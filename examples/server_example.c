/**
 * @file server_example.c
 * @brief Modbus TCP Server – example / hardware bringup template.
 *
 * How to use:
 *   1. Edit the "MODIFY HERE" sections below.
 *   2. Build:  make examples
 *   3. Run:    ./build/server_example
 *
 * The server listens for a Modbus TCP master (e.g. Modbus Poll, another device)
 * and lets it read / write the register bank defined in this file.
 *
 * Sections to modify:
 *   ① Network settings  – port, unit ID
 *   ② Register map      – define your register names and addresses
 *   ③ Read callback     – map register addresses to your real hardware values
 *   ④ Write callback    – apply written values to your real hardware
 */

#define _GNU_SOURCE

#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

#include "modbus_tcp_server.h"

/* ════════════════════════════════════════════════════════════════════════════
 * ① NETWORK SETTINGS  (MODIFY HERE)
 * ════════════════════════════════════════════════════════════════════════════ */

#define SERVER_PORT         502u  /* Use 502 in production (requires root / CAP_NET_BIND_SERVICE) */
#define SERVER_UNIT_ID      1u      /* Modbus unit / slave ID                           */

/* ════════════════════════════════════════════════════════════════════════════
 * ② REGISTER MAP  (MODIFY HERE)
 *
 * Define your register addresses as named constants.
 * All addresses are 0-based (Modbus address 40001 = offset 0). *
 * R   = read-only registers (served by on_read callback)
 * R/W = read/write registers (served by on_read and on_write callbacks) * ════════════════════════════════════════════════════════════════════════════ */

#define REG_TEMPERATURE         0x00  /* R   – temperature × 10  (e.g. 253 = 25.3 °C)    */
#define REG_HUMIDITY            0x01  /* R   – relative humidity % (0–100)                */
#define REG_STATUS              0x02  /* R   – device status bitmask                      */
#define REG_UPTIME_LOW          0x03  /* R   – uptime seconds, low  16 bits               */
#define REG_UPTIME_HIGH         0x04  /* R   – uptime seconds, high 16 bits               */
#define REG_SETPOINT            0x0A /* R/W – target setpoint                            */
#define REG_CONTROL             0x0B /* R/W – control flags                              */
#define REG_DEVICE_ID           0x14 /* R   – firmware / device identifier               */

/**
 * Total addressable register count. Requests outside [0, REG_BANK_SIZE) are
 * rejected with an Illegal Data Address exception to protect against buffer
 * overflows and undefined behavior. Adjust this to match your register layout
 * (must be at least REG_DEVICE_ID + 1).
 */
#define REG_BANK_SIZE           32u

/* ── Application context passed through cfg.userdata ───────────────────── */

typedef struct {
    uint16_t        reg_bank[REG_BANK_SIZE];
    pthread_mutex_t reg_lock;
} server_app_ctx_t;

/* ── Graceful shutdown ──────────────────────────────────────────────────── */

static volatile int g_running = 1;

static void handle_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ════════════════════════════════════════════════════════════════════════════
 * LOG CONTEXT  – passed as .log_userdata to the configuration
 *
 * Carries:
 *   node_name – printed in every log line (useful when running multiple
 *               server instances in the same process or log file)
 *   min_level – messages below this level are suppressed
 *
 * MODIFY HERE: adjust node_name or min_level for your deployment.
 * ════════════════════════════════════════════════════════════════════════════ */

typedef struct {
    const char        *node_name;
    mb_tcp_log_level_t min_level;
} server_log_ctx_t;

/* ════════════════════════════════════════════════════════════════════════════
 * LINK CONTEXT  – passed as .link_userdata to the configuration
 *
 * Tracks the number of currently connected Modbus masters so the application
 * can make decisions based on connection state (e.g. hold outputs safe while
 * no master is connected).
 *
 * MODIFY HERE: add fields your application needs (e.g. last client IP string,
 * timestamps, per-client session state).
 * ════════════════════════════════════════════════════════════════════════════ */

typedef struct {
    volatile int active_connections;
} link_ctx_t;

/* ════════════════════════════════════════════════════════════════════════════
 * LOG SINK  – prints timestamped messages to stderr
 * ════════════════════════════════════════════════════════════════════════════ */

static void server_log(void *userdata, mb_tcp_log_level_t level,
                        const char *fmt, va_list ap)
{
    /* userdata = cfg.log_userdata (passed from configuration).
     * In this example, it points to server_log_ctx_t which holds node_name
     * and min_level. Can be NULL if not needed by the logging function. */
    server_log_ctx_t *log_ctx = (server_log_ctx_t *)userdata;

    if (level < log_ctx->min_level) {
        return;
    }

    static const char * const level_tag[] = {
        [MB_TCP_LOG_DEBUG] = "DEBUG",
        [MB_TCP_LOG_INFO]  = "INFO ",
        [MB_TCP_LOG_WARN]  = "WARN ",
        [MB_TCP_LOG_ERROR] = "ERROR",
    };

    char time_buf[16];
    time_t now = time(NULL);
    struct tm tm_info;
    localtime_r(&now, &tm_info);
    strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &tm_info);

    fprintf(stderr, "[%s][%s][%s] ", time_buf, log_ctx->node_name,
            level_tag[level]);
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
}

/* ════════════════════════════════════════════════════════════════════════════
 * LINK CALLBACK  – notified on every client connect / disconnect
 *
 * Uses link_ctx_t (passed via .link_userdata) to maintain an accurate count
 * of active master connections.  The count can be read from any thread to
 * decide whether the device should hold outputs in a safe state.
 * ════════════════════════════════════════════════════════════════════════════ */

static void on_link_change(void *userdata, int fd, int connected)
{
    /* userdata = cfg.link_userdata (passed from configuration).
     * In this example, it points to link_ctx_t which tracks active connections.
     * Can be NULL if connection tracking is not needed. */
    link_ctx_t *link_ctx = (link_ctx_t *)userdata;

    if (connected) {
        link_ctx->active_connections++;
        fprintf(stderr, "[INFO ] Client connected    (fd=%d) — active masters: %d\n",
                fd, link_ctx->active_connections);
    } else {
        if (link_ctx->active_connections > 0) {
            link_ctx->active_connections--;
        }
        fprintf(stderr, "[INFO ] Client disconnected (fd=%d) — active masters: %d\n",
                fd, link_ctx->active_connections);
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * ③ READ CALLBACK  (MODIFY HERE)
 *
 * Called by the server when a Modbus master issues FC03 or FC04.
 * Fill `out[0..qty-1]` with current register values.
 *
 * function_code is MODBUS_FUNC_READ_HOLDING_REGISTERS (0x03) for FC03 or
 * MODBUS_FUNC_READ_INPUT_REGISTERS (0x04) for FC04.  Use it to serve
 * Holding and Input register spaces from separate data sources when needed.
 *
 * If this callback is NULL (not set in config), the server automatically
 * sends a Server Device Failure exception to the master.
 *
 * Return values:
 *   0                              – success
 *   MODBUS_EX_ILLEGAL_DATA_ADDRESS – address out of range (sends exception 0x02)
 *   MODBUS_EX_ILLEGAL_DATA_VALUE   – value constraint violation (sends exception 0x03)
 *   MODBUS_EX_SERVER_DEVICE_FAILURE or any other non-zero – internal error (sends exception 0x04)
 * ════════════════════════════════════════════════════════════════════════════ */

static int on_read(uint8_t function_code, uint16_t addr, uint16_t qty,
                   uint16_t *out, void *userdata)
{
    /* userdata = cfg.userdata (passed from configuration).
     * In this example it points to server_app_ctx_t, which owns the register
     * bank and its mutex. */
    server_app_ctx_t *app = (server_app_ctx_t *)userdata;
    if (!app) {
        return MODBUS_EX_SERVER_DEVICE_FAILURE;
    }

    /* MODIFY HERE: if your device distinguishes Holding Registers (FC03) from
     * Input Registers (FC04), branch on function_code here and serve each from
     * its own data source.  In this example both map to the same register bank. */
    (void)function_code;

    if ((uint32_t)addr + (uint32_t)qty > REG_BANK_SIZE) {
        fprintf(stderr, "[WARN ] Read out of range: addr=%u qty=%u\n",
                (unsigned)addr, (unsigned)qty);
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    /* ── Refresh read-only registers with live values before serving ────
     * THREAD SAFETY: Acquire lock before reading/writing the register bank
     * to ensure consistency when multiple masters or threads access registers. */
    pthread_mutex_lock(&app->reg_lock);

    /* MODIFY HERE: replace these stubs with real sensor reads. */
    app->reg_bank[REG_TEMPERATURE] = 253u;                  /* 25.3 °C stub  */
    app->reg_bank[REG_HUMIDITY]    = 60u;                   /* 60 % stub     */
    app->reg_bank[REG_STATUS]      = 0x0001u;               /* "online" flag */

    uint32_t uptime_sec = (uint32_t)time(NULL);             /* simple uptime */
    app->reg_bank[REG_UPTIME_LOW]  = (uint16_t)(uptime_sec & 0xFFFFu);
    app->reg_bank[REG_UPTIME_HIGH] = (uint16_t)(uptime_sec >> 16);

    app->reg_bank[REG_DEVICE_ID]   = 0x0100u;               /* version 1.0   */

    for (uint16_t i = 0u; i < qty; i++) {
        out[i] = app->reg_bank[addr + i];
    }

    pthread_mutex_unlock(&app->reg_lock);
    return 0;
}

/* ════════════════════════════════════════════════════════════════════════════
 * ④ WRITE CALLBACK  (MODIFY HERE)
 *
 * Called by the server when a Modbus master issues FC06 / FC16.
 * Apply `data[0..qty-1]` to the appropriate hardware outputs.
 *
 * If this callback is NULL (not set in config), the server automatically
 * sends a Server Device Failure exception to the master.
 *
 * Return values:
 *   0                              – success
 *   MODBUS_EX_ILLEGAL_DATA_ADDRESS – address out of range (sends exception 0x02)
 *   MODBUS_EX_ILLEGAL_DATA_VALUE   – value constraint violation (sends exception 0x03)
 *   MODBUS_EX_SERVER_DEVICE_FAILURE or any other non-zero – internal error (sends exception 0x04)
 * ════════════════════════════════════════════════════════════════════════════ */

static int on_write(uint16_t addr, uint16_t qty, const uint16_t *data, void *userdata)
{
    /* userdata = cfg.userdata (passed from configuration).
     * In this example it points to server_app_ctx_t, which owns the register
     * bank and its mutex. */
    server_app_ctx_t *app = (server_app_ctx_t *)userdata;
    if (!app) {
        return MODBUS_EX_SERVER_DEVICE_FAILURE;
    }

    /* CRITICAL: Validate address range before accessing reg_bank.
     * Cast to uint32_t to prevent integer overflow when adding addr + qty. */
    if ((uint32_t)addr + (uint32_t)qty > REG_BANK_SIZE) {
        fprintf(stderr, "[WARN ] Write out of range: addr=%u qty=%u\n",
                (unsigned)addr, (unsigned)qty);
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    /* ── Apply writes to register bank ──
     * THREAD SAFETY: Acquire lock before writing to the register bank
     * to ensure consistency when multiple masters or threads access registers. */
    pthread_mutex_lock(&app->reg_lock);
    for (uint16_t i = 0u; i < qty; i++) {
        app->reg_bank[addr + i] = data[i];
    }
    pthread_mutex_unlock(&app->reg_lock);

    /* MODIFY HERE: act on specific register writes. */
    for (uint16_t i = 0u; i < qty; i++) {
        uint16_t reg_addr = (uint16_t)(addr + i);
        switch (reg_addr) {
            case REG_SETPOINT:
                fprintf(stderr, "[INFO ] Setpoint updated → %u\n", data[i]);
                /* TODO: apply data[i] to your hardware setpoint */
                break;
            case REG_CONTROL:
                fprintf(stderr, "[INFO ] Control register → 0x%04X\n", data[i]);
                /* TODO: interpret control flags and drive outputs */
                break;
            default:
                break;
        }
    }

    return 0;
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    /* ── Application context ──────────────────────────────────────────────
     * Passed through cfg.userdata and received by on_read / on_write.
     * Keep application state here instead of using global variables. */
    server_app_ctx_t app_ctx;
    memset(&app_ctx, 0, sizeof(app_ctx));
    if (pthread_mutex_init(&app_ctx.reg_lock, NULL) != 0) {
        fprintf(stderr, "[ERROR] Failed to initialise register lock\n");
        return 1;
    }

    /* ── Log context ───────────────────────────────────────────────────────
     * MODIFY HERE: set node_name to identify this server instance in logs,
     * and adjust min_level to control verbosity.     *
     * To disable logging entirely, set cfg.logv = NULL below.     * ──────────────────────────────────────────────────────────────────── */
    server_log_ctx_t log_ctx = {
        .node_name = "MODBUS-SRV",      /* MODIFY HERE */
        .min_level = MB_TCP_LOG_DEBUG,  /* MODIFY HERE */
    };

    /* ── Link context ───────────────────────────────────────────────────────
     * Tracks connected masters.  Read link_ctx.active_connections from any
     * thread to know whether at least one master is currently online.     *
     * If you don't need connection tracking, set cfg.on_link = NULL below
     * and cfg.link_userdata = NULL (no callback will be invoked).     * ──────────────────────────────────────────────────────────────────── */
    link_ctx_t link_ctx = {
        .active_connections = 0,
    };

    /* ── Start the Modbus TCP server ───────────────────────────────────── */
    mb_tcp_server_ctx_t    server = {0};
    mb_tcp_server_config_t cfg = {0};

    mb_tcp_server_config_init(&cfg);       /* fill optional fields with safe defaults */

    /* ── Required: protocol-critical fields ──────────────────────────── */
    cfg.port    = SERVER_PORT;
    cfg.unit_id = SERVER_UNIT_ID;

    /* ── Required for useful behaviour: register callbacks ───────────── */
    cfg.on_read  = on_read;
    cfg.on_write = on_write;
    cfg.userdata = &app_ctx;               /* passed to on_read / on_write */

    /* ── Optional: logging ───────────────────────────────────────────── */
    cfg.logv         = server_log;         /* set to NULL for silent mode */
    cfg.log_userdata = &log_ctx;

    /* ── Optional: connection event tracking ─────────────────────────── */
    cfg.on_link       = on_link_change;    /* set to NULL to disable */
    cfg.link_userdata = &link_ctx;

    /* ── Optional: tunables (already set to defaults by config_init) ─── */
    /* cfg.max_clients    = 3;             uncomment to cap connections    */
    /* cfg.recv_timeout_ms = 10000;        uncomment to extend timeout     */

    if (mb_tcp_server_start(&server, &cfg) != 0) {
        fprintf(stderr, "[ERROR] Failed to start server on port %u\n",
                (unsigned)SERVER_PORT);
        pthread_mutex_destroy(&app_ctx.reg_lock);
        return 1;
    }

    fprintf(stderr, "[INFO ] Modbus TCP server running on port %u "
                    "(unit ID %u) — press Ctrl-C to stop\n",
            (unsigned)SERVER_PORT, (unsigned)SERVER_UNIT_ID);
    fprintf(stderr, "[INFO ] Register map:\n");
    fprintf(stderr, "          REG_TEMPERATURE  = %u (R)\n",   REG_TEMPERATURE);
    fprintf(stderr, "          REG_HUMIDITY     = %u (R)\n",   REG_HUMIDITY);
    fprintf(stderr, "          REG_STATUS       = %u (R)\n",   REG_STATUS);
    fprintf(stderr, "          REG_UPTIME_LOW   = %u (R)\n",   REG_UPTIME_LOW);
    fprintf(stderr, "          REG_UPTIME_HIGH  = %u (R)\n",   REG_UPTIME_HIGH);
    fprintf(stderr, "          REG_SETPOINT     = %u (R/W)\n", REG_SETPOINT);
    fprintf(stderr, "          REG_CONTROL      = %u (R/W)\n", REG_CONTROL);
    fprintf(stderr, "          REG_DEVICE_ID    = %u (R)\n",   REG_DEVICE_ID);

    /* ── Wait for Ctrl-C ───────────────────────────────────────────────── */
    while (g_running) {
        sleep(1);
    }

    fprintf(stderr, "\n[INFO ] Shutting down...\n");
    mb_tcp_server_stop(&server);
    pthread_mutex_destroy(&app_ctx.reg_lock);
    fprintf(stderr, "[INFO ] Server stopped.\n");
    return 0;
}
