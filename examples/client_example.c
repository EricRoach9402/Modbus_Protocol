/**
 * @file client_example.c
 * @brief Modbus TCP Client – example / hardware bringup template.
 *
 * How to use:
 *   1. Edit the "MODIFY HERE" sections below.
 *   2. Build:  make examples
 *   3. Run:    ./build/client_example
 *
 * The client connects to a remote Modbus TCP server, polls a set of registers
 * on a configurable interval, and writes a value once at startup.
 *
 * Sections to modify:
 *   ① Network settings  – IP, port, unit ID, timeouts
 *   ② Register map      – define your register names and addresses
 *   ③ Poll loop         – which registers to read each cycle, how to interpret them
 *   ④ Write example     – what to write at startup or on demand
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

#include "modbus_tcp_client.h"

/* ════════════════════════════════════════════════════════════════════════════
 * ① NETWORK SETTINGS  (MODIFY HERE)
 * ════════════════════════════════════════════════════════════════════════════ */

#define SERVER_IP               "192.168.1.90"   /* Remote server IPv4 address */
#define SERVER_PORT             502         /* Match server_example.c; use 502 in production */
#define SERVER_UNIT_ID          1u             /* Modbus unit / slave ID      */

#define CONNECT_TIMEOUT_SEC     5u             /* TCP handshake timeout       */
#define RESPONSE_TIMEOUT_MS     2000u          /* Per-request timeout         */

/** Polling interval in seconds between read cycles. */
#define POLL_INTERVAL_SEC       2u

/* Optionally bind the socket to a specific interface (e.g. "eth0").
 * Set to NULL to let the OS choose based on routing. */
#define BIND_IFACE              NULL

/* ════════════════════════════════════════════════════════════════════════════
 * ② REGISTER MAP  (MODIFY HERE)
 *
 * Mirror the server-side register map.
 * All addresses are 0-based (Modbus address 40001 = offset 0).
 *
 * R   = read-only (use mb_tcp_client_read_holding_registers)
 * R/W = read or write (use read or write functions as appropriate)
 * ════════════════════════════════════════════════════════════════════════════ */

#define REG_TEMPERATURE         0x00   /* R   – temperature × 10  (e.g. 253 = 25.3 °C) */
#define REG_HUMIDITY            0x01   /* R   – relative humidity %                     */
#define REG_STATUS              0x02   /* R   – device status bitmask                   */
#define REG_UPTIME_LOW          0x03   /* R   – uptime seconds, low  16 bits            */
#define REG_UPTIME_HIGH         0x04   /* R   – uptime seconds, high 16 bits            */
#define REG_SETPOINT            0x0A   /* R/W – target setpoint                         */
#define REG_CONTROL             0x0B   /* R/W – control flags                           */
#define REG_DEVICE_ID           0x14   /* R   – firmware / device identifier            */

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
 *   device_name – printed in every log line so multi-client apps stay clear
 *   min_level   – messages below this level are suppressed
 *
 * MODIFY HERE: adjust min_level or add fields your application needs.
 * ════════════════════════════════════════════════════════════════════════════ */

typedef struct {
    const char        *device_name;
    mb_tcp_log_level_t min_level;
} client_log_ctx_t;

/* ════════════════════════════════════════════════════════════════════════════
 * LOG SINK  – prints timestamped messages to stderr
 * ════════════════════════════════════════════════════════════════════════════ */

static void client_log(void *userdata, mb_tcp_log_level_t level,
                        const char *fmt, va_list ap)
{
    /* userdata = cfg.log_userdata (passed from configuration).
     * In this example, it points to client_log_ctx_t which holds device_name
     * and min_level. Can be NULL if not needed by the logging function. */
    client_log_ctx_t *log_ctx = (client_log_ctx_t *)userdata;

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

    fprintf(stderr, "[%s][%s][%s] ", time_buf, log_ctx->device_name,
            level_tag[level]);
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
}

/* ── Error-code helper ──────────────────────────────────────────────────── */

/** Translates a client return code to a human-readable string for logging. */
static const char *client_err_str(int rc)
{
    switch (rc) {
        case MB_TCP_CLIENT_OK:               return "OK";
        case MB_TCP_CLIENT_ERR_ARG:          return "ERR_ARG";
        case MB_TCP_CLIENT_ERR_NOT_CONNECTED: return "ERR_NOT_CONNECTED";
        case MB_TCP_CLIENT_ERR_TRANSPORT:    return "ERR_TRANSPORT";
        case MB_TCP_CLIENT_ERR_TIMEOUT:      return "ERR_TIMEOUT";
        case MB_TCP_CLIENT_ERR_FRAME:        return "ERR_FRAME";
        case MB_TCP_CLIENT_ERR_TID:          return "ERR_TID";
        default:
            if (rc > 0) {
                return "MODBUS_EXCEPTION";
            }
            return "UNKNOWN";
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * ③ POLL LOOP  (MODIFY HERE)
 *
 * Called once per POLL_INTERVAL_SEC.
 * Read whichever registers your application cares about and interpret them.
 * ════════════════════════════════════════════════════════════════════════════ */

static void poll_once(mb_tcp_client_ctx_t *cli)
{
    int rc;

    /* All read/write functions return:
     *   - MB_TCP_CLIENT_OK (0)           : success
     *   - Positive value (1-127)         : Modbus exception code (server rejected)
     *   - Negative value (MB_TCP_CLIENT_ERR_*) : transport/framing error
     * The helper function client_err_str() converts these to human-readable strings. */

    /* ── Read a contiguous block: REG_TEMPERATURE … REG_UPTIME_HIGH ─────
     * It's more efficient to read several adjacent registers in one request
     * rather than making separate calls. Adjust start address and count
     * to match your register layout.
     * The buffer is indexed by register offset within the block:
     *   block[0] = REG_TEMPERATURE, block[1] = REG_HUMIDITY, etc.
     * ──────────────────────────────────────────── */
    uint16_t block[5] = {0u};  /* REG_TEMPERATURE=0 through REG_UPTIME_HIGH=4 */
    rc = mb_tcp_client_read_holding_registers(cli, REG_TEMPERATURE, 5u, block);
    if (rc == MB_TCP_CLIENT_OK) {
        /* Interpret register values according to your protocol definition.
         * In this example:
         *   - Temperature is stored as (°C × 10), so divide by 10 to get °C.
         *   - Humidity and status are already in the correct format.
         *   - Uptime is split across two 16-bit registers; combine them. */
        float    temperature = (float)block[REG_TEMPERATURE] / 10.0f;
        uint16_t humidity    = block[REG_HUMIDITY];
        uint16_t status      = block[REG_STATUS];
        uint32_t uptime      = ((uint32_t)block[REG_UPTIME_HIGH] << 16)
                             | (uint32_t)block[REG_UPTIME_LOW];

        printf("[POLL] Temperature: %.1f °C  |  Humidity: %u %%  |  "
               "Status: 0x%04X  |  Uptime: %u s\n",
               temperature, (unsigned)humidity,
               (unsigned)status, (unsigned)uptime);
    } else {
        fprintf(stderr, "[POLL] Read block failed: %s (rc=%d)\n",
                client_err_str(rc), rc);
    }

    /* ── Read individual register: setpoint ──────────────────────────── */
    uint16_t setpoint = 0u;
    rc = mb_tcp_client_read_holding_registers(cli, REG_SETPOINT, 1u, &setpoint);
    if (rc == MB_TCP_CLIENT_OK) {
        printf("[POLL] Setpoint: %u\n", (unsigned)setpoint);
    } else {
        fprintf(stderr, "[POLL] Read setpoint failed: %s (rc=%d)\n",
                client_err_str(rc), rc);
    }

    uint16_t control = 0u;
    rc = mb_tcp_client_read_holding_registers(cli, REG_CONTROL, 1u, &control);
    if (rc == MB_TCP_CLIENT_OK) {
        printf("[POLL] Control: 0x%04X\n", (unsigned)control);
    } else {
        fprintf(stderr, "[POLL] Read control failed: %s (rc=%d)\n",
                client_err_str(rc), rc);
    }

    uint16_t device_id = 0u;
    rc = mb_tcp_client_read_holding_registers(cli, REG_DEVICE_ID, 1u, &device_id);
    if (rc == MB_TCP_CLIENT_OK) {
        printf("[POLL] Device ID: 0x%04X\n", (unsigned)device_id);
    } else {
        fprintf(stderr, "[POLL] Read device ID failed: %s (rc=%d)\n",
                client_err_str(rc), rc);
    }

    /* MODIFY HERE: add more reads as your register map requires. */
}

/* ════════════════════════════════════════════════════════════════════════════
 * ④ WRITE EXAMPLE  (MODIFY HERE)
 *
 * Demonstrates writing a single register and a multi-register block.
 * Called once after the connection is established.
 * ════════════════════════════════════════════════════════════════════════════ */

static void initial_write(mb_tcp_client_ctx_t *cli)
{
    int rc;

    /* FC06 – Write Single Register (writes exactly 1 register).
     * Use this if the device requires or prefers the single-register function code.
     * Some older devices or specific protocols may mandate FC06. */
    uint16_t setpoint = 300u;   /* MODIFY HERE: your startup setpoint value */
    rc = mb_tcp_client_write_single_register(cli, REG_SETPOINT, setpoint);
    if (rc == MB_TCP_CLIENT_OK) {
        printf("[INIT] Setpoint written: %u\n", (unsigned)setpoint);
    } else {
        fprintf(stderr, "[INIT] Write setpoint failed: %s (rc=%d)\n",
                client_err_str(rc), rc);
    }

    /* FC16 – Write Multiple Registers (writes 1 or more contiguous registers).
     * More efficient when writing multiple adjacent registers in one request.
     * FC16 can write a single register too (qty=1) if preferred; it's valid
     * per the Modbus specification. Choose FC06 vs FC16 based on device requirements. */
    uint16_t init_regs[2] = {
        300u,       /* REG_SETPOINT  – MODIFY HERE */
        0x0001u,    /* REG_CONTROL   – MODIFY HERE: startup control flags */
    };
    rc = mb_tcp_client_write_multiple_registers(cli, REG_SETPOINT, 2u, init_regs);
    if (rc == MB_TCP_CLIENT_OK) {
        printf("[INIT] Setpoint+Control written (FC16): setpoint=%u control=0x%04X\n",
               (unsigned)init_regs[0], (unsigned)init_regs[1]);
    } else {
        fprintf(stderr, "[INIT] FC16 write failed: %s (rc=%d)\n",
                client_err_str(rc), rc);
    }
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    /* ── Log context ───────────────────────────────────────────────────────
     * MODIFY HERE: set device_name to identify this client in log output,
     * and adjust min_level to control verbosity:
     *   MB_TCP_LOG_DEBUG  – everything (connection details, TID, etc.)
     *   MB_TCP_LOG_INFO   – informational messages and above
     *   MB_TCP_LOG_WARN   – warnings and errors only
     *   MB_TCP_LOG_ERROR  – errors only
     *
     * To disable logging entirely, set cfg.logv = NULL (see below).
     * ──────────────────────────────────────────────────────────────────── */
    client_log_ctx_t log_ctx = {
        .device_name = "SENSOR-01",       /* MODIFY HERE */
        .min_level   = MB_TCP_LOG_DEBUG,  /* MODIFY HERE */
    };

    /* ── Build configuration ───────────────────────────────────────────── */
    mb_tcp_client_config_t cfg = {
        .remote_host         = SERVER_IP,
        .port                = SERVER_PORT,
        .unit_id             = SERVER_UNIT_ID,
        .bind_iface          = BIND_IFACE,          /* NULL = system default */
        .connect_timeout_sec = CONNECT_TIMEOUT_SEC,
        .response_timeout_ms = RESPONSE_TIMEOUT_MS,
        .logv                = client_log,          /* Set to NULL for silent mode */
        .log_userdata        = &log_ctx,            /* Can be NULL if logv doesn't need it */
    };

    printf("[INFO] Connecting to Modbus TCP server at %s:%u (unit ID %u)...\n",
           SERVER_IP, (unsigned)SERVER_PORT, (unsigned)SERVER_UNIT_ID);

    mb_tcp_client_ctx_t client = {0};

    if (mb_tcp_client_connect(&client, &cfg) != 0) {
        fprintf(stderr, "[ERROR] Connection failed — is the server running?\n");
        return 1;
    }

    printf("[INFO] Connected.  Polling every %u second(s).  "
           "Press Ctrl-C to stop.\n", (unsigned)POLL_INTERVAL_SEC);

    /* ── One-time startup writes ───────────────────────────────────────── */
    initial_write(&client);

    /* ── Main polling loop ─────────────────────────────────────────────── */
    while (g_running) {
        poll_once(&client);
        sleep(POLL_INTERVAL_SEC);
    }

    printf("\n[INFO] Shutting down...\n");
    mb_tcp_client_disconnect(&client);
    printf("[INFO] Disconnected.\n");
    return 0;
}
