/**
 * @file rtu_server_example.c
 * @brief Modbus RTU Server - hardware bringup template.
 */

#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "modbus_rtu_server.h"

#define SERIAL_DEVICE      "/dev/ttyUSB0"
#define SERIAL_BAUD_RATE   9600u
#define SERVER_UNIT_ID     1u

#define REG_TEMPERATURE    0x00u
#define REG_STATUS         0x01u
#define REG_SETPOINT       0x0Au
#define REG_BANK_SIZE      32u

typedef struct {
    uint16_t reg_bank[REG_BANK_SIZE];
    pthread_mutex_t reg_lock;
} server_app_ctx_t;

static volatile int g_running = 1;

static void handle_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

static void server_log(void *userdata, mb_rtu_log_level_t level,
                       const char *fmt, va_list ap)
{
    (void)userdata;

    static const char * const level_tag[] = {
        [MB_RTU_LOG_DEBUG] = "DEBUG",
        [MB_RTU_LOG_INFO]  = "INFO ",
        [MB_RTU_LOG_WARN]  = "WARN ",
        [MB_RTU_LOG_ERROR] = "ERROR",
    };

    fprintf(stderr, "[RTU-SERVER][%s] ", level_tag[level]);
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
}

static int on_read(uint8_t function_code, uint16_t addr, uint16_t qty,
                   uint16_t *out, void *userdata)
{
    server_app_ctx_t *app = (server_app_ctx_t *)userdata;
    (void)function_code;

    if (!app || (uint32_t)addr + (uint32_t)qty > REG_BANK_SIZE) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    pthread_mutex_lock(&app->reg_lock);

    app->reg_bank[REG_TEMPERATURE] = 253u;
    app->reg_bank[REG_STATUS] = 0x0001u;

    for (uint16_t i = 0u; i < qty; i++) {
        out[i] = app->reg_bank[addr + i];
    }

    pthread_mutex_unlock(&app->reg_lock);
    return 0;
}

static int on_write(uint16_t addr, uint16_t qty, const uint16_t *data, void *userdata)
{
    server_app_ctx_t *app = (server_app_ctx_t *)userdata;

    if (!app || !data || (uint32_t)addr + (uint32_t)qty > REG_BANK_SIZE) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    pthread_mutex_lock(&app->reg_lock);
    for (uint16_t i = 0u; i < qty; i++) {
        app->reg_bank[addr + i] = data[i];
    }
    pthread_mutex_unlock(&app->reg_lock);

    if (addr <= REG_SETPOINT && (uint32_t)addr + (uint32_t)qty > REG_SETPOINT) {
        fprintf(stderr, "[INFO] Setpoint updated\n");
    }

    return 0;
}

int main(void)
{
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    server_app_ctx_t app_ctx;
    memset(&app_ctx, 0, sizeof(app_ctx));
    if (pthread_mutex_init(&app_ctx.reg_lock, NULL) != 0) {
        return 1;
    }

    mb_rtu_server_config_t cfg;
    mb_rtu_server_config_init(&cfg);
    cfg.device = SERIAL_DEVICE;
    cfg.baud_rate = SERIAL_BAUD_RATE;
    cfg.unit_id = SERVER_UNIT_ID;
    cfg.on_read = on_read;
    cfg.on_write = on_write;
    cfg.userdata = &app_ctx;
    cfg.logv = server_log;

    mb_rtu_server_ctx_t server = {0};
    if (mb_rtu_server_start(&server, &cfg) != 0) {
        fprintf(stderr, "[ERROR] Failed to start RTU server on %s\n", SERIAL_DEVICE);
        pthread_mutex_destroy(&app_ctx.reg_lock);
        return 1;
    }

    fprintf(stderr, "[INFO] Modbus RTU server running on %s, unit ID %u\n",
            SERIAL_DEVICE, (unsigned)SERVER_UNIT_ID);

    while (g_running) {
        sleep(1);
    }

    mb_rtu_server_stop(&server);
    pthread_mutex_destroy(&app_ctx.reg_lock);
    return 0;
}
