/**
 * @file rtu_client_example.c
 * @brief Modbus RTU Client - hardware bringup template.
 */

#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "modbus_rtu_client.h"

#define SERIAL_DEVICE        "/dev/ttyUSB0"
#define SERIAL_BAUD_RATE     9600u
#define SERVER_UNIT_ID       1u
#define RESPONSE_TIMEOUT_MS  1000u
#define POLL_INTERVAL_SEC    2u

#define REG_TEMPERATURE      0x00u
#define REG_SETPOINT         0x0Au

static volatile int g_running = 1;

static void handle_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

static void client_log(void *userdata, mb_rtu_log_level_t level,
                       const char *fmt, va_list ap)
{
    (void)userdata;

    static const char * const level_tag[] = {
        [MB_RTU_LOG_DEBUG] = "DEBUG",
        [MB_RTU_LOG_INFO]  = "INFO ",
        [MB_RTU_LOG_WARN]  = "WARN ",
        [MB_RTU_LOG_ERROR] = "ERROR",
    };

    fprintf(stderr, "[RTU-CLIENT][%s] ", level_tag[level]);
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
}

static const char *client_err_str(int rc)
{
    switch (rc) {
        case MB_RTU_CLIENT_OK: return "OK";
        case MB_RTU_CLIENT_ERR_ARG: return "ERR_ARG";
        case MB_RTU_CLIENT_ERR_NOT_CONNECTED: return "ERR_NOT_CONNECTED";
        case MB_RTU_CLIENT_ERR_TRANSPORT: return "ERR_TRANSPORT";
        case MB_RTU_CLIENT_ERR_TIMEOUT: return "ERR_TIMEOUT";
        case MB_RTU_CLIENT_ERR_FRAME: return "ERR_FRAME";
        case MB_RTU_CLIENT_ERR_UNIT_ID: return "ERR_UNIT_ID";
        default:
            if (rc > 0) {
                return "MODBUS_EXCEPTION";
            }
            return "UNKNOWN";
    }
}

int main(void)
{
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    mb_rtu_client_config_t cfg = {
        .device = SERIAL_DEVICE,
        .baud_rate = SERIAL_BAUD_RATE,
        .data_bits = 8u,
        .stop_bits = 1u,
        .parity = MB_RTU_PARITY_NONE,
        .unit_id = SERVER_UNIT_ID,
        .response_timeout_ms = RESPONSE_TIMEOUT_MS,
        .logv = client_log,
    };

    mb_rtu_client_ctx_t client = {0};
    if (mb_rtu_client_connect(&client, &cfg) != 0) {
        fprintf(stderr, "[ERROR] Failed to open RTU device %s\n", SERIAL_DEVICE);
        return 1;
    }

    uint16_t setpoint = 300u;
    int rc = mb_rtu_client_write_single_register(&client, REG_SETPOINT, setpoint);
    if (rc != MB_RTU_CLIENT_OK) {
        fprintf(stderr, "[INIT] Write setpoint failed: %s (rc=%d)\n",
                client_err_str(rc), rc);
    }

    while (g_running) {
        uint16_t temperature_raw = 0u;
        rc = mb_rtu_client_read_holding_registers(&client, REG_TEMPERATURE,
                                                  1u, &temperature_raw);
        if (rc == MB_RTU_CLIENT_OK) {
            printf("[POLL] Temperature: %.1f C\n",
                   (double)temperature_raw / 10.0);
        } else {
            fprintf(stderr, "[POLL] Read failed: %s (rc=%d)\n",
                    client_err_str(rc), rc);
        }

        sleep(POLL_INTERVAL_SEC);
    }

    mb_rtu_client_disconnect(&client);
    return 0;
}
