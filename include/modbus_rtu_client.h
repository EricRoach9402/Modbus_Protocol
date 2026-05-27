/**
 * @file modbus_rtu_client.h
 * @brief Modbus RTU client (master) application-facing API.
 *
 * Mirrors modbus_tcp_client.h where possible. Callers configure a serial port
 * instead of remote_host/port, then use the same blocking register operations.
 */

#ifndef MODBUS_RTU_CLIENT_H
#define MODBUS_RTU_CLIENT_H

#include <pthread.h>
#include <stdint.h>

#include "modbus_rtu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -- Error / return codes ------------------------------------------------ */

#define MB_RTU_CLIENT_OK                 0
#define MB_RTU_CLIENT_ERR_ARG           (-1)
#define MB_RTU_CLIENT_ERR_NOT_CONNECTED (-2)
#define MB_RTU_CLIENT_ERR_TRANSPORT     (-3)
#define MB_RTU_CLIENT_ERR_TIMEOUT       (-4)
#define MB_RTU_CLIENT_ERR_FRAME         (-5)
#define MB_RTU_CLIENT_ERR_UNIT_ID       (-6)

/* -- Configuration ------------------------------------------------------- */

typedef struct mb_rtu_client_config {
    const char     *device;              /**< Serial device path, e.g. "/dev/ttyUSB0". */
    uint32_t        baud_rate;           /**< 0 = 9600. */
    uint8_t         data_bits;           /**< 0 = 8. */
    uint8_t         stop_bits;           /**< 0 = 1. */
    mb_rtu_parity_t parity;              /**< Default: MB_RTU_PARITY_NONE. */
    uint8_t         unit_id;             /**< RTU slave address sent in every request. */
    uint32_t        response_timeout_ms; /**< Per-request timeout; 0 = default (1 s). */

    mb_rtu_logv_fn  logv;                /**< Optional log sink; NULL = silent mode. */
    void           *log_userdata;        /**< Context passed to logv; can be NULL. */
} mb_rtu_client_config_t;

/* -- Runtime context ----------------------------------------------------- */

typedef struct mb_rtu_client_ctx {
    int                    fd;           /**< Open serial fd; -1 when disconnected. */
    pthread_mutex_t        lock;         /**< Serializes concurrent requests. */
    mb_rtu_client_config_t cfg;          /**< Copy of configuration. */
} mb_rtu_client_ctx_t;

/* -- Lifecycle ----------------------------------------------------------- */

int mb_rtu_client_connect(mb_rtu_client_ctx_t *ctx, const mb_rtu_client_config_t *cfg);
void mb_rtu_client_disconnect(mb_rtu_client_ctx_t *ctx);

/* -- Register access ----------------------------------------------------- */

int mb_rtu_client_read_holding_registers(mb_rtu_client_ctx_t *ctx,
                                         uint16_t addr, uint16_t qty,
                                         uint16_t *out);

int mb_rtu_client_write_single_register(mb_rtu_client_ctx_t *ctx,
                                        uint16_t addr, uint16_t value);

int mb_rtu_client_write_multiple_registers(mb_rtu_client_ctx_t *ctx,
                                           uint16_t addr, uint16_t qty,
                                           const uint16_t *data);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_CLIENT_H */
