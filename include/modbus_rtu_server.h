/**
 * @file modbus_rtu_server.h
 * @brief Modbus RTU server (slave) application-facing API.
 *
 * Provides the same register callback model as the TCP server. The RTU layer
 * owns serial-port setup, CRC validation, and frame dispatch.
 */

#ifndef MODBUS_RTU_SERVER_H
#define MODBUS_RTU_SERVER_H

#include <stdint.h>

#include "modbus_rtu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Pass as unit_id to accept incoming requests for any non-broadcast slave ID.
 * Broadcast address 0x00 is still treated with Modbus broadcast semantics.
 */
#define MB_RTU_SERVER_UNIT_ID_ANY   0xFFu

typedef int (*mb_rtu_srv_read_fn)(uint8_t function_code, uint16_t addr, uint16_t qty,
                                  uint16_t *out, void *userdata);

typedef int (*mb_rtu_srv_write_fn)(uint16_t addr, uint16_t qty,
                                   const uint16_t *data, void *userdata);

#define MB_RTU_SERVER_DEFAULT_RECV_TIMEOUT_MS  MODBUS_RTU_DEFAULT_TIMEOUT_MS

typedef struct mb_rtu_server_config {
    const char     *device;              /**< Serial device path, e.g. "/dev/ttyUSB0". */
    uint32_t        baud_rate;           /**< 0 = 9600. */
    uint8_t         data_bits;           /**< 0 = 8. */
    uint8_t         stop_bits;           /**< 0 = 1. */
    mb_rtu_parity_t parity;              /**< Default: MB_RTU_PARITY_NONE. */
    uint8_t         unit_id;             /**< Accepted RTU slave address. */
    uint32_t        recv_timeout_ms;     /**< 0 = default (1 s). */

    mb_rtu_srv_read_fn  on_read;         /**< Handler for FC03 / FC04 reads. */
    mb_rtu_srv_write_fn on_write;        /**< Handler for FC06 / FC16 writes. */
    void               *userdata;        /**< Context passed to callbacks. */

    mb_rtu_logv_fn      logv;            /**< Optional log sink; NULL = silent mode. */
    void               *log_userdata;    /**< Context passed to logv. */

    mb_rtu_link_fn      on_link;         /**< Optional serial-open/close callback. */
    void               *link_userdata;   /**< Context passed to on_link. */
} mb_rtu_server_config_t;

typedef struct mb_rtu_server_ctx {
    mb_rtu_ctx_t           transport;
    mb_rtu_server_config_t cfg;
    /** Internal non-blocking request assembly state. Do not modify directly. */
    uint8_t                rx_buffer[MODBUS_RTU_MAX_ADU_LENGTH];
    size_t                 rx_len;
    uint64_t               last_rx_ms;
} mb_rtu_server_ctx_t;

void mb_rtu_server_config_init(mb_rtu_server_config_t *cfg);
int mb_rtu_server_start(mb_rtu_server_ctx_t *ctx, const mb_rtu_server_config_t *cfg);
void mb_rtu_server_stop(mb_rtu_server_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_SERVER_H */
