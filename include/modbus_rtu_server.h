/**
 * @file modbus_rtu_server.h
 * @brief Modbus RTU server (slave) application-facing API.
 *
 * Provides the same register callback model as the TCP server. The RTU layer
 * owns serial-port setup, CRC validation, and frame dispatch.
 *
 * Response timing (inter_frame_delay_us):
 *   0                      → T3.5 from serial settings (mb_rtu_t35_us)
 *   MB_RTU_TIMING_DISABLED → no pre-response wait
 *   any other value        → explicit delay in microseconds
 *
 * Measured from the end of the received request (last RX byte).
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
#define MB_RTU_SERVER_UNIT_ID_ANY 0xFFu

typedef int (*mb_rtu_srv_read_fn)(uint8_t function_code, uint16_t addr, uint16_t qty,
    uint16_t *out, void *userdata);

typedef int (*mb_rtu_srv_write_fn)(uint16_t addr, uint16_t qty,
    const uint16_t *data, void *userdata);

/** FC01 / FC02 read: fill @p qty bytes of @p out (pre-zeroed) with 0 / nonzero. */
typedef int (*mb_rtu_srv_read_bits_fn)(uint8_t function_code, uint16_t addr, uint16_t qty,
    uint8_t *out, void *userdata);

/** FC05 / FC15 write: @p bits holds @p qty bytes, each 0 or 1. */
typedef int (*mb_rtu_srv_write_bits_fn)(uint16_t addr, uint16_t qty,
    const uint8_t *bits, void *userdata);

#define MB_RTU_SERVER_DEFAULT_RECV_TIMEOUT_MS MODBUS_RTU_DEFAULT_TIMEOUT_MS

typedef struct mb_rtu_server_config {
    const char *device; /**< Serial device path, e.g. "/dev/ttyUSB0". */
    uint32_t baud_rate; /**< 0 = 9600. */
    uint8_t data_bits; /**< 0 = 8. */
    uint8_t stop_bits; /**< 0 = 1. */
    mb_rtu_parity_t parity; /**< Default: MB_RTU_PARITY_NONE. */
    uint8_t unit_id; /**< Accepted RTU slave address. */
    uint32_t recv_timeout_ms; /**< 0 = default (1 s). */
    /**
     * Minimum silence (T3.5) before sending a response.
     * 0 = mb_rtu_t35_us(); MB_RTU_TIMING_DISABLED = off.
     */
    uint32_t inter_frame_delay_us;
    mb_rtu_rs485_mode_t rs485_mode; /**< Kernel RS-485 DE control; default none. */
    /**
     * Nonzero if the transceiver echoes transmitted bytes back into RX
     * (half-duplex without RE gating).  The echo of each response is discarded.
     */
    uint8_t local_echo;

    /*
     * Callbacks; NULL → the matching function codes reply ILLEGAL_FUNCTION.
     * Return 0 or a Modbus exception code (MODBUS_EX_*); other values map to
     * SERVER_DEVICE_FAILURE.
     */
    mb_rtu_srv_read_fn on_read; /**< Handler for FC03 / FC04 reads. */
    mb_rtu_srv_write_fn on_write; /**< Handler for FC06 / FC16 writes. */
    mb_rtu_srv_read_bits_fn on_read_bits; /**< Handler for FC01 / FC02 reads. */
    mb_rtu_srv_write_bits_fn on_write_bits; /**< Handler for FC05 / FC15 writes. */
    void *userdata; /**< Context passed to callbacks. */

    mb_rtu_tx_enable_fn on_tx_enable; /**< Optional DE / TX-enable hook (e.g. GPIO). */
    void *tx_userdata; /**< Context passed to on_tx_enable. */

    mb_rtu_logv_fn logv; /**< Optional log sink; NULL = silent mode. */
    void *log_userdata; /**< Context passed to logv. */

    mb_rtu_link_fn on_link; /**< Optional serial-open/close callback. */
    void *link_userdata; /**< Context passed to on_link. */
} mb_rtu_server_config_t;

typedef struct mb_rtu_server_ctx {
    mb_rtu_ctx_t transport;
    mb_rtu_server_config_t cfg;
    /** Internal non-blocking request assembly state. Do not modify directly. */
    uint8_t rx_buffer[MODBUS_RTU_MAX_ADU_LENGTH];
    size_t rx_len;
    uint64_t last_rx_us; /**< MONOTONIC us of last RX byte; 0 = idle. */
    uint32_t t35_us; /**< Cached auto T3.5 for this serial config. */
} mb_rtu_server_ctx_t;

void mb_rtu_server_config_init(mb_rtu_server_config_t *cfg);

/**
 * Open the serial port and start the server thread.
 * Frames that fail CRC or are malformed (including other slaves' responses
 * on a multi-drop bus) are skipped byte by byte until a valid request is found.
 *
 * @param ctx  Zero-initialized (or previously stopped) context.
 * @return 0 on success, -1 on error (including ctx already started).
 */
int mb_rtu_server_start(mb_rtu_server_ctx_t *ctx, const mb_rtu_server_config_t *cfg);

/** Idempotent: safe on a zero-initialized ctx, after a failed start, or when called twice. */
void mb_rtu_server_stop(mb_rtu_server_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_SERVER_H */
