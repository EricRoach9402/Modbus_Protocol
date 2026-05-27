/**
 * @file modbus_rtu.h
 * @brief Standalone Modbus RTU framing, CRC, serial I/O, and optional server thread.
 *
 * The public client/server APIs mirror the Modbus TCP modules as closely as
 * practical. This lower layer owns serial-port setup and reusable RTU frame
 * helpers.
 */

#ifndef MODBUS_PROTOCOL_RTU_H
#define MODBUS_PROTOCOL_RTU_H

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>

#include "modbus_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum mb_rtu_log_level {
    MB_RTU_LOG_DEBUG = 0,
    MB_RTU_LOG_INFO,
    MB_RTU_LOG_WARN,
    MB_RTU_LOG_ERROR
} mb_rtu_log_level_t;

/**
 * Optional log sink (vfprintf-style). @p userdata is the caller's context.
 */
typedef void (*mb_rtu_logv_fn)(void *userdata, mb_rtu_log_level_t level,
                               const char *fmt, va_list ap);

/** Optional: notified when the serial link is opened (connected != 0) or closed. */
typedef void (*mb_rtu_link_fn)(void *userdata, int fd, int connected);

typedef enum mb_rtu_parity {
    MB_RTU_PARITY_NONE = 0,
    MB_RTU_PARITY_EVEN,
    MB_RTU_PARITY_ODD
} mb_rtu_parity_t;

typedef struct mb_rtu_serial_config {
    const char *device;              /**< Serial device path, e.g. "/dev/ttyUSB0". */
    uint32_t baud_rate;              /**< 0 = 9600. Supports common POSIX baud rates. */
    uint8_t data_bits;               /**< 0 = 8. Supported values: 7 or 8. */
    uint8_t stop_bits;               /**< 0 = 1. Supported values: 1 or 2. */
    mb_rtu_parity_t parity;          /**< Default: MB_RTU_PARITY_NONE. */
} mb_rtu_serial_config_t;

typedef struct mb_rtu_config {
    mb_rtu_serial_config_t serial;
    uint32_t recv_timeout_ms;        /**< 0 = MODBUS_RTU_DEFAULT_TIMEOUT_MS. */
    void *userdata;

    mb_rtu_logv_fn logv;
    void *log_userdata;

    mb_rtu_link_fn on_link;
    void *link_userdata;

    void (*on_init)(void *userdata);
    /** Periodic transport callback. Must return quickly; negative rc is forwarded to on_error. */
    int  (*on_process)(void *userdata, int serial_fd);
    /** Error callback for real failures only; link state is reported through on_link. */
    void (*on_error)(void *userdata, int error_code);
} mb_rtu_config_t;

typedef struct mb_rtu_ctx {
    pthread_t thread;
    volatile int keep_running;
    pthread_mutex_t lock;
    mb_rtu_config_t cfg;
    int serial_fd;
} mb_rtu_ctx_t;

/** Zero ctx before first use; copies @p cfg (pointer fields must remain valid until stop). */
int mb_rtu_start(mb_rtu_ctx_t *ctx, const mb_rtu_config_t *cfg);
void mb_rtu_stop(mb_rtu_ctx_t *ctx);

/** Standalone serial open/close helpers used by the blocking client API. */
int mb_rtu_open(int *fd_out, const mb_rtu_serial_config_t *serial);
void mb_rtu_close(int *fd);

/** CRC16/MODBUS helpers. CRC bytes are stored low byte first per RTU spec. */
uint16_t mb_rtu_crc16(const uint8_t *data, size_t length);
int mb_rtu_append_crc(uint8_t *frame, size_t frame_len_without_crc, size_t frame_cap);
int mb_rtu_validate_crc(const uint8_t *frame, size_t frame_len);

/**
 * Build a Modbus RTU request for a 5-byte PDU after the unit id:
 * FC, start_addr (BE), quantity/value (BE). Used for FC 01-06 layouts.
 *
 * @param req_cap must be >= MODBUS_RTU_MIN_REQ_LENGTH.
 * @return total ADU byte length on success, -1 on error.
 */
int mb_rtu_build_request_basis(uint8_t unit_id, int function, int addr,
                               uint16_t nb, uint8_t *req, size_t req_cap);

/** Build a FC06 (Write Single Register) RTU ADU. */
int mb_rtu_build_fc06_request(uint8_t unit_id, uint16_t addr, uint16_t value,
                              uint8_t *req, size_t req_cap);

/**
 * Build a FC16 (Write Multiple Registers) RTU ADU.
 *
 * @param req_cap must be >= 9 + num_registers * 2.
 * @return total ADU byte length on success, -1 on error.
 */
int mb_rtu_build_fc16_request(uint8_t unit_id, uint16_t addr, uint16_t num_registers,
                              const uint16_t *values, uint8_t *req, size_t req_cap);

/**
 * Parse a validated Modbus RTU request: reads FC, address, and quantity.
 * FC06 forces quantity = 1.
 */
int mb_rtu_parse_request(const uint8_t *request, size_t length, uint8_t *function_code,
                         uint16_t *address, uint16_t *quantity);

/**
 * Build FC03 / FC04 style read response: unit + FC + byte_count + register data + CRC.
 * @return total response length, or -1 on error.
 */
int mb_rtu_build_read_registers_response(uint8_t function_code, uint8_t unit_id,
                                         const uint16_t *data, uint16_t quantity,
                                         uint8_t *response, size_t response_cap);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_PROTOCOL_RTU_H */
