/**
 * @file modbus_rtu_client.h
 * @brief Modbus RTU client (master) application-facing API.
 *
 * Mirrors modbus_tcp_client.h where possible. Callers configure a serial port
 * instead of remote_host/port, then use the same blocking register operations.
 *
 * Thread safety / shared bus:
 *   Multiple threads may share one mb_rtu_client_ctx_t; requests on that
 *   context are serialized by ctx->lock.  mb_rtu_client_disconnect() waits
 *   for an in-flight request to finish, but the caller must ensure no thread
 *   starts a new request on the context during or after disconnect.
 *
 *   Multiple contexts on the same device (typically different unit_id
 *   values for slaves on one RS-485 bus) share one open file descriptor and
 *   one process-wide bus lock keyed by the canonical path (realpath(), so
 *   symlinks such as /dev/serial/by-id/... map to the same bus).
 *   Transactions on that device are serialized across all such contexts.
 *   All contexts on the same device must use matching serial settings
 *   (baud / data / stop / parity / rs485_mode).  There is no limit on the
 *   number of distinct devices; a device's record is freed when its last
 *   context disconnects.
 *
 * Bus timing (measured from the end of the previous transaction on the path):
 *   - inter_frame_delay_us: T3.5 silence before every request frame.
 *   - bus_handoff_silence_ms: longer silence when the unit_id changes.
 *   The larger of the two applies.
 *
 * Timing field convention (both client fields below):
 *   0                      → library default
 *   MB_RTU_TIMING_DISABLED → feature off (no wait)
 *   any other value        → explicit delay
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

#define MB_RTU_CLIENT_OK 0
#define MB_RTU_CLIENT_ERR_ARG (-1)
#define MB_RTU_CLIENT_ERR_NOT_CONNECTED (-2)
#define MB_RTU_CLIENT_ERR_TRANSPORT (-3)
#define MB_RTU_CLIENT_ERR_TIMEOUT (-4)
#define MB_RTU_CLIENT_ERR_FRAME (-5)
#define MB_RTU_CLIENT_ERR_UNIT_ID (-6)
#define MB_RTU_CLIENT_ERR_CRC (-7)

/** Default inter-unit_id bus silence when bus_handoff_silence_ms is 0. */
#define MB_RTU_CLIENT_DEFAULT_HANDOFF_SILENCE_MS 30u

/* -- Configuration ------------------------------------------------------- */

typedef struct mb_rtu_client_config {
    const char *device; /**< Serial device path, e.g. "/dev/ttyUSB0". */
    uint32_t baud_rate; /**< 0 = 9600. */
    uint8_t data_bits; /**< 0 = 8. */
    uint8_t stop_bits; /**< 0 = 1. */
    mb_rtu_parity_t parity; /**< Default: MB_RTU_PARITY_NONE. */
    uint8_t unit_id; /**< RTU slave address sent in every request. */
    /**
     * Response deadline after the request is sent; 0 = default (1 s).
     * The total budget is this value plus the wire time of a maximum-size
     * RTU frame at the configured baud, so large reads at low baud rates
     * are not cut short.
     */
    uint32_t response_timeout_ms;
    /**
     * Minimum silence (T3.5) before each request frame.
     * 0 = mb_rtu_t35_us(); MB_RTU_TIMING_DISABLED = off.
     */
    uint32_t inter_frame_delay_us;
    /**
     * Minimum silence before a transaction whose unit_id differs from the
     * previous transaction on the same shared device path.
     * 0 = MB_RTU_CLIENT_DEFAULT_HANDOFF_SILENCE_MS;
     * MB_RTU_TIMING_DISABLED = off.
     */
    uint32_t bus_handoff_silence_ms;
    /** Kernel RS-485 DE control (must match across contexts on one device). */
    mb_rtu_rs485_mode_t rs485_mode;
    /**
     * Nonzero if the transceiver echoes transmitted bytes back into RX.
     * The echo is read and compared with the request before the response;
     * a mismatch (bus collision) returns MB_RTU_CLIENT_ERR_TRANSPORT.
     */
    uint8_t local_echo;

    mb_rtu_tx_enable_fn on_tx_enable; /**< Optional DE / TX-enable hook (e.g. GPIO). */
    void *tx_userdata; /**< Context passed to on_tx_enable. */

    mb_rtu_logv_fn logv; /**< Optional log sink; NULL = silent mode. */
    void *log_userdata; /**< Context passed to logv; can be NULL. */
} mb_rtu_client_config_t;

/* -- Runtime context ----------------------------------------------------- */

/** Opaque handle to this device path's shared port / bus-lock record. */
struct mb_rtu_shared_bus;

typedef struct mb_rtu_client_ctx {
    int fd; /**< Shared serial fd; -1 when detached. */
    pthread_mutex_t lock; /**< Serializes concurrent requests on this ctx. */
    struct mb_rtu_shared_bus *shared_bus; /**< Shared port + bus lock for device path. */
    int initialized; /**< Internal: nonzero between a successful connect and disconnect. */
    mb_rtu_client_config_t cfg; /**< Copy of configuration. */
} mb_rtu_client_ctx_t;

/* -- Lifecycle ----------------------------------------------------------- */

/**
 * @brief Open or attach to the serial port for this context.
 *
 * The first context on a given device path opens the port; additional
 * contexts on the same path reuse that fd (matching serial settings required).
 * A failed connect leaves the context disconnected, so it may be retried.
 *
 * @param ctx  Zero-initialized (or previously disconnected) context.
 * @return 0 on success, -1 on error (including ctx already connected).
 */
int mb_rtu_client_connect(mb_rtu_client_ctx_t *ctx, const mb_rtu_client_config_t *cfg);

/**
 * @brief Detach this context from the serial port and release resources.
 *
 * Decrements the shared-port reference count; the port is closed when the
 * last context on that device path disconnects.  Waits for an in-flight
 * request to finish.  Idempotent: safe on a zero-initialized context, after
 * a failed connect, or when called twice.
 */
void mb_rtu_client_disconnect(mb_rtu_client_ctx_t *ctx);

/**
 * Change the slave address used by subsequent requests without reopening
 * the port.  Waits for an in-flight request on this context to finish.
 *
 * @return MB_RTU_CLIENT_OK, MB_RTU_CLIENT_ERR_ARG or MB_RTU_CLIENT_ERR_NOT_CONNECTED.
 */
int mb_rtu_client_set_unit_id(mb_rtu_client_ctx_t *ctx, uint8_t unit_id);

/*
 * -- Register / coil access ----------------------------------------------
 *
 * Same contracts as the mb_tcp_client_* equivalents.  Return
 * MB_RTU_CLIENT_OK, a positive Modbus exception code, or a negative
 * MB_RTU_CLIENT_ERR_* code.  With unit_id 0 (broadcast) only the write
 * functions are allowed; they return MB_RTU_CLIENT_OK once the frame is sent.
 * Bit buffers use one byte per coil / input (0 = OFF, nonzero = ON).
 */

int mb_rtu_client_read_holding_registers(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    uint16_t *out);

int mb_rtu_client_read_input_registers(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    uint16_t *out);

int mb_rtu_client_write_single_register(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t value);

int mb_rtu_client_write_multiple_registers(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    const uint16_t *data);

int mb_rtu_client_read_coils(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty, uint8_t *out);

int mb_rtu_client_read_discrete_inputs(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty, uint8_t *out);

int mb_rtu_client_write_single_coil(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint8_t value);

int mb_rtu_client_write_multiple_coils(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    const uint8_t *values);

/* -- Device presence probe ----------------------------------------------- */

/**
 * Probe whether a slave is actually present and responding on the bus.
 *
 * mb_rtu_client_connect() only opens the local serial device node; on RTU
 * there is no handshake, so an absent slave still "connects" successfully.
 * This issues one FC03 read of @p qty registers at @p addr and returns
 * whether a valid response was received, letting callers distinguish
 * "port open" from "device online".
 *
 * @param addr A register address known to exist on the slave.
 * @param qty  Number of registers to read (typically 1).
 * @return MB_RTU_CLIENT_OK if the slave responded, a positive Modbus
 *         exception code, or a negative MB_RTU_CLIENT_ERR_* code.
 */
int mb_rtu_client_probe_device(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_CLIENT_H */
