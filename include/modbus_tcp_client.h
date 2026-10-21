/**
 * @file modbus_tcp_client.h
 * @brief Modbus TCP client (master) application-facing API.
 *
 * Provides blocking, thread-safe read/write functions for accessing registers
 * on a remote Modbus TCP server.  Callers never interact with raw sockets or
 * Modbus frames directly.
 *
 * Relationship to other layers:
 *   modbus_tcp.h/.c    -- TCP transport (socket helpers, framing utilities)
 *   modbus_tcp_client  -- THIS FILE: application API sitting above the transport
 *
 * Thread safety:
 *   Multiple threads may share one mb_tcp_client_ctx_t.  Requests are
 *   serialized internally: only one request is in flight at a time.
 *   mb_tcp_client_disconnect() waits for an in-flight request to finish, but
 *   the caller must ensure no thread starts a new request on the context
 *   during or after disconnect.
 *
 * RTU support follows the same pattern via modbus_rtu_client.h.
 */

#ifndef MODBUS_TCP_CLIENT_H
#define MODBUS_TCP_CLIENT_H

#include <pthread.h>
#include <stdint.h>
#include "modbus_tcp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Error / return codes ──────────────────────────────────────────────── */

/**
 * Return values for all mb_tcp_client_read_* / mb_tcp_client_write_* functions.
 *
 *   0          – success
 *   > 0        – Modbus exception code (MODBUS_EX_* from modbus_defines.h)
 *   < 0        – transport or framing error (one of the codes below)
 *
 * A timeout with no reply keeps the connection; a late reply to that request
 * is discarded by TID on the next call.  When the byte stream can no longer
 * be trusted (send / recv failure, malformed MBAP length, or a timeout in the
 * middle of a frame) the socket is closed and later calls return
 * MB_TCP_CLIENT_ERR_NOT_CONNECTED; call mb_tcp_client_disconnect() then
 * mb_tcp_client_connect() to recover.
 */
#define MB_TCP_CLIENT_OK 0
#define MB_TCP_CLIENT_ERR_ARG (-1) /**< Invalid argument. */
/** Not connected, or the connection was closed after a stream error. */
#define MB_TCP_CLIENT_ERR_NOT_CONNECTED (-2)
#define MB_TCP_CLIENT_ERR_TRANSPORT (-3) /**< Socket send/recv error. */
#define MB_TCP_CLIENT_ERR_TIMEOUT (-4) /**< Response not received in time. */
#define MB_TCP_CLIENT_ERR_FRAME (-5) /**< Malformed or unexpected response. */
/** Only replies with another TID arrived before the timeout. */
#define MB_TCP_CLIENT_ERR_TID (-6)

/* ── Configuration ─────────────────────────────────────────────────────── */

/**
 * Client configuration.  All pointer fields must remain valid until
 * mb_tcp_client_disconnect() returns.
 *
 * Fields that accept NULL:
 *   - bind_iface: NULL = let OS choose interface via routing
 *   - logv: NULL = suppress all log messages (silent mode)
 *   - log_userdata: application-defined context; can be NULL if not used by logv
 */
typedef struct mb_tcp_client_config {
    const char *remote_host; /**< Server IPv4 address string (required). */
    uint16_t port; /**< Server port (default Modbus: 502). */
    const char *bind_iface; /**< Bind to interface (SO_BINDTODEVICE); NULL → system default. */
    uint8_t unit_id; /**< Modbus unit ID sent in every request. */
    uint32_t connect_timeout_sec; /**< TCP connect timeout; 0 = default (5 s). */
    uint32_t response_timeout_ms; /**< Per-request timeout; 0 = default (1 s). */

    mb_tcp_logv_fn logv; /**< Optional log sink; NULL → silent mode. */
    void *log_userdata; /**< Context passed to logv; can be NULL. */
} mb_tcp_client_config_t;

/* ── Runtime context ───────────────────────────────────────────────────── */

/**
 * Client runtime context.  Zero-initialize before calling mb_tcp_client_connect().
 * Do not modify fields directly after connect.
 */
typedef struct mb_tcp_client_ctx {
    int sock; /**< Connected socket fd; -1 when disconnected. */
    uint16_t tid; /**< Monotonic transaction ID counter. */
    pthread_mutex_t lock; /**< Serializes concurrent requests. */
    int initialized; /**< Internal: nonzero between a successful connect and disconnect. */
    mb_tcp_client_config_t cfg; /**< Copy of configuration. */
} mb_tcp_client_ctx_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

/**
 * Connect to the Modbus TCP server specified in cfg.
 *
 * Initializes the mutex and socket; sets the per-request receive timeout.
 * A failed connect leaves the context disconnected, so it may be retried.
 *
 * @param ctx  Zero-initialized (or previously disconnected) context; must
 *             remain valid until disconnect.
 * @param cfg  Configuration; pointer fields are used by reference.
 * @return 0 on success, -1 on error (including ctx already connected).
 */
int mb_tcp_client_connect(mb_tcp_client_ctx_t *ctx, const mb_tcp_client_config_t *cfg);

/**
 * Close the connection and release resources.
 * Waits for an in-flight request to finish.  Idempotent: safe on a
 * zero-initialized context, after a failed connect, or when called twice.
 */
void mb_tcp_client_disconnect(mb_tcp_client_ctx_t *ctx);

/**
 * Change the unit ID used by subsequent requests without reconnecting
 * (e.g. several slaves behind one Modbus TCP gateway).
 * Waits for an in-flight request to finish.
 *
 * @return MB_TCP_CLIENT_OK, MB_TCP_CLIENT_ERR_ARG or MB_TCP_CLIENT_ERR_NOT_CONNECTED.
 */
int mb_tcp_client_set_unit_id(mb_tcp_client_ctx_t *ctx, uint8_t unit_id);

/* ── Register access ───────────────────────────────────────────────────── */

/**
 * FC03 – Read Holding Registers.
 *
 * @param ctx   Connected context.
 * @param addr  Starting register address (0-based).
 * @param qty   Number of registers to read (1 – MODBUS_MAX_READ_REGISTERS).
 * @param out   Caller-owned buffer; receives qty values in host byte order.
 * @return MB_TCP_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_TCP_CLIENT_ERR_* code.
 */
int mb_tcp_client_read_holding_registers(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    uint16_t *out);

/** FC04 – Read Input Registers.  Same contract as mb_tcp_client_read_holding_registers(). */
int mb_tcp_client_read_input_registers(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    uint16_t *out);

/**
 * FC06 – Write Single Register.
 *
 * Always sends FC06 regardless of context.  Use this when the remote device
 * requires or prefers the single-register write function code.
 *
 * @param ctx    Connected context.
 * @param addr   Register address (0-based).
 * @param value  Register value in host byte order.
 * @return MB_TCP_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_TCP_CLIENT_ERR_* code.
 */
int mb_tcp_client_write_single_register(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t value);

/**
 * FC16 – Write Multiple Registers.
 *
 * Always sends FC16 regardless of qty.  Sending qty=1 via FC16 is explicitly
 * permitted by the Modbus specification and is a valid reason to prefer this
 * function over mb_tcp_client_write_single_register().
 *
 * @param ctx   Connected context.
 * @param addr  Starting register address (0-based).
 * @param qty   Number of registers to write (1 – MODBUS_MAX_WRITE_REGISTERS).
 * @param data  Register values in host byte order.
 * @return MB_TCP_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_TCP_CLIENT_ERR_* code.
 */
int mb_tcp_client_write_multiple_registers(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    const uint16_t *data);

/* ── Coil / discrete input access ──────────────────────────────────────── */

/**
 * FC01 – Read Coils.
 *
 * @param qty  Number of coils (1 – MODBUS_MAX_READ_BITS).
 * @param out  Caller-owned buffer of @p qty bytes; each receives 0 or 1.
 * @return MB_TCP_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_TCP_CLIENT_ERR_* code.
 */
int mb_tcp_client_read_coils(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty, uint8_t *out);

/** FC02 – Read Discrete Inputs.  Same contract as mb_tcp_client_read_coils(). */
int mb_tcp_client_read_discrete_inputs(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty, uint8_t *out);

/** FC05 – Write Single Coil.  @p value nonzero = ON. */
int mb_tcp_client_write_single_coil(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint8_t value);

/**
 * FC15 – Write Multiple Coils.
 *
 * @param qty     Number of coils (1 – MODBUS_MAX_WRITE_BITS).
 * @param values  @p qty bytes, nonzero = ON.
 */
int mb_tcp_client_write_multiple_coils(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    const uint8_t *values);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_TCP_CLIENT_H */
