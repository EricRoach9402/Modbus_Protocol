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
 * Return values for mb_tcp_client_read_holding_registers(),
 * mb_tcp_client_write_single_register(), and
 * mb_tcp_client_write_multiple_registers().
 *
 *   0          – success
 *   > 0        – Modbus exception code (MODBUS_EX_* from modbus_defines.h)
 *   < 0        – transport or framing error (one of the codes below)
 */
#define MB_TCP_CLIENT_OK                0
#define MB_TCP_CLIENT_ERR_ARG          (-1)   /**< Invalid argument.               */
#define MB_TCP_CLIENT_ERR_NOT_CONNECTED (-2)  /**< Not connected; call connect first. */
#define MB_TCP_CLIENT_ERR_TRANSPORT    (-3)   /**< Socket send/recv error.         */
#define MB_TCP_CLIENT_ERR_TIMEOUT      (-4)   /**< Response not received in time.  */
#define MB_TCP_CLIENT_ERR_FRAME        (-5)   /**< Malformed or unexpected response. */
#define MB_TCP_CLIENT_ERR_TID          (-6)   /**< Transaction ID mismatch.        */

/* ── Configuration ─────────────────────────────────────────────────────── */

/**
 * Client configuration.  All pointer fields must remain valid until
 * mb_tcp_client_disconnect() returns.
 *
 * Fields that accept NULL:
 *   - bind_iface:    NULL = let OS choose interface via routing
 *   - logv:          NULL = suppress all log messages (silent mode)
 *   - log_userdata:  application-defined context; can be NULL if not used by logv
 */
typedef struct mb_tcp_client_config {
    const char  *remote_host;           /**< Server IPv4 address string (required). */
    uint16_t     port;                  /**< Server port (default Modbus: 502).      */
    const char  *bind_iface;            /**< Bind to interface (SO_BINDTODEVICE);
                                             NULL → system default.                  */
    uint8_t      unit_id;               /**< Modbus unit ID sent in every request.   */
    uint32_t     connect_timeout_sec;   /**< TCP connect timeout; 0 = default (5 s). */
    uint32_t     response_timeout_ms;   /**< Per-request timeout; 0 = default (1 s). */

    mb_tcp_logv_fn logv;                /**< Optional log sink; NULL → silent mode. */
    void          *log_userdata;        /**< Context passed to logv; can be NULL.    */
} mb_tcp_client_config_t;

/* ── Runtime context ───────────────────────────────────────────────────── */

/**
 * Client runtime context.  Zero-initialize before calling mb_tcp_client_connect().
 * Do not modify fields directly after connect.
 */
typedef struct mb_tcp_client_ctx {
    int                    sock;    /**< Connected socket fd; -1 when disconnected. */
    uint16_t               tid;     /**< Monotonic transaction ID counter.          */
    pthread_mutex_t        lock;    /**< Serializes concurrent requests.            */
    mb_tcp_client_config_t cfg;     /**< Copy of configuration.                     */
} mb_tcp_client_ctx_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

/**
 * Connect to the Modbus TCP server specified in cfg.
 *
 * Initializes the mutex and socket; sets the per-request receive timeout.
 *
 * @param ctx  Zero-initialized context; must remain valid until disconnect.
 * @param cfg  Configuration; pointer fields are used by reference.
 * @return 0 on success, -1 on error.
 */
int mb_tcp_client_connect(mb_tcp_client_ctx_t *ctx, const mb_tcp_client_config_t *cfg);

/**
 * Close the connection and release resources.
 * Safe to call even if connect failed.  Blocking until the socket is closed.
 */
void mb_tcp_client_disconnect(mb_tcp_client_ctx_t *ctx);

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

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_TCP_CLIENT_H */
