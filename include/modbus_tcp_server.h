/**
 * @file modbus_tcp_server.h
 * @brief Modbus TCP server (slave) application-facing API.
 *
 * Provides a callback-based interface for exposing Modbus registers over TCP.
 * Callers implement on_read / on_write and never interact with raw sockets or
 * Modbus frames directly.
 *
 * Relationship to other layers:
 *   modbus_tcp.h/.c   -- TCP transport (threads, sockets, framing helpers)
 *   modbus_tcp_server  -- THIS FILE: application API sitting above the transport
 *
 * Future RTU support will follow the same pattern via modbus_rtu_server.h.
 */

#ifndef MODBUS_TCP_SERVER_H
#define MODBUS_TCP_SERVER_H

#include <stdint.h>
#include "modbus_tcp.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Pass as unit_id to accept incoming requests regardless of their unit ID. */
#define MB_TCP_SERVER_UNIT_ID_ANY   0u

/**
 * Called when a Modbus master reads holding or input registers (FC03 / FC04).
 *
 * @param addr      Starting register address (0-based).
 * @param qty       Number of registers requested (1 – MODBUS_MAX_READ_REGISTERS).
 * @param out       Caller-owned buffer to fill with @p qty values (host byte order).
 * @param userdata  Application context supplied in mb_tcp_server_config_t.
 * @return 0 on success; any other value causes a Server Device Failure exception.
 */
typedef int (*mb_srv_read_fn)(uint16_t addr, uint16_t qty, uint16_t *out, void *userdata);

/**
 * Called when a Modbus master writes registers (FC06 single / FC16 multiple).
 *
 * @param addr      Starting register address (0-based).
 * @param qty       Number of registers written (always 1 for FC06).
 * @param data      Register values in host byte order.
 * @param userdata  Application context supplied in mb_tcp_server_config_t.
 * @return 0 on success; any other value causes a Server Device Failure exception.
 */
typedef int (*mb_srv_write_fn)(uint16_t addr, uint16_t qty,
                               const uint16_t *data, void *userdata);

/**
 * Server configuration.  All pointer fields must remain valid until
 * mb_tcp_server_stop() returns.
 *
 * Fields that accept NULL:
 *   - on_read:       NULL → return Server Device Failure exception to master
 *   - on_write:      NULL → return Server Device Failure exception to master
 *   - userdata:      application-defined context; can be NULL if not used by callbacks
 *   - logv:          NULL → suppress all log messages (silent mode)
 *   - log_userdata:  context passed to logv; can be NULL if not used
 *   - on_link:       NULL → no notification on client connect/disconnect
 *   - link_userdata: context passed to on_link; can be NULL if on_link is NULL
 */
typedef struct mb_tcp_server_config {
    uint16_t port;
    /** Accepted unit ID.  Use MB_TCP_SERVER_UNIT_ID_ANY to accept all unit IDs. */
    uint8_t  unit_id;

    /** Handler for FC03 (holding) and FC04 (input) reads.  NULL → exception reply. */
    mb_srv_read_fn  on_read;
    /** Handler for FC06 (single) and FC16 (multiple) writes.  NULL → exception reply. */
    mb_srv_write_fn on_write;
    void           *userdata;           /**< Context passed to on_read/on_write; can be NULL. */

    mb_tcp_logv_fn  logv;               /**< Optional log sink; NULL → silent mode. */
    void           *log_userdata;       /**< Context passed to logv; can be NULL.    */

    mb_tcp_link_fn  on_link;            /**< Optional connection event callback; NULL → ignore. */
    void           *link_userdata;      /**< Context passed to on_link; can be NULL.  */
} mb_tcp_server_config_t;

/**
 * Server runtime context.  Zero-initialize before calling mb_tcp_server_start().
 * Do not modify fields directly after start.
 */
typedef struct mb_tcp_server_ctx {
    mb_tcp_ctx_t           transport;   /**< Owned transport context (internal). */
    mb_tcp_server_config_t cfg;         /**< Copy of configuration. */
} mb_tcp_server_ctx_t;

/**
 * Start the Modbus TCP server: bind port, launch listener thread.
 *
 * @param ctx  Zero-initialized context; must remain valid until mb_tcp_server_stop().
 * @param cfg  Configuration; pointer fields are copied by reference.
 * @return 0 on success, -1 on error.
 */
int mb_tcp_server_start(mb_tcp_server_ctx_t *ctx, const mb_tcp_server_config_t *cfg);

/**
 * Stop the server, close all client connections, and join the listener thread.
 * Blocking call.
 */
void mb_tcp_server_stop(mb_tcp_server_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_TCP_SERVER_H */
