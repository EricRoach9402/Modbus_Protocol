/**
 * @file modbus_tcp.h
 * @brief Standalone Modbus TCP framing, requests/responses, and optional I/O threads.
 *
 * No dependency on CM_MODBUS. Optional logging and link callbacks are supplied by the caller.
 */

#ifndef MODBUS_PROTOCOL_TCP_H
#define MODBUS_PROTOCOL_TCP_H

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>

#include "modbus_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum mb_tcp_log_level {
    MB_TCP_LOG_DEBUG = 0,
    MB_TCP_LOG_INFO,
    MB_TCP_LOG_WARN,
    MB_TCP_LOG_ERROR
} mb_tcp_log_level_t;

/**
 * Optional log sink (vfprintf-style). @p userdata is the caller's context.
 */
typedef void (*mb_tcp_logv_fn)(void *userdata, mb_tcp_log_level_t level, const char *fmt, va_list ap);

/** Optional: notified when TCP link is up (connected != 0) or lost. @p fd is the socket or -1. */
typedef void (*mb_tcp_link_fn)(void *userdata, int fd, int connected);

/** keepalive_idle_sec sentinel: do not enable SO_KEEPALIVE. */
#define MB_TCP_KEEPALIVE_DISABLED UINT32_MAX

#define MB_TCP_DEFAULT_KEEPALIVE_IDLE_SEC 5u
#define MB_TCP_DEFAULT_KEEPALIVE_INTERVAL_SEC 5u
#define MB_TCP_DEFAULT_KEEPALIVE_COUNT 1u
#define MB_TCP_DEFAULT_IDLE_POLL_MS 5000u

typedef enum mb_tcp_mode {
    /** Listen on @p port (bind all interfaces); accept Modbus TCP masters. */
    MB_TCP_MODE_LISTEN_SERVER = 0,
    /** Connect to @p remote_host:@p port (outbound Modbus TCP client). */
    MB_TCP_MODE_CONNECT_CLIENT = 1
} mb_tcp_mode_t;

typedef struct mb_tcp_config {
    mb_tcp_mode_t mode;
    const char *remote_host; /**< IPv4 string, required for MB_TCP_MODE_CONNECT_CLIENT */
    uint16_t port;
    /** Bind outgoing socket to interface (Linux SO_BINDTODEVICE). NULL = skip. */
    const char *bind_iface;
    unsigned int connect_timeout_sec; /**< default 5 if 0 */
    unsigned int connect_retry_sec;   /**< default 3 if 0 */
    /**
     * Receive timeout applied to each accepted client socket (server mode only).
     * Protects recv from blocking indefinitely on a slow or malicious client.
     * 0 → caller-supplied default is used by the upper layer (e.g. mb_tcp_server).
     */
    uint32_t recv_timeout_ms;
    /**
     * Simultaneously connected clients (server mode only).
     * 0 → MB_TCP_DEFAULT_MAX_CLIENTS.  Slots are allocated at start.
     * Sockets whose fd number is >= FD_SETSIZE are rejected (select() limit).
     */
    uint16_t max_clients;
    /**
     * TCP keepalive on accepted sockets (server mode only).
     * keepalive_idle_sec: 0 → MB_TCP_DEFAULT_KEEPALIVE_IDLE_SEC;
     *                     MB_TCP_KEEPALIVE_DISABLED → SO_KEEPALIVE off.
     * keepalive_interval_sec: 0 → MB_TCP_DEFAULT_KEEPALIVE_INTERVAL_SEC.
     * keepalive_count: 0 → MB_TCP_DEFAULT_KEEPALIVE_COUNT.
     */
    uint32_t keepalive_idle_sec;
    uint32_t keepalive_interval_sec;
    uint32_t keepalive_count;
    /** Server idle select() period for dead-client checks; 0 → MB_TCP_DEFAULT_IDLE_POLL_MS. */
    uint32_t idle_poll_ms;
    void *userdata;

    mb_tcp_logv_fn logv;
    void *log_userdata;

    mb_tcp_link_fn on_link;
    void *link_userdata;

    void (*on_init)(void *userdata);
    /** Server: @p client_fd is the accepted socket. Client: @p client_fd is the connected socket. */
    int (*on_process)(void *userdata, int client_fd);
    /** @p connected: 1 = link up, 0 = link down (see implementation for semantics). */
    void (*on_error)(void *userdata, int connected);
} mb_tcp_config_t;

typedef struct mb_tcp_ctx {
    pthread_t thread;
    volatile int keep_running;
    uint16_t tid;
    pthread_mutex_t lock;
    mb_tcp_config_t cfg;
    int listen_sock;
    int stream_sock;
    int *client_fds;                    /**< Internal: max_clients slots, -1 = free. */
    uint16_t client_cap;                /**< Internal: number of client_fds slots. */
    int initialized;                    /**< Internal: nonzero between start and stop. */
} mb_tcp_ctx_t;

/**
 * Zero ctx before first use; copies @p cfg (pointer fields must remain valid until mb_tcp_stop).
 * @return 0 on success, -1 on error (including ctx already started).
 */
int mb_tcp_start(mb_tcp_ctx_t *ctx, const mb_tcp_config_t *cfg);

/** Idempotent: safe on a zero-initialized ctx, after a failed start, or when called twice. */
void mb_tcp_stop(mb_tcp_ctx_t *ctx);

/**
 * Send all @p len bytes, retrying on short writes and EINTR.
 * Never raises SIGPIPE (uses MSG_NOSIGNAL).
 * @return 0 on success, -1 on error.
 */
int mb_tcp_send_all(int fd, const uint8_t *buf, size_t len);

/**
 * Build a Modbus TCP ADU for a 5-byte PDU after the unit id:
 * FC, start_addr (BE), quantity/value (BE). Used for FC 01–04 and similar layouts.
 * Sets MBAP Length = 6 (unit + 5-byte PDU).
 *
 * @param req_cap must be >= MODBUS_TCP_MIN_READ_REQ_LENGTH (12).
 * @return 0 on success, -1 on error.
 */
int mb_tcp_build_request_basis(mb_tcp_ctx_t *ctx, uint8_t modbus_uid, int function, int addr,
                               uint16_t nb, uint8_t *req, size_t req_cap);

/**
 * Build a FC06 (Write Single Register) ADU into @p req.
 * Total frame is always MODBUS_TCP_MIN_READ_REQ_LENGTH (12) bytes.
 *
 * @param req_cap must be >= MODBUS_TCP_MIN_READ_REQ_LENGTH.
 * @return 0 on success, -1 on error.
 */
int mb_tcp_build_fc06_request(mb_tcp_ctx_t *ctx, uint8_t modbus_uid,
                               uint16_t addr, uint16_t value,
                               uint8_t *req, size_t req_cap);

/**
 * Build a FC16 (Write Multiple Registers) ADU into @p req.
 * Always uses FC16, even when @p num_registers is 1.
 *
 * @param num_registers 1 – MODBUS_MAX_WRITE_REGISTERS.
 * @param req_cap must be >= MODBUS_TCP_MBAP_HEADER_LEN + 7 + num_registers * 2.
 * @return total ADU byte length on success, -1 on error.
 */
int mb_tcp_build_fc16_request(mb_tcp_ctx_t *ctx, uint8_t modbus_uid,
                               uint16_t addr, uint16_t num_registers,
                               const uint16_t *values,
                               uint8_t *req, size_t req_cap);

/**
 * Parse a Modbus TCP request: validates MBAP length vs buffer, then reads FC, address, quantity.
 * FC 06 forces quantity = 1.
 */
int mb_tcp_parse_request(const uint8_t *request, size_t length, uint8_t *function_code,
                         uint16_t *address, uint16_t *quantity);

/**
 * Build FC 03 / FC 04 style read response: MBAP + FC + byte_count + register data (BE).
 * @return total response length, or -1 on error.
 */
int mb_tcp_build_read_registers_response(uint16_t tid, uint8_t function_code, uint8_t unit_id,
                                         const uint16_t *data, uint16_t quantity, uint8_t *response,
                                         size_t response_cap);

/** Standalone connect (optional helper; no threads). @p bind_iface may be NULL. */
int mb_tcp_connect(int *sock_out, const char *bind_iface, const char *remote_host, uint16_t port,
                   unsigned int timeout_sec);

void mb_tcp_close(int *fd);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_PROTOCOL_TCP_H */
