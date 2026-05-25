/**
 * @file modbus_tcp_client.c
 * @brief Modbus TCP client API – implementation.
 *
 * Provides blocking, thread-safe read/write helpers.  Each public function
 * holds the context mutex for the full request–response cycle, ensuring only
 * one transaction is in flight at a time.
 *
 * Request building is done locally using only constants from modbus_defines.h;
 * this avoids coupling the client to the mb_tcp_ctx_t TID mechanism.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <sys/socket.h>

#include "modbus_tcp_client.h"

/* ── Default tunables ──────────────────────────────────────────────────── */

#define DEFAULT_CONNECT_TIMEOUT_SEC     5u
#define DEFAULT_RESPONSE_TIMEOUT_MS  1000u

/* ── Modbus TCP ADU byte-offset map ────────────────────────────────────────
 *
 *  Byte  Field
 *  ----  ----------------------------------------
 *   0    Transaction ID  (high byte)
 *   1    Transaction ID  (low  byte)
 *   2    Protocol ID     (high byte, always 0x00)
 *   3    Protocol ID     (low  byte, always 0x00)
 *   4    MBAP Length     (high byte)
 *   5    MBAP Length     (low  byte)
 *   6    Unit ID
 *   7    Function Code
 *   8    Start Address   (high byte)
 *   9    Start Address   (low  byte)
 *  10    Quantity        (high byte) / FC06 value high
 *  11    Quantity        (low  byte) / FC06 value low
 *  12    Byte Count      (FC16 request only)
 *  13+   Register Data   (FC16 request only)
 * ──────────────────────────────────────────────────────────────────────── */
#define ADU_OFFSET_TID_HIGH         0u
#define ADU_OFFSET_TID_LOW          1u
#define ADU_OFFSET_PROTO_HIGH       2u
#define ADU_OFFSET_PROTO_LOW        3u
#define ADU_OFFSET_LEN_HIGH         4u
#define ADU_OFFSET_LEN_LOW          5u
#define ADU_OFFSET_UNIT_ID          6u
#define ADU_OFFSET_FC               7u
#define ADU_OFFSET_ADDR_HIGH        8u
#define ADU_OFFSET_ADDR_LOW         9u
#define ADU_OFFSET_QTY_HIGH        10u
#define ADU_OFFSET_QTY_LOW         11u
#define ADU_OFFSET_FC16_BYTE_COUNT 12u
#define ADU_OFFSET_FC16_DATA_START 13u

/* FC16 / FC06 success response layout (fixed 12 bytes):
 *   MBAP(6) + unit_id(1) + FC(1) + start_addr(2) + qty_or_value(2) */
#define FC16_RESPONSE_LEN          12u

/* Minimum ADU length for any valid response, including Modbus exception:
 *   MBAP(6) + unit_id(1) + FC|0x80(1) + exception_code(1) = 9 bytes */
#define MIN_RESPONSE_LEN            9u

/* FC03 response layout:
 *   MBAP(6) + unit_id(1) + FC(1) + byte_count(1) + data(qty*2)
 *   → minimum with qty=1:  6 + 1 + 1 + 1 + 2 = 11 bytes */
#define FC03_RESPONSE_HEADER_LEN    9u   /* bytes before the data payload  */
#define FC03_RESPONSE_BYTE_COUNT_OFFSET  8u

/* ── Logging helper ────────────────────────────────────────────────────── */

static void cli_logv(mb_tcp_client_ctx_t *ctx, mb_tcp_log_level_t level,
                     const char *fmt, ...)
{
    if (!ctx->cfg.logv) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    ctx->cfg.logv(ctx->cfg.log_userdata, level, fmt, ap);
    va_end(ap);
}

/* ── Network I/O helpers ───────────────────────────────────────────────── */

/**
 * Receive exactly len bytes from fd.
 *
 * @return  0   success
 *         -1   transport error or peer closed connection
 *         -2   receive timeout (errno is EAGAIN / EWOULDBLOCK / ETIMEDOUT)
 */
static int recv_exact(int fd, uint8_t *buf, size_t len)
{
    size_t received = 0;
    while (received < len) {
        ssize_t n = recv(fd, buf + received, len - received, 0);
        if (n <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ETIMEDOUT) {
                return -2;  /* timeout */
            }
            return -1;      /* disconnect or error */
        }
        received += (size_t)n;
    }
    return 0;
}

/* Map recv_exact return codes to MB_TCP_CLIENT_ERR_* codes. */
static int recv_err_to_client_err(int recv_rc)
{
    if (recv_rc == -2) {
        return MB_TCP_CLIENT_ERR_TIMEOUT;
    }
    return MB_TCP_CLIENT_ERR_TRANSPORT;
}

/* ── TID management ─────────────────────────────────────────────────────
 * Must be called while the context mutex is held.
 */
static uint16_t next_tid(mb_tcp_client_ctx_t *ctx)
{
    if (ctx->tid < UINT16_MAX) {
        ctx->tid++;
    } else {
        ctx->tid = 1u;
    }
    return ctx->tid;
}

/* ── Request builders ──────────────────────────────────────────────────── */

/**
 * Write a FC03 (Read Holding Registers) request into buf.
 * Requires buf_cap >= MODBUS_TCP_MIN_READ_REQ_LENGTH.
 */
static void build_fc03_request(uint16_t tid, uint8_t unit_id,
                                uint16_t addr, uint16_t qty,
                                uint8_t *buf)
{
    buf[ADU_OFFSET_TID_HIGH]  = (uint8_t)(tid >> 8);
    buf[ADU_OFFSET_TID_LOW]   = (uint8_t)(tid & 0xFFu);
    buf[ADU_OFFSET_PROTO_HIGH] = 0x00u;
    buf[ADU_OFFSET_PROTO_LOW]  = 0x00u;
    buf[ADU_OFFSET_LEN_HIGH]  = 0x00u;
    buf[ADU_OFFSET_LEN_LOW]   = MODBUS_TCP_MBAP_LEN_STANDARD_REQ;
    buf[ADU_OFFSET_UNIT_ID]   = unit_id;
    buf[ADU_OFFSET_FC]        = (uint8_t)MODBUS_FUNC_READ_HOLDING_REGISTERS;
    buf[ADU_OFFSET_ADDR_HIGH] = (uint8_t)(addr >> 8);
    buf[ADU_OFFSET_ADDR_LOW]  = (uint8_t)(addr & 0xFFu);
    buf[ADU_OFFSET_QTY_HIGH]  = (uint8_t)(qty >> 8);
    buf[ADU_OFFSET_QTY_LOW]   = (uint8_t)(qty & 0xFFu);
}

/**
 * Write a FC06 (Write Single Register) request into buf.
 * Requires buf_cap >= MODBUS_TCP_MIN_READ_REQ_LENGTH.
 */
static void build_fc06_request(uint16_t tid, uint8_t unit_id,
                                uint16_t addr, uint16_t value,
                                uint8_t *buf)
{
    buf[ADU_OFFSET_TID_HIGH]  = (uint8_t)(tid >> 8);
    buf[ADU_OFFSET_TID_LOW]   = (uint8_t)(tid & 0xFFu);
    buf[ADU_OFFSET_PROTO_HIGH] = 0x00u;
    buf[ADU_OFFSET_PROTO_LOW]  = 0x00u;
    buf[ADU_OFFSET_LEN_HIGH]  = 0x00u;
    buf[ADU_OFFSET_LEN_LOW]   = MODBUS_TCP_MBAP_LEN_STANDARD_REQ;
    buf[ADU_OFFSET_UNIT_ID]   = unit_id;
    buf[ADU_OFFSET_FC]        = (uint8_t)MODBUS_FUNC_WRITE_SINGLE_REGISTER;
    buf[ADU_OFFSET_ADDR_HIGH] = (uint8_t)(addr >> 8);
    buf[ADU_OFFSET_ADDR_LOW]  = (uint8_t)(addr & 0xFFu);
    buf[ADU_OFFSET_QTY_HIGH]  = (uint8_t)(value >> 8);
    buf[ADU_OFFSET_QTY_LOW]   = (uint8_t)(value & 0xFFu);
}

/**
 * Write a FC16 (Write Multiple Registers) request into buf.
 * Returns total request byte count.
 * Requires buf_cap >= (MODBUS_TCP_MBAP_HEADER_LEN + 7 + qty * 2).
 */
static size_t build_fc16_request(uint16_t tid, uint8_t unit_id,
                                  uint16_t addr, uint16_t qty,
                                  const uint16_t *data, uint8_t *buf)
{
    uint8_t byte_count = (uint8_t)(qty * 2u);
    /* MBAP Length = unit(1) + FC(1) + addr(2) + qty(2) + byte_count(1) + data */
    uint16_t mbap_len  = (uint16_t)(7u + (uint16_t)byte_count);

    buf[ADU_OFFSET_TID_HIGH]       = (uint8_t)(tid >> 8);
    buf[ADU_OFFSET_TID_LOW]        = (uint8_t)(tid & 0xFFu);
    buf[ADU_OFFSET_PROTO_HIGH]     = 0x00u;
    buf[ADU_OFFSET_PROTO_LOW]      = 0x00u;
    buf[ADU_OFFSET_LEN_HIGH]       = (uint8_t)(mbap_len >> 8);
    buf[ADU_OFFSET_LEN_LOW]        = (uint8_t)(mbap_len & 0xFFu);
    buf[ADU_OFFSET_UNIT_ID]        = unit_id;
    buf[ADU_OFFSET_FC]             = (uint8_t)MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS;
    buf[ADU_OFFSET_ADDR_HIGH]      = (uint8_t)(addr >> 8);
    buf[ADU_OFFSET_ADDR_LOW]       = (uint8_t)(addr & 0xFFu);
    buf[ADU_OFFSET_QTY_HIGH]       = (uint8_t)(qty >> 8);
    buf[ADU_OFFSET_QTY_LOW]        = (uint8_t)(qty & 0xFFu);
    buf[ADU_OFFSET_FC16_BYTE_COUNT] = byte_count;

    for (uint16_t i = 0u; i < qty; i++) {
        size_t offset = ADU_OFFSET_FC16_DATA_START + (size_t)(i * 2u);
        buf[offset]     = (uint8_t)(data[i] >> 8);
        buf[offset + 1u] = (uint8_t)(data[i] & 0xFFu);
    }

    return MODBUS_TCP_MBAP_HEADER_LEN + (size_t)mbap_len;
}

/* ── Response receiver ─────────────────────────────────────────────────── */

/**
 * Receive one complete ADU response from sock into buf (up to buf_cap bytes).
 * Reads MBAP header first, then the body.
 *
 * @param[out] total_len  Set to total bytes received on success.
 * @return 0, -1 (transport error), or -2 (timeout).
 */
static int recv_response(int sock, uint8_t *buf, size_t buf_cap, size_t *total_len)
{
    int rc = recv_exact(sock, buf, MODBUS_TCP_MBAP_HEADER_LEN);
    if (rc != 0) {
        return rc;
    }

    uint16_t mbap_remaining = ((uint16_t)buf[ADU_OFFSET_LEN_HIGH] << 8)
                            | (uint16_t)buf[ADU_OFFSET_LEN_LOW];

    if (mbap_remaining == 0u ||
        mbap_remaining > (buf_cap - MODBUS_TCP_MBAP_HEADER_LEN)) {
        return -1;  /* malformed length */
    }

    rc = recv_exact(sock, buf + MODBUS_TCP_MBAP_HEADER_LEN, mbap_remaining);
    if (rc != 0) {
        return rc;
    }

    *total_len = MODBUS_TCP_MBAP_HEADER_LEN + (size_t)mbap_remaining;
    return 0;
}

/* ── Write-response validator ───────────────────────────────────────────── */

/**
 * Common response validation for FC06 and FC16 write operations.
 *
 * Both write function codes share the same 12-byte response layout:
 *   MBAP(6) + unit_id(1) + FC(1) + echo_addr(2) + echo_qty_or_value(2)
 *
 * Checks performed:
 *  1. Minimum length (must be at least FC16_RESPONSE_LEN bytes).
 *  2. Transaction ID matches the request TID.
 *  3. No Modbus exception flag set in the response FC byte.
 *  4. Response FC byte matches the function code that was sent.
 *
 * @param ctx          Client context (used for logging only).
 * @param req          The original request buffer (req[ADU_OFFSET_FC] is read).
 * @param resp         The received response buffer.
 * @param total_len    Total bytes received.
 * @param expected_tid The TID that was sent in the request.
 * @return MB_TCP_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_TCP_CLIENT_ERR_* code.
 */
static int validate_write_response(mb_tcp_client_ctx_t *ctx,
                                    const uint8_t *req, const uint8_t *resp,
                                    size_t total_len, uint16_t expected_tid)
{
    /* A Modbus exception response (9 bytes) is shorter than a normal write
     * response (12 bytes).  Reject anything too short to be either. */
    if (total_len < MIN_RESPONSE_LEN) {
        return MB_TCP_CLIENT_ERR_FRAME;
    }

    uint16_t resp_tid = ((uint16_t)resp[ADU_OFFSET_TID_HIGH] << 8)
                      | (uint16_t)resp[ADU_OFFSET_TID_LOW];
    if (resp_tid != expected_tid) {
        cli_logv(ctx, MB_TCP_LOG_WARN,
                 "FC%02X TID mismatch: sent %u, received %u",
                 (unsigned)req[ADU_OFFSET_FC],
                 (unsigned)expected_tid, (unsigned)resp_tid);
        return MB_TCP_CLIENT_ERR_TID;
    }

    uint8_t resp_fc = resp[ADU_OFFSET_FC];
    if (resp_fc & (uint8_t)MODBUS_EXCEPTION_FLAG) {
        uint8_t exception_code = resp[ADU_OFFSET_ADDR_HIGH];
        cli_logv(ctx, MB_TCP_LOG_WARN,
                 "FC%02X exception code %u",
                 (unsigned)req[ADU_OFFSET_FC], (unsigned)exception_code);
        return (int)exception_code;
    }

    /* Normal success response: must be at least 12 bytes and echo the FC. */
    if (total_len < FC16_RESPONSE_LEN) {
        return MB_TCP_CLIENT_ERR_FRAME;
    }

    if (resp_fc != req[ADU_OFFSET_FC]) {
        return MB_TCP_CLIENT_ERR_FRAME;
    }

    return MB_TCP_CLIENT_OK;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int mb_tcp_client_connect(mb_tcp_client_ctx_t *ctx, const mb_tcp_client_config_t *cfg)
{
    if (!ctx || !cfg || !cfg->remote_host || cfg->remote_host[0] == '\0' ||
        cfg->port == 0u) {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg  = *cfg;
    ctx->sock = -1;
    ctx->tid  = 0u;

    if (pthread_mutex_init(&ctx->lock, NULL) != 0) {
        return -1;
    }

    uint32_t connect_sec = (cfg->connect_timeout_sec != 0u)
                           ? cfg->connect_timeout_sec
                           : DEFAULT_CONNECT_TIMEOUT_SEC;

    if (mb_tcp_connect(&ctx->sock, cfg->bind_iface, cfg->remote_host,
                       cfg->port, connect_sec) != 0) {
        pthread_mutex_destroy(&ctx->lock);
        ctx->sock = -1;
        return -1;
    }

    /* Override the socket receive timeout with the per-request value. */
    uint32_t resp_ms = (cfg->response_timeout_ms != 0u)
                       ? cfg->response_timeout_ms
                       : DEFAULT_RESPONSE_TIMEOUT_MS;

    struct timeval response_tv = {
        .tv_sec  = (time_t)(resp_ms / 1000u),
        .tv_usec = (suseconds_t)((resp_ms % 1000u) * 1000u),
    };
    setsockopt(ctx->sock, SOL_SOCKET, SO_RCVTIMEO,
               &response_tv, sizeof(response_tv));

    cli_logv(ctx, MB_TCP_LOG_DEBUG,
             "connected to %s:%u", cfg->remote_host, (unsigned)cfg->port);
    return 0;
}

void mb_tcp_client_disconnect(mb_tcp_client_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }
    mb_tcp_close(&ctx->sock);
    pthread_mutex_destroy(&ctx->lock);
}

int mb_tcp_client_read_holding_registers(mb_tcp_client_ctx_t *ctx,
                                          uint16_t addr, uint16_t qty,
                                          uint16_t *out)
{
    if (!ctx || !out || qty == 0u || qty > MODBUS_MAX_READ_REGISTERS) {
        return MB_TCP_CLIENT_ERR_ARG;
    }
    if (ctx->sock < 0) {
        return MB_TCP_CLIENT_ERR_NOT_CONNECTED;
    }

    uint8_t req[MODBUS_TCP_MIN_READ_REQ_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    pthread_mutex_lock(&ctx->lock);

    uint16_t tid = next_tid(ctx);
    build_fc03_request(tid, ctx->cfg.unit_id, addr, qty, req);

    if (send(ctx->sock, req, sizeof(req), 0) != (ssize_t)sizeof(req)) {
        cli_logv(ctx, MB_TCP_LOG_ERROR, "FC03 send failed");
        pthread_mutex_unlock(&ctx->lock);
        return MB_TCP_CLIENT_ERR_TRANSPORT;
    }

    size_t total_len = 0u;
    int rc = recv_response(ctx->sock, resp, sizeof(resp), &total_len);
    if (rc != 0) {
        cli_logv(ctx, MB_TCP_LOG_ERROR, "FC03 recv failed (rc=%d)", rc);
        pthread_mutex_unlock(&ctx->lock);
        return recv_err_to_client_err(rc);
    }

    pthread_mutex_unlock(&ctx->lock);

    /* Validate TID. */
    uint16_t resp_tid = ((uint16_t)resp[ADU_OFFSET_TID_HIGH] << 8)
                      | (uint16_t)resp[ADU_OFFSET_TID_LOW];
    if (resp_tid != tid) {
        cli_logv(ctx, MB_TCP_LOG_WARN,
                 "FC03 TID mismatch: sent %u, received %u",
                 (unsigned)tid, (unsigned)resp_tid);
        return MB_TCP_CLIENT_ERR_TID;
    }

    /* Check for exception response. */
    uint8_t resp_fc = resp[ADU_OFFSET_FC];
    if (resp_fc & (uint8_t)MODBUS_EXCEPTION_FLAG) {
        uint8_t exception_code = resp[ADU_OFFSET_ADDR_HIGH]; /* exception byte */
        cli_logv(ctx, MB_TCP_LOG_WARN,
                 "FC03 exception code %u", (unsigned)exception_code);
        return (int)exception_code;  /* positive Modbus exception code */
    }

    if (resp_fc != (uint8_t)MODBUS_FUNC_READ_HOLDING_REGISTERS) {
        return MB_TCP_CLIENT_ERR_FRAME;
    }

    /* Validate byte count in response. */
    if (total_len < FC03_RESPONSE_HEADER_LEN) {
        return MB_TCP_CLIENT_ERR_FRAME;
    }
    uint8_t byte_count = resp[FC03_RESPONSE_BYTE_COUNT_OFFSET];
    if (byte_count != (uint8_t)(qty * 2u)) {
        return MB_TCP_CLIENT_ERR_FRAME;
    }

    /* Extract register values (big-endian → host byte order). */
    for (uint16_t i = 0u; i < qty; i++) {
        size_t offset = FC03_RESPONSE_HEADER_LEN + (size_t)(i * 2u);
        out[i] = ((uint16_t)resp[offset] << 8) | (uint16_t)resp[offset + 1u];
    }

    return MB_TCP_CLIENT_OK;
}

int mb_tcp_client_write_single_register(mb_tcp_client_ctx_t *ctx,
                                         uint16_t addr, uint16_t value)
{
    if (!ctx) {
        return MB_TCP_CLIENT_ERR_ARG;
    }
    if (ctx->sock < 0) {
        return MB_TCP_CLIENT_ERR_NOT_CONNECTED;
    }

    uint8_t req[MODBUS_TCP_MIN_READ_REQ_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    pthread_mutex_lock(&ctx->lock);

    uint16_t tid = next_tid(ctx);
    build_fc06_request(tid, ctx->cfg.unit_id, addr, value, req);

    if (send(ctx->sock, req, sizeof(req), 0) != (ssize_t)sizeof(req)) {
        cli_logv(ctx, MB_TCP_LOG_ERROR, "FC06 send failed");
        pthread_mutex_unlock(&ctx->lock);
        return MB_TCP_CLIENT_ERR_TRANSPORT;
    }

    size_t total_len = 0u;
    int rc = recv_response(ctx->sock, resp, sizeof(resp), &total_len);
    if (rc != 0) {
        cli_logv(ctx, MB_TCP_LOG_ERROR, "FC06 recv failed (rc=%d)", rc);
        pthread_mutex_unlock(&ctx->lock);
        return recv_err_to_client_err(rc);
    }

    pthread_mutex_unlock(&ctx->lock);

    return validate_write_response(ctx, req, resp, total_len, tid);
}

int mb_tcp_client_write_multiple_registers(mb_tcp_client_ctx_t *ctx,
                                            uint16_t addr, uint16_t qty,
                                            const uint16_t *data)
{
    if (!ctx || !data || qty == 0u || qty > MODBUS_MAX_WRITE_REGISTERS) {
        return MB_TCP_CLIENT_ERR_ARG;
    }
    if (ctx->sock < 0) {
        return MB_TCP_CLIENT_ERR_NOT_CONNECTED;
    }

    uint8_t req[MODBUS_TCP_MAX_ADU_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    pthread_mutex_lock(&ctx->lock);

    uint16_t tid = next_tid(ctx);
    size_t req_len = build_fc16_request(tid, ctx->cfg.unit_id, addr, qty, data, req);

    if (send(ctx->sock, req, req_len, 0) != (ssize_t)req_len) {
        cli_logv(ctx, MB_TCP_LOG_ERROR, "FC16 send failed");
        pthread_mutex_unlock(&ctx->lock);
        return MB_TCP_CLIENT_ERR_TRANSPORT;
    }

    size_t total_len = 0u;
    int rc = recv_response(ctx->sock, resp, sizeof(resp), &total_len);
    if (rc != 0) {
        cli_logv(ctx, MB_TCP_LOG_ERROR, "FC16 recv failed (rc=%d)", rc);
        pthread_mutex_unlock(&ctx->lock);
        return recv_err_to_client_err(rc);
    }

    pthread_mutex_unlock(&ctx->lock);

    return validate_write_response(ctx, req, resp, total_len, tid);
}
