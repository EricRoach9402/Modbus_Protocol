/**
 * @file modbus_tcp_client.c
 * @brief Modbus TCP client API – implementation.
 *
 * Provides blocking, thread-safe read/write helpers.  Each public function
 * holds the context mutex for the full request–response cycle, ensuring only
 * one transaction is in flight at a time.
 *
 * PDUs are built and validated by modbus_pdu; this file only adds / checks
 * the MBAP header.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <poll.h>
#include <stdarg.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>

#include "modbus_pdu.h"
#include "modbus_tcp_client.h"

/* ── Default tunables ──────────────────────────────────────────────────── */

#define DEFAULT_CONNECT_TIMEOUT_SEC 5u
#define DEFAULT_RESPONSE_TIMEOUT_MS 1000u

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
 *   7+   PDU (function code + data)
 * ──────────────────────────────────────────────────────────────────────── */
#define ADU_OFFSET_TID_HIGH 0u
#define ADU_OFFSET_TID_LOW 1u
#define ADU_OFFSET_PROTO_HIGH 2u
#define ADU_OFFSET_PROTO_LOW 3u
#define ADU_OFFSET_LEN_HIGH 4u
#define ADU_OFFSET_LEN_LOW 5u
#define ADU_OFFSET_UNIT_ID 6u
#define ADU_OFFSET_PDU 7u

/* recv_response(): complete frame received but Protocol ID is not Modbus (0). */
#define RECV_PROTOCOL_ID_ERROR (-3)
/* recv_response(): timed out mid-frame; the byte stream is no longer aligned. */
#define RECV_PARTIAL_TIMEOUT (-4)

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
        if (n == 0) {
            return -1; /* peer closed connection */
        }
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ETIMEDOUT) {
                return -2; /* timeout */
            }
            return -1;
        }
        received += (size_t)n;
    }
    return 0;
}

/* Map recv_response return codes to MB_TCP_CLIENT_ERR_* codes. */
static int recv_err_to_client_err(int recv_rc)
{
    if (recv_rc == -2 || recv_rc == RECV_PARTIAL_TIMEOUT) {
        return MB_TCP_CLIENT_ERR_TIMEOUT;
    }
    if (recv_rc == RECV_PROTOCOL_ID_ERROR) {
        return MB_TCP_CLIENT_ERR_FRAME;
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

/* ── Response receiver ─────────────────────────────────────────────────── */

static uint64_t monotonic_ms(void)
{
    struct timespec now;

    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)now.tv_sec * 1000u + (uint64_t)now.tv_nsec / 1000000u;
}

static uint32_t effective_response_timeout_ms(const mb_tcp_client_ctx_t *ctx)
{
    return (ctx->cfg.response_timeout_ms != 0u)
           ? ctx->cfg.response_timeout_ms
           : DEFAULT_RESPONSE_TIMEOUT_MS;
}

/** @return 0 when @p fd is readable, -2 at @p deadline_ms, -1 on error. */
static int wait_readable(int fd, uint64_t deadline_ms)
{
    for (;;) {
        uint64_t now_ms = monotonic_ms();
        if (now_ms >= deadline_ms) {
            return -2;
        }

        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        int rc = poll(&pfd, 1, (int)(deadline_ms - now_ms));
        if (rc > 0) {
            return 0;
        }
        if (rc == 0) {
            return -2;
        }
        if (errno != EINTR) {
            return -1;
        }
    }
}

/**
 * Receive one complete ADU response from sock into buf (up to buf_cap bytes).
 * Waits until @p deadline_ms for the first byte, then reads MBAP header and body.
 *
 * @param[out] total_len  Set to total bytes received on success.
 * @return 0, -1 (transport error or malformed length), -2 (no data before the
 *         deadline; stream still aligned), RECV_PARTIAL_TIMEOUT, or
 *         RECV_PROTOCOL_ID_ERROR.
 */
static int recv_response(int sock, uint8_t *buf, size_t buf_cap, uint64_t deadline_ms,
    size_t *total_len)
{
    int rc = wait_readable(sock, deadline_ms);
    if (rc != 0) {
        return rc;
    }

    rc = recv_exact(sock, buf, MODBUS_TCP_MBAP_HEADER_LEN);
    if (rc != 0) {
        return (rc == -2) ? RECV_PARTIAL_TIMEOUT : rc;
    }

    uint16_t mbap_remaining = ((uint16_t)buf[ADU_OFFSET_LEN_HIGH] << 8)
                            | (uint16_t)buf[ADU_OFFSET_LEN_LOW];

    if (mbap_remaining == 0u ||
        mbap_remaining > (buf_cap - MODBUS_TCP_MBAP_HEADER_LEN)) {
        return -1; /* malformed length */
    }

    rc = recv_exact(sock, buf + MODBUS_TCP_MBAP_HEADER_LEN, mbap_remaining);
    if (rc != 0) {
        return (rc == -2) ? RECV_PARTIAL_TIMEOUT : rc;
    }

    *total_len = MODBUS_TCP_MBAP_HEADER_LEN + (size_t)mbap_remaining;

    if (buf[ADU_OFFSET_PROTO_HIGH] != 0u || buf[ADU_OFFSET_PROTO_LOW] != 0u) {
        return RECV_PROTOCOL_ID_ERROR;
    }
    return 0;
}

/* ── Transaction ───────────────────────────────────────────────────────── */

/**
 * Send one request PDU and validate the response.
 *
 * On MB_TCP_CLIENT_OK, @p resp holds the full response ADU; its PDU starts at
 * ADU_OFFSET_PDU and has already passed mb_pdu_check_response().
 *
 * @return MB_TCP_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_TCP_CLIENT_ERR_* code.
 */
static int transact(mb_tcp_client_ctx_t *ctx, const uint8_t *pdu, size_t pdu_len,
    uint8_t *resp)
{
    uint8_t req[MODBUS_TCP_MAX_ADU_LENGTH];
    size_t total_len = 0u;
    uint8_t fc = pdu[0];

    if (!ctx->initialized) {
        return MB_TCP_CLIENT_ERR_NOT_CONNECTED;
    }

    uint16_t mbap_len = (uint16_t)(1u + pdu_len);
    req[ADU_OFFSET_PROTO_HIGH] = 0x00u;
    req[ADU_OFFSET_PROTO_LOW] = 0x00u;
    req[ADU_OFFSET_LEN_HIGH] = (uint8_t)(mbap_len >> 8);
    req[ADU_OFFSET_LEN_LOW] = (uint8_t)(mbap_len & 0xFFu);
    memcpy(req + ADU_OFFSET_PDU, pdu, pdu_len);

    pthread_mutex_lock(&ctx->lock);

    if (ctx->sock < 0) {
        pthread_mutex_unlock(&ctx->lock);
        return MB_TCP_CLIENT_ERR_NOT_CONNECTED;
    }

    uint16_t tid = next_tid(ctx);
    req[ADU_OFFSET_TID_HIGH] = (uint8_t)(tid >> 8);
    req[ADU_OFFSET_TID_LOW] = (uint8_t)(tid & 0xFFu);
    req[ADU_OFFSET_UNIT_ID] = ctx->cfg.unit_id;

    if (mb_tcp_send_all(ctx->sock, req, ADU_OFFSET_PDU + pdu_len) != 0) {
        mb_tcp_close(&ctx->sock);
        pthread_mutex_unlock(&ctx->lock);
        cli_logv(ctx, MB_TCP_LOG_ERROR, "FC%02X send failed, connection closed", (unsigned)fc);
        return MB_TCP_CLIENT_ERR_TRANSPORT;
    }

    /* Late replies to earlier timed-out requests carry an older TID: skip them. */
    uint64_t deadline_ms = monotonic_ms() + effective_response_timeout_ms(ctx);
    int stale_seen = 0;
    int rc;
    for (;;) {
        rc = recv_response(ctx->sock, resp, MODBUS_TCP_MAX_ADU_LENGTH, deadline_ms, &total_len);
        if (rc != 0) {
            break;
        }
        uint16_t resp_tid = ((uint16_t)resp[ADU_OFFSET_TID_HIGH] << 8)
                          | (uint16_t)resp[ADU_OFFSET_TID_LOW];
        if (resp_tid == tid) {
            break;
        }
        cli_logv(ctx, MB_TCP_LOG_WARN, "FC%02X discarding response TID %u (expected %u)",
            (unsigned)fc, (unsigned)resp_tid, (unsigned)tid);
        stale_seen = 1;
    }

    if (rc == -1 || rc == RECV_PARTIAL_TIMEOUT) {
        mb_tcp_close(&ctx->sock);
    }
    pthread_mutex_unlock(&ctx->lock);

    if (rc == -2 && stale_seen) {
        return MB_TCP_CLIENT_ERR_TID;
    }
    if (rc != 0) {
        cli_logv(ctx, MB_TCP_LOG_ERROR, "FC%02X recv failed (rc=%d)%s", (unsigned)fc, rc,
            (rc == -1 || rc == RECV_PARTIAL_TIMEOUT) ? ", connection closed" : "");
        return recv_err_to_client_err(rc);
    }
    if (total_len <= ADU_OFFSET_PDU) {
        return MB_TCP_CLIENT_ERR_FRAME;
    }

    rc = mb_pdu_check_response(pdu, resp + ADU_OFFSET_PDU, total_len - ADU_OFFSET_PDU);
    if (rc == MB_PDU_ERR_FRAME) {
        cli_logv(ctx, MB_TCP_LOG_WARN, "FC%02X malformed or mismatched response",
            (unsigned)fc);
        return MB_TCP_CLIENT_ERR_FRAME;
    }
    if (rc > 0) {
        cli_logv(ctx, MB_TCP_LOG_WARN, "FC%02X exception code %d", (unsigned)fc, rc);
    }
    return rc;
}

static int read_registers(mb_tcp_client_ctx_t *ctx, uint8_t fc,
    uint16_t addr, uint16_t qty, uint16_t *out)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    if (!ctx || !out || qty == 0u || qty > MODBUS_MAX_READ_REGISTERS) {
        return MB_TCP_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_read(pdu, fc, addr, qty);
    int rc = transact(ctx, pdu, pdu_len, resp);
    if (rc == MB_TCP_CLIENT_OK) {
        mb_pdu_get_registers(resp + ADU_OFFSET_PDU, qty, out);
    }
    return rc;
}

static int read_bits(mb_tcp_client_ctx_t *ctx, uint8_t fc,
    uint16_t addr, uint16_t qty, uint8_t *out)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    if (!ctx || !out || qty == 0u || qty > MODBUS_MAX_READ_BITS) {
        return MB_TCP_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_read(pdu, fc, addr, qty);
    int rc = transact(ctx, pdu, pdu_len, resp);
    if (rc == MB_TCP_CLIENT_OK) {
        mb_pdu_get_bits(resp + ADU_OFFSET_PDU, qty, out);
    }
    return rc;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int mb_tcp_client_connect(mb_tcp_client_ctx_t *ctx, const mb_tcp_client_config_t *cfg)
{
    if (!ctx || !cfg || !cfg->remote_host || cfg->remote_host[0] == '\0' ||
        cfg->port == 0u) {
        return -1;
    }
    if (ctx->initialized) {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->sock = -1;
    ctx->tid = 0u;

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
        .tv_sec = (time_t)(resp_ms / 1000u),
        .tv_usec = (suseconds_t)((resp_ms % 1000u) * 1000u),
    };
    setsockopt(ctx->sock, SOL_SOCKET, SO_RCVTIMEO,
        &response_tv, sizeof(response_tv));

    ctx->initialized = 1;
    cli_logv(ctx, MB_TCP_LOG_DEBUG,
        "connected to %s:%u", cfg->remote_host, (unsigned)cfg->port);
    return 0;
}

void mb_tcp_client_disconnect(mb_tcp_client_ctx_t *ctx)
{
    if (!ctx || !ctx->initialized) {
        return;
    }

    pthread_mutex_lock(&ctx->lock);
    mb_tcp_close(&ctx->sock);
    ctx->initialized = 0;
    pthread_mutex_unlock(&ctx->lock);
    pthread_mutex_destroy(&ctx->lock);
}

int mb_tcp_client_set_unit_id(mb_tcp_client_ctx_t *ctx, uint8_t unit_id)
{
    if (!ctx) {
        return MB_TCP_CLIENT_ERR_ARG;
    }
    if (!ctx->initialized) {
        return MB_TCP_CLIENT_ERR_NOT_CONNECTED;
    }

    pthread_mutex_lock(&ctx->lock);
    ctx->cfg.unit_id = unit_id;
    pthread_mutex_unlock(&ctx->lock);
    return MB_TCP_CLIENT_OK;
}

int mb_tcp_client_read_holding_registers(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    uint16_t *out)
{
    return read_registers(ctx, MODBUS_FUNC_READ_HOLDING_REGISTERS, addr, qty, out);
}

int mb_tcp_client_read_input_registers(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    uint16_t *out)
{
    return read_registers(ctx, MODBUS_FUNC_READ_INPUT_REGISTERS, addr, qty, out);
}

int mb_tcp_client_read_coils(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty, uint8_t *out)
{
    return read_bits(ctx, MODBUS_FUNC_READ_COILS, addr, qty, out);
}

int mb_tcp_client_read_discrete_inputs(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty, uint8_t *out)
{
    return read_bits(ctx, MODBUS_FUNC_READ_DISCRETE_INPUTS, addr, qty, out);
}

int mb_tcp_client_write_single_register(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t value)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    if (!ctx) {
        return MB_TCP_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_write_single(pdu, MODBUS_FUNC_WRITE_SINGLE_REGISTER,
        addr, value);
    return transact(ctx, pdu, pdu_len, resp);
}

int mb_tcp_client_write_single_coil(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint8_t value)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    if (!ctx) {
        return MB_TCP_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_write_single(pdu, MODBUS_FUNC_WRITE_SINGLE_COIL, addr,
        value ? MODBUS_COIL_ON : MODBUS_COIL_OFF);
    return transact(ctx, pdu, pdu_len, resp);
}

int mb_tcp_client_write_multiple_registers(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    const uint16_t *data)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    if (!ctx || !data || qty == 0u || qty > MODBUS_MAX_WRITE_REGISTERS) {
        return MB_TCP_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_write_registers(pdu, addr, qty, data);
    return transact(ctx, pdu, pdu_len, resp);
}

int mb_tcp_client_write_multiple_coils(mb_tcp_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    const uint8_t *values)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    if (!ctx || !values || qty == 0u || qty > MODBUS_MAX_WRITE_BITS) {
        return MB_TCP_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_write_coils(pdu, addr, qty, values);
    return transact(ctx, pdu, pdu_len, resp);
}
