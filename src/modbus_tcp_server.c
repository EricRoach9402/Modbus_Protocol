/**
 * @file modbus_tcp_server.c
 * @brief Modbus TCP server API – implementation.
 *
 * Implements the on_process callback consumed by the transport layer.
 * For each client request the callback:
 *   1. Receives the complete Modbus TCP ADU (MBAP header then PDU body).
 *   2. Validates Protocol ID and unit ID.
 *   3. Dispatches the PDU through modbus_pdu (FC01-06, FC15, FC16).
 *   4. Sends back the response or Modbus exception with the request's MBAP.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <sys/socket.h>

#include "modbus_pdu.h"
#include "modbus_tcp_server.h"

/* ── Modbus TCP ADU byte-offset map ────────────────────────────────────────
 *
 *  Byte  Field
 *  ----  ----------------------------------------
 *   0    Transaction ID  (high byte)
 *   1    Transaction ID  (low  byte)
 *   2    Protocol ID     (high byte, always 0x00)
 *   3    Protocol ID     (low  byte, always 0x00)
 *   4    MBAP Length     (high byte)
 *   5    MBAP Length     (low  byte)  ← bytes remaining after this point
 *   6    Unit ID
 *   7+   PDU (function code + data)
 * ──────────────────────────────────────────────────────────────────────── */
#define ADU_OFFSET_PROTO_HIGH 2u
#define ADU_OFFSET_PROTO_LOW 3u
#define ADU_OFFSET_LEN_HIGH 4u
#define ADU_OFFSET_LEN_LOW 5u
#define ADU_OFFSET_UNIT_ID 6u
#define ADU_OFFSET_PDU 7u

/* ── Logging helper ────────────────────────────────────────────────────── */

static void srv_logv(mb_tcp_server_ctx_t *ctx, mb_tcp_log_level_t level,
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
 * Retries on short reads until the full buffer is filled.
 *
 * @return 0 on success, -1 if the peer closed the connection or an error occurred.
 */
static int recv_exact(int fd, uint8_t *buf, size_t len)
{
    size_t received = 0;
    while (received < len) {
        ssize_t n = recv(fd, buf + received, len - received, 0);
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            return -1;
        }
        received += (size_t)n;
    }
    return 0;
}

/* ── on_process: transport-layer callback, one call per readable socket ── */

static int server_on_process(void *userdata, int client_fd)
{
    mb_tcp_server_ctx_t *ctx = (mb_tcp_server_ctx_t *)userdata;
    uint8_t adu[MODBUS_TCP_MAX_ADU_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    /* Step 1 – receive MBAP header to learn the remaining frame length. */
    if (recv_exact(client_fd, adu, MODBUS_TCP_MBAP_HEADER_LEN) < 0) {
        return -1;
    }

    uint16_t mbap_remaining = ((uint16_t)adu[ADU_OFFSET_LEN_HIGH] << 8)
                            | (uint16_t)adu[ADU_OFFSET_LEN_LOW];

    /* Unit ID + at least the function code. */
    if (mbap_remaining < 2u ||
        mbap_remaining > (MODBUS_TCP_MAX_ADU_LENGTH - MODBUS_TCP_MBAP_HEADER_LEN)) {
        srv_logv(ctx, MB_TCP_LOG_WARN,
            "invalid MBAP length field: %u", (unsigned)mbap_remaining);
        return -1;
    }

    /* Step 2 – receive unit ID + PDU body. */
    if (recv_exact(client_fd, adu + MODBUS_TCP_MBAP_HEADER_LEN, mbap_remaining) < 0) {
        return -1;
    }

    uint8_t unit_id = adu[ADU_OFFSET_UNIT_ID];

    /* Non-zero Protocol ID is not Modbus: drop the frame without replying. */
    if (adu[ADU_OFFSET_PROTO_HIGH] != 0u || adu[ADU_OFFSET_PROTO_LOW] != 0u) {
        srv_logv(ctx, MB_TCP_LOG_WARN, "non-Modbus protocol id 0x%02X%02X ignored",
            (unsigned)adu[ADU_OFFSET_PROTO_HIGH], (unsigned)adu[ADU_OFFSET_PROTO_LOW]);
        return 0;
    }

    /* Step 3 – validate unit ID.  Silently ignore mismatches (Modbus spec). */
    if (ctx->cfg.unit_id != (uint8_t)MB_TCP_SERVER_UNIT_ID_ANY &&
        unit_id != ctx->cfg.unit_id) {
        srv_logv(ctx, MB_TCP_LOG_WARN,
            "unit_id mismatch: received %u, expected %u",
            (unsigned)unit_id, (unsigned)ctx->cfg.unit_id);
        return 0;
    }

    /* Step 4 – dispatch the PDU and frame the response with the request's MBAP. */
    const mb_pdu_handlers_t handlers = {
        .read_registers = ctx->cfg.on_read,
        .write_registers = ctx->cfg.on_write,
        .read_bits = ctx->cfg.on_read_bits,
        .write_bits = ctx->cfg.on_write_bits,
        .userdata = ctx->cfg.userdata,
    };
    size_t pdu_len = (size_t)mbap_remaining - 1u;
    size_t resp_pdu_len = mb_pdu_handle_request(&handlers, adu + ADU_OFFSET_PDU, pdu_len,
        resp + ADU_OFFSET_PDU);

    if (resp[ADU_OFFSET_PDU] & (uint8_t)MODBUS_EXCEPTION_FLAG) {
        srv_logv(ctx, MB_TCP_LOG_WARN, "FC%02X -> exception %u",
            (unsigned)adu[ADU_OFFSET_PDU], (unsigned)resp[ADU_OFFSET_PDU + 1u]);
    }

    uint16_t resp_mbap_len = (uint16_t)(1u + resp_pdu_len);
    memcpy(resp, adu, ADU_OFFSET_LEN_HIGH);
    resp[ADU_OFFSET_LEN_HIGH] = (uint8_t)(resp_mbap_len >> 8);
    resp[ADU_OFFSET_LEN_LOW] = (uint8_t)(resp_mbap_len & 0xFFu);
    resp[ADU_OFFSET_UNIT_ID] = unit_id;

    size_t resp_len = ADU_OFFSET_PDU + resp_pdu_len;
    if (mb_tcp_send_all(client_fd, resp, resp_len) != 0) {
        srv_logv(ctx, MB_TCP_LOG_ERROR, "send response failed: %s", strerror(errno));
        return -1;
    }
    return 0;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void mb_tcp_server_config_init(mb_tcp_server_config_t *cfg)
{
    if (!cfg) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->recv_timeout_ms = MB_TCP_SERVER_DEFAULT_RECV_TIMEOUT_MS;
}

int mb_tcp_server_start(mb_tcp_server_ctx_t *ctx, const mb_tcp_server_config_t *cfg)
{
    if (!ctx || !cfg) {
        return -1;
    }
    if (ctx->transport.initialized) {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;

    if (ctx->cfg.recv_timeout_ms == 0u) {
        ctx->cfg.recv_timeout_ms = MB_TCP_SERVER_DEFAULT_RECV_TIMEOUT_MS;
    }

    mb_tcp_config_t transport_cfg = {
        .mode = MB_TCP_MODE_LISTEN_SERVER,
        .port = cfg->port,
        .recv_timeout_ms = ctx->cfg.recv_timeout_ms,
        .max_clients = cfg->max_clients,
        .keepalive_idle_sec = cfg->keepalive_idle_sec,
        .keepalive_interval_sec = cfg->keepalive_interval_sec,
        .keepalive_count = cfg->keepalive_count,
        .idle_poll_ms = cfg->idle_poll_ms,
        .userdata = ctx,
        .on_process = server_on_process,
        .logv = cfg->logv,
        .log_userdata = cfg->log_userdata,
        .on_link = cfg->on_link,
        .link_userdata = cfg->link_userdata,
    };

    return mb_tcp_start(&ctx->transport, &transport_cfg);
}

void mb_tcp_server_stop(mb_tcp_server_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }
    mb_tcp_stop(&ctx->transport);
}
