/**
 * @file modbus_tcp_server.c
 * @brief Modbus TCP server API – implementation.
 *
 * Implements the on_process callback consumed by the transport layer.
 * For each client request the callback:
 *   1. Receives the complete Modbus TCP ADU (MBAP header then PDU body).
 *   2. Validates frame length and unit ID.
 *   3. Dispatches to the caller-supplied on_read / on_write handler.
 *   4. Sends back the appropriate response or Modbus exception.
 *
 * Supported function codes:
 *   FC03 – Read Holding Registers
 *   FC04 – Read Input Registers
 *   FC06 – Write Single Register
 *   FC16 (0x10) – Write Multiple Registers
 */

#define _GNU_SOURCE

#include <stdarg.h>
#include <string.h>
#include <sys/socket.h>

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
 *   7    Function Code
 *   8    Start Address   (high byte)
 *   9    Start Address   (low  byte)
 *  10    Quantity        (high byte)  / FC06 value high
 *  11    Quantity        (low  byte)  / FC06 value low
 *  12    Byte Count                   (FC16 only)
 *  13+   Register Data               (FC16 only, qty × 2 bytes)
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
/* FC06: value sits at the same position as quantity. */
#define ADU_OFFSET_FC06_VALUE_HIGH  ADU_OFFSET_QTY_HIGH
#define ADU_OFFSET_FC06_VALUE_LOW   ADU_OFFSET_QTY_LOW
/* FC16: byte-count field and start of register data. */
#define ADU_OFFSET_FC16_BYTE_COUNT 12u
#define ADU_OFFSET_FC16_DATA_START 13u

/* ── Fixed-length response sizes ──────────────────────────────────────────
 *  Exception:  MBAP(6) + unit(1) + FC|0x80(1) + exc_code(1)      =  9
 *  FC16 reply: MBAP(6) + unit(1) + FC(1) + addr(2) + qty(2)      = 12
 * ──────────────────────────────────────────────────────────────────────── */
#define EXCEPTION_RESPONSE_LEN      9u
#define FC16_RESPONSE_LEN          12u

/* MBAP Length field values for these fixed responses. */
#define EXCEPTION_MBAP_LEN          3u   /* unit(1) + FC(1) + exc(1)              */
#define FC16_RESPONSE_MBAP_LEN      6u   /* unit(1) + FC(1) + addr(2) + qty(2)    */

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

/* ── Exception code mapping ────────────────────────────────────────────── */

/**
 * Maps a callback return value to a Modbus exception code.
 *
 * Recognized exception codes (MODBUS_EX_ILLEGAL_FUNCTION through
 * MODBUS_EX_SERVER_DEVICE_FAILURE) are passed through directly.
 * Any other non-zero value falls back to MODBUS_EX_SERVER_DEVICE_FAILURE.
 */
static uint8_t exception_from_callback_rc(int rc)
{
    if (rc >= (int)MODBUS_EX_ILLEGAL_FUNCTION &&
        rc <= (int)MODBUS_EX_SERVER_DEVICE_FAILURE) {
        return (uint8_t)rc;
    }
    return (uint8_t)MODBUS_EX_SERVER_DEVICE_FAILURE;
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
        if (n <= 0) {
            return -1;
        }
        received += (size_t)n;
    }
    return 0;
}

/* ── Response builders ─────────────────────────────────────────────────── */

/**
 * Write a Modbus exception response into buf.
 * @return EXCEPTION_RESPONSE_LEN, or -1 if buf_cap is too small.
 */
static int build_exception_response(uint16_t tid, uint8_t unit_id, uint8_t fc,
                                    uint8_t exception_code,
                                    uint8_t *buf, size_t buf_cap)
{
    if (buf_cap < EXCEPTION_RESPONSE_LEN) {
        return -1;
    }
    buf[ADU_OFFSET_TID_HIGH]  = (uint8_t)(tid >> 8);
    buf[ADU_OFFSET_TID_LOW]   = (uint8_t)(tid & 0xFFu);
    buf[ADU_OFFSET_PROTO_HIGH] = 0x00u;
    buf[ADU_OFFSET_PROTO_LOW]  = 0x00u;
    buf[ADU_OFFSET_LEN_HIGH]  = 0x00u;
    buf[ADU_OFFSET_LEN_LOW]   = EXCEPTION_MBAP_LEN;
    buf[ADU_OFFSET_UNIT_ID]   = unit_id;
    buf[ADU_OFFSET_FC]        = fc | (uint8_t)MODBUS_EXCEPTION_FLAG;
    buf[ADU_OFFSET_ADDR_HIGH] = exception_code;   /* exception code at byte 8 */
    return (int)EXCEPTION_RESPONSE_LEN;
}

/**
 * Write a FC16 (Write Multiple Registers) success response into buf.
 * @return FC16_RESPONSE_LEN, or -1 if buf_cap is too small.
 */
static int build_fc16_response(uint16_t tid, uint8_t unit_id,
                               uint16_t start_addr, uint16_t qty,
                               uint8_t *buf, size_t buf_cap)
{
    if (buf_cap < FC16_RESPONSE_LEN) {
        return -1;
    }
    buf[ADU_OFFSET_TID_HIGH]   = (uint8_t)(tid >> 8);
    buf[ADU_OFFSET_TID_LOW]    = (uint8_t)(tid & 0xFFu);
    buf[ADU_OFFSET_PROTO_HIGH] = 0x00u;
    buf[ADU_OFFSET_PROTO_LOW]  = 0x00u;
    buf[ADU_OFFSET_LEN_HIGH]   = 0x00u;
    buf[ADU_OFFSET_LEN_LOW]    = FC16_RESPONSE_MBAP_LEN;
    buf[ADU_OFFSET_UNIT_ID]    = unit_id;
    buf[ADU_OFFSET_FC]         = (uint8_t)MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS;
    buf[ADU_OFFSET_ADDR_HIGH]  = (uint8_t)(start_addr >> 8);
    buf[ADU_OFFSET_ADDR_LOW]   = (uint8_t)(start_addr & 0xFFu);
    buf[ADU_OFFSET_QTY_HIGH]   = (uint8_t)(qty >> 8);
    buf[ADU_OFFSET_QTY_LOW]    = (uint8_t)(qty & 0xFFu);
    return (int)FC16_RESPONSE_LEN;
}

/* ── on_process: transport-layer callback, one call per readable socket ── */

static int server_on_process(void *userdata, int client_fd)
{
    mb_tcp_server_ctx_t *ctx = (mb_tcp_server_ctx_t *)userdata;
    uint8_t adu[MODBUS_TCP_MAX_ADU_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];
    int     resp_len = -1;

    /* Step 1 – receive MBAP header to learn the remaining frame length. */
    if (recv_exact(client_fd, adu, MODBUS_TCP_MBAP_HEADER_LEN) < 0) {
        return -1;
    }

    uint16_t mbap_remaining = ((uint16_t)adu[ADU_OFFSET_LEN_HIGH] << 8)
                            | (uint16_t)adu[ADU_OFFSET_LEN_LOW];

    if (mbap_remaining == 0u ||
        mbap_remaining > (MODBUS_TCP_MAX_ADU_LENGTH - MODBUS_TCP_MBAP_HEADER_LEN)) {
        srv_logv(ctx, MB_TCP_LOG_WARN,
                 "invalid MBAP length field: %u", (unsigned)mbap_remaining);
        return -1;
    }

    /* Step 2 – receive unit ID + PDU body. */
    if (recv_exact(client_fd, adu + MODBUS_TCP_MBAP_HEADER_LEN, mbap_remaining) < 0) {
        return -1;
    }

    size_t   total_len = MODBUS_TCP_MBAP_HEADER_LEN + (size_t)mbap_remaining;
    uint16_t tid       = ((uint16_t)adu[ADU_OFFSET_TID_HIGH] << 8)
                       | (uint16_t)adu[ADU_OFFSET_TID_LOW];
    uint8_t  unit_id   = adu[ADU_OFFSET_UNIT_ID];

    /* Step 3 – validate unit ID.  Silently ignore mismatches (Modbus spec). */
    if (ctx->cfg.unit_id != (uint8_t)MB_TCP_SERVER_UNIT_ID_ANY &&
        unit_id != ctx->cfg.unit_id) {
        srv_logv(ctx, MB_TCP_LOG_WARN,
                 "unit_id mismatch: received %u, expected %u",
                 (unsigned)unit_id, (unsigned)ctx->cfg.unit_id);
        return 0;
    }

    /* Step 4 – parse function code, start address, and quantity. */
    uint8_t  fc;
    uint16_t start_addr;
    uint16_t qty;
    if (mb_tcp_parse_request(adu, total_len, &fc, &start_addr, &qty) < 0) {
        srv_logv(ctx, MB_TCP_LOG_WARN,
                 "failed to parse request (raw FC 0x%02X)", (unsigned)adu[ADU_OFFSET_FC]);
        resp_len = build_exception_response(tid, unit_id, adu[ADU_OFFSET_FC],
                                            MODBUS_EX_ILLEGAL_FUNCTION,
                                            resp, sizeof(resp));
        goto send_response;
    }

    /* Step 5 – dispatch by function code. */
    switch (fc) {

        /* ── FC03 / FC04: read holding or input registers ─────────────── */
        case MODBUS_FUNC_READ_HOLDING_REGISTERS:
        case MODBUS_FUNC_READ_INPUT_REGISTERS: {
            if (qty == 0u || qty > MODBUS_MAX_READ_REGISTERS) {
                srv_logv(ctx, MB_TCP_LOG_WARN,
                         "FC%02X: invalid quantity %u", (unsigned)fc, (unsigned)qty);
                resp_len = build_exception_response(tid, unit_id, fc,
                                                    MODBUS_EX_ILLEGAL_DATA_VALUE,
                                                    resp, sizeof(resp));
                break;
            }
            uint16_t data[MODBUS_MAX_READ_REGISTERS];
            int rc = ctx->cfg.on_read
                     ? ctx->cfg.on_read(fc, start_addr, qty, data, ctx->cfg.userdata)
                     : -1;
            if (rc != 0) {
                resp_len = build_exception_response(tid, unit_id, fc,
                                                    exception_from_callback_rc(rc),
                                                    resp, sizeof(resp));
            } else {
                resp_len = mb_tcp_build_read_registers_response(
                    tid, fc, unit_id, data, qty, resp, sizeof(resp));
            }
            break;
        }

        /* ── FC06: write single register ──────────────────────────────── */
        case MODBUS_FUNC_WRITE_SINGLE_REGISTER: {
            uint16_t value = ((uint16_t)adu[ADU_OFFSET_FC06_VALUE_HIGH] << 8)
                           | (uint16_t)adu[ADU_OFFSET_FC06_VALUE_LOW];
            int rc = ctx->cfg.on_write
                     ? ctx->cfg.on_write(start_addr, 1u, &value, ctx->cfg.userdata)
                     : -1;
            if (rc != 0) {
                resp_len = build_exception_response(tid, unit_id, fc,
                                                    exception_from_callback_rc(rc),
                                                    resp, sizeof(resp));
            } else {
                /* FC06 success: echo the request frame unchanged. */
                memcpy(resp, adu, total_len);
                resp_len = (int)total_len;
            }
            break;
        }

        /* ── FC16: write multiple registers ───────────────────────────── */
        case MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS: {
            if (qty == 0u || qty > MODBUS_MAX_WRITE_REGISTERS) {
                srv_logv(ctx, MB_TCP_LOG_WARN,
                         "FC16: invalid quantity %u", (unsigned)qty);
                resp_len = build_exception_response(tid, unit_id, fc,
                                                    MODBUS_EX_ILLEGAL_DATA_VALUE,
                                                    resp, sizeof(resp));
                break;
            }
            uint8_t byte_count = adu[ADU_OFFSET_FC16_BYTE_COUNT];
            if (byte_count != (uint8_t)(qty * 2u)) {
                srv_logv(ctx, MB_TCP_LOG_WARN,
                         "FC16: byte_count %u inconsistent with qty %u",
                         (unsigned)byte_count, (unsigned)qty);
                resp_len = build_exception_response(tid, unit_id, fc,
                                                    MODBUS_EX_ILLEGAL_DATA_VALUE,
                                                    resp, sizeof(resp));
                break;
            }
            uint16_t data[MODBUS_MAX_WRITE_REGISTERS];
            for (uint16_t i = 0u; i < qty; i++) {
                size_t offset = ADU_OFFSET_FC16_DATA_START + (size_t)(i * 2u);
                data[i] = ((uint16_t)adu[offset] << 8) | (uint16_t)adu[offset + 1u];
            }
            int rc = ctx->cfg.on_write
                     ? ctx->cfg.on_write(start_addr, qty, data, ctx->cfg.userdata)
                     : -1;
            if (rc != 0) {
                resp_len = build_exception_response(tid, unit_id, fc,
                                                    exception_from_callback_rc(rc),
                                                    resp, sizeof(resp));
            } else {
                resp_len = build_fc16_response(tid, unit_id, start_addr, qty,
                                               resp, sizeof(resp));
            }
            break;
        }

        default:
            srv_logv(ctx, MB_TCP_LOG_WARN,
                     "unsupported FC 0x%02X", (unsigned)fc);
            resp_len = build_exception_response(tid, unit_id, fc,
                                                MODBUS_EX_ILLEGAL_FUNCTION,
                                                resp, sizeof(resp));
            break;
    }

send_response:
    if (resp_len > 0) {
        ssize_t sent = send(client_fd, resp, (size_t)resp_len, 0);
        if (sent != (ssize_t)resp_len) {
            srv_logv(ctx, MB_TCP_LOG_ERROR,
                     "send incomplete: %zd of %d bytes sent", sent, resp_len);
            return -1;
        }
    }
    return 0;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int mb_tcp_server_start(mb_tcp_server_ctx_t *ctx, const mb_tcp_server_config_t *cfg)
{
    if (!ctx || !cfg) {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;

    mb_tcp_config_t transport_cfg = {
        .mode             = MB_TCP_MODE_LISTEN_SERVER,
        .port             = cfg->port,
        .recv_timeout_ms  = (cfg->recv_timeout_ms != 0u)
                            ? cfg->recv_timeout_ms
                            : MB_TCP_SERVER_DEFAULT_RECV_TIMEOUT_MS,
        .max_clients      = cfg->max_clients,
        .userdata         = ctx,
        .on_process       = server_on_process,
        .logv             = cfg->logv,
        .log_userdata     = cfg->log_userdata,
        .on_link          = cfg->on_link,
        .link_userdata    = cfg->link_userdata,
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
