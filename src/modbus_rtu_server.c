/**
 * @file modbus_rtu_server.c
 * @brief Modbus RTU server API - implementation.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "modbus_rtu_server.h"

#define RTU_OFFSET_UNIT_ID        0u
#define RTU_OFFSET_FC             1u
#define RTU_OFFSET_ADDR_HIGH      2u
#define RTU_OFFSET_ADDR_LOW       3u
#define RTU_OFFSET_QTY_HIGH       4u
#define RTU_OFFSET_QTY_LOW        5u
#define RTU_OFFSET_FC06_VAL_HIGH  RTU_OFFSET_QTY_HIGH
#define RTU_OFFSET_FC06_VAL_LOW   RTU_OFFSET_QTY_LOW
#define RTU_OFFSET_FC16_BYTE_CNT  6u
#define RTU_OFFSET_FC16_DATA      7u

#define RTU_EXCEPTION_LEN         5u
#define RTU_WRITE_RESPONSE_LEN    8u
#define RTU_IO_FRAME_ERROR       (-10)

static void srv_logv(mb_rtu_server_ctx_t *ctx, mb_rtu_log_level_t level,
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

static uint8_t exception_from_callback_rc(int rc)
{
    if (rc >= (int)MODBUS_EX_ILLEGAL_FUNCTION &&
        rc <= (int)MODBUS_EX_SERVER_DEVICE_FAILURE) {
        return (uint8_t)rc;
    }
    return (uint8_t)MODBUS_EX_SERVER_DEVICE_FAILURE;
}

static uint32_t effective_timeout_ms(const mb_rtu_server_ctx_t *ctx)
{
    return (ctx->cfg.recv_timeout_ms != 0u)
           ? ctx->cfg.recv_timeout_ms
           : MB_RTU_SERVER_DEFAULT_RECV_TIMEOUT_MS;
}

static uint64_t current_time_ms(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0u;
    }

    return ((uint64_t)ts.tv_sec * 1000u) + ((uint64_t)ts.tv_nsec / 1000000u);
}

static void reset_rx_state(mb_rtu_server_ctx_t *ctx)
{
    ctx->rx_len = 0u;
    ctx->last_rx_ms = 0u;
}

static int read_available_bytes(mb_rtu_server_ctx_t *ctx, int fd, uint64_t now_ms)
{
    while (ctx->rx_len < sizeof(ctx->rx_buffer)) {
        ssize_t n = read(fd, ctx->rx_buffer + ctx->rx_len,
                         sizeof(ctx->rx_buffer) - ctx->rx_len);

        if (n > 0) {
            ctx->rx_len += (size_t)n;
            ctx->last_rx_ms = now_ms;
            continue;
        }
        if (n == 0) {
            return 0;
        }
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;
            }
            return -1;
        }
    }

    return 0;
}

static int get_complete_request_len(const uint8_t *adu, size_t rx_len,
                                    size_t adu_cap, size_t *total_len)
{
    if (rx_len < 2u) {
        return 0;
    }

    uint8_t fc = adu[RTU_OFFSET_FC];
    if (fc == MODBUS_FUNC_READ_HOLDING_REGISTERS ||
        fc == MODBUS_FUNC_READ_INPUT_REGISTERS ||
        fc == MODBUS_FUNC_WRITE_SINGLE_REGISTER) {
        if (adu_cap < MODBUS_RTU_MIN_REQ_LENGTH) {
            return RTU_IO_FRAME_ERROR;
        }
        if (rx_len < MODBUS_RTU_MIN_REQ_LENGTH) {
            return 0;
        }
        *total_len = MODBUS_RTU_MIN_REQ_LENGTH;
        return 1;
    }

    if (fc == MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS) {
        if (rx_len < RTU_OFFSET_FC16_DATA) {
            return 0;
        }

        uint8_t byte_count = adu[RTU_OFFSET_FC16_BYTE_CNT];
        size_t frame_len = 9u + (size_t)byte_count;
        if (byte_count == 0u || frame_len > adu_cap) {
            return RTU_IO_FRAME_ERROR;
        }
        if (rx_len < frame_len) {
            return 0;
        }
        *total_len = frame_len;
        return 1;
    }

    /* Most unsupported public function requests still use a fixed 8-byte RTU ADU. */
    if (adu_cap < MODBUS_RTU_MIN_REQ_LENGTH) {
        return RTU_IO_FRAME_ERROR;
    }
    if (rx_len < MODBUS_RTU_MIN_REQ_LENGTH) {
        return 0;
    }
    *total_len = MODBUS_RTU_MIN_REQ_LENGTH;
    return 1;
}

static int extract_request_frame(mb_rtu_server_ctx_t *ctx,
                                 uint8_t *adu, size_t adu_cap,
                                 size_t *total_len)
{
    size_t frame_len = 0u;
    int rc = get_complete_request_len(ctx->rx_buffer, ctx->rx_len,
                                      sizeof(ctx->rx_buffer), &frame_len);
    if (rc <= 0) {
        return rc;
    }
    if (!adu || frame_len > adu_cap) {
        return RTU_IO_FRAME_ERROR;
    }

    memcpy(adu, ctx->rx_buffer, frame_len);

    size_t remaining = ctx->rx_len - frame_len;
    if (remaining > 0u) {
        memmove(ctx->rx_buffer, ctx->rx_buffer + frame_len, remaining);
    }

    ctx->rx_len = remaining;
    ctx->last_rx_ms = (remaining > 0u) ? current_time_ms() : 0u;
    *total_len = frame_len;
    return 1;
}

static int write_all(int fd, const uint8_t *buf, size_t len)
{
    size_t sent = 0u;

    while (sent < len) {
        ssize_t n = write(fd, buf + sent, len - sent);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        if (n == 0) {
            return -1;
        }
        sent += (size_t)n;
    }

    if (tcdrain(fd) != 0) {
        return -1;
    }
    return 0;
}

static int build_exception_response(uint8_t unit_id, uint8_t fc, uint8_t exception_code,
                                    uint8_t *buf, size_t buf_cap)
{
    if (!buf || buf_cap < RTU_EXCEPTION_LEN) {
        return -1;
    }

    buf[RTU_OFFSET_UNIT_ID] = unit_id;
    buf[RTU_OFFSET_FC] = fc | (uint8_t)MODBUS_EXCEPTION_FLAG;
    buf[2] = exception_code;

    if (mb_rtu_append_crc(buf, RTU_EXCEPTION_LEN - MODBUS_RTU_CRC_LEN, buf_cap) != 0) {
        return -1;
    }
    return (int)RTU_EXCEPTION_LEN;
}

static int build_fc16_response(uint8_t unit_id, uint8_t fc,
                               uint16_t start_addr, uint16_t qty,
                               uint8_t *buf, size_t buf_cap)
{
    if (!buf || buf_cap < RTU_WRITE_RESPONSE_LEN) {
        return -1;
    }

    buf[RTU_OFFSET_UNIT_ID] = unit_id;
    buf[RTU_OFFSET_FC] = fc;
    buf[RTU_OFFSET_ADDR_HIGH] = (uint8_t)(start_addr >> 8);
    buf[RTU_OFFSET_ADDR_LOW] = (uint8_t)(start_addr & 0xFFu);
    buf[RTU_OFFSET_QTY_HIGH] = (uint8_t)(qty >> 8);
    buf[RTU_OFFSET_QTY_LOW] = (uint8_t)(qty & 0xFFu);

    if (mb_rtu_append_crc(buf, RTU_WRITE_RESPONSE_LEN - MODBUS_RTU_CRC_LEN,
                          buf_cap) != 0) {
        return -1;
    }
    return (int)RTU_WRITE_RESPONSE_LEN;
}

static int unit_id_matches(const mb_rtu_server_ctx_t *ctx, uint8_t unit_id)
{
    if (unit_id == MODBUS_RTU_BROADCAST_ADDRESS) {
        return 1;
    }
    if (ctx->cfg.unit_id == MB_RTU_SERVER_UNIT_ID_ANY) {
        return 1;
    }
    return unit_id == ctx->cfg.unit_id;
}

static int server_on_process(void *userdata, int serial_fd)
{
    mb_rtu_server_ctx_t *ctx = (mb_rtu_server_ctx_t *)userdata;
    uint8_t adu[MODBUS_RTU_MAX_ADU_LENGTH];
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];
    size_t total_len = 0u;
    int resp_len = -1;

    uint64_t now_ms = current_time_ms();
    if (ctx->rx_len > 0u && ctx->last_rx_ms > 0u &&
        now_ms >= ctx->last_rx_ms &&
        (now_ms - ctx->last_rx_ms) > (uint64_t)effective_timeout_ms(ctx)) {
        srv_logv(ctx, MB_RTU_LOG_WARN, "rtu frame assembly timeout");
        reset_rx_state(ctx);
    }

    int rc = read_available_bytes(ctx, serial_fd, now_ms);
    if (rc < 0) {
        return -1;
    }

    rc = extract_request_frame(ctx, adu, sizeof(adu), &total_len);
    if (rc == 0) {
        return 0;
    }
    if (rc == RTU_IO_FRAME_ERROR) {
        srv_logv(ctx, MB_RTU_LOG_WARN, "invalid rtu frame");
        reset_rx_state(ctx);
        return 0;
    }
    if (rc < 0) {
        return -1;
    }

    if (mb_rtu_validate_crc(adu, total_len) != 0) {
        srv_logv(ctx, MB_RTU_LOG_WARN, "crc validation failed");
        return 0;
    }

    uint8_t unit_id = adu[RTU_OFFSET_UNIT_ID];
    uint8_t fc = adu[RTU_OFFSET_FC];
    int is_broadcast = (unit_id == MODBUS_RTU_BROADCAST_ADDRESS);

    if (!unit_id_matches(ctx, unit_id)) {
        srv_logv(ctx, MB_RTU_LOG_WARN,
                 "unit_id mismatch: received %u, expected %u",
                 (unsigned)unit_id, (unsigned)ctx->cfg.unit_id);
        return 0;
    }

    if (is_broadcast &&
        fc != MODBUS_FUNC_WRITE_SINGLE_REGISTER &&
        fc != MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS) {
        return 0;
    }

    uint8_t parsed_fc = 0u;
    uint16_t start_addr = 0u;
    uint16_t qty = 0u;
    if (mb_rtu_parse_request(adu, total_len, &parsed_fc, &start_addr, &qty) != 0) {
        resp_len = build_exception_response(unit_id, fc, MODBUS_EX_ILLEGAL_FUNCTION,
                                            resp, sizeof(resp));
        goto send_response;
    }

    switch (parsed_fc) {
        case MODBUS_FUNC_READ_HOLDING_REGISTERS:
        case MODBUS_FUNC_READ_INPUT_REGISTERS: {
            if (qty == 0u || qty > MODBUS_MAX_READ_REGISTERS) {
                resp_len = build_exception_response(unit_id, parsed_fc,
                                                    MODBUS_EX_ILLEGAL_DATA_VALUE,
                                                    resp, sizeof(resp));
                break;
            }

            uint16_t data[MODBUS_MAX_READ_REGISTERS];
            rc = ctx->cfg.on_read
                 ? ctx->cfg.on_read(parsed_fc, start_addr, qty, data, ctx->cfg.userdata)
                 : -1;
            if (rc != 0) {
                resp_len = build_exception_response(unit_id, parsed_fc,
                                                    exception_from_callback_rc(rc),
                                                    resp, sizeof(resp));
            } else {
                resp_len = mb_rtu_build_read_registers_response(parsed_fc, unit_id,
                                                                data, qty,
                                                                resp, sizeof(resp));
            }
            break;
        }

        case MODBUS_FUNC_WRITE_SINGLE_REGISTER: {
            uint16_t value = ((uint16_t)adu[RTU_OFFSET_FC06_VAL_HIGH] << 8)
                           | (uint16_t)adu[RTU_OFFSET_FC06_VAL_LOW];
            rc = ctx->cfg.on_write
                 ? ctx->cfg.on_write(start_addr, 1u, &value, ctx->cfg.userdata)
                 : -1;
            if (rc != 0) {
                resp_len = build_exception_response(unit_id, parsed_fc,
                                                    exception_from_callback_rc(rc),
                                                    resp, sizeof(resp));
            } else {
                memcpy(resp, adu, total_len);
                resp_len = (int)total_len;
            }
            break;
        }

        case MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS: {
            if (qty == 0u || qty > MODBUS_MAX_WRITE_REGISTERS) {
                resp_len = build_exception_response(unit_id, parsed_fc,
                                                    MODBUS_EX_ILLEGAL_DATA_VALUE,
                                                    resp, sizeof(resp));
                break;
            }

            uint8_t byte_count = adu[RTU_OFFSET_FC16_BYTE_CNT];
            if (byte_count != (uint8_t)(qty * 2u) ||
                total_len != 9u + (size_t)byte_count) {
                resp_len = build_exception_response(unit_id, parsed_fc,
                                                    MODBUS_EX_ILLEGAL_DATA_VALUE,
                                                    resp, sizeof(resp));
                break;
            }

            uint16_t data[MODBUS_MAX_WRITE_REGISTERS];
            for (uint16_t i = 0u; i < qty; i++) {
                size_t offset = RTU_OFFSET_FC16_DATA + (size_t)(i * 2u);
                data[i] = ((uint16_t)adu[offset] << 8) | (uint16_t)adu[offset + 1u];
            }

            rc = ctx->cfg.on_write
                 ? ctx->cfg.on_write(start_addr, qty, data, ctx->cfg.userdata)
                 : -1;
            if (rc != 0) {
                resp_len = build_exception_response(unit_id, parsed_fc,
                                                    exception_from_callback_rc(rc),
                                                    resp, sizeof(resp));
            } else {
                resp_len = build_fc16_response(unit_id, parsed_fc, start_addr, qty,
                                               resp, sizeof(resp));
            }
            break;
        }

        default:
            resp_len = build_exception_response(unit_id, parsed_fc,
                                                MODBUS_EX_ILLEGAL_FUNCTION,
                                                resp, sizeof(resp));
            break;
    }

send_response:
    if (is_broadcast) {
        return 0;
    }

    if (resp_len > 0 && write_all(serial_fd, resp, (size_t)resp_len) != 0) {
        srv_logv(ctx, MB_RTU_LOG_ERROR, "send response failed");
        return -1;
    }

    return 0;
}

void mb_rtu_server_config_init(mb_rtu_server_config_t *cfg)
{
    if (!cfg) {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->baud_rate = 9600u;
    cfg->data_bits = 8u;
    cfg->stop_bits = 1u;
    cfg->parity = MB_RTU_PARITY_NONE;
    cfg->recv_timeout_ms = MB_RTU_SERVER_DEFAULT_RECV_TIMEOUT_MS;
}

int mb_rtu_server_start(mb_rtu_server_ctx_t *ctx, const mb_rtu_server_config_t *cfg)
{
    if (!ctx || !cfg || !cfg->device || cfg->device[0] == '\0') {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    if (ctx->cfg.recv_timeout_ms == 0u) {
        ctx->cfg.recv_timeout_ms = MB_RTU_SERVER_DEFAULT_RECV_TIMEOUT_MS;
    }

    mb_rtu_config_t transport_cfg = {
        .serial = {
            .device = cfg->device,
            .baud_rate = cfg->baud_rate,
            .data_bits = cfg->data_bits,
            .stop_bits = cfg->stop_bits,
            .parity = cfg->parity,
        },
        .recv_timeout_ms = ctx->cfg.recv_timeout_ms,
        .userdata = ctx,
        .on_process = server_on_process,
        .logv = cfg->logv,
        .log_userdata = cfg->log_userdata,
        .on_link = cfg->on_link,
        .link_userdata = cfg->link_userdata,
    };

    return mb_rtu_start(&ctx->transport, &transport_cfg);
}

void mb_rtu_server_stop(mb_rtu_server_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    mb_rtu_stop(&ctx->transport);
}
