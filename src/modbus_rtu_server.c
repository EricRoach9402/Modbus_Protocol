/**
 * @file modbus_rtu_server.c
 * @brief Modbus RTU server API - implementation.
 *
 * Frames requests from the serial byte stream (with CRC resync), dispatches
 * the PDU through modbus_pdu (FC01-06, FC15, FC16) and frames the response.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "modbus_pdu.h"
#include "modbus_rtu_server.h"

#define RTU_OFFSET_UNIT_ID        0u
#define RTU_OFFSET_FC             1u
#define RTU_OFFSET_PDU            1u
#define RTU_OFFSET_WRITE_BYTE_CNT 6u

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

static uint32_t effective_timeout_ms(const mb_rtu_server_ctx_t *ctx)
{
    return (ctx->cfg.recv_timeout_ms != 0u)
           ? ctx->cfg.recv_timeout_ms
           : MB_RTU_SERVER_DEFAULT_RECV_TIMEOUT_MS;
}

static uint32_t effective_inter_frame_us(const mb_rtu_server_ctx_t *ctx)
{
    return mb_rtu_effective_timing(ctx->cfg.inter_frame_delay_us, ctx->t35_us);
}

static uint64_t current_time_us(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0u;
    }

    return ((uint64_t)ts.tv_sec * 1000000u) + ((uint64_t)ts.tv_nsec / 1000u);
}

static void sleep_us(uint64_t us)
{
    struct timespec delay = {
        .tv_sec = (time_t)(us / 1000000u),
        .tv_nsec = (long)((us % 1000000u) * 1000u),
    };

    while (nanosleep(&delay, &delay) != 0 && errno == EINTR) {
    }
}

/**
 * Wait until @p since_us + @p required_us has elapsed (request-end → response).
 * If @p since_us is unknown (0), waits the full required delay.
 */
static void wait_response_silence(uint64_t since_us, uint32_t required_us)
{
    uint64_t now_us;
    uint64_t elapsed_us;

    if (required_us == 0u) {
        return;
    }

    if (since_us == 0u) {
        sleep_us(required_us);
        return;
    }

    now_us = current_time_us();
    if (now_us < since_us) {
        sleep_us(required_us);
        return;
    }

    elapsed_us = now_us - since_us;
    if (elapsed_us < (uint64_t)required_us) {
        sleep_us((uint64_t)required_us - elapsed_us);
    }
}

static void reset_rx_state(mb_rtu_server_ctx_t *ctx)
{
    ctx->rx_len = 0u;
    ctx->last_rx_us = 0u;
}

static int read_available_bytes(mb_rtu_server_ctx_t *ctx, int fd, uint64_t now_us)
{
    while (ctx->rx_len < sizeof(ctx->rx_buffer)) {
        ssize_t n = read(fd, ctx->rx_buffer + ctx->rx_len,
                         sizeof(ctx->rx_buffer) - ctx->rx_len);

        if (n > 0) {
            ctx->rx_len += (size_t)n;
            ctx->last_rx_us = now_us;
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
    if (fc == MODBUS_FUNC_WRITE_MULTIPLE_COILS || fc == MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS) {
        if (rx_len <= RTU_OFFSET_WRITE_BYTE_CNT) {
            return 0;
        }

        uint8_t byte_count = adu[RTU_OFFSET_WRITE_BYTE_CNT];
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

    /* FC01-FC06 (and most unsupported public FCs) use a fixed 8-byte RTU ADU. */
    if (adu_cap < MODBUS_RTU_MIN_REQ_LENGTH) {
        return RTU_IO_FRAME_ERROR;
    }
    if (rx_len < MODBUS_RTU_MIN_REQ_LENGTH) {
        return 0;
    }
    *total_len = MODBUS_RTU_MIN_REQ_LENGTH;
    return 1;
}

static void drop_rx_bytes(mb_rtu_server_ctx_t *ctx, size_t count)
{
    size_t remaining = ctx->rx_len - count;

    if (remaining > 0u) {
        memmove(ctx->rx_buffer, ctx->rx_buffer + count, remaining);
    }
    ctx->rx_len = remaining;
}

/**
 * Extract the next CRC-valid request frame from the RX buffer.
 * A candidate that is malformed or fails CRC (e.g. another slave's response
 * on a multi-drop bus) is skipped one byte at a time until a valid frame
 * boundary is found.
 *
 * @return 1 with @p adu filled, 0 if more bytes are needed.
 */
static int extract_request_frame(mb_rtu_server_ctx_t *ctx,
                                 uint8_t *adu, size_t adu_cap,
                                 size_t *total_len)
{
    size_t frame_len = 0u;
    size_t dropped = 0u;
    int rc;

    for (;;) {
        rc = get_complete_request_len(ctx->rx_buffer, ctx->rx_len,
                                      sizeof(ctx->rx_buffer), &frame_len);
        if (rc == 0) {
            break;
        }
        if (rc == 1 && frame_len <= adu_cap &&
            mb_rtu_validate_crc(ctx->rx_buffer, frame_len) == 0) {
            break;
        }
        drop_rx_bytes(ctx, 1u);
        dropped++;
    }

    if (dropped > 0u) {
        srv_logv(ctx, MB_RTU_LOG_DEBUG, "rtu resync: skipped %zu byte(s)", dropped);
    }
    if (rc == 0) {
        if (ctx->rx_len == 0u) {
            ctx->last_rx_us = 0u;
        }
        return 0;
    }

    memcpy(adu, ctx->rx_buffer, frame_len);
    drop_rx_bytes(ctx, frame_len);
    if (ctx->rx_len == 0u) {
        ctx->last_rx_us = 0u;
    }
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
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                fd_set writefds;
                FD_ZERO(&writefds);
                FD_SET(fd, &writefds);
                select(fd + 1, NULL, &writefds, NULL, NULL);
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

/** Send one response frame, toggling the optional TX-enable hook around it. */
static int send_frame(mb_rtu_server_ctx_t *ctx, int fd, const uint8_t *buf, size_t len)
{
    if (ctx->cfg.on_tx_enable) {
        ctx->cfg.on_tx_enable(ctx->cfg.tx_userdata, fd, 1);
    }
    int rc = write_all(fd, buf, len);
    if (ctx->cfg.on_tx_enable) {
        ctx->cfg.on_tx_enable(ctx->cfg.tx_userdata, fd, 0);
    }
    return rc;
}

/** Consume the transceiver's local echo of @p len transmitted bytes. */
static void discard_echo(mb_rtu_server_ctx_t *ctx, int fd, size_t len)
{
    uint8_t scratch[MODBUS_RTU_MAX_ADU_LENGTH];
    uint64_t deadline_us = current_time_us() + (uint64_t)effective_timeout_ms(ctx) * 1000u;
    size_t got = 0u;

    while (got < len) {
        uint64_t now_us = current_time_us();
        if (now_us >= deadline_us) {
            srv_logv(ctx, MB_RTU_LOG_WARN, "local echo incomplete: %zu of %zu bytes",
                     got, len);
            return;
        }

        uint64_t remaining_us = deadline_us - now_us;
        fd_set readfds;
        struct timeval tv = {
            .tv_sec = (time_t)(remaining_us / 1000000u),
            .tv_usec = (suseconds_t)(remaining_us % 1000000u),
        };
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        if (select(fd + 1, &readfds, NULL, NULL, &tv) <= 0) {
            continue;
        }

        ssize_t n = read(fd, scratch, len - got);
        if (n > 0) {
            got += (size_t)n;
        } else if (n == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) {
            return;
        }
    }
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

    uint64_t now_us = current_time_us();
    uint64_t timeout_us = (uint64_t)effective_timeout_ms(ctx) * 1000u;

    if (ctx->rx_len > 0u && ctx->last_rx_us > 0u &&
        now_us >= ctx->last_rx_us &&
        (now_us - ctx->last_rx_us) > timeout_us) {
        srv_logv(ctx, MB_RTU_LOG_DEBUG, "rtu frame assembly timeout");
        reset_rx_state(ctx);
    }

    int rc = read_available_bytes(ctx, serial_fd, now_us);
    if (rc < 0) {
        return -1;
    }

    /* Capture request-end time before extract clears last_rx_us. */
    uint64_t request_end_us = ctx->last_rx_us;

    if (extract_request_frame(ctx, adu, sizeof(adu), &total_len) == 0) {
        return 0;
    }

    uint8_t unit_id = adu[RTU_OFFSET_UNIT_ID];
    uint8_t fc = adu[RTU_OFFSET_FC];
    int is_broadcast = (unit_id == MODBUS_RTU_BROADCAST_ADDRESS);

    if (!unit_id_matches(ctx, unit_id)) {
        srv_logv(ctx, MB_RTU_LOG_DEBUG,
                 "unit_id mismatch: received %u, expected %u",
                 (unsigned)unit_id, (unsigned)ctx->cfg.unit_id);
        return 0;
    }

    if (is_broadcast && !mb_pdu_is_write(fc)) {
        return 0;
    }

    const mb_pdu_handlers_t handlers = {
        .read_registers = ctx->cfg.on_read,
        .write_registers = ctx->cfg.on_write,
        .read_bits = ctx->cfg.on_read_bits,
        .write_bits = ctx->cfg.on_write_bits,
        .userdata = ctx->cfg.userdata,
    };
    size_t pdu_len = total_len - RTU_OFFSET_PDU - MODBUS_RTU_CRC_LEN;
    size_t resp_pdu_len = mb_pdu_handle_request(&handlers, adu + RTU_OFFSET_PDU, pdu_len,
                                                resp + RTU_OFFSET_PDU);

    if (is_broadcast) {
        return 0;
    }

    resp[RTU_OFFSET_UNIT_ID] = unit_id;
    size_t resp_len = RTU_OFFSET_PDU + resp_pdu_len;
    if (mb_rtu_append_crc(resp, resp_len, sizeof(resp)) != 0) {
        return -1;
    }
    resp_len += MODBUS_RTU_CRC_LEN;

    wait_response_silence(request_end_us, effective_inter_frame_us(ctx));
    if (send_frame(ctx, serial_fd, resp, resp_len) != 0) {
        srv_logv(ctx, MB_RTU_LOG_ERROR, "send response failed");
        return -1;
    }
    if (ctx->cfg.local_echo) {
        discard_echo(ctx, serial_fd, resp_len);
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
    if (ctx->transport.initialized) {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    if (ctx->cfg.recv_timeout_ms == 0u) {
        ctx->cfg.recv_timeout_ms = MB_RTU_SERVER_DEFAULT_RECV_TIMEOUT_MS;
    }

    mb_rtu_serial_config_t serial = {
        .device = cfg->device,
        .baud_rate = cfg->baud_rate,
        .data_bits = cfg->data_bits,
        .stop_bits = cfg->stop_bits,
        .parity = cfg->parity,
        .rs485_mode = cfg->rs485_mode,
    };
    ctx->t35_us = mb_rtu_t35_us(&serial);

    mb_rtu_config_t transport_cfg = {
        .serial = serial,
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
