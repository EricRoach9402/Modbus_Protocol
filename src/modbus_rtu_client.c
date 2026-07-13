/**
 * @file modbus_rtu_client.c
 * @brief Modbus RTU client API - implementation.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "modbus_rtu_client.h"

#define DEFAULT_RESPONSE_TIMEOUT_MS  MODBUS_RTU_DEFAULT_TIMEOUT_MS

#define RTU_OFFSET_UNIT_ID        0u
#define RTU_OFFSET_FC             1u
#define RTU_OFFSET_ADDR_HIGH      2u
#define RTU_OFFSET_ADDR_LOW       3u
#define RTU_OFFSET_QTY_HIGH       4u
#define RTU_OFFSET_QTY_LOW        5u
#define RTU_READ_BYTE_COUNT_OFF   2u
#define RTU_READ_DATA_OFF         3u

#define RTU_EXCEPTION_LEN         5u
#define RTU_WRITE_RESPONSE_LEN    8u
#define RTU_IO_FRAME_ERROR       (-10)

static void cli_logv(mb_rtu_client_ctx_t *ctx, mb_rtu_log_level_t level,
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

static uint32_t effective_timeout_ms(const mb_rtu_client_ctx_t *ctx)
{
    return (ctx->cfg.response_timeout_ms != 0u)
           ? ctx->cfg.response_timeout_ms
           : DEFAULT_RESPONSE_TIMEOUT_MS;
}

static int wait_readable(int fd, uint32_t timeout_ms)
{
    fd_set readfds;
    struct timeval tv;

    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    tv.tv_sec = (time_t)(timeout_ms / 1000u);
    tv.tv_usec = (suseconds_t)((timeout_ms % 1000u) * 1000u);

    int rc = select(fd + 1, &readfds, NULL, NULL, &tv);
    if (rc == 0) {
        return -2;
    }
    if (rc < 0) {
        if (errno == EINTR) {
            return -2;
        }
        return -1;
    }
    return 0;
}

static int read_exact_timeout(int fd, uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    size_t received = 0u;

    while (received < len) {
        int ready = wait_readable(fd, timeout_ms);
        if (ready != 0) {
            return ready;
        }

        ssize_t n = read(fd, buf + received, len - received);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        if (n == 0) {
            return -2;
        }

        received += (size_t)n;
    }

    return 0;
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

static int io_err_to_client_err(int rc)
{
    if (rc == -2) {
        return MB_RTU_CLIENT_ERR_TIMEOUT;
    }
    if (rc == RTU_IO_FRAME_ERROR) {
        return MB_RTU_CLIENT_ERR_FRAME;
    }
    return MB_RTU_CLIENT_ERR_TRANSPORT;
}

static int recv_response(mb_rtu_client_ctx_t *ctx, uint8_t expected_fc,
                         uint8_t *buf, size_t buf_cap, size_t *total_len)
{
    if (buf_cap < RTU_EXCEPTION_LEN) {
        return RTU_IO_FRAME_ERROR;
    }

    uint32_t timeout_ms = effective_timeout_ms(ctx);
    int rc = read_exact_timeout(ctx->fd, buf, 2u, timeout_ms);
    if (rc != 0) {
        return rc;
    }

    if (buf[RTU_OFFSET_UNIT_ID] != ctx->cfg.unit_id) {
        return RTU_IO_FRAME_ERROR;
    }

    uint8_t resp_fc = buf[RTU_OFFSET_FC];
    if ((resp_fc & (uint8_t)MODBUS_EXCEPTION_FLAG) != 0u) {
        rc = read_exact_timeout(ctx->fd, buf + 2u, RTU_EXCEPTION_LEN - 2u, timeout_ms);
        if (rc != 0) {
            return rc;
        }
        *total_len = RTU_EXCEPTION_LEN;
        return (mb_rtu_validate_crc(buf, *total_len) == 0) ? 0 : RTU_IO_FRAME_ERROR;
    }

    if (resp_fc != expected_fc) {
        return RTU_IO_FRAME_ERROR;
    }

    if (expected_fc == (uint8_t)MODBUS_FUNC_READ_HOLDING_REGISTERS ||
        expected_fc == (uint8_t)MODBUS_FUNC_READ_INPUT_REGISTERS) {
        rc = read_exact_timeout(ctx->fd, buf + 2u, 1u, timeout_ms);
        if (rc != 0) {
            return rc;
        }

        uint8_t byte_count = buf[RTU_READ_BYTE_COUNT_OFF];
        size_t frame_len = 3u + (size_t)byte_count + MODBUS_RTU_CRC_LEN;
        if (byte_count == 0u || frame_len > buf_cap) {
            return RTU_IO_FRAME_ERROR;
        }

        rc = read_exact_timeout(ctx->fd, buf + 3u,
                                (size_t)byte_count + MODBUS_RTU_CRC_LEN,
                                timeout_ms);
        if (rc != 0) {
            return rc;
        }
        *total_len = frame_len;
        return (mb_rtu_validate_crc(buf, *total_len) == 0) ? 0 : RTU_IO_FRAME_ERROR;
    }

    if (buf_cap < RTU_WRITE_RESPONSE_LEN) {
        return RTU_IO_FRAME_ERROR;
    }

    rc = read_exact_timeout(ctx->fd, buf + 2u, RTU_WRITE_RESPONSE_LEN - 2u, timeout_ms);
    if (rc != 0) {
        return rc;
    }
    *total_len = RTU_WRITE_RESPONSE_LEN;
    return (mb_rtu_validate_crc(buf, *total_len) == 0) ? 0 : RTU_IO_FRAME_ERROR;
}

static int validate_write_response(mb_rtu_client_ctx_t *ctx,
                                   const uint8_t *req, const uint8_t *resp,
                                   size_t total_len)
{
    if (total_len < RTU_EXCEPTION_LEN) {
        return MB_RTU_CLIENT_ERR_FRAME;
    }

    if (resp[RTU_OFFSET_UNIT_ID] != ctx->cfg.unit_id) {
        return MB_RTU_CLIENT_ERR_UNIT_ID;
    }

    uint8_t resp_fc = resp[RTU_OFFSET_FC];
    if ((resp_fc & (uint8_t)MODBUS_EXCEPTION_FLAG) != 0u) {
        cli_logv(ctx, MB_RTU_LOG_WARN,
                 "FC%02X exception code %u",
                 (unsigned)req[RTU_OFFSET_FC], (unsigned)resp[2]);
        return (int)resp[2];
    }

    if (total_len != RTU_WRITE_RESPONSE_LEN || resp_fc != req[RTU_OFFSET_FC]) {
        return MB_RTU_CLIENT_ERR_FRAME;
    }

    if (memcmp(resp + RTU_OFFSET_ADDR_HIGH, req + RTU_OFFSET_ADDR_HIGH, 4u) != 0) {
        return MB_RTU_CLIENT_ERR_FRAME;
    }

    return MB_RTU_CLIENT_OK;
}

int mb_rtu_client_connect(mb_rtu_client_ctx_t *ctx, const mb_rtu_client_config_t *cfg)
{
    if (!ctx || !cfg || !cfg->device || cfg->device[0] == '\0') {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->fd = -1;

    if (pthread_mutex_init(&ctx->lock, NULL) != 0) {
        return -1;
    }

    mb_rtu_serial_config_t serial = {
        .device = cfg->device,
        .baud_rate = cfg->baud_rate,
        .data_bits = cfg->data_bits,
        .stop_bits = cfg->stop_bits,
        .parity = cfg->parity,
    };

    if (mb_rtu_open(&ctx->fd, &serial) != 0) {
        pthread_mutex_destroy(&ctx->lock);
        ctx->fd = -1;
        return -1;
    }

    cli_logv(ctx, MB_RTU_LOG_DEBUG, "connected to %s", cfg->device);
    return 0;
}

void mb_rtu_client_disconnect(mb_rtu_client_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    mb_rtu_close(&ctx->fd);
    pthread_mutex_destroy(&ctx->lock);
}

int mb_rtu_client_read_holding_registers(mb_rtu_client_ctx_t *ctx,
                                         uint16_t addr, uint16_t qty,
                                         uint16_t *out)
{
    if (!ctx || !out || qty == 0u || qty > MODBUS_MAX_READ_REGISTERS ||
        ctx->cfg.unit_id == MODBUS_RTU_BROADCAST_ADDRESS) {
        return MB_RTU_CLIENT_ERR_ARG;
    }
    if (ctx->fd < 0) {
        return MB_RTU_CLIENT_ERR_NOT_CONNECTED;
    }

    uint8_t req[MODBUS_RTU_MIN_REQ_LENGTH];
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];

    int req_len = mb_rtu_build_request_basis(ctx->cfg.unit_id,
                                             MODBUS_FUNC_READ_HOLDING_REGISTERS,
                                             addr, qty, req, sizeof(req));
    if (req_len < 0) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    pthread_mutex_lock(&ctx->lock);

    tcflush(ctx->fd, TCIOFLUSH);
    if (write_all(ctx->fd, req, (size_t)req_len) != 0) {
        cli_logv(ctx, MB_RTU_LOG_ERROR, "FC03 send failed");
        pthread_mutex_unlock(&ctx->lock);
        return MB_RTU_CLIENT_ERR_TRANSPORT;
    }

    size_t total_len = 0u;
    int rc = recv_response(ctx, MODBUS_FUNC_READ_HOLDING_REGISTERS,
                           resp, sizeof(resp), &total_len);
    pthread_mutex_unlock(&ctx->lock);

    if (rc != 0) {
        cli_logv(ctx, MB_RTU_LOG_ERROR, "FC03 recv failed (rc=%d)", rc);
        return io_err_to_client_err(rc);
    }

    uint8_t resp_fc = resp[RTU_OFFSET_FC];
    if ((resp_fc & (uint8_t)MODBUS_EXCEPTION_FLAG) != 0u) {
        return (int)resp[2];
    }

    if (total_len < (5u + (size_t)qty * 2u) ||
        resp[RTU_READ_BYTE_COUNT_OFF] != (uint8_t)(qty * 2u)) {
        return MB_RTU_CLIENT_ERR_FRAME;
    }

    for (uint16_t i = 0u; i < qty; i++) {
        size_t offset = RTU_READ_DATA_OFF + (size_t)(i * 2u);
        out[i] = ((uint16_t)resp[offset] << 8) | (uint16_t)resp[offset + 1u];
    }

    return MB_RTU_CLIENT_OK;
}

int mb_rtu_client_write_single_register(mb_rtu_client_ctx_t *ctx,
                                        uint16_t addr, uint16_t value)
{
    if (!ctx) {
        return MB_RTU_CLIENT_ERR_ARG;
    }
    if (ctx->fd < 0) {
        return MB_RTU_CLIENT_ERR_NOT_CONNECTED;
    }

    uint8_t req[MODBUS_RTU_MIN_REQ_LENGTH];
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];
    int req_len = mb_rtu_build_fc06_request(ctx->cfg.unit_id, addr, value,
                                            req, sizeof(req));
    if (req_len < 0) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    pthread_mutex_lock(&ctx->lock);

    tcflush(ctx->fd, TCIOFLUSH);
    if (write_all(ctx->fd, req, (size_t)req_len) != 0) {
        cli_logv(ctx, MB_RTU_LOG_ERROR, "FC06 send failed");
        pthread_mutex_unlock(&ctx->lock);
        return MB_RTU_CLIENT_ERR_TRANSPORT;
    }

    if (ctx->cfg.unit_id == MODBUS_RTU_BROADCAST_ADDRESS) {
        pthread_mutex_unlock(&ctx->lock);
        return MB_RTU_CLIENT_OK;
    }

    size_t total_len = 0u;
    int rc = recv_response(ctx, MODBUS_FUNC_WRITE_SINGLE_REGISTER,
                           resp, sizeof(resp), &total_len);
    pthread_mutex_unlock(&ctx->lock);

    if (rc != 0) {
        cli_logv(ctx, MB_RTU_LOG_ERROR, "FC06 recv failed (rc=%d)", rc);
        return io_err_to_client_err(rc);
    }

    return validate_write_response(ctx, req, resp, total_len);
}

int mb_rtu_client_probe_device(mb_rtu_client_ctx_t *ctx,
                               uint16_t addr, uint16_t qty)
{
    uint16_t probe_buf[MODBUS_MAX_READ_REGISTERS];

    if (qty == 0u || qty > MODBUS_MAX_READ_REGISTERS) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    return mb_rtu_client_read_holding_registers(ctx, addr, qty, probe_buf);
}

int mb_rtu_client_write_multiple_registers(mb_rtu_client_ctx_t *ctx,
                                           uint16_t addr, uint16_t qty,
                                           const uint16_t *data)
{
    if (!ctx || !data || qty == 0u || qty > MODBUS_MAX_WRITE_REGISTERS) {
        return MB_RTU_CLIENT_ERR_ARG;
    }
    if (ctx->fd < 0) {
        return MB_RTU_CLIENT_ERR_NOT_CONNECTED;
    }

    uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH];
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];
    int req_len = mb_rtu_build_fc16_request(ctx->cfg.unit_id, addr, qty, data,
                                            req, sizeof(req));
    if (req_len < 0) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    pthread_mutex_lock(&ctx->lock);

    tcflush(ctx->fd, TCIOFLUSH);
    if (write_all(ctx->fd, req, (size_t)req_len) != 0) {
        cli_logv(ctx, MB_RTU_LOG_ERROR, "FC16 send failed");
        pthread_mutex_unlock(&ctx->lock);
        return MB_RTU_CLIENT_ERR_TRANSPORT;
    }

    if (ctx->cfg.unit_id == MODBUS_RTU_BROADCAST_ADDRESS) {
        pthread_mutex_unlock(&ctx->lock);
        return MB_RTU_CLIENT_OK;
    }

    size_t total_len = 0u;
    int rc = recv_response(ctx, MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS,
                           resp, sizeof(resp), &total_len);
    pthread_mutex_unlock(&ctx->lock);

    if (rc != 0) {
        cli_logv(ctx, MB_RTU_LOG_ERROR, "FC16 recv failed (rc=%d)", rc);
        return io_err_to_client_err(rc);
    }

    return validate_write_response(ctx, req, resp, total_len);
}
