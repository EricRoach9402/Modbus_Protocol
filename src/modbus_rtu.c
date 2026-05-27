/**
 * @file modbus_rtu.c
 * @brief Standalone Modbus RTU serial transport and framing helpers.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "modbus_rtu.h"

#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif

#define RTU_OFFSET_UNIT_ID        0u
#define RTU_OFFSET_FC             1u
#define RTU_OFFSET_ADDR_HIGH      2u
#define RTU_OFFSET_ADDR_LOW       3u
#define RTU_OFFSET_QTY_HIGH       4u
#define RTU_OFFSET_QTY_LOW        5u
#define RTU_OFFSET_FC16_BYTE_CNT  6u
#define RTU_OFFSET_FC16_DATA      7u

#define RTU_STANDARD_PDU_LEN      5u
#define RTU_READ_RESPONSE_BASE    5u
#define RTU_PROCESS_PAUSE_NS      1000000L

static void mb_rtu_logv(mb_rtu_ctx_t *ctx, mb_rtu_log_level_t level, const char *fmt, ...)
{
    if (!ctx || !ctx->cfg.logv) {
        return;
    }

    va_list ap;
    va_start(ap, fmt);
    ctx->cfg.logv(ctx->cfg.log_userdata, level, fmt, ap);
    va_end(ap);
}

static void mb_rtu_link_call(mb_rtu_ctx_t *ctx, int fd, int connected)
{
    if (ctx && ctx->cfg.on_link) {
        ctx->cfg.on_link(ctx->cfg.link_userdata, fd, connected);
    }
}

static void pause_process_loop(void)
{
    struct timespec pause_time = {
        .tv_sec = 0,
        .tv_nsec = RTU_PROCESS_PAUSE_NS,
    };

    nanosleep(&pause_time, NULL);
}

static speed_t baud_to_speed(uint32_t baud_rate)
{
    switch (baud_rate) {
        case 0u:
        case 9600u: return B9600;
        case 1200u: return B1200;
        case 2400u: return B2400;
        case 4800u: return B4800;
        case 19200u: return B19200;
        case 38400u: return B38400;
        case 57600u: return B57600;
        case 115200u: return B115200;
#ifdef B230400
        case 230400u: return B230400;
#endif
#ifdef B460800
        case 460800u: return B460800;
#endif
        default: return (speed_t)0;
    }
}

static uint8_t effective_data_bits(uint8_t data_bits)
{
    return (data_bits == 0u) ? 8u : data_bits;
}

static uint8_t effective_stop_bits(uint8_t stop_bits)
{
    return (stop_bits == 0u) ? 1u : stop_bits;
}

static int configure_serial(int fd, const mb_rtu_serial_config_t *serial)
{
    struct termios options;
    speed_t speed = baud_to_speed(serial->baud_rate);
    uint8_t data_bits = effective_data_bits(serial->data_bits);
    uint8_t stop_bits = effective_stop_bits(serial->stop_bits);

    if (speed == (speed_t)0 || (data_bits != 7u && data_bits != 8u) ||
        (stop_bits != 1u && stop_bits != 2u)) {
        errno = EINVAL;
        return -1;
    }

    memset(&options, 0, sizeof(options));
    options.c_cflag = CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (data_bits == 7u) {
        options.c_cflag |= CS7;
    } else {
        options.c_cflag |= CS8;
    }

    if (serial->parity == MB_RTU_PARITY_EVEN) {
        options.c_cflag |= PARENB;
    } else if (serial->parity == MB_RTU_PARITY_ODD) {
        options.c_cflag |= PARENB | PARODD;
    } else if (serial->parity != MB_RTU_PARITY_NONE) {
        errno = EINVAL;
        return -1;
    }

    if (stop_bits == 2u) {
        options.c_cflag |= CSTOPB;
    }

    if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0) {
        return -1;
    }

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return 0;
}

static void *mb_rtu_thread(void *arg)
{
    mb_rtu_ctx_t *ctx = (mb_rtu_ctx_t *)arg;

    while (ctx->keep_running) {
        int fd = ctx->serial_fd;
        if (fd < 0) {
            break;
        }

        if (ctx->cfg.on_process) {
            int rc = ctx->cfg.on_process(ctx->cfg.userdata, fd);
            if (rc < 0 && ctx->keep_running) {
                mb_rtu_logv(ctx, MB_RTU_LOG_ERROR, "on_process failed");
                if (ctx->cfg.on_error) {
                    ctx->cfg.on_error(ctx->cfg.userdata, rc);
                }
            }
            pause_process_loop();
        } else {
            struct timespec pause_time = {
                .tv_sec = 0,
                .tv_nsec = 100000000L,
            };
            nanosleep(&pause_time, NULL);
        }
    }

    mb_rtu_logv(ctx, MB_RTU_LOG_DEBUG, "rtu thread exit");
    return NULL;
}

int mb_rtu_open(int *fd_out, const mb_rtu_serial_config_t *serial)
{
    if (!fd_out || !serial || !serial->device || serial->device[0] == '\0') {
        return -1;
    }

    int fd = open(serial->device, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) {
        return -1;
    }

    if (configure_serial(fd, serial) != 0) {
        close(fd);
        return -1;
    }

    *fd_out = fd;
    return 0;
}

void mb_rtu_close(int *fd)
{
    if (fd && *fd >= 0) {
        close(*fd);
        *fd = -1;
    }
}

int mb_rtu_start(mb_rtu_ctx_t *ctx, const mb_rtu_config_t *cfg)
{
    if (!ctx || !cfg || !cfg->serial.device || cfg->serial.device[0] == '\0') {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->serial_fd = -1;
    ctx->cfg = *cfg;
    if (ctx->cfg.recv_timeout_ms == 0u) {
        ctx->cfg.recv_timeout_ms = MODBUS_RTU_DEFAULT_TIMEOUT_MS;
    }

    if (pthread_mutex_init(&ctx->lock, NULL) != 0) {
        return -1;
    }

    if (mb_rtu_open(&ctx->serial_fd, &ctx->cfg.serial) != 0) {
        pthread_mutex_destroy(&ctx->lock);
        return -1;
    }

    /*
     * Set O_NONBLOCK on the server-thread fd so that read_available_bytes()
     * gets proper EAGAIN semantics on every POSIX-compliant platform.
     * VMIN=0 / VTIME=0 is not a guaranteed substitute for O_NONBLOCK on all
     * embedded BSPs.  The client path (mb_rtu_client_connect) calls
     * mb_rtu_open() directly and keeps the fd in blocking mode because its
     * read_exact_timeout() is select-gated before every read().
     */
    int flags = fcntl(ctx->serial_fd, F_GETFL, 0);
    if (flags < 0 || fcntl(ctx->serial_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        mb_rtu_close(&ctx->serial_fd);
        pthread_mutex_destroy(&ctx->lock);
        return -1;
    }

    ctx->keep_running = 1;

    if (ctx->cfg.on_init) {
        ctx->cfg.on_init(ctx->cfg.userdata);
    }

    if (pthread_create(&ctx->thread, NULL, mb_rtu_thread, ctx) != 0) {
        mb_rtu_close(&ctx->serial_fd);
        pthread_mutex_destroy(&ctx->lock);
        ctx->keep_running = 0;
        return -1;
    }

    mb_rtu_link_call(ctx, ctx->serial_fd, 1);
    return 0;
}

void mb_rtu_stop(mb_rtu_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    int was_running = ctx->keep_running;
    ctx->keep_running = 0;

    if (ctx->serial_fd >= 0) {
        mb_rtu_link_call(ctx, ctx->serial_fd, 0);

        /*
         * Set serial_fd to -1 before close() so that new loop iterations in
         * mb_rtu_thread see -1 and exit without calling on_process.  There is
         * still a narrow window where the thread has already read the old fd
         * value into a local variable and is about to call on_process; in that
         * case the subsequent read() on the closed fd returns EBADF (-1), which
         * propagates as a transport error and the thread exits cleanly because
         * keep_running is already 0.  This is an accepted design trade-off:
         * avoiding it would require a mutex around every fd access in the loop,
         * which adds overhead not justified for a clean-shutdown path.
         */
        int saved_fd = ctx->serial_fd;
        ctx->serial_fd = -1;
        close(saved_fd);
    }

    if (was_running) {
        pthread_join(ctx->thread, NULL);
    }

    pthread_mutex_destroy(&ctx->lock);
}

uint16_t mb_rtu_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;

    if (!data && length > 0u) {
        return 0u;
    }

    for (size_t i = 0u; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0u; bit < 8u; bit++) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

int mb_rtu_append_crc(uint8_t *frame, size_t frame_len_without_crc, size_t frame_cap)
{
    if (!frame || frame_cap < frame_len_without_crc + MODBUS_RTU_CRC_LEN) {
        return -1;
    }

    uint16_t crc = mb_rtu_crc16(frame, frame_len_without_crc);
    frame[frame_len_without_crc] = (uint8_t)(crc & 0xFFu);
    frame[frame_len_without_crc + 1u] = (uint8_t)(crc >> 8);
    return 0;
}

int mb_rtu_validate_crc(const uint8_t *frame, size_t frame_len)
{
    if (!frame || frame_len < MODBUS_RTU_CRC_LEN) {
        return -1;
    }

    size_t payload_len = frame_len - MODBUS_RTU_CRC_LEN;
    uint16_t expected = mb_rtu_crc16(frame, payload_len);
    uint16_t received = (uint16_t)frame[payload_len]
                      | ((uint16_t)frame[payload_len + 1u] << 8);

    return (expected == received) ? 0 : -1;
}

int mb_rtu_build_request_basis(uint8_t unit_id, int function, int addr,
                               uint16_t nb, uint8_t *req, size_t req_cap)
{
    if (!req || req_cap < MODBUS_RTU_MIN_REQ_LENGTH) {
        return -1;
    }
    if (function < 0 || function > 255 || addr < 0 || addr > 65535) {
        return -1;
    }

    req[RTU_OFFSET_UNIT_ID]   = unit_id;
    req[RTU_OFFSET_FC]        = (uint8_t)function;
    req[RTU_OFFSET_ADDR_HIGH] = (uint8_t)((unsigned)addr >> 8);
    req[RTU_OFFSET_ADDR_LOW]  = (uint8_t)((unsigned)addr & 0xFFu);
    req[RTU_OFFSET_QTY_HIGH]  = (uint8_t)(nb >> 8);
    req[RTU_OFFSET_QTY_LOW]   = (uint8_t)(nb & 0xFFu);

    if (mb_rtu_append_crc(req, 1u + RTU_STANDARD_PDU_LEN, req_cap) != 0) {
        return -1;
    }

    return (int)MODBUS_RTU_MIN_REQ_LENGTH;
}

int mb_rtu_build_fc06_request(uint8_t unit_id, uint16_t addr, uint16_t value,
                              uint8_t *req, size_t req_cap)
{
    return mb_rtu_build_request_basis(unit_id, MODBUS_FUNC_WRITE_SINGLE_REGISTER,
                                      addr, value, req, req_cap);
}

int mb_rtu_build_fc16_request(uint8_t unit_id, uint16_t addr, uint16_t num_registers,
                              const uint16_t *values, uint8_t *req, size_t req_cap)
{
    if (!req || !values || num_registers == 0u ||
        num_registers > MODBUS_MAX_WRITE_REGISTERS) {
        return -1;
    }

    uint8_t byte_count = (uint8_t)(num_registers * 2u);
    size_t total_len = 9u + (size_t)byte_count;
    if (req_cap < total_len || total_len > MODBUS_RTU_MAX_ADU_LENGTH) {
        return -1;
    }

    req[RTU_OFFSET_UNIT_ID]       = unit_id;
    req[RTU_OFFSET_FC]            = (uint8_t)MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS;
    req[RTU_OFFSET_ADDR_HIGH]     = (uint8_t)(addr >> 8);
    req[RTU_OFFSET_ADDR_LOW]      = (uint8_t)(addr & 0xFFu);
    req[RTU_OFFSET_QTY_HIGH]      = (uint8_t)(num_registers >> 8);
    req[RTU_OFFSET_QTY_LOW]       = (uint8_t)(num_registers & 0xFFu);
    req[RTU_OFFSET_FC16_BYTE_CNT] = byte_count;

    for (uint16_t i = 0u; i < num_registers; i++) {
        size_t offset = RTU_OFFSET_FC16_DATA + (size_t)(i * 2u);
        req[offset] = (uint8_t)(values[i] >> 8);
        req[offset + 1u] = (uint8_t)(values[i] & 0xFFu);
    }

    if (mb_rtu_append_crc(req, total_len - MODBUS_RTU_CRC_LEN, req_cap) != 0) {
        return -1;
    }

    return (int)total_len;
}

int mb_rtu_parse_request(const uint8_t *request, size_t length, uint8_t *function_code,
                         uint16_t *address, uint16_t *quantity)
{
    if (!request || !function_code || !address || !quantity ||
        length < MODBUS_RTU_MIN_REQ_LENGTH) {
        return -1;
    }

    if (mb_rtu_validate_crc(request, length) != 0) {
        return -1;
    }

    *function_code = request[RTU_OFFSET_FC];
    *address = ((uint16_t)request[RTU_OFFSET_ADDR_HIGH] << 8)
             | (uint16_t)request[RTU_OFFSET_ADDR_LOW];

    if (*function_code == MODBUS_FUNC_WRITE_SINGLE_REGISTER) {
        *quantity = 1u;
    } else {
        *quantity = ((uint16_t)request[RTU_OFFSET_QTY_HIGH] << 8)
                  | (uint16_t)request[RTU_OFFSET_QTY_LOW];
    }

    return 0;
}

int mb_rtu_build_read_registers_response(uint8_t function_code, uint8_t unit_id,
                                         const uint16_t *data, uint16_t quantity,
                                         uint8_t *response, size_t response_cap)
{
    if (!response || !data || quantity > MODBUS_MAX_READ_REGISTERS) {
        return -1;
    }

    size_t total_len = RTU_READ_RESPONSE_BASE + (size_t)quantity * 2u;
    if (response_cap < total_len || total_len > MODBUS_RTU_MAX_ADU_LENGTH) {
        return -1;
    }

    response[RTU_OFFSET_UNIT_ID] = unit_id;
    response[RTU_OFFSET_FC] = function_code;
    response[2] = (uint8_t)(quantity * 2u);

    for (uint16_t i = 0u; i < quantity; i++) {
        size_t offset = 3u + (size_t)(i * 2u);
        response[offset] = (uint8_t)(data[i] >> 8);
        response[offset + 1u] = (uint8_t)(data[i] & 0xFFu);
    }

    if (mb_rtu_append_crc(response, total_len - MODBUS_RTU_CRC_LEN, response_cap) != 0) {
        return -1;
    }

    return (int)total_len;
}
