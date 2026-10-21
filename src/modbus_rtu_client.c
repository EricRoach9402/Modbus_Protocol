/**
 * @file modbus_rtu_client.c
 * @brief Modbus RTU client API - implementation.
 *
 * Contexts that share the same device share one open fd and one bus mutex
 * (see mb_rtu_shared_bus).  PDUs are built and validated by modbus_pdu;
 * this file adds RTU framing (slave address + CRC), bus timing and DE control.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "modbus_pdu.h"
#include "modbus_rtu_client.h"

#define DEFAULT_RESPONSE_TIMEOUT_MS MODBUS_RTU_DEFAULT_TIMEOUT_MS

#define RTU_OFFSET_UNIT_ID 0u
#define RTU_OFFSET_FC 1u
#define RTU_OFFSET_PDU 1u
#define RTU_READ_BYTE_COUNT_OFF 2u

#define RTU_EXCEPTION_LEN 5u
#define RTU_WRITE_RESPONSE_LEN 8u
#define RTU_IO_FRAME_ERROR (-10)
#define RTU_IO_CRC_ERROR (-11)
#define RTU_IO_UNIT_ID_ERROR (-12)

/* -- Shared serial-bus registry ------------------------------------------ */

typedef struct mb_rtu_shared_bus {
    struct mb_rtu_shared_bus *next;
    char path[PATH_MAX];
    pthread_mutex_t mutex;
    int fd;
    unsigned int refcnt;
    uint32_t baud_rate;
    uint8_t data_bits;
    uint8_t stop_bits;
    mb_rtu_parity_t parity;
    mb_rtu_rs485_mode_t rs485_mode;
    uint32_t t35_us;
    bool has_last_transaction;
    uint8_t last_unit_id;
    struct timespec last_release_time;
} mb_rtu_shared_bus_t;

/*
 * Lock order: shared_buses_registry_lock, then bus->mutex.  The registry lock
 * covers list lookup, insertion, refcnt changes and removal.
 */
static mb_rtu_shared_bus_t *shared_buses;
static pthread_mutex_t shared_buses_registry_lock = PTHREAD_MUTEX_INITIALIZER;

static uint32_t effective_baud_rate(uint32_t baud_rate)
{
    return (baud_rate == 0u) ? 9600u : baud_rate;
}

static uint8_t effective_data_bits(uint8_t data_bits)
{
    return (data_bits == 0u) ? 8u : data_bits;
}

static uint8_t effective_stop_bits(uint8_t stop_bits)
{
    return (stop_bits == 0u) ? 1u : stop_bits;
}

static uint32_t effective_handoff_silence_ms(const mb_rtu_client_ctx_t *ctx)
{
    return mb_rtu_effective_timing(ctx->cfg.bus_handoff_silence_ms,
        MB_RTU_CLIENT_DEFAULT_HANDOFF_SILENCE_MS);
}

/** Caller must hold the shared bus mutex (t35_us is set at port open). */
static uint32_t effective_inter_frame_us(const mb_rtu_client_ctx_t *ctx)
{
    return mb_rtu_effective_timing(ctx->cfg.inter_frame_delay_us,
        ctx->shared_bus->t35_us);
}

/** Canonical registry key: realpath() when resolvable, else the path verbatim. */
static int resolve_bus_key(const char *device, char *key)
{
    if (realpath(device, key) != NULL) {
        return 0;
    }
    if (strlen(device) >= PATH_MAX) {
        return -1;
    }
    strcpy(key, device);
    return 0;
}

/** Caller must hold the registry lock. */
static mb_rtu_shared_bus_t *find_bus_locked(const char *key)
{
    for (mb_rtu_shared_bus_t *bus = shared_buses; bus; bus = bus->next) {
        if (strcmp(bus->path, key) == 0) {
            return bus;
        }
    }
    return NULL;
}

static bool bus_settings_match(const mb_rtu_shared_bus_t *bus,
    const mb_rtu_serial_config_t *serial)
{
    return bus->baud_rate == effective_baud_rate(serial->baud_rate) &&
           bus->data_bits == effective_data_bits(serial->data_bits) &&
           bus->stop_bits == effective_stop_bits(serial->stop_bits) &&
           bus->parity == serial->parity &&
           bus->rs485_mode == serial->rs485_mode;
}

/** Open the port and link a new bus record.  Caller must hold the registry lock. */
static mb_rtu_shared_bus_t *open_bus_locked(const char *key,
    const mb_rtu_serial_config_t *serial)
{
    mb_rtu_shared_bus_t *bus = calloc(1u, sizeof(*bus));
    int fd = -1;

    if (!bus) {
        return NULL;
    }
    if (mb_rtu_open(&fd, serial) != 0) {
        free(bus);
        return NULL;
    }
    if (pthread_mutex_init(&bus->mutex, NULL) != 0) {
        mb_rtu_close(&fd);
        free(bus);
        return NULL;
    }

    strcpy(bus->path, key);
    bus->fd = fd;
    bus->baud_rate = effective_baud_rate(serial->baud_rate);
    bus->data_bits = effective_data_bits(serial->data_bits);
    bus->stop_bits = effective_stop_bits(serial->stop_bits);
    bus->parity = serial->parity;
    bus->rs485_mode = serial->rs485_mode;
    bus->t35_us = mb_rtu_t35_us(serial);
    bus->next = shared_buses;
    shared_buses = bus;
    return bus;
}

/**
 * Attach to the shared bus for @p serial->device, opening the port on first
 * use.  Later attaches reuse the fd and must match serial settings.
 */
static mb_rtu_shared_bus_t *shared_bus_attach(const mb_rtu_serial_config_t *serial)
{
    char key[PATH_MAX];
    mb_rtu_shared_bus_t *bus;

    if (resolve_bus_key(serial->device, key) != 0) {
        return NULL;
    }

    pthread_mutex_lock(&shared_buses_registry_lock);

    bus = find_bus_locked(key);
    if (bus) {
        if (!bus_settings_match(bus, serial)) {
            bus = NULL;
        }
    } else {
        bus = open_bus_locked(key, serial);
    }

    if (bus) {
        bus->refcnt++;
    }

    pthread_mutex_unlock(&shared_buses_registry_lock);
    return bus;
}

/** Drop one reference; close the port and free the record on the last detach. */
static void shared_bus_detach(mb_rtu_shared_bus_t *bus)
{
    pthread_mutex_lock(&shared_buses_registry_lock);

    if (bus->refcnt > 0u) {
        bus->refcnt--;
    }
    if (bus->refcnt == 0u) {
        for (mb_rtu_shared_bus_t **link = &shared_buses; *link; link = &(*link)->next) {
            if (*link == bus) {
                *link = bus->next;
                break;
            }
        }
        mb_rtu_close(&bus->fd);
        pthread_mutex_destroy(&bus->mutex);
        free(bus);
    }

    pthread_mutex_unlock(&shared_buses_registry_lock);
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
 * Wait until the bus has been silent for T3.5, or for the handoff silence
 * when the previous unit_id differed, whichever is longer.
 * Caller must hold bus->mutex.
 */
static void wait_bus_silence(mb_rtu_shared_bus_t *bus, uint8_t unit_id,
    uint32_t inter_frame_us, uint32_t handoff_ms)
{
    struct timespec now;
    uint64_t required_us = inter_frame_us;
    int64_t elapsed_us;

    if (!bus->has_last_transaction) {
        return;
    }

    if (bus->last_unit_id != unit_id &&
        (uint64_t)handoff_ms * 1000u > required_us) {
        required_us = (uint64_t)handoff_ms * 1000u;
    }
    if (required_us == 0u) {
        return;
    }

    clock_gettime(CLOCK_MONOTONIC, &now);
    elapsed_us = (int64_t)(now.tv_sec - bus->last_release_time.tv_sec) * 1000000 +
                 (now.tv_nsec - bus->last_release_time.tv_nsec) / 1000;

    if (elapsed_us >= 0 && (uint64_t)elapsed_us < required_us) {
        sleep_us(required_us - (uint64_t)elapsed_us);
    }
}

static void mark_bus_transaction_done(mb_rtu_shared_bus_t *bus, uint8_t unit_id)
{
    clock_gettime(CLOCK_MONOTONIC, &bus->last_release_time);
    bus->last_unit_id = unit_id;
    bus->has_last_transaction = true;
}

/* -- Logging / I/O helpers ----------------------------------------------- */

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

static void format_hex_string(const uint8_t *buf, size_t len,
    char *out, size_t out_cap)
{
    size_t pos = 0u;

    if (!out || out_cap == 0u) {
        return;
    }

    out[0] = '\0';
    for (size_t i = 0u; i < len && (pos + 3u) < out_cap; i++) {
        int n = snprintf(out + pos, out_cap - pos, "%02X%s",
            buf[i], (i + 1u < len) ? " " : "");
        if (n < 0) {
            break;
        }
        pos += (size_t)n;
    }
}

static void log_transaction_failure(mb_rtu_client_ctx_t *ctx, int err, uint8_t unit_id,
    const uint8_t *req, size_t req_len,
    const uint8_t *resp, size_t resp_len)
{
    char tx_hex[MODBUS_RTU_MAX_ADU_LENGTH * 3u];
    char rx_hex[MODBUS_RTU_MAX_ADU_LENGTH * 3u];

    format_hex_string(req, req_len, tx_hex, sizeof(tx_hex));
    format_hex_string(resp, resp_len, rx_hex, sizeof(rx_hex));
    cli_logv(ctx, MB_RTU_LOG_WARN,
        "uid=%u transaction failed (err=%d): tx[%zu]={%s} rx[%zu]={%s}",
        (unsigned)unit_id, err, req_len, tx_hex, resp_len, rx_hex);
}

static uint64_t monotonic_us(void)
{
    struct timespec now;

    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)now.tv_sec * 1000000u + (uint64_t)now.tv_nsec / 1000u;
}

/** Wire time of @p nbytes characters with the bus's framing settings. */
static uint64_t frame_wire_time_us(const mb_rtu_shared_bus_t *bus, size_t nbytes)
{
    uint64_t char_bits = 1u + bus->data_bits + bus->stop_bits +
                         ((bus->parity != MB_RTU_PARITY_NONE) ? 1u : 0u);

    return (char_bits * nbytes * 1000000u + bus->baud_rate - 1u) / bus->baud_rate;
}

/** Absolute CLOCK_MONOTONIC deadline (µs) for receiving one full response. */
static uint64_t response_deadline_us(const mb_rtu_client_ctx_t *ctx)
{
    uint32_t timeout_ms = (ctx->cfg.response_timeout_ms != 0u)
                          ? ctx->cfg.response_timeout_ms
                          : DEFAULT_RESPONSE_TIMEOUT_MS;

    return monotonic_us() + (uint64_t)timeout_ms * 1000u +
           frame_wire_time_us(ctx->shared_bus, MODBUS_RTU_MAX_ADU_LENGTH);
}

static int wait_readable(int fd, uint64_t deadline_us)
{
    for (;;) {
        fd_set readfds;
        struct timeval tv;
        uint64_t now_us = monotonic_us();

        if (now_us >= deadline_us) {
            return -2;
        }
        uint64_t remaining_us = deadline_us - now_us;

        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        tv.tv_sec = (time_t)(remaining_us / 1000000u);
        tv.tv_usec = (suseconds_t)(remaining_us % 1000000u);

        int rc = select(fd + 1, &readfds, NULL, NULL, &tv);
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

static int read_exact_timeout(int fd, uint8_t *buf, size_t len, uint64_t deadline_us)
{
    size_t received = 0u;

    while (received < len) {
        int ready = wait_readable(fd, deadline_us);
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

/** Send one request frame, toggling the optional TX-enable hook around it. */
static int send_frame(mb_rtu_client_ctx_t *ctx, const uint8_t *buf, size_t len)
{
    if (ctx->cfg.on_tx_enable) {
        ctx->cfg.on_tx_enable(ctx->cfg.tx_userdata, ctx->fd, 1);
    }
    int rc = write_all(ctx->fd, buf, len);
    if (ctx->cfg.on_tx_enable) {
        ctx->cfg.on_tx_enable(ctx->cfg.tx_userdata, ctx->fd, 0);
    }
    return rc;
}

static int io_err_to_client_err(int rc)
{
    switch (rc) {
        case -2:
            return MB_RTU_CLIENT_ERR_TIMEOUT;
        case RTU_IO_CRC_ERROR:
            return MB_RTU_CLIENT_ERR_CRC;
        case RTU_IO_FRAME_ERROR:
            return MB_RTU_CLIENT_ERR_FRAME;
        case RTU_IO_UNIT_ID_ERROR:
            return MB_RTU_CLIENT_ERR_UNIT_ID;
        default:
            return MB_RTU_CLIENT_ERR_TRANSPORT;
    }
}

/**
 * Begin a bus transaction: caller already holds ctx->lock.
 * Locks the shared bus and waits out the required bus silence.
 */
static void bus_transaction_begin(mb_rtu_client_ctx_t *ctx)
{
    pthread_mutex_lock(&ctx->shared_bus->mutex);
    wait_bus_silence(ctx->shared_bus, ctx->cfg.unit_id,
        effective_inter_frame_us(ctx),
        effective_handoff_silence_ms(ctx));
}

static void bus_transaction_end(mb_rtu_client_ctx_t *ctx)
{
    mark_bus_transaction_done(ctx->shared_bus, ctx->cfg.unit_id);
    pthread_mutex_unlock(&ctx->shared_bus->mutex);
}

/**
 * Receive one response ADU for a request with function code @p expected_fc.
 * Length is derived from the function code (and byte count for reads).
 *
 * @param[out] total_len  Bytes received so far (also set on failure, for logging).
 * @return 0, -1, -2 (timeout) or RTU_IO_* error.
 */
static int recv_response(mb_rtu_client_ctx_t *ctx, uint8_t unit_id, uint8_t expected_fc,
    uint8_t *buf, size_t *total_len)
{
    uint64_t deadline_us = response_deadline_us(ctx);
    size_t frame_len;

    *total_len = 0u;
    int rc = read_exact_timeout(ctx->fd, buf, 2u, deadline_us);
    if (rc != 0) {
        return rc;
    }
    *total_len = 2u;

    uint8_t resp_fc = buf[RTU_OFFSET_FC];
    if (resp_fc == (uint8_t)(expected_fc | MODBUS_EXCEPTION_FLAG)) {
        frame_len = RTU_EXCEPTION_LEN;
    } else if (resp_fc != expected_fc) {
        return RTU_IO_FRAME_ERROR;
    } else if (expected_fc <= MODBUS_FUNC_READ_INPUT_REGISTERS) {
        rc = read_exact_timeout(ctx->fd, buf + 2u, 1u, deadline_us);
        if (rc != 0) {
            return rc;
        }
        *total_len = 3u;
        frame_len = 3u + (size_t)buf[RTU_READ_BYTE_COUNT_OFF] + MODBUS_RTU_CRC_LEN;
        if (frame_len > MODBUS_RTU_MAX_ADU_LENGTH) {
            return RTU_IO_FRAME_ERROR;
        }
    } else {
        frame_len = RTU_WRITE_RESPONSE_LEN;
    }

    rc = read_exact_timeout(ctx->fd, buf + *total_len, frame_len - *total_len, deadline_us);
    if (rc != 0) {
        return rc;
    }
    *total_len = frame_len;

    if (mb_rtu_validate_crc(buf, frame_len) != 0) {
        return RTU_IO_CRC_ERROR;
    }
    if (buf[RTU_OFFSET_UNIT_ID] != unit_id) {
        return RTU_IO_UNIT_ID_ERROR;
    }
    return 0;
}

/**
 * Send one request PDU and, unless broadcast, receive and validate the reply.
 *
 * On MB_RTU_CLIENT_OK, the response PDU starts at @p resp + RTU_OFFSET_PDU
 * and has already passed mb_pdu_check_response().
 *
 * @return MB_RTU_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_RTU_CLIENT_ERR_* code.
 */
static int transact(mb_rtu_client_ctx_t *ctx, const uint8_t *pdu, size_t pdu_len,
    uint8_t *resp)
{
    uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH];
    uint8_t echo[MODBUS_RTU_MAX_ADU_LENGTH];
    size_t resp_len = 0u;
    uint8_t fc = pdu[0];
    int rc;

    if (!ctx->initialized) {
        return MB_RTU_CLIENT_ERR_NOT_CONNECTED;
    }

    pthread_mutex_lock(&ctx->lock);

    uint8_t unit_id = ctx->cfg.unit_id;
    int is_broadcast = (unit_id == MODBUS_RTU_BROADCAST_ADDRESS);
    if (is_broadcast && !mb_pdu_is_write(fc)) {
        pthread_mutex_unlock(&ctx->lock);
        return MB_RTU_CLIENT_ERR_ARG;
    }

    req[RTU_OFFSET_UNIT_ID] = unit_id;
    memcpy(req + RTU_OFFSET_PDU, pdu, pdu_len);
    size_t req_len = RTU_OFFSET_PDU + pdu_len;
    mb_rtu_append_crc(req, req_len, sizeof(req));
    req_len += MODBUS_RTU_CRC_LEN;

    bus_transaction_begin(ctx);

    tcflush(ctx->fd, TCIOFLUSH);
    if (send_frame(ctx, req, req_len) != 0) {
        bus_transaction_end(ctx);
        pthread_mutex_unlock(&ctx->lock);
        cli_logv(ctx, MB_RTU_LOG_ERROR, "FC%02X send failed", (unsigned)fc);
        return MB_RTU_CLIENT_ERR_TRANSPORT;
    }

    if (ctx->cfg.local_echo) {
        rc = read_exact_timeout(ctx->fd, echo, req_len, response_deadline_us(ctx));
        if (rc != 0 || memcmp(echo, req, req_len) != 0) {
            bus_transaction_end(ctx);
            pthread_mutex_unlock(&ctx->lock);
            cli_logv(ctx, MB_RTU_LOG_ERROR, "FC%02X local echo missing or corrupted",
                (unsigned)fc);
            return MB_RTU_CLIENT_ERR_TRANSPORT;
        }
    }

    if (is_broadcast) {
        bus_transaction_end(ctx);
        pthread_mutex_unlock(&ctx->lock);
        return MB_RTU_CLIENT_OK;
    }

    rc = recv_response(ctx, unit_id, fc, resp, &resp_len);
    bus_transaction_end(ctx);
    pthread_mutex_unlock(&ctx->lock);

    if (rc != 0) {
        rc = io_err_to_client_err(rc);
        log_transaction_failure(ctx, rc, unit_id, req, req_len, resp, resp_len);
        return rc;
    }

    rc = mb_pdu_check_response(pdu, resp + RTU_OFFSET_PDU,
        resp_len - RTU_OFFSET_PDU - MODBUS_RTU_CRC_LEN);
    if (rc == MB_PDU_ERR_FRAME) {
        log_transaction_failure(ctx, MB_RTU_CLIENT_ERR_FRAME, unit_id,
            req, req_len, resp, resp_len);
        return MB_RTU_CLIENT_ERR_FRAME;
    }
    if (rc > 0) {
        cli_logv(ctx, MB_RTU_LOG_WARN, "FC%02X exception code %d", (unsigned)fc, rc);
    }
    return rc;
}

static int read_registers(mb_rtu_client_ctx_t *ctx, uint8_t fc,
    uint16_t addr, uint16_t qty, uint16_t *out)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];

    if (!ctx || !out || qty == 0u || qty > MODBUS_MAX_READ_REGISTERS) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_read(pdu, fc, addr, qty);
    int rc = transact(ctx, pdu, pdu_len, resp);
    if (rc == MB_RTU_CLIENT_OK) {
        mb_pdu_get_registers(resp + RTU_OFFSET_PDU, qty, out);
    }
    return rc;
}

static int read_bits(mb_rtu_client_ctx_t *ctx, uint8_t fc,
    uint16_t addr, uint16_t qty, uint8_t *out)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];

    if (!ctx || !out || qty == 0u || qty > MODBUS_MAX_READ_BITS) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_read(pdu, fc, addr, qty);
    int rc = transact(ctx, pdu, pdu_len, resp);
    if (rc == MB_RTU_CLIENT_OK) {
        mb_pdu_get_bits(resp + RTU_OFFSET_PDU, qty, out);
    }
    return rc;
}

/* -- Public API ---------------------------------------------------------- */

int mb_rtu_client_connect(mb_rtu_client_ctx_t *ctx, const mb_rtu_client_config_t *cfg)
{
    mb_rtu_shared_bus_t *bus;
    mb_rtu_serial_config_t serial;

    if (!ctx || !cfg || !cfg->device || cfg->device[0] == '\0') {
        return -1;
    }
    if (ctx->initialized) {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->fd = -1;
    ctx->shared_bus = NULL;

    if (pthread_mutex_init(&ctx->lock, NULL) != 0) {
        return -1;
    }

    serial.device = cfg->device;
    serial.baud_rate = cfg->baud_rate;
    serial.data_bits = cfg->data_bits;
    serial.stop_bits = cfg->stop_bits;
    serial.parity = cfg->parity;
    serial.rs485_mode = cfg->rs485_mode;

    bus = shared_bus_attach(&serial);
    if (!bus) {
        pthread_mutex_destroy(&ctx->lock);
        return -1;
    }

    ctx->shared_bus = bus;
    ctx->fd = bus->fd;
    ctx->initialized = 1;
    cli_logv(ctx, MB_RTU_LOG_DEBUG, "connected to %s (shared bus)", cfg->device);
    return 0;
}

void mb_rtu_client_disconnect(mb_rtu_client_ctx_t *ctx)
{
    if (!ctx || !ctx->initialized) {
        return;
    }

    pthread_mutex_lock(&ctx->lock);

    shared_bus_detach(ctx->shared_bus);
    ctx->shared_bus = NULL;
    ctx->fd = -1;
    ctx->initialized = 0;

    pthread_mutex_unlock(&ctx->lock);
    pthread_mutex_destroy(&ctx->lock);
}

int mb_rtu_client_set_unit_id(mb_rtu_client_ctx_t *ctx, uint8_t unit_id)
{
    if (!ctx) {
        return MB_RTU_CLIENT_ERR_ARG;
    }
    if (!ctx->initialized) {
        return MB_RTU_CLIENT_ERR_NOT_CONNECTED;
    }

    pthread_mutex_lock(&ctx->lock);
    ctx->cfg.unit_id = unit_id;
    pthread_mutex_unlock(&ctx->lock);
    return MB_RTU_CLIENT_OK;
}

int mb_rtu_client_read_holding_registers(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    uint16_t *out)
{
    return read_registers(ctx, MODBUS_FUNC_READ_HOLDING_REGISTERS, addr, qty, out);
}

int mb_rtu_client_read_input_registers(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    uint16_t *out)
{
    return read_registers(ctx, MODBUS_FUNC_READ_INPUT_REGISTERS, addr, qty, out);
}

int mb_rtu_client_read_coils(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty, uint8_t *out)
{
    return read_bits(ctx, MODBUS_FUNC_READ_COILS, addr, qty, out);
}

int mb_rtu_client_read_discrete_inputs(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty, uint8_t *out)
{
    return read_bits(ctx, MODBUS_FUNC_READ_DISCRETE_INPUTS, addr, qty, out);
}

int mb_rtu_client_write_single_register(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t value)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];

    if (!ctx) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_write_single(pdu, MODBUS_FUNC_WRITE_SINGLE_REGISTER,
        addr, value);
    return transact(ctx, pdu, pdu_len, resp);
}

int mb_rtu_client_write_single_coil(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint8_t value)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];

    if (!ctx) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_write_single(pdu, MODBUS_FUNC_WRITE_SINGLE_COIL, addr,
        value ? MODBUS_COIL_ON : MODBUS_COIL_OFF);
    return transact(ctx, pdu, pdu_len, resp);
}

int mb_rtu_client_write_multiple_registers(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    const uint16_t *data)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];

    if (!ctx || !data || qty == 0u || qty > MODBUS_MAX_WRITE_REGISTERS) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_write_registers(pdu, addr, qty, data);
    return transact(ctx, pdu, pdu_len, resp);
}

int mb_rtu_client_write_multiple_coils(mb_rtu_client_ctx_t *ctx,
    uint16_t addr, uint16_t qty,
    const uint8_t *values)
{
    uint8_t pdu[MODBUS_MAX_PDU_LENGTH];
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];

    if (!ctx || !values || qty == 0u || qty > MODBUS_MAX_WRITE_BITS) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    size_t pdu_len = mb_pdu_build_write_coils(pdu, addr, qty, values);
    return transact(ctx, pdu, pdu_len, resp);
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
