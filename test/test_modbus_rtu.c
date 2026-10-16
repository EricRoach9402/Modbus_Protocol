/**
 * @file test_modbus_rtu.c
 * @brief Validation for Modbus RTU framing helpers.
 */

#define _GNU_SOURCE

#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include "modbus_rtu.h"
#include "modbus_rtu_client.h"
#include "modbus_rtu_server.h"

static int g_pass_count = 0;
static int g_fail_count = 0;

static void check(const char *description, int condition)
{
    if (condition) {
        printf("  [PASS]  %s\n", description);
        g_pass_count++;
    } else {
        printf("  [FAIL]  %s\n", description);
        g_fail_count++;
    }
}

static void test_crc16(void)
{
    printf("\n-- CRC16/MODBUS --\n");

    const uint8_t payload[] = {
        0x01u, 0x03u, 0x00u, 0x00u, 0x00u, 0x0Au,
    };
    uint16_t crc = mb_rtu_crc16(payload, sizeof(payload));

    check("known CRC for 01 03 00 00 00 0A is 0xCDC5", crc == 0xCDC5u);

    uint8_t frame[8] = {
        0x01u, 0x03u, 0x00u, 0x00u, 0x00u, 0x0Au, 0xC5u, 0xCDu,
    };
    check("validate known frame CRC", mb_rtu_validate_crc(frame, sizeof(frame)) == 0);

    frame[6] ^= 0x01u;
    check("reject corrupted CRC", mb_rtu_validate_crc(frame, sizeof(frame)) != 0);
}

static void test_request_builders(void)
{
    printf("\n-- Request Builders --\n");

    uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH];
    int len = mb_rtu_build_request_basis(1u, MODBUS_FUNC_READ_HOLDING_REGISTERS,
                                         0u, 10u, req, sizeof(req));
    check("FC03 request length is 8", len == 8);
    check("FC03 request CRC validates", mb_rtu_validate_crc(req, (size_t)len) == 0);
    check("FC03 request matches known bytes",
          len == 8 &&
          req[0] == 0x01u && req[1] == 0x03u &&
          req[2] == 0x00u && req[3] == 0x00u &&
          req[4] == 0x00u && req[5] == 0x0Au &&
          req[6] == 0xC5u && req[7] == 0xCDu);

    len = mb_rtu_build_fc06_request(1u, 0x000Au, 0x1234u, req, sizeof(req));
    check("FC06 request length is 8", len == 8);
    check("FC06 request CRC validates", mb_rtu_validate_crc(req, (size_t)len) == 0);

    uint16_t values[3] = {0x1111u, 0x2222u, 0x3333u};
    len = mb_rtu_build_fc16_request(1u, 0x0020u, 3u, values, req, sizeof(req));
    check("FC16 request length is 15", len == 15);
    check("FC16 request CRC validates", mb_rtu_validate_crc(req, (size_t)len) == 0);
    check("FC16 byte count is qty * 2", len == 15 && req[6] == 6u);
}

static void test_parse_request(void)
{
    printf("\n-- Request Parser --\n");

    uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH];
    uint8_t fc = 0u;
    uint16_t addr = 0u;
    uint16_t qty = 0u;

    int len = mb_rtu_build_request_basis(1u, MODBUS_FUNC_READ_HOLDING_REGISTERS,
                                         0x0010u, 5u, req, sizeof(req));
    int rc = mb_rtu_parse_request(req, (size_t)len, &fc, &addr, &qty);
    check("parse FC03 request", rc == 0);
    check("parse FC03 fields",
          fc == MODBUS_FUNC_READ_HOLDING_REGISTERS && addr == 0x0010u && qty == 5u);

    len = mb_rtu_build_fc06_request(1u, 0x000Au, 0x2222u, req, sizeof(req));
    rc = mb_rtu_parse_request(req, (size_t)len, &fc, &addr, &qty);
    check("parse FC06 request", rc == 0);
    check("parse FC06 forces qty=1",
          fc == MODBUS_FUNC_WRITE_SINGLE_REGISTER && addr == 0x000Au && qty == 1u);
}

static void test_read_response_builder(void)
{
    printf("\n-- Read Response Builder --\n");

    uint16_t data[2] = {0x1234u, 0xABCDu};
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];

    int len = mb_rtu_build_read_registers_response(MODBUS_FUNC_READ_HOLDING_REGISTERS,
                                                   1u, data, 2u, resp, sizeof(resp));
    check("FC03 response length is 9", len == 9);
    check("FC03 response CRC validates", mb_rtu_validate_crc(resp, (size_t)len) == 0);
    check("FC03 response data is big-endian",
          len == 9 && resp[2] == 4u &&
          resp[3] == 0x12u && resp[4] == 0x34u &&
          resp[5] == 0xABu && resp[6] == 0xCDu);
}

static void test_t35(void)
{
    printf("\n-- T3.5 Inter-frame Silence --\n");

    mb_rtu_serial_config_t serial = {
        .baud_rate = 9600u,
        .data_bits = 8u,
        .stop_bits = 1u,
        .parity = MB_RTU_PARITY_NONE,
    };
    check("9600 8N1 -> 3646 us", mb_rtu_t35_us(&serial) == 3646u);

    serial.parity = MB_RTU_PARITY_EVEN;
    check("9600 8E1 -> 4011 us", mb_rtu_t35_us(&serial) == 4011u);

    serial.baud_rate = 0u;
    serial.data_bits = 0u;
    serial.stop_bits = 0u;
    serial.parity = MB_RTU_PARITY_NONE;
    check("zero fields use 9600 8N1 defaults", mb_rtu_t35_us(&serial) == 3646u);

    serial.baud_rate = 19200u;
    check("19200 8N1 -> 1823 us", mb_rtu_t35_us(&serial) == 1823u);

    serial.baud_rate = 115200u;
    check("above 19200 baud -> fixed 1750 us", mb_rtu_t35_us(&serial) == 1750u);

    check("NULL serial -> 0", mb_rtu_t35_us(NULL) == 0u);
}

static void test_effective_timing(void)
{
    printf("\n-- Timing Field Resolution --\n");

    check("0 -> default value",
          mb_rtu_effective_timing(0u, 30u) == 30u);
    check("DISABLED -> 0 (off)",
          mb_rtu_effective_timing(MB_RTU_TIMING_DISABLED, 30u) == 0u);
    check("explicit value kept",
          mb_rtu_effective_timing(10000u, 30u) == 10000u);
    check("DISABLED with zero default still off",
          mb_rtu_effective_timing(MB_RTU_TIMING_DISABLED, 0u) == 0u);
}

/**
 * mb_rtu_client_probe_device() – client-side argument / state validation.
 *
 * Exercises the guard clauses only (qty range, not-connected) without a real
 * serial device, mirroring how the TCP suite validates client-side rejection
 * paths in test_not_connected_guard() / test_exception_handling().
 */
static void test_probe_device_guards(void)
{
    printf("\n-- mb_rtu_client_probe_device() Guards --\n");

    mb_rtu_client_ctx_t ctx = {0};
    ctx.fd = -1;                 /* never opened; matches post-disconnect state */
    ctx.cfg.unit_id = 1u;

    int rc = mb_rtu_client_probe_device(&ctx, 0u, 0u);
    check("qty=0 rejected client-side -> MB_RTU_CLIENT_ERR_ARG",
          rc == MB_RTU_CLIENT_ERR_ARG);

    rc = mb_rtu_client_probe_device(&ctx, 0u, MODBUS_MAX_READ_REGISTERS + 1u);
    check("qty > MODBUS_MAX_READ_REGISTERS rejected -> MB_RTU_CLIENT_ERR_ARG",
          rc == MB_RTU_CLIENT_ERR_ARG);

    rc = mb_rtu_client_probe_device(&ctx, 0u, 1u);
    check("valid qty on unopened ctx -> MB_RTU_CLIENT_ERR_NOT_CONNECTED",
          rc == MB_RTU_CLIENT_ERR_NOT_CONNECTED);
}

/**
 * connect/disconnect lifecycle on a pseudo-terminal standing in for a
 * serial port.
 */
static void test_client_lifecycle(void)
{
    printf("\n-- Client Connect / Disconnect Lifecycle --\n");

    mb_rtu_client_ctx_t ctx = {0};
    mb_rtu_client_disconnect(&ctx);
    check("disconnect on zero-initialized ctx is a no-op", ctx.initialized == 0);

    mb_rtu_client_config_t cfg = {
        .device = "/dev/nonexistent-modbus-test",
        .unit_id = 1u,
    };
    check("connect to missing device fails", mb_rtu_client_connect(&ctx, &cfg) != 0);
    mb_rtu_client_disconnect(&ctx);
    check("disconnect after failed connect is a no-op", ctx.initialized == 0);

    int master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) {
        check("pty setup", 0);
        if (master >= 0) {
            close(master);
        }
        return;
    }
    cfg.device = ptsname(master);

    check("connect to pty succeeds", mb_rtu_client_connect(&ctx, &cfg) == 0);
    check("second connect on connected ctx is rejected",
          mb_rtu_client_connect(&ctx, &cfg) != 0);
    check("fd still open after rejected reconnect", ctx.fd >= 0);

    mb_rtu_client_ctx_t peer = {0};
    mb_rtu_client_config_t peer_cfg = cfg;
    peer_cfg.unit_id = 2u;
    check("second ctx on same device shares the fd",
          mb_rtu_client_connect(&peer, &peer_cfg) == 0 && peer.fd == ctx.fd);

    mb_rtu_client_disconnect(&ctx);
    mb_rtu_client_disconnect(&ctx);
    check("double disconnect is safe", ctx.initialized == 0 && ctx.fd < 0);
    check("shared fd stays open for remaining ctx",
          fcntl(peer.fd, F_GETFD) != -1);

    check("reconnect after disconnect (no memset) succeeds",
          mb_rtu_client_connect(&ctx, &cfg) == 0);

    char link_path[64];
    snprintf(link_path, sizeof(link_path), "/tmp/mb_rtu_test_link_%d", (int)getpid());
    unlink(link_path);
    if (symlink(cfg.device, link_path) == 0) {
        mb_rtu_client_ctx_t via_link = {0};
        mb_rtu_client_config_t link_cfg = cfg;
        link_cfg.device = link_path;
        check("symlink to same device shares the fd",
              mb_rtu_client_connect(&via_link, &link_cfg) == 0 && via_link.fd == ctx.fd);
        mb_rtu_client_disconnect(&via_link);
        unlink(link_path);
    } else {
        check("symlink setup", 0);
    }

    mb_rtu_client_disconnect(&ctx);
    mb_rtu_client_disconnect(&peer);
    close(master);
}

/** Open a pty master; returns its fd and stores the slave path in @p path. */
static int open_test_pty(char *path, size_t path_cap)
{
    int master = posix_openpt(O_RDWR | O_NOCTTY);

    if (master < 0) {
        return -1;
    }
    if (grantpt(master) != 0 || unlockpt(master) != 0 ||
        ptsname_r(master, path, path_cap) != 0) {
        close(master);
        return -1;
    }
    return master;
}

#define MANY_DEVICES 12

/**
 * The shared-bus registry has no fixed slot count: many distinct devices can
 * be open at once, and records are released on the last disconnect.
 */
static void test_many_shared_buses(void)
{
    printf("\n-- Many Distinct Devices --\n");

    int masters[MANY_DEVICES];
    char paths[MANY_DEVICES][64];
    mb_rtu_client_ctx_t ctxs[MANY_DEVICES];
    int all_ok = 1;

    memset(ctxs, 0, sizeof(ctxs));
    for (int i = 0; i < MANY_DEVICES; i++) {
        masters[i] = open_test_pty(paths[i], sizeof(paths[i]));
        if (masters[i] < 0) {
            all_ok = 0;
        }
    }
    if (!all_ok) {
        check("pty setup", 0);
        for (int i = 0; i < MANY_DEVICES; i++) {
            if (masters[i] >= 0) {
                close(masters[i]);
            }
        }
        return;
    }

    for (int i = 0; i < MANY_DEVICES; i++) {
        mb_rtu_client_config_t cfg = { .device = paths[i], .unit_id = 1u };
        if (mb_rtu_client_connect(&ctxs[i], &cfg) != 0) {
            all_ok = 0;
        }
    }
    check("12 distinct devices open concurrently", all_ok);

    int distinct = 1;
    for (int i = 1; i < MANY_DEVICES; i++) {
        if (ctxs[i].fd == ctxs[0].fd) {
            distinct = 0;
        }
    }
    check("each device has its own fd", distinct);

    mb_rtu_client_disconnect(&ctxs[0]);
    mb_rtu_client_config_t again_cfg = { .device = paths[0], .unit_id = 1u };
    check("reconnect after last disconnect reopens the device",
          mb_rtu_client_connect(&ctxs[0], &again_cfg) == 0 && ctxs[0].fd >= 0);

    for (int i = 0; i < MANY_DEVICES; i++) {
        mb_rtu_client_disconnect(&ctxs[i]);
        close(masters[i]);
    }
}

static uint64_t now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

/** Read exactly @p len bytes from @p fd within @p timeout_ms. */
static int read_with_timeout(int fd, uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    size_t got = 0u;
    uint64_t deadline = now_ms() + timeout_ms;

    while (got < len) {
        uint64_t now = now_ms();
        if (now >= deadline) {
            return -1;
        }
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv = {
            .tv_sec = (time_t)((deadline - now) / 1000u),
            .tv_usec = (suseconds_t)(((deadline - now) % 1000u) * 1000u),
        };
        if (select(fd + 1, &rfds, NULL, NULL, &tv) <= 0) {
            return -1;
        }
        ssize_t n = read(fd, buf + got, len - got);
        if (n <= 0) {
            return -1;
        }
        got += (size_t)n;
    }
    return 0;
}

typedef struct {
    int master;
    uint32_t byte_interval_ms;
} drip_slave_t;

/** Slave stub: consume one FC03 request, then send a valid reply byte by byte. */
static void *drip_slave_thread(void *arg)
{
    drip_slave_t *slave = (drip_slave_t *)arg;
    uint8_t req[MODBUS_RTU_MIN_REQ_LENGTH];
    uint8_t resp[8] = { 0x01u, 0x03u, 0x02u, 0x00u, 0x2Au };

    if (read_with_timeout(slave->master, req, sizeof(req), 2000u) != 0) {
        return NULL;
    }
    mb_rtu_append_crc(resp, 5u, sizeof(resp));

    for (size_t i = 0u; i < 7u; i++) {
        struct timespec pause_time = {
            .tv_sec = 0,
            .tv_nsec = (long)slave->byte_interval_ms * 1000000L,
        };
        nanosleep(&pause_time, NULL);
        if (write(slave->master, &resp[i], 1u) != 1) {
            return NULL;
        }
    }
    return NULL;
}

/**
 * response_timeout_ms bounds the whole response, not each received chunk:
 * a slave dribbling bytes slower than the deadline must time out.
 */
static void test_response_deadline(void)
{
    printf("\n-- Client Response Deadline --\n");

    char path[64];
    int master = open_test_pty(path, sizeof(path));
    if (master < 0) {
        check("pty setup", 0);
        return;
    }

    mb_rtu_client_ctx_t ctx = {0};
    mb_rtu_client_config_t cfg = {
        .device = path,
        .baud_rate = 115200u,
        .unit_id = 1u,
        .response_timeout_ms = 200u,
    };
    if (mb_rtu_client_connect(&ctx, &cfg) != 0) {
        check("connect to pty", 0);
        close(master);
        return;
    }

    /* 7 bytes x 100 ms = 700 ms total, each gap well under the 200 ms timeout. */
    drip_slave_t slave = { .master = master, .byte_interval_ms = 100u };
    pthread_t thread;
    pthread_create(&thread, NULL, drip_slave_thread, &slave);

    uint16_t value = 0u;
    uint64_t start = now_ms();
    int rc = mb_rtu_client_read_holding_registers(&ctx, 0u, 1u, &value);
    uint64_t elapsed = now_ms() - start;

    check("slow-dripping response -> MB_RTU_CLIENT_ERR_TIMEOUT",
          rc == MB_RTU_CLIENT_ERR_TIMEOUT);
    check("timeout fires near the deadline, not per byte", elapsed < 500u);

    pthread_join(thread, NULL);
    mb_rtu_client_disconnect(&ctx);
    close(master);
}

static int resync_on_read(uint8_t function_code, uint16_t addr, uint16_t qty,
                          uint16_t *out, void *userdata)
{
    (void)function_code;
    (void)userdata;
    for (uint16_t i = 0u; i < qty; i++) {
        out[i] = (uint16_t)(0x1000u + addr + i);
    }
    return 0;
}

/**
 * Server lifecycle and multi-drop resync: another slave's response sitting in
 * front of our request must be skipped, and our request still answered.
 */
static void test_server_resync(void)
{
    printf("\n-- RTU Server Lifecycle / Resync --\n");

    mb_rtu_server_ctx_t srv = {0};
    mb_rtu_server_stop(&srv);
    check("stop on zero-initialized ctx is a no-op", srv.transport.initialized == 0);

    mb_rtu_server_config_t cfg;
    mb_rtu_server_config_init(&cfg);
    cfg.device = "/dev/nonexistent-modbus-test";
    cfg.unit_id = 1u;
    cfg.on_read = resync_on_read;
    check("start on missing device fails", mb_rtu_server_start(&srv, &cfg) != 0);
    mb_rtu_server_stop(&srv);
    mb_rtu_server_stop(&srv);
    check("stop after failed start and double stop are safe",
          srv.transport.initialized == 0);

    char path[64];
    int master = open_test_pty(path, sizeof(path));
    if (master < 0) {
        check("pty setup", 0);
        return;
    }
    cfg.device = path;
    cfg.baud_rate = 115200u;
    if (mb_rtu_server_start(&srv, &cfg) != 0) {
        check("start on pty", 0);
        close(master);
        return;
    }
    check("start on running ctx is rejected", mb_rtu_server_start(&srv, &cfg) != 0);

    /* Slave 2's FC03 reply (9 bytes) immediately followed by our FC03 request. */
    uint8_t stream[MODBUS_RTU_MAX_ADU_LENGTH] = {
        0x02u, 0x03u, 0x04u, 0x00u, 0x0Au, 0x00u, 0x0Bu,
    };
    size_t stream_len = 7u;
    mb_rtu_append_crc(stream, stream_len, sizeof(stream));
    stream_len += MODBUS_RTU_CRC_LEN;

    uint8_t req[MODBUS_RTU_MIN_REQ_LENGTH];
    int req_len = mb_rtu_build_request_basis(1u, MODBUS_FUNC_READ_HOLDING_REGISTERS,
                                             0x0005u, 1u, req, sizeof(req));
    memcpy(stream + stream_len, req, (size_t)req_len);
    stream_len += (size_t)req_len;

    uint8_t resp[7];
    int ok = write(master, stream, stream_len) == (ssize_t)stream_len &&
             read_with_timeout(master, resp, sizeof(resp), 1000u) == 0;
    check("request after foreign response is answered", ok);
    check("resynced response is valid",
          ok && mb_rtu_validate_crc(resp, sizeof(resp)) == 0 &&
          resp[0] == 0x01u && resp[1] == 0x03u &&
          resp[3] == 0x10u && resp[4] == 0x05u);

    /* A corrupted request (bad CRC) followed by a good one. */
    uint8_t corrupt[MODBUS_RTU_MIN_REQ_LENGTH * 2u];
    memcpy(corrupt, req, (size_t)req_len);
    corrupt[req_len - 1] ^= 0xFFu;
    memcpy(corrupt + req_len, req, (size_t)req_len);
    ok = write(master, corrupt, sizeof(corrupt)) == (ssize_t)sizeof(corrupt) &&
         read_with_timeout(master, resp, sizeof(resp), 1000u) == 0;
    check("request after CRC-corrupted frame is answered",
          ok && mb_rtu_validate_crc(resp, sizeof(resp)) == 0);

    mb_rtu_server_stop(&srv);
    mb_rtu_server_stop(&srv);
    check("double stop after run is safe", srv.transport.initialized == 0);
    close(master);
}

/* -- Client <-> server end-to-end over two ptys joined by a relay --------- */

typedef struct {
    int master_client;
    int master_server;
    volatile int running;
    int echo_client;
    int echo_server;
} pty_relay_t;

static void relay_copy(int from, int to, int echo)
{
    uint8_t buf[MODBUS_RTU_MAX_ADU_LENGTH];
    ssize_t n = read(from, buf, sizeof(buf));

    if (n <= 0) {
        return;
    }
    if (write(to, buf, (size_t)n) != n) {
        return;
    }
    if (echo && write(from, buf, (size_t)n) != n) {
        return;
    }
}

/** Forward bytes both ways; optionally echo each side's TX back to it. */
static void *pty_relay_thread(void *arg)
{
    pty_relay_t *relay = (pty_relay_t *)arg;
    int max_fd = (relay->master_client > relay->master_server)
                 ? relay->master_client : relay->master_server;

    while (relay->running) {
        fd_set rfds;
        struct timeval tv = { .tv_sec = 0, .tv_usec = 20000 };
        FD_ZERO(&rfds);
        FD_SET(relay->master_client, &rfds);
        FD_SET(relay->master_server, &rfds);
        if (select(max_fd + 1, &rfds, NULL, NULL, &tv) <= 0) {
            continue;
        }
        if (FD_ISSET(relay->master_client, &rfds)) {
            relay_copy(relay->master_client, relay->master_server, relay->echo_client);
        }
        if (FD_ISSET(relay->master_server, &rfds)) {
            relay_copy(relay->master_server, relay->master_client, relay->echo_server);
        }
    }
    return NULL;
}

#define E2E_BANK_SIZE 64u

static uint16_t e2e_regs[E2E_BANK_SIZE];
static uint8_t e2e_coils[E2E_BANK_SIZE];

static int e2e_read(uint8_t function_code, uint16_t addr, uint16_t qty,
                    uint16_t *out, void *userdata)
{
    (void)userdata;
    if ((uint32_t)addr + qty > E2E_BANK_SIZE) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    for (uint16_t i = 0u; i < qty; i++) {
        out[i] = (uint16_t)(e2e_regs[addr + i] +
                            ((function_code == MODBUS_FUNC_READ_INPUT_REGISTERS) ? 1u : 0u));
    }
    return 0;
}

static int e2e_write(uint16_t addr, uint16_t qty, const uint16_t *data, void *userdata)
{
    (void)userdata;
    if ((uint32_t)addr + qty > E2E_BANK_SIZE) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    memcpy(&e2e_regs[addr], data, (size_t)qty * sizeof(uint16_t));
    return 0;
}

static int e2e_read_bits(uint8_t function_code, uint16_t addr, uint16_t qty,
                         uint8_t *out, void *userdata)
{
    (void)userdata;
    if ((uint32_t)addr + qty > E2E_BANK_SIZE) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    for (uint16_t i = 0u; i < qty; i++) {
        uint16_t a = (uint16_t)(addr + i);
        out[i] = (function_code == MODBUS_FUNC_READ_COILS) ? e2e_coils[a] : (a % 2u == 0u);
    }
    return 0;
}

static int e2e_write_bits(uint16_t addr, uint16_t qty, const uint8_t *bits, void *userdata)
{
    (void)userdata;
    if ((uint32_t)addr + qty > E2E_BANK_SIZE) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    memcpy(&e2e_coils[addr], bits, qty);
    return 0;
}

typedef struct {
    volatile int on_count;
    volatile int off_count;
    volatile int level;
} tx_trace_t;

static void trace_tx_enable(void *userdata, int fd, int enable)
{
    tx_trace_t *trace = (tx_trace_t *)userdata;
    (void)fd;
    if (enable) {
        trace->on_count++;
    } else {
        trace->off_count++;
    }
    trace->level = enable;
}

typedef struct {
    int master_client;
    int master_server;
    char client_path[64];
    char server_path[64];
    pty_relay_t relay;
    pthread_t relay_thread;
    mb_rtu_server_ctx_t server;
    mb_rtu_client_ctx_t client;
} e2e_bench_t;

static int e2e_setup(e2e_bench_t *bench, int echo, tx_trace_t *client_tx, tx_trace_t *server_tx)
{
    memset(bench, 0, sizeof(*bench));
    bench->master_client = open_test_pty(bench->client_path, sizeof(bench->client_path));
    bench->master_server = open_test_pty(bench->server_path, sizeof(bench->server_path));
    if (bench->master_client < 0 || bench->master_server < 0) {
        return -1;
    }

    bench->relay.master_client = bench->master_client;
    bench->relay.master_server = bench->master_server;
    bench->relay.running = 1;
    bench->relay.echo_client = echo;
    bench->relay.echo_server = echo;
    pthread_create(&bench->relay_thread, NULL, pty_relay_thread, &bench->relay);

    mb_rtu_server_config_t srv_cfg;
    mb_rtu_server_config_init(&srv_cfg);
    srv_cfg.device = bench->server_path;
    srv_cfg.baud_rate = 115200u;
    srv_cfg.unit_id = 1u;
    srv_cfg.local_echo = (uint8_t)echo;
    srv_cfg.on_read = e2e_read;
    srv_cfg.on_write = e2e_write;
    srv_cfg.on_read_bits = e2e_read_bits;
    srv_cfg.on_write_bits = e2e_write_bits;
    srv_cfg.on_tx_enable = server_tx ? trace_tx_enable : NULL;
    srv_cfg.tx_userdata = server_tx;
    if (mb_rtu_server_start(&bench->server, &srv_cfg) != 0) {
        return -1;
    }

    mb_rtu_client_config_t cli_cfg = {
        .device = bench->client_path,
        .baud_rate = 115200u,
        .unit_id = 1u,
        .response_timeout_ms = 300u,
        .local_echo = (uint8_t)echo,
        .on_tx_enable = client_tx ? trace_tx_enable : NULL,
        .tx_userdata = client_tx,
    };
    return mb_rtu_client_connect(&bench->client, &cli_cfg);
}

static void e2e_teardown(e2e_bench_t *bench)
{
    mb_rtu_client_disconnect(&bench->client);
    mb_rtu_server_stop(&bench->server);
    if (bench->relay.running) {
        bench->relay.running = 0;
        pthread_join(bench->relay_thread, NULL);
    }
    if (bench->master_client >= 0) {
        close(bench->master_client);
    }
    if (bench->master_server >= 0) {
        close(bench->master_server);
    }
}

/** Every supported function code through a real RTU client and server. */
static void test_rtu_end_to_end(void)
{
    printf("\n-- RTU Client <-> Server End-to-End --\n");

    tx_trace_t client_tx = {0};
    tx_trace_t server_tx = {0};
    e2e_bench_t bench;
    memset(e2e_regs, 0, sizeof(e2e_regs));
    memset(e2e_coils, 0, sizeof(e2e_coils));

    if (e2e_setup(&bench, 0, &client_tx, &server_tx) != 0) {
        check("e2e setup", 0);
        e2e_teardown(&bench);
        return;
    }
    mb_rtu_client_ctx_t *cli = &bench.client;

    static const uint16_t values[3] = { 0x1111u, 0x2222u, 0x3333u };
    uint16_t regs[3] = {0u, 0u, 0u};
    check("FC16 write 3 registers",
          mb_rtu_client_write_multiple_registers(cli, 10u, 3u, values) == MB_RTU_CLIENT_OK);
    check("FC03 read back",
          mb_rtu_client_read_holding_registers(cli, 10u, 3u, regs) == MB_RTU_CLIENT_OK &&
          memcmp(regs, values, sizeof(values)) == 0);
    check("FC06 write single register",
          mb_rtu_client_write_single_register(cli, 11u, 0xBEEFu) == MB_RTU_CLIENT_OK);
    check("FC04 read input registers (holding + 1)",
          mb_rtu_client_read_input_registers(cli, 11u, 1u, regs) == MB_RTU_CLIENT_OK &&
          regs[0] == 0xBEF0u);

    static const uint8_t pattern[10] = { 1, 1, 0, 1, 0, 0, 0, 1, 1, 0 };
    uint8_t bits[16];
    check("FC15 write 10 coils",
          mb_rtu_client_write_multiple_coils(cli, 3u, 10u, pattern) == MB_RTU_CLIENT_OK);
    check("FC05 write coil 4 ON",
          mb_rtu_client_write_single_coil(cli, 4u, 1u) == MB_RTU_CLIENT_OK);
    memset(bits, 0xAA, sizeof(bits));
    check("FC01 read back coils",
          mb_rtu_client_read_coils(cli, 3u, 10u, bits) == MB_RTU_CLIENT_OK &&
          bits[0] == 1u && bits[1] == 1u && bits[2] == 0u && bits[7] == 1u && bits[9] == 0u);
    check("FC02 read discrete inputs",
          mb_rtu_client_read_discrete_inputs(cli, 0u, 4u, bits) == MB_RTU_CLIENT_OK &&
          bits[0] == 1u && bits[1] == 0u && bits[2] == 1u && bits[3] == 0u);
    check("FC01 out-of-range -> ILLEGAL_DATA_ADDRESS",
          mb_rtu_client_read_coils(cli, 60u, 8u, bits) == MODBUS_EX_ILLEGAL_DATA_ADDRESS);

    mb_rtu_client_set_unit_id(cli, MODBUS_RTU_BROADCAST_ADDRESS);
    check("broadcast read rejected client-side",
          mb_rtu_client_read_coils(cli, 0u, 1u, bits) == MB_RTU_CLIENT_ERR_ARG);
    check("broadcast FC05 returns OK without a reply",
          mb_rtu_client_write_single_coil(cli, 30u, 1u) == MB_RTU_CLIENT_OK);
    mb_rtu_client_set_unit_id(cli, 5u);
    check("unknown unit id -> TIMEOUT",
          mb_rtu_client_read_holding_registers(cli, 0u, 1u, regs) == MB_RTU_CLIENT_ERR_TIMEOUT);
    mb_rtu_client_set_unit_id(cli, 1u);
    check("broadcast write executed by server; unit id restored without reconnect",
          mb_rtu_client_read_coils(cli, 30u, 1u, bits) == MB_RTU_CLIENT_OK && bits[0] == 1u);

    check("client TX-enable hook toggled around every frame",
          client_tx.on_count > 0 && client_tx.on_count == client_tx.off_count &&
          client_tx.level == 0);
    check("server TX-enable hook toggled around every response",
          server_tx.on_count > 0 && server_tx.on_count == server_tx.off_count &&
          server_tx.level == 0);

    e2e_teardown(&bench);

    printf("\n-- RTU Local Echo --\n");
    if (e2e_setup(&bench, 1, NULL, NULL) != 0) {
        check("echo e2e setup", 0);
        e2e_teardown(&bench);
        return;
    }
    check("FC06 with local echo on both sides",
          mb_rtu_client_write_single_register(&bench.client, 5u, 0x0505u) == MB_RTU_CLIENT_OK);
    check("FC03 with local echo on both sides",
          mb_rtu_client_read_holding_registers(&bench.client, 5u, 1u, regs) ==
          MB_RTU_CLIENT_OK && regs[0] == 0x0505u);
    check("server did not re-execute its own echoed response",
          mb_rtu_client_read_holding_registers(&bench.client, 5u, 1u, regs) ==
          MB_RTU_CLIENT_OK);
    e2e_teardown(&bench);
}

/** Kernel RS-485 mode is applied at open: a pty has no TIOCSRS485, so it must fail. */
static void test_rs485_mode(void)
{
    printf("\n-- RS-485 Kernel Mode --\n");

    char path[64];
    int master = open_test_pty(path, sizeof(path));
    if (master < 0) {
        check("pty setup", 0);
        return;
    }

    mb_rtu_client_ctx_t ctx = {0};
    mb_rtu_client_config_t cfg = {
        .device = path,
        .unit_id = 1u,
        .rs485_mode = MB_RTU_RS485_RTS_ON_SEND,
    };
    check("RS-485 mode on a non-RS-485 driver fails connect",
          mb_rtu_client_connect(&ctx, &cfg) != 0);

    cfg.rs485_mode = MB_RTU_RS485_NONE;
    check("same device without RS-485 mode connects",
          mb_rtu_client_connect(&ctx, &cfg) == 0);
    mb_rtu_client_disconnect(&ctx);
    close(master);
}

int main(void)
{
    printf("==========================================\n");
    printf("  Modbus RTU API  -  Framing Validation\n");
    printf("==========================================\n");

    test_crc16();
    test_request_builders();
    test_parse_request();
    test_read_response_builder();
    test_t35();
    test_effective_timing();
    test_probe_device_guards();
    test_client_lifecycle();
    test_many_shared_buses();
    test_response_deadline();
    test_server_resync();
    test_rtu_end_to_end();
    test_rs485_mode();

    printf("\n==========================================\n");
    printf("  Results:  %d passed  |  %d failed\n",
           g_pass_count, g_fail_count);
    printf("==========================================\n\n");

    return (g_fail_count == 0) ? 0 : 1;
}
