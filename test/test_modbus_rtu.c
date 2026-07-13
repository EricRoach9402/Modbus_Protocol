/**
 * @file test_modbus_rtu.c
 * @brief Validation for Modbus RTU framing helpers.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "modbus_rtu.h"
#include "modbus_rtu_client.h"

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

int main(void)
{
    printf("==========================================\n");
    printf("  Modbus RTU API  -  Framing Validation\n");
    printf("==========================================\n");

    test_crc16();
    test_request_builders();
    test_parse_request();
    test_read_response_builder();
    test_probe_device_guards();

    printf("\n==========================================\n");
    printf("  Results:  %d passed  |  %d failed\n",
           g_pass_count, g_fail_count);
    printf("==========================================\n\n");

    return (g_fail_count == 0) ? 0 : 1;
}
