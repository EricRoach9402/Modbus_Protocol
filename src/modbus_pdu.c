/**
 * @file modbus_pdu.c
 * @brief Internal transport-independent Modbus PDU encode / decode / dispatch.
 *
 * PDU layout (offsets from the function code):
 *   0     function code
 *   1-2   start address
 *   3-4   quantity, or value for FC05 / FC06
 *   5     byte count (FC15 / FC16 requests)
 *   6+    data       (FC15 / FC16 requests)
 * Read responses: 0 = FC, 1 = byte count, 2+ = data.
 */

#include <string.h>

#include "modbus_pdu.h"

#define PDU_OFFSET_FC 0u
#define PDU_OFFSET_ADDR 1u
#define PDU_OFFSET_QTY 3u
#define PDU_OFFSET_BYTE_COUNT 5u
#define PDU_OFFSET_WRITE_DATA 6u
#define PDU_OFFSET_READ_BYTE_COUNT 1u
#define PDU_OFFSET_READ_DATA 2u

#define PDU_FIXED_REQ_LEN 5u
#define PDU_WRITE_RESPONSE_LEN 5u
#define PDU_EXCEPTION_LEN 2u

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFu);
}

/** Nonzero when [addr, addr + qty) runs past the 16-bit address space. */
static int range_overflows(uint16_t addr, uint16_t qty)
{
    return (uint32_t)addr + qty > 0x10000u;
}

static size_t bit_byte_count(uint16_t qty)
{
    return ((size_t)qty + 7u) / 8u;
}

static void pack_bits(const uint8_t *values, uint16_t qty, uint8_t *out)
{
    memset(out, 0, bit_byte_count(qty));
    for (uint16_t i = 0u; i < qty; i++) {
        if (values[i]) {
            out[i / 8u] |= (uint8_t)(1u << (i % 8u));
        }
    }
}

static void unpack_bits(const uint8_t *packed, uint16_t qty, uint8_t *out)
{
    for (uint16_t i = 0u; i < qty; i++) {
        out[i] = (uint8_t)((packed[i / 8u] >> (i % 8u)) & 1u);
    }
}

static size_t build_exception(uint8_t *resp, uint8_t function_code, uint8_t exception_code)
{
    resp[0] = function_code | (uint8_t)MODBUS_EXCEPTION_FLAG;
    resp[1] = exception_code;
    return PDU_EXCEPTION_LEN;
}

/** Map a callback return value to an exception code; unknown values → device failure. */
static uint8_t exception_from_callback_rc(int rc)
{
    switch (rc) {
        case MODBUS_EX_ILLEGAL_FUNCTION:
        case MODBUS_EX_ILLEGAL_DATA_ADDRESS:
        case MODBUS_EX_ILLEGAL_DATA_VALUE:
        case MODBUS_EX_SERVER_DEVICE_FAILURE:
        case MODBUS_EX_ACKNOWLEDGE:
        case MODBUS_EX_SERVER_DEVICE_BUSY:
        case MODBUS_EX_MEMORY_PARITY_ERROR:
        case MODBUS_EX_GATEWAY_PATH_UNAVAILABLE:
        case MODBUS_EX_GATEWAY_TARGET_FAILED:
            return (uint8_t)rc;
        default:
            return (uint8_t)MODBUS_EX_SERVER_DEVICE_FAILURE;
    }
}

/** Echo FC + address + quantity / value as the write success response. */
static size_t build_write_echo(const uint8_t *req, uint8_t *resp)
{
    memcpy(resp, req, PDU_WRITE_RESPONSE_LEN);
    return PDU_WRITE_RESPONSE_LEN;
}

size_t mb_pdu_build_read(uint8_t *pdu, uint8_t function_code, uint16_t addr, uint16_t qty)
{
    pdu[PDU_OFFSET_FC] = function_code;
    put_u16(pdu + PDU_OFFSET_ADDR, addr);
    put_u16(pdu + PDU_OFFSET_QTY, qty);
    return PDU_FIXED_REQ_LEN;
}

size_t mb_pdu_build_write_single(uint8_t *pdu, uint8_t function_code,
                                 uint16_t addr, uint16_t value)
{
    return mb_pdu_build_read(pdu, function_code, addr, value);
}

size_t mb_pdu_build_write_registers(uint8_t *pdu, uint16_t addr, uint16_t qty,
                                    const uint16_t *data)
{
    mb_pdu_build_read(pdu, MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS, addr, qty);
    pdu[PDU_OFFSET_BYTE_COUNT] = (uint8_t)(qty * 2u);
    for (uint16_t i = 0u; i < qty; i++) {
        put_u16(pdu + PDU_OFFSET_WRITE_DATA + (size_t)i * 2u, data[i]);
    }
    return PDU_OFFSET_WRITE_DATA + (size_t)qty * 2u;
}

size_t mb_pdu_build_write_coils(uint8_t *pdu, uint16_t addr, uint16_t qty,
                                const uint8_t *values)
{
    size_t byte_count = bit_byte_count(qty);

    mb_pdu_build_read(pdu, MODBUS_FUNC_WRITE_MULTIPLE_COILS, addr, qty);
    pdu[PDU_OFFSET_BYTE_COUNT] = (uint8_t)byte_count;
    pack_bits(values, qty, pdu + PDU_OFFSET_WRITE_DATA);
    return PDU_OFFSET_WRITE_DATA + byte_count;
}

int mb_pdu_check_response(const uint8_t *req, const uint8_t *resp, size_t resp_len)
{
    uint8_t fc = req[PDU_OFFSET_FC];
    uint16_t qty = get_u16(req + PDU_OFFSET_QTY);
    size_t byte_count;

    if (resp_len < 1u) {
        return MB_PDU_ERR_FRAME;
    }
    if (resp[PDU_OFFSET_FC] == (uint8_t)(fc | MODBUS_EXCEPTION_FLAG)) {
        if (resp_len != PDU_EXCEPTION_LEN || resp[1] == 0u) {
            return MB_PDU_ERR_FRAME;
        }
        return (int)resp[1];
    }
    if (resp[PDU_OFFSET_FC] != fc) {
        return MB_PDU_ERR_FRAME;
    }

    switch (fc) {
        case MODBUS_FUNC_READ_COILS:
        case MODBUS_FUNC_READ_DISCRETE_INPUTS:
        case MODBUS_FUNC_READ_HOLDING_REGISTERS:
        case MODBUS_FUNC_READ_INPUT_REGISTERS:
            byte_count = (fc <= MODBUS_FUNC_READ_DISCRETE_INPUTS)
                         ? bit_byte_count(qty) : (size_t)qty * 2u;
            if (resp_len != PDU_OFFSET_READ_DATA + byte_count ||
                resp[PDU_OFFSET_READ_BYTE_COUNT] != (uint8_t)byte_count) {
                return MB_PDU_ERR_FRAME;
            }
            return 0;

        case MODBUS_FUNC_WRITE_SINGLE_COIL:
        case MODBUS_FUNC_WRITE_SINGLE_REGISTER:
        case MODBUS_FUNC_WRITE_MULTIPLE_COILS:
        case MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS:
            if (resp_len != PDU_WRITE_RESPONSE_LEN ||
                memcmp(resp + PDU_OFFSET_ADDR, req + PDU_OFFSET_ADDR, 4u) != 0) {
                return MB_PDU_ERR_FRAME;
            }
            return 0;

        default:
            return MB_PDU_ERR_FRAME;
    }
}

void mb_pdu_get_registers(const uint8_t *resp, uint16_t qty, uint16_t *out)
{
    for (uint16_t i = 0u; i < qty; i++) {
        out[i] = get_u16(resp + PDU_OFFSET_READ_DATA + (size_t)i * 2u);
    }
}

void mb_pdu_get_bits(const uint8_t *resp, uint16_t qty, uint8_t *out)
{
    unpack_bits(resp + PDU_OFFSET_READ_DATA, qty, out);
}

int mb_pdu_is_write(uint8_t function_code)
{
    return function_code == MODBUS_FUNC_WRITE_SINGLE_COIL ||
           function_code == MODBUS_FUNC_WRITE_SINGLE_REGISTER ||
           function_code == MODBUS_FUNC_WRITE_MULTIPLE_COILS ||
           function_code == MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS;
}

static size_t handle_read_registers(const mb_pdu_handlers_t *h, uint8_t fc,
                                    uint16_t addr, uint16_t qty, uint8_t *resp)
{
    uint16_t data[MODBUS_MAX_READ_REGISTERS];

    if (!h->read_registers) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_FUNCTION);
    }
    if (qty == 0u || qty > MODBUS_MAX_READ_REGISTERS) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_DATA_VALUE);
    }
    if (range_overflows(addr, qty)) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_DATA_ADDRESS);
    }

    int rc = h->read_registers(fc, addr, qty, data, h->userdata);
    if (rc != 0) {
        return build_exception(resp, fc, exception_from_callback_rc(rc));
    }

    resp[PDU_OFFSET_FC] = fc;
    resp[PDU_OFFSET_READ_BYTE_COUNT] = (uint8_t)(qty * 2u);
    for (uint16_t i = 0u; i < qty; i++) {
        put_u16(resp + PDU_OFFSET_READ_DATA + (size_t)i * 2u, data[i]);
    }
    return PDU_OFFSET_READ_DATA + (size_t)qty * 2u;
}

static size_t handle_read_bits(const mb_pdu_handlers_t *h, uint8_t fc,
                               uint16_t addr, uint16_t qty, uint8_t *resp)
{
    uint8_t bits[MODBUS_MAX_READ_BITS];

    if (!h->read_bits) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_FUNCTION);
    }
    if (qty == 0u || qty > MODBUS_MAX_READ_BITS) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_DATA_VALUE);
    }
    if (range_overflows(addr, qty)) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_DATA_ADDRESS);
    }

    memset(bits, 0, qty);
    int rc = h->read_bits(fc, addr, qty, bits, h->userdata);
    if (rc != 0) {
        return build_exception(resp, fc, exception_from_callback_rc(rc));
    }

    size_t byte_count = bit_byte_count(qty);
    resp[PDU_OFFSET_FC] = fc;
    resp[PDU_OFFSET_READ_BYTE_COUNT] = (uint8_t)byte_count;
    pack_bits(bits, qty, resp + PDU_OFFSET_READ_DATA);
    return PDU_OFFSET_READ_DATA + byte_count;
}

static size_t handle_write_single_coil(const mb_pdu_handlers_t *h, const uint8_t *req,
                                       uint8_t *resp)
{
    uint8_t fc = req[PDU_OFFSET_FC];
    uint16_t value = get_u16(req + PDU_OFFSET_QTY);
    uint8_t bit;

    if (!h->write_bits) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_FUNCTION);
    }
    if (value == MODBUS_COIL_ON) {
        bit = 1u;
    } else if (value == MODBUS_COIL_OFF) {
        bit = 0u;
    } else {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_DATA_VALUE);
    }

    int rc = h->write_bits(get_u16(req + PDU_OFFSET_ADDR), 1u, &bit, h->userdata);
    if (rc != 0) {
        return build_exception(resp, fc, exception_from_callback_rc(rc));
    }
    return build_write_echo(req, resp);
}

static size_t handle_write_single_register(const mb_pdu_handlers_t *h, const uint8_t *req,
                                           uint8_t *resp)
{
    uint8_t fc = req[PDU_OFFSET_FC];
    uint16_t value = get_u16(req + PDU_OFFSET_QTY);

    if (!h->write_registers) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_FUNCTION);
    }

    int rc = h->write_registers(get_u16(req + PDU_OFFSET_ADDR), 1u, &value, h->userdata);
    if (rc != 0) {
        return build_exception(resp, fc, exception_from_callback_rc(rc));
    }
    return build_write_echo(req, resp);
}

static size_t handle_write_multiple(const mb_pdu_handlers_t *h, const uint8_t *req,
                                    size_t req_len, uint8_t *resp)
{
    uint8_t fc = req[PDU_OFFSET_FC];
    int is_coils = (fc == MODBUS_FUNC_WRITE_MULTIPLE_COILS);
    uint16_t qty = get_u16(req + PDU_OFFSET_QTY);
    uint16_t max_qty = is_coils ? MODBUS_MAX_WRITE_BITS : MODBUS_MAX_WRITE_REGISTERS;

    if (is_coils ? !h->write_bits : !h->write_registers) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_FUNCTION);
    }
    if (req_len <= PDU_OFFSET_BYTE_COUNT || qty == 0u || qty > max_qty) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_DATA_VALUE);
    }

    size_t byte_count = is_coils ? bit_byte_count(qty) : (size_t)qty * 2u;
    if (req[PDU_OFFSET_BYTE_COUNT] != (uint8_t)byte_count ||
        req_len != PDU_OFFSET_WRITE_DATA + byte_count) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_DATA_VALUE);
    }

    uint16_t addr = get_u16(req + PDU_OFFSET_ADDR);
    if (range_overflows(addr, qty)) {
        return build_exception(resp, fc, MODBUS_EX_ILLEGAL_DATA_ADDRESS);
    }

    const uint8_t *payload = req + PDU_OFFSET_WRITE_DATA;
    int rc;

    if (is_coils) {
        uint8_t bits[MODBUS_MAX_WRITE_BITS];
        unpack_bits(payload, qty, bits);
        rc = h->write_bits(addr, qty, bits, h->userdata);
    } else {
        uint16_t data[MODBUS_MAX_WRITE_REGISTERS];
        for (uint16_t i = 0u; i < qty; i++) {
            data[i] = get_u16(payload + (size_t)i * 2u);
        }
        rc = h->write_registers(addr, qty, data, h->userdata);
    }

    if (rc != 0) {
        return build_exception(resp, fc, exception_from_callback_rc(rc));
    }
    return build_write_echo(req, resp);
}

size_t mb_pdu_handle_request(const mb_pdu_handlers_t *handlers,
                             const uint8_t *req, size_t req_len, uint8_t *resp)
{
    uint8_t fc = req[PDU_OFFSET_FC];

    switch (fc) {
        case MODBUS_FUNC_READ_COILS:
        case MODBUS_FUNC_READ_DISCRETE_INPUTS:
        case MODBUS_FUNC_READ_HOLDING_REGISTERS:
        case MODBUS_FUNC_READ_INPUT_REGISTERS:
        case MODBUS_FUNC_WRITE_SINGLE_COIL:
        case MODBUS_FUNC_WRITE_SINGLE_REGISTER:
        case MODBUS_FUNC_WRITE_MULTIPLE_COILS:
        case MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS:
            if (req_len < PDU_FIXED_REQ_LEN) {
                return build_exception(resp, fc, MODBUS_EX_ILLEGAL_DATA_VALUE);
            }
            break;
        default:
            return build_exception(resp, fc, MODBUS_EX_ILLEGAL_FUNCTION);
    }

    uint16_t addr = get_u16(req + PDU_OFFSET_ADDR);
    uint16_t qty = get_u16(req + PDU_OFFSET_QTY);

    switch (fc) {
        case MODBUS_FUNC_READ_COILS:
        case MODBUS_FUNC_READ_DISCRETE_INPUTS:
            return handle_read_bits(handlers, fc, addr, qty, resp);
        case MODBUS_FUNC_READ_HOLDING_REGISTERS:
        case MODBUS_FUNC_READ_INPUT_REGISTERS:
            return handle_read_registers(handlers, fc, addr, qty, resp);
        case MODBUS_FUNC_WRITE_SINGLE_COIL:
            return handle_write_single_coil(handlers, req, resp);
        case MODBUS_FUNC_WRITE_SINGLE_REGISTER:
            return handle_write_single_register(handlers, req, resp);
        default:
            return handle_write_multiple(handlers, req, req_len, resp);
    }
}
