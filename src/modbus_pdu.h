/**
 * @file modbus_pdu.h
 * @brief Internal transport-independent Modbus PDU encode / decode / dispatch.
 *
 * A PDU is the function code plus its data, without the TCP MBAP header or
 * the RTU slave address / CRC.  TCP and RTU clients and servers share this
 * layer so every function code is implemented once.  Not part of the public API.
 */

#ifndef MODBUS_PDU_H
#define MODBUS_PDU_H

#include <stddef.h>
#include <stdint.h>

#include "modbus_defines.h"

/** mb_pdu_check_response(): response is malformed or does not match the request. */
#define MB_PDU_ERR_FRAME (-1)

/** Server-side application callbacks for one request dispatch. */
typedef struct mb_pdu_handlers {
    int (*read_registers)(uint8_t function_code, uint16_t addr, uint16_t qty,
        uint16_t *out, void *userdata);
    int (*write_registers)(uint16_t addr, uint16_t qty, const uint16_t *data, void *userdata);
    int (*read_bits)(uint8_t function_code, uint16_t addr, uint16_t qty,
        uint8_t *out, void *userdata);
    int (*write_bits)(uint16_t addr, uint16_t qty, const uint8_t *bits, void *userdata);
    void *userdata;
} mb_pdu_handlers_t;

/** FC01-FC04 request.  @return PDU length (5). */
size_t mb_pdu_build_read(uint8_t *pdu, uint8_t function_code, uint16_t addr, uint16_t qty);

/** FC05 / FC06 request.  @return PDU length (5). */
size_t mb_pdu_build_write_single(uint8_t *pdu, uint8_t function_code,
    uint16_t addr, uint16_t value);

/** FC16 request; @p pdu must hold MODBUS_MAX_PDU_LENGTH bytes.  @return PDU length. */
size_t mb_pdu_build_write_registers(uint8_t *pdu, uint16_t addr, uint16_t qty,
    const uint16_t *data);

/** FC15 request (one byte per coil, nonzero = ON).  @return PDU length. */
size_t mb_pdu_build_write_coils(uint8_t *pdu, uint16_t addr, uint16_t qty,
    const uint8_t *values);

/**
 * Validate a response PDU against the request PDU that produced it.
 * @return 0 if valid, a positive Modbus exception code, or MB_PDU_ERR_FRAME.
 */
int mb_pdu_check_response(const uint8_t *req, const uint8_t *resp, size_t resp_len);

/** Extract @p qty registers from a validated FC03 / FC04 response. */
void mb_pdu_get_registers(const uint8_t *resp, uint16_t qty, uint16_t *out);

/** Extract @p qty bits (0 / 1 per byte) from a validated FC01 / FC02 response. */
void mb_pdu_get_bits(const uint8_t *resp, uint16_t qty, uint8_t *out);

/** Nonzero for function codes a slave executes on broadcast (05, 06, 15, 16). */
int mb_pdu_is_write(uint8_t function_code);

/**
 * Dispatch one request PDU to @p handlers and build the response PDU
 * (normal or exception) into @p resp, which must hold MODBUS_MAX_PDU_LENGTH bytes.
 * @return response PDU length (always > 0).
 */
size_t mb_pdu_handle_request(const mb_pdu_handlers_t *handlers,
    const uint8_t *req, size_t req_len, uint8_t *resp);

#endif /* MODBUS_PDU_H */
