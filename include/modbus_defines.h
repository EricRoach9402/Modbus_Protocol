/**
 * @file modbus_defines.h
 * @brief Modbus function codes, TCP/RTU framing constants, and exception codes.
 */

#ifndef MODBUS_PROTOCOL_DEFINES_H
#define MODBUS_PROTOCOL_DEFINES_H

/* ── Modbus function codes ─────────────────────────────────────────────── */

#define MODBUS_FUNC_READ_COILS                  0x01u
#define MODBUS_FUNC_READ_DISCRETE_INPUTS        0x02u
#define MODBUS_FUNC_READ_HOLDING_REGISTERS      0x03u
#define MODBUS_FUNC_READ_INPUT_REGISTERS        0x04u
#define MODBUS_FUNC_WRITE_SINGLE_COIL           0x05u
#define MODBUS_FUNC_WRITE_SINGLE_REGISTER       0x06u
#define MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS    0x10u

/* ── Modbus TCP framing constants ──────────────────────────────────────── */

/** Number of MBAP header bytes at the start of every Modbus TCP ADU. */
#define MODBUS_TCP_MBAP_HEADER_LEN              6u

/** Alias kept for backward compatibility. */
#define MODBUS_TCP_MBAP_PREFIX_LEN              MODBUS_TCP_MBAP_HEADER_LEN

/**
 * MBAP Length field value for a standard 5-byte PDU:
 *   unit_id (1) + FC (1) + start_addr (2) + qty_or_value (2) = 6 bytes.
 * Applies to FC01–FC06 and other single-range request types.
 */
#define MODBUS_TCP_MBAP_LEN_STANDARD_REQ        6u

/** Total byte length of a standard read / write-single request ADU (MBAP + PDU). */
#define MODBUS_TCP_MIN_READ_REQ_LENGTH          12u

/** Backward-compatible alias. */
#define MODBUS_TCP_MIN_LENGTH                   MODBUS_TCP_MIN_READ_REQ_LENGTH

/** Maximum Modbus TCP ADU size in bytes (Modbus spec + MBAP overhead). */
#define MODBUS_TCP_MAX_ADU_LENGTH               260u

/* -- Modbus RTU framing constants --------------------------------------- */

/** Slave address used by Modbus RTU broadcast frames. Slaves do not reply. */
#define MODBUS_RTU_BROADCAST_ADDRESS            0x00u

/** CRC16 byte count appended at the end of every Modbus RTU ADU. */
#define MODBUS_RTU_CRC_LEN                      2u

/** Total byte length of a standard read / write-single RTU request ADU. */
#define MODBUS_RTU_MIN_REQ_LENGTH               8u

/** Maximum Modbus RTU ADU size in bytes (address + PDU + CRC). */
#define MODBUS_RTU_MAX_ADU_LENGTH               256u

/** Default RTU response / receive timeout for serial transactions. */
#define MODBUS_RTU_DEFAULT_TIMEOUT_MS           1000u

/* ── Modbus register count limits (Modbus specification) ───────────────── */

/** Maximum number of registers readable in one FC03 / FC04 request. */
#define MODBUS_MAX_READ_REGISTERS               125u

/** Maximum number of registers writable in one FC16 request. */
#define MODBUS_MAX_WRITE_REGISTERS              123u

/* ── Modbus exception handling ─────────────────────────────────────────── */

/** Bit OR'd with the function code in an exception response. */
#define MODBUS_EXCEPTION_FLAG                   0x80u

#define MODBUS_EX_ILLEGAL_FUNCTION              0x01u
#define MODBUS_EX_ILLEGAL_DATA_ADDRESS          0x02u
#define MODBUS_EX_ILLEGAL_DATA_VALUE            0x03u
#define MODBUS_EX_SERVER_DEVICE_FAILURE         0x04u

/* ── Library tunables ──────────────────────────────────────────────────── */

#define MB_TCP_DEFAULT_BUFFER_SIZE              2048
#define MB_TCP_MAX_CLIENTS                      10

#endif /* MODBUS_PROTOCOL_DEFINES_H */
