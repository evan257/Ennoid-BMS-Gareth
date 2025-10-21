// clang-format off
// AUTO-GENERATED - DO NOT EDIT
// Generated from sre.dbc
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef MIN_OF
#define MIN_OF(x,y) ((x) < (y) ? (x) : (y))
#endif
#ifndef MAX_OF
#define MAX_OF(x,y) ((x) > (y) ? (x) : (y))
#endif

/**
 * Extern dependency to use dbc_encode_and_send_*() API
 * This is designed such that the generated code does not depend on your CAN driver
 * @param argument_from_dbc_encode_and_send is a pass through argument from dbc_encode_and_send_*()
 * @returns the value returned from is returned from dbc_encode_and_send_*()
 */
extern bool dbc_send_can_message(void * argument_from_dbc_encode_and_send, uint32_t message_id, const uint8_t bytes[8], uint8_t dlc);

/// Missing in Action (MIA) structure
typedef struct {
  uint32_t mia_counter; ///< Counter used to track MIA
} dbc_mia_info_t;

/**
 * dbc_encode_*() API returns this instance to indicate message ID and DLC that should be used
 * dbc_decode_*() API checks against the provided header to make sure we are decoding the right message
 */
typedef struct {
  uint32_t message_id;  ///< CAN bus message identification
  uint8_t message_dlc;  ///< Data Length Code of the CAN message
} dbc_message_header_t;

// Message headers containing CAN message IDs and their DLCs; @see dbc_message_header_t
static const dbc_message_header_t dbc_header_BMS_Status                          = {     1568U, 1 };
static const dbc_message_header_t dbc_header_BMS_Pack_Info                       = {     1571U, 8 };
static const dbc_message_header_t dbc_header_BMS_Load_Info                       = {     1572U, 8 };
static const dbc_message_header_t dbc_header_BMS_Cell_Voltage_High_Low           = {     1573U, 8 };
static const dbc_message_header_t dbc_header_BMS_Cell_Voltage_Average            = {     1574U, 8 };
static const dbc_message_header_t dbc_header_BMS_Status_Soc                      = {     1575U, 8 };
static const dbc_message_header_t dbc_header_BMS_Temperature_Average             = {     1576U, 8 };
static const dbc_message_header_t dbc_header_BMS_Cell_Temperature                = {     1577U, 8 };
static const dbc_message_header_t dbc_header_BMS_Board_Temperature               = {     1584U, 8 };
static const dbc_message_header_t dbc_header_BMS_Load_Power                      = {     1585U, 4 };

/**
 * BMS_Status: Sent by 'BMS'
 *   - BMS Status and Fault Message
 */
typedef struct {
  dbc_mia_info_t mia_info;

  uint8_t BMS_Cell_Over_Voltage;
  uint8_t BMS_Cell_Under_Voltage;
  uint8_t BMS_Cell_Over_Temperature;
  uint8_t BMS_Board_Over_Temperature;
  uint8_t BMS_Power_Exceed_80kW;
} dbc_BMS_Status_s;

/**
 * BMS_Pack_Info: Sent by 'BMS'
 *   - Accumulator Pack Voltage and Current
 */
typedef struct {
  dbc_mia_info_t mia_info;

  float BMS_Pack_Voltage; // unit: V
  float BMS_Pack_Current; // unit: A
} dbc_BMS_Pack_Info_s;

/**
 * BMS_Load_Info: Sent by 'BMS'
 *   - Accumulator Load Voltage and Current
 */
typedef struct {
  dbc_mia_info_t mia_info;

  float BMS_Load_Voltage; // unit: V
  float BMS_Load_Current; // unit: A
} dbc_BMS_Load_Info_s;

/**
 * BMS_Cell_Voltage_High_Low: Sent by 'BMS'
 *   - Cell Voltage High and Low
 */
typedef struct {
  dbc_mia_info_t mia_info;

  float BMS_Cell_Voltage_High; // unit: V
  float BMS_Cell_Voltage_Low; // unit: V
} dbc_BMS_Cell_Voltage_High_Low_s;

/**
 * BMS_Cell_Voltage_Average: Sent by 'BMS'
 *   - Cell Voltage Average and Mismatch
 */
typedef struct {
  dbc_mia_info_t mia_info;

  float BMS_Cell_Voltage_Average; // unit: V
  float BMS_Cell_Voltage_Mismatch; // unit: V
} dbc_BMS_Cell_Voltage_Average_s;

/**
 * BMS_Status_Soc: Sent by 'BMS'
 *   - SOC Percentage and Battery Capacity
 */
typedef struct {
  dbc_mia_info_t mia_info;

  float BMS_SOC; // unit: %
  float BMS_Capacity; // unit: Ah
} dbc_BMS_Status_Soc_s;

/**
 * BMS_Temperature_Average: Sent by 'BMS'
 *   - Cell and BMS Board Temperature Average
 */
typedef struct {
  dbc_mia_info_t mia_info;

  float BMS_Cell_Temperature_Average; // unit: DegreeC
  float BMS_Board_Temperature_Average; // unit: DegreeC
} dbc_BMS_Temperature_Average_s;

/**
 * BMS_Cell_Temperature: Sent by 'BMS'
 *   - Cell Temperature High and Low
 */
typedef struct {
  dbc_mia_info_t mia_info;

  float BMS_Cell_Temperature_High; // unit: DegreeC
  float BMS_Cell_Temperature_Low; // unit: DegreeC
} dbc_BMS_Cell_Temperature_s;

/**
 * BMS_Board_Temperature: Sent by 'BMS'
 *   - Accumulator Load Power
 */
typedef struct {
  dbc_mia_info_t mia_info;

  float BMS_Board_Temperature_High; // unit: DegreeC
  float BMS_Board_Temperature_Low; // unit: DegreeC
} dbc_BMS_Board_Temperature_s;

/**
 * BMS_Load_Power: Sent by 'BMS'
 */
typedef struct {
  dbc_mia_info_t mia_info;

  float BMS_Load_Power; // unit: W
} dbc_BMS_Load_Power_s;

// -----------------------------------------------------------------------------
// When a message's MIA counter reaches this value
// corresponding MIA replacements occur
// -----------------------------------------------------------------------------
extern const uint32_t dbc_mia_threshold_BMS_Status;
extern const uint32_t dbc_mia_threshold_BMS_Pack_Info;
extern const uint32_t dbc_mia_threshold_BMS_Load_Info;
extern const uint32_t dbc_mia_threshold_BMS_Cell_Voltage_High_Low;
extern const uint32_t dbc_mia_threshold_BMS_Cell_Voltage_Average;
extern const uint32_t dbc_mia_threshold_BMS_Status_Soc;
extern const uint32_t dbc_mia_threshold_BMS_Temperature_Average;
extern const uint32_t dbc_mia_threshold_BMS_Cell_Temperature;
extern const uint32_t dbc_mia_threshold_BMS_Board_Temperature;
extern const uint32_t dbc_mia_threshold_BMS_Load_Power;

// -----------------------------------------------------------------------------
// User must define these externed instances in their code to use MIA functions
// These are copied during dbc_service_mia_*() when message MIA timeout occurs
// -----------------------------------------------------------------------------
extern const dbc_BMS_Status_s           dbc_mia_replacement_BMS_Status;
extern const dbc_BMS_Pack_Info_s        dbc_mia_replacement_BMS_Pack_Info;
extern const dbc_BMS_Load_Info_s        dbc_mia_replacement_BMS_Load_Info;
extern const dbc_BMS_Cell_Voltage_High_Low_s dbc_mia_replacement_BMS_Cell_Voltage_High_Low;
extern const dbc_BMS_Cell_Voltage_Average_s dbc_mia_replacement_BMS_Cell_Voltage_Average;
extern const dbc_BMS_Status_Soc_s       dbc_mia_replacement_BMS_Status_Soc;
extern const dbc_BMS_Temperature_Average_s dbc_mia_replacement_BMS_Temperature_Average;
extern const dbc_BMS_Cell_Temperature_s dbc_mia_replacement_BMS_Cell_Temperature;
extern const dbc_BMS_Board_Temperature_s dbc_mia_replacement_BMS_Board_Temperature;
extern const dbc_BMS_Load_Power_s       dbc_mia_replacement_BMS_Load_Power;


/**
 * Encode to transmit BMS_Status:
 *   Transmitter: 'BMS' with message ID 1568 composed of 1 bytes
 */
static inline dbc_message_header_t dbc_encode_BMS_Status(uint8_t bytes[8], const dbc_BMS_Status_s *message) {
  uint64_t raw = 0;
  memset(bytes, 0, 8);

  // Encode to raw 1-bit signal with scale=1
  raw = ((uint64_t)((((int64_t)message->BMS_Cell_Over_Voltage)))) & 0x01;
  bytes[0] |= (((uint8_t)(raw >>  0) & 0x01)     ); // 1 bits at B0

  // Encode to raw 1-bit signal with scale=1
  raw = ((uint64_t)((((int64_t)message->BMS_Cell_Under_Voltage)))) & 0x01;
  bytes[0] |= (((uint8_t)(raw >>  0) & 0x01) << 1); // 1 bits at B1

  // Encode to raw 1-bit signal with scale=1
  raw = ((uint64_t)((((int64_t)message->BMS_Cell_Over_Temperature)))) & 0x01;
  bytes[0] |= (((uint8_t)(raw >>  0) & 0x01) << 2); // 1 bits at B2

  // Encode to raw 1-bit signal with scale=1
  raw = ((uint64_t)((((int64_t)message->BMS_Board_Over_Temperature)))) & 0x01;
  bytes[0] |= (((uint8_t)(raw >>  0) & 0x01) << 3); // 1 bits at B3

  // Encode to raw 1-bit signal with scale=1
  raw = ((uint64_t)((((int64_t)message->BMS_Power_Exceed_80kW)))) & 0x01;
  bytes[0] |= (((uint8_t)(raw >>  0) & 0x01) << 4); // 1 bits at B4

  return dbc_header_BMS_Status;
}

/// @see dbc_encode_BMS_Status(); this is its variant to encode and call dbc_send_can_message() to send the message
static inline bool dbc_encode_and_send_BMS_Status(void *argument_for_dbc_send_can_message, const dbc_BMS_Status_s *message) {
  uint8_t bytes[8];
  const dbc_message_header_t header = dbc_encode_BMS_Status(bytes, message);
  return dbc_send_can_message(argument_for_dbc_send_can_message, header.message_id, bytes, header.message_dlc);
}

/**
 * Encode to transmit BMS_Pack_Info:
 *   Transmitter: 'BMS' with message ID 1571 composed of 8 bytes
 */
static inline dbc_message_header_t dbc_encode_BMS_Pack_Info(uint8_t bytes[8], const dbc_BMS_Pack_Info_s *message) {
  uint64_t raw = 0;
  memset(bytes, 0, 8);

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [0 -> 300.0]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Pack_Voltage,300.0f),0.0f)) / 0.1f) + 0.5f)) & 0xffffffff;

  bytes[0] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B0
  bytes[1] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B8
  bytes[2] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B16
  bytes[3] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B24

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [-3000.0 -> 3000.0]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Pack_Current,3000.0f),-3000.0f)) / 0.1f) + 0.5f)) & 0xffffffff;

  bytes[4] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B32
  bytes[5] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B40
  bytes[6] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B48
  bytes[7] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B56

  return dbc_header_BMS_Pack_Info;
}

/// @see dbc_encode_BMS_Pack_Info(); this is its variant to encode and call dbc_send_can_message() to send the message
static inline bool dbc_encode_and_send_BMS_Pack_Info(void *argument_for_dbc_send_can_message, const dbc_BMS_Pack_Info_s *message) {
  uint8_t bytes[8];
  const dbc_message_header_t header = dbc_encode_BMS_Pack_Info(bytes, message);
  return dbc_send_can_message(argument_for_dbc_send_can_message, header.message_id, bytes, header.message_dlc);
}

/**
 * Encode to transmit BMS_Load_Info:
 *   Transmitter: 'BMS' with message ID 1572 composed of 8 bytes
 */
static inline dbc_message_header_t dbc_encode_BMS_Load_Info(uint8_t bytes[8], const dbc_BMS_Load_Info_s *message) {
  uint64_t raw = 0;
  memset(bytes, 0, 8);

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [0 -> 300.0]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Load_Voltage,300.0f),0.0f)) / 0.1f) + 0.5f)) & 0xffffffff;
  
  bytes[0] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B0
  bytes[1] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B8
  bytes[2] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B16
  bytes[3] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B24

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [-3000.0 -> 3000.0]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Load_Current,3000.0f),-3000.0f)) / 0.1f) + 0.5f)) & 0xffffffff;

  bytes[4] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B32
  bytes[5] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B40
  bytes[6] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B48
  bytes[7] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B56

  return dbc_header_BMS_Load_Info;
}

/// @see dbc_encode_BMS_Load_Info(); this is its variant to encode and call dbc_send_can_message() to send the message
static inline bool dbc_encode_and_send_BMS_Load_Info(void *argument_for_dbc_send_can_message, const dbc_BMS_Load_Info_s *message) {
  uint8_t bytes[8];
  const dbc_message_header_t header = dbc_encode_BMS_Load_Info(bytes, message);
  return dbc_send_can_message(argument_for_dbc_send_can_message, header.message_id, bytes, header.message_dlc);
}

/**
 * Encode to transmit BMS_Cell_Voltage_High_Low:
 *   Transmitter: 'BMS' with message ID 1573 composed of 8 bytes
 */
static inline dbc_message_header_t dbc_encode_BMS_Cell_Voltage_High_Low(uint8_t bytes[8], const dbc_BMS_Cell_Voltage_High_Low_s *message) {
  uint64_t raw = 0;
  memset(bytes, 0, 8);

  // Encode to raw 32-bit SIGNED signal with scale=0.01 within range of [0 -> 6.55]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Cell_Voltage_High,6.55f),0.0f)) / 0.01f) + 0.5f)) & 0xffffffff;
  bytes[0] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B0
  bytes[1] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B8
  bytes[2] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B16
  bytes[3] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B24

  // Encode to raw 32-bit SIGNED signal with scale=0.01 within range of [0 -> 6.55]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Cell_Voltage_Low,6.55f),0.0f)) / 0.01f) + 0.5f)) & 0xffffffff;
  bytes[4] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B32
  bytes[5] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B40
  bytes[6] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B48
  bytes[7] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B56

  return dbc_header_BMS_Cell_Voltage_High_Low;
}

/// @see dbc_encode_BMS_Cell_Voltage_High_Low(); this is its variant to encode and call dbc_send_can_message() to send the message
static inline bool dbc_encode_and_send_BMS_Cell_Voltage_High_Low(void *argument_for_dbc_send_can_message, const dbc_BMS_Cell_Voltage_High_Low_s *message) {
  uint8_t bytes[8];
  const dbc_message_header_t header = dbc_encode_BMS_Cell_Voltage_High_Low(bytes, message);
  return dbc_send_can_message(argument_for_dbc_send_can_message, header.message_id, bytes, header.message_dlc);
}

/**
 * Encode to transmit BMS_Cell_Voltage_Average:
 *   Transmitter: 'BMS' with message ID 1574 composed of 8 bytes
 */
static inline dbc_message_header_t dbc_encode_BMS_Cell_Voltage_Average(uint8_t bytes[8], const dbc_BMS_Cell_Voltage_Average_s *message) {
  uint64_t raw = 0;
  memset(bytes, 0, 8);

  // Encode to raw 32-bit SIGNED signal with scale=0.01 within range of [0 -> 6.55]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Cell_Voltage_Average,6.55f),0.0f)) / 0.01f) + 0.5f)) & 0xffffffff;
  bytes[0] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B0
  bytes[1] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B8
  bytes[2] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B16
  bytes[3] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B24

  // Encode to raw 32-bit SIGNED signal with scale=0.01 within range of [0 -> 6.55]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Cell_Voltage_Mismatch,6.55f),0.0f)) / 0.01f) + 0.5f)) & 0xffffffff;
  bytes[4] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B32
  bytes[5] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B40
  bytes[6] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B48
  bytes[7] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B56

  return dbc_header_BMS_Cell_Voltage_Average;
}

/// @see dbc_encode_BMS_Cell_Voltage_Average(); this is its variant to encode and call dbc_send_can_message() to send the message
static inline bool dbc_encode_and_send_BMS_Cell_Voltage_Average(void *argument_for_dbc_send_can_message, const dbc_BMS_Cell_Voltage_Average_s *message) {
  uint8_t bytes[8];
  const dbc_message_header_t header = dbc_encode_BMS_Cell_Voltage_Average(bytes, message);
  return dbc_send_can_message(argument_for_dbc_send_can_message, header.message_id, bytes, header.message_dlc);
}

/**
 * Encode to transmit BMS_Status_Soc:
 *   Transmitter: 'BMS' with message ID 1575 composed of 8 bytes
 */
static inline dbc_message_header_t dbc_encode_BMS_Status_Soc(uint8_t bytes[8], const dbc_BMS_Status_Soc_s *message) {
  uint64_t raw = 0;
  memset(bytes, 0, 8);

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [0 -> 100]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_SOC,100.0f),0.0f)) / 0.1f) + 0.5f)) & 0xffffffff;
  bytes[0] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B0
  bytes[1] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B8
  bytes[2] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B16
  bytes[3] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B24

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [0 -> 30]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Capacity,30.0f),0.0f)) / 0.1f) + 0.5f)) & 0xffffffff;
  bytes[4] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B32
  bytes[5] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B40
  bytes[6] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B48
  bytes[7] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B56

  return dbc_header_BMS_Status_Soc;
}

/// @see dbc_encode_BMS_Status_Soc(); this is its variant to encode and call dbc_send_can_message() to send the message
static inline bool dbc_encode_and_send_BMS_Status_Soc(void *argument_for_dbc_send_can_message, const dbc_BMS_Status_Soc_s *message) {
  uint8_t bytes[8];
  const dbc_message_header_t header = dbc_encode_BMS_Status_Soc(bytes, message);
  return dbc_send_can_message(argument_for_dbc_send_can_message, header.message_id, bytes, header.message_dlc);
}

/**
 * Encode to transmit BMS_Temperature_Average:
 *   Transmitter: 'BMS' with message ID 1576 composed of 8 bytes
 */
static inline dbc_message_header_t dbc_encode_BMS_Temperature_Average(uint8_t bytes[8], const dbc_BMS_Temperature_Average_s *message) {
  uint64_t raw = 0;
  memset(bytes, 0, 8);

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [-100.0 -> 100.0]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Cell_Temperature_Average,100.0f),-100.0f)) / 0.1f) + 0.5f)) & 0xffffffff;

  bytes[0] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B0
  bytes[1] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B8
  bytes[2] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B16
  bytes[3] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B24

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [-100.0 -> 100.0]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Board_Temperature_Average,100.0f),-100.0f)) / 0.1f) + 0.5f)) & 0xffffffff;

  bytes[4] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B32
  bytes[5] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B40
  bytes[6] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B48
  bytes[7] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B56

  return dbc_header_BMS_Temperature_Average;
}

/// @see dbc_encode_BMS_Temperature_Average(); this is its variant to encode and call dbc_send_can_message() to send the message
static inline bool dbc_encode_and_send_BMS_Temperature_Average(void *argument_for_dbc_send_can_message, const dbc_BMS_Temperature_Average_s *message) {
  uint8_t bytes[8];
  const dbc_message_header_t header = dbc_encode_BMS_Temperature_Average(bytes, message);
  return dbc_send_can_message(argument_for_dbc_send_can_message, header.message_id, bytes, header.message_dlc);
}

/**
 * Encode to transmit BMS_Cell_Temperature:
 *   Transmitter: 'BMS' with message ID 1577 composed of 8 bytes
 */
static inline dbc_message_header_t dbc_encode_BMS_Cell_Temperature(uint8_t bytes[8], const dbc_BMS_Cell_Temperature_s *message) {
  uint64_t raw = 0;
  memset(bytes, 0, 8);

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [-99.9 -> 99.9]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Cell_Temperature_High,99.9f),-99.9f)) / 0.1f) + 0.5f)) & 0xffffffff;
  bytes[0] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B0
  bytes[1] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B8
  bytes[2] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B16
  bytes[3] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B24

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [-99.9 -> 99.9]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Cell_Temperature_Low,99.9f),-99.9f)) / 0.1f) + 0.5f)) & 0xffffffff;
  bytes[4] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B32
  bytes[5] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B40
  bytes[6] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B48
  bytes[7] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B56

  return dbc_header_BMS_Cell_Temperature;
}

/// @see dbc_encode_BMS_Cell_Temperature(); this is its variant to encode and call dbc_send_can_message() to send the message
static inline bool dbc_encode_and_send_BMS_Cell_Temperature(void *argument_for_dbc_send_can_message, const dbc_BMS_Cell_Temperature_s *message) {
  uint8_t bytes[8];
  const dbc_message_header_t header = dbc_encode_BMS_Cell_Temperature(bytes, message);
  return dbc_send_can_message(argument_for_dbc_send_can_message, header.message_id, bytes, header.message_dlc);
}

/**
 * Encode to transmit BMS_Board_Temperature:
 *   Transmitter: 'BMS' with message ID 1584 composed of 8 bytes
 */
static inline dbc_message_header_t dbc_encode_BMS_Board_Temperature(uint8_t bytes[8], const dbc_BMS_Board_Temperature_s *message) {
  uint64_t raw = 0;
  memset(bytes, 0, 8);

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [-100.0 -> 100.0]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Board_Temperature_High,100.0f),-100.0f)) / 0.1f) + 0.5f)) & 0xffffffff;
  
  bytes[0] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B0
  bytes[1] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B8
  bytes[2] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B16
  bytes[3] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B24

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [-100.0 -> 100.0]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Board_Temperature_Low,100.0f),-100.0f)) / 0.1f) + 0.5f)) & 0xffffffff;

  bytes[4] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B32
  bytes[5] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B40
  bytes[6] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B48
  bytes[7] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B56

  return dbc_header_BMS_Board_Temperature;
}

/// @see dbc_encode_BMS_Board_Temperature(); this is its variant to encode and call dbc_send_can_message() to send the message
static inline bool dbc_encode_and_send_BMS_Board_Temperature(void *argument_for_dbc_send_can_message, const dbc_BMS_Board_Temperature_s *message) {
  uint8_t bytes[8];
  const dbc_message_header_t header = dbc_encode_BMS_Board_Temperature(bytes, message);
  return dbc_send_can_message(argument_for_dbc_send_can_message, header.message_id, bytes, header.message_dlc);
}

/**
 * Encode to transmit BMS_Load_Power:
 *   Transmitter: 'BMS' with message ID 1585 composed of 4 bytes
 */
static inline dbc_message_header_t dbc_encode_BMS_Load_Power(uint8_t bytes[8], const dbc_BMS_Load_Power_s *message) {
  uint64_t raw = 0;
  memset(bytes, 0, 8);

  // Encode to raw 32-bit SIGNED signal with scale=0.1 within range of [-79999.9 -> 79999.9]
  raw = ((uint64_t)(((MAX_OF(MIN_OF(message->BMS_Load_Power,79999.9f),-79999.9f)) / 0.1f) + 0.5f)) & 0xffffffff;
  bytes[0] |= (((uint8_t)(raw >>  0) & 0xff)     ); // 8 bits at B0
  bytes[1] |= (((uint8_t)(raw >>  8) & 0xff)     ); // 8 bits at B8
  bytes[2] |= (((uint8_t)(raw >> 16) & 0xff)     ); // 8 bits at B16
  bytes[3] |= (((uint8_t)(raw >> 24) & 0xff)     ); // 8 bits at B24

  return dbc_header_BMS_Load_Power;
}

/// @see dbc_encode_BMS_Load_Power(); this is its variant to encode and call dbc_send_can_message() to send the message
static inline bool dbc_encode_and_send_BMS_Load_Power(void *argument_for_dbc_send_can_message, const dbc_BMS_Load_Power_s *message) {
  uint8_t bytes[8];
  const dbc_message_header_t header = dbc_encode_BMS_Load_Power(bytes, message);
  return dbc_send_can_message(argument_for_dbc_send_can_message, header.message_id, bytes, header.message_dlc);
}

/**
 * Decode received message BMS_Status: Sent by BMS
 *   Provided a dbc_message_header_t, this will attempt to decode the received message and return true upon success
 */
static inline bool dbc_decode_BMS_Status(dbc_BMS_Status_s *message, const dbc_message_header_t header, const uint8_t bytes[8]) {
  const bool success = true;

  if ((header.message_id != dbc_header_BMS_Status.message_id) || (header.message_dlc != dbc_header_BMS_Status.message_dlc)) {
    return !success;
  }

  uint64_t raw = 0;
  raw  = ((uint64_t)((bytes[0]) & 0x01)); // 1 bits from B0
  message->BMS_Cell_Over_Voltage = ((raw));

  raw  = ((uint64_t)((bytes[0] >> 1) & 0x01)); // 1 bits from B1
  message->BMS_Cell_Under_Voltage = ((raw));

  raw  = ((uint64_t)((bytes[0] >> 2) & 0x01)); // 1 bits from B2
  message->BMS_Cell_Over_Temperature = ((raw));

  raw  = ((uint64_t)((bytes[0] >> 3) & 0x01)); // 1 bits from B3
  message->BMS_Board_Over_Temperature = ((raw));

  raw  = ((uint64_t)((bytes[0] >> 4) & 0x01)); // 1 bits from B4
  message->BMS_Power_Exceed_80kW = ((raw));

  message->mia_info.mia_counter = 0;
  return success;
}

/**
 * Decode received message BMS_Pack_Info: Sent by BMS
 *   Provided a dbc_message_header_t, this will attempt to decode the received message and return true upon success
 */
static inline bool dbc_decode_BMS_Pack_Info(dbc_BMS_Pack_Info_s *message, const dbc_message_header_t header, const uint8_t bytes[8]) {
  const bool success = true;

  if ((header.message_id != dbc_header_BMS_Pack_Info.message_id) || (header.message_dlc != dbc_header_BMS_Pack_Info.message_dlc)) {
    return !success;
  }

  uint64_t raw = 0;
  raw  = ((uint64_t)((bytes[0]))); // 8 bits from B0
  raw |= ((uint64_t)((bytes[1]))) << 8; // 8 bits from B8
  raw |= ((uint64_t)((bytes[2]))) << 16; // 8 bits from B16
  raw |= ((uint64_t)((bytes[3]))) << 24; // 8 bits from B24
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Pack_Voltage = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Pack_Voltage = ((raw * 0.1f));
  }

  raw  = ((uint64_t)((bytes[4]))); // 8 bits from B32
  raw |= ((uint64_t)((bytes[5]))) << 8; // 8 bits from B40
  raw |= ((uint64_t)((bytes[6]))) << 16; // 8 bits from B48
  raw |= ((uint64_t)((bytes[7]))) << 24; // 8 bits from B56
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Pack_Current = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Pack_Current = ((raw * 0.1f));
  }

  message->mia_info.mia_counter = 0;
  return success;
}

/**
 * Decode received message BMS_Load_Info: Sent by BMS
 *   Provided a dbc_message_header_t, this will attempt to decode the received message and return true upon success
 */
static inline bool dbc_decode_BMS_Load_Info(dbc_BMS_Load_Info_s *message, const dbc_message_header_t header, const uint8_t bytes[8]) {
  const bool success = true;

  if ((header.message_id != dbc_header_BMS_Load_Info.message_id) || (header.message_dlc != dbc_header_BMS_Load_Info.message_dlc)) {
    return !success;
  }

  uint64_t raw = 0;
  raw  = ((uint64_t)((bytes[0]))); // 8 bits from B0
  raw |= ((uint64_t)((bytes[1]))) << 8; // 8 bits from B8
  raw |= ((uint64_t)((bytes[2]))) << 16; // 8 bits from B16
  raw |= ((uint64_t)((bytes[3]))) << 24; // 8 bits from B24
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Load_Voltage = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Load_Voltage = ((raw * 0.1f));
  }

  raw  = ((uint64_t)((bytes[4]))); // 8 bits from B32
  raw |= ((uint64_t)((bytes[5]))) << 8; // 8 bits from B40
  raw |= ((uint64_t)((bytes[6]))) << 16; // 8 bits from B48
  raw |= ((uint64_t)((bytes[7]))) << 24; // 8 bits from B56
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Load_Current = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Load_Current = ((raw * 0.1f));
  }

  message->mia_info.mia_counter = 0;
  return success;
}

/**
 * Decode received message BMS_Cell_Voltage_High_Low: Sent by BMS
 *   Provided a dbc_message_header_t, this will attempt to decode the received message and return true upon success
 */
static inline bool dbc_decode_BMS_Cell_Voltage_High_Low(dbc_BMS_Cell_Voltage_High_Low_s *message, const dbc_message_header_t header, const uint8_t bytes[8]) {
  const bool success = true;

  if ((header.message_id != dbc_header_BMS_Cell_Voltage_High_Low.message_id) || (header.message_dlc != dbc_header_BMS_Cell_Voltage_High_Low.message_dlc)) {
    return !success;
  }

  uint64_t raw = 0;
  raw  = ((uint64_t)((bytes[0]))); // 8 bits from B0
  raw |= ((uint64_t)((bytes[1]))) << 8; // 8 bits from B8
  raw |= ((uint64_t)((bytes[2]))) << 16; // 8 bits from B16
  raw |= ((uint64_t)((bytes[3]))) << 24; // 8 bits from B24
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Cell_Voltage_High = ((((UINT32_MAX << 31) | raw) * 0.01f));
  } else {
    message->BMS_Cell_Voltage_High = ((raw * 0.01f));
  }

  raw  = ((uint64_t)((bytes[4]))); // 8 bits from B32
  raw |= ((uint64_t)((bytes[5]))) << 8; // 8 bits from B40
  raw |= ((uint64_t)((bytes[6]))) << 16; // 8 bits from B48
  raw |= ((uint64_t)((bytes[7]))) << 24; // 8 bits from B56
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Cell_Voltage_Low = ((((UINT32_MAX << 31) | raw) * 0.01f));
  } else {
    message->BMS_Cell_Voltage_Low = ((raw * 0.01f));
  }

  message->mia_info.mia_counter = 0;
  return success;
}

/**
 * Decode received message BMS_Cell_Voltage_Average: Sent by BMS
 *   Provided a dbc_message_header_t, this will attempt to decode the received message and return true upon success
 */
static inline bool dbc_decode_BMS_Cell_Voltage_Average(dbc_BMS_Cell_Voltage_Average_s *message, const dbc_message_header_t header, const uint8_t bytes[8]) {
  const bool success = true;

  if ((header.message_id != dbc_header_BMS_Cell_Voltage_Average.message_id) || (header.message_dlc != dbc_header_BMS_Cell_Voltage_Average.message_dlc)) {
    return !success;
  }

  uint64_t raw = 0;
  raw  = ((uint64_t)((bytes[0]))); // 8 bits from B0
  raw |= ((uint64_t)((bytes[1]))) << 8; // 8 bits from B8
  raw |= ((uint64_t)((bytes[2]))) << 16; // 8 bits from B16
  raw |= ((uint64_t)((bytes[3]))) << 24; // 8 bits from B24
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Cell_Voltage_Average = ((((UINT32_MAX << 31) | raw) * 0.01f));
  } else {
    message->BMS_Cell_Voltage_Average = ((raw * 0.01f));
  }

  raw  = ((uint64_t)((bytes[4]))); // 8 bits from B32
  raw |= ((uint64_t)((bytes[5]))) << 8; // 8 bits from B40
  raw |= ((uint64_t)((bytes[6]))) << 16; // 8 bits from B48
  raw |= ((uint64_t)((bytes[7]))) << 24; // 8 bits from B56
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Cell_Voltage_Mismatch = ((((UINT32_MAX << 31) | raw) * 0.01f));
  } else {
    message->BMS_Cell_Voltage_Mismatch = ((raw * 0.01f));
  }

  message->mia_info.mia_counter = 0;
  return success;
}

/**
 * Decode received message BMS_Status_Soc: Sent by BMS
 *   Provided a dbc_message_header_t, this will attempt to decode the received message and return true upon success
 */
static inline bool dbc_decode_BMS_Status_Soc(dbc_BMS_Status_Soc_s *message, const dbc_message_header_t header, const uint8_t bytes[8]) {
  const bool success = true;

  if ((header.message_id != dbc_header_BMS_Status_Soc.message_id) || (header.message_dlc != dbc_header_BMS_Status_Soc.message_dlc)) {
    return !success;
  }

  uint64_t raw = 0;
  raw  = ((uint64_t)((bytes[0]))); // 8 bits from B0
  raw |= ((uint64_t)((bytes[1]))) << 8; // 8 bits from B8
  raw |= ((uint64_t)((bytes[2]))) << 16; // 8 bits from B16
  raw |= ((uint64_t)((bytes[3]))) << 24; // 8 bits from B24
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_SOC = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_SOC = ((raw * 0.1f));
  }

  raw  = ((uint64_t)((bytes[4]))); // 8 bits from B32
  raw |= ((uint64_t)((bytes[5]))) << 8; // 8 bits from B40
  raw |= ((uint64_t)((bytes[6]))) << 16; // 8 bits from B48
  raw |= ((uint64_t)((bytes[7]))) << 24; // 8 bits from B56
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Capacity = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Capacity = ((raw * 0.1f));
  }

  message->mia_info.mia_counter = 0;
  return success;
}

/**
 * Decode received message BMS_Temperature_Average: Sent by BMS
 *   Provided a dbc_message_header_t, this will attempt to decode the received message and return true upon success
 */
static inline bool dbc_decode_BMS_Temperature_Average(dbc_BMS_Temperature_Average_s *message, const dbc_message_header_t header, const uint8_t bytes[8]) {
  const bool success = true;

  if ((header.message_id != dbc_header_BMS_Temperature_Average.message_id) || (header.message_dlc != dbc_header_BMS_Temperature_Average.message_dlc)) {
    return !success;
  }

  uint64_t raw = 0;
  raw  = ((uint64_t)((bytes[0]))); // 8 bits from B0
  raw |= ((uint64_t)((bytes[1]))) << 8; // 8 bits from B8
  raw |= ((uint64_t)((bytes[2]))) << 16; // 8 bits from B16
  raw |= ((uint64_t)((bytes[3]))) << 24; // 8 bits from B24
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Cell_Temperature_Average = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Cell_Temperature_Average = ((raw * 0.1f));
  }

  raw  = ((uint64_t)((bytes[4]))); // 8 bits from B32
  raw |= ((uint64_t)((bytes[5]))) << 8; // 8 bits from B40
  raw |= ((uint64_t)((bytes[6]))) << 16; // 8 bits from B48
  raw |= ((uint64_t)((bytes[7]))) << 24; // 8 bits from B56
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Board_Temperature_Average = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Board_Temperature_Average = ((raw * 0.1f));
  }

  message->mia_info.mia_counter = 0;
  return success;
}

/**
 * Decode received message BMS_Cell_Temperature: Sent by BMS
 *   Provided a dbc_message_header_t, this will attempt to decode the received message and return true upon success
 */
static inline bool dbc_decode_BMS_Cell_Temperature(dbc_BMS_Cell_Temperature_s *message, const dbc_message_header_t header, const uint8_t bytes[8]) {
  const bool success = true;

  if ((header.message_id != dbc_header_BMS_Cell_Temperature.message_id) || (header.message_dlc != dbc_header_BMS_Cell_Temperature.message_dlc)) {
    return !success;
  }

  uint64_t raw = 0;
  raw  = ((uint64_t)((bytes[0]))); // 8 bits from B0
  raw |= ((uint64_t)((bytes[1]))) << 8; // 8 bits from B8
  raw |= ((uint64_t)((bytes[2]))) << 16; // 8 bits from B16
  raw |= ((uint64_t)((bytes[3]))) << 24; // 8 bits from B24
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Cell_Temperature_High = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Cell_Temperature_High = ((raw * 0.1f));
  }

  raw  = ((uint64_t)((bytes[4]))); // 8 bits from B32
  raw |= ((uint64_t)((bytes[5]))) << 8; // 8 bits from B40
  raw |= ((uint64_t)((bytes[6]))) << 16; // 8 bits from B48
  raw |= ((uint64_t)((bytes[7]))) << 24; // 8 bits from B56
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Cell_Temperature_Low = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Cell_Temperature_Low = ((raw * 0.1f));
  }

  message->mia_info.mia_counter = 0;
  return success;
}

/**
 * Decode received message BMS_Board_Temperature: Sent by BMS
 *   Provided a dbc_message_header_t, this will attempt to decode the received message and return true upon success
 */
static inline bool dbc_decode_BMS_Board_Temperature(dbc_BMS_Board_Temperature_s *message, const dbc_message_header_t header, const uint8_t bytes[8]) {
  const bool success = true;

  if ((header.message_id != dbc_header_BMS_Board_Temperature.message_id) || (header.message_dlc != dbc_header_BMS_Board_Temperature.message_dlc)) {
    return !success;
  }

  uint64_t raw = 0;
  raw  = ((uint64_t)((bytes[0]))); // 8 bits from B0
  raw |= ((uint64_t)((bytes[1]))) << 8; // 8 bits from B8
  raw |= ((uint64_t)((bytes[2]))) << 16; // 8 bits from B16
  raw |= ((uint64_t)((bytes[3]))) << 24; // 8 bits from B24
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Board_Temperature_High = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Board_Temperature_High = ((raw * 0.1f));
  }

  raw  = ((uint64_t)((bytes[4]))); // 8 bits from B32
  raw |= ((uint64_t)((bytes[5]))) << 8; // 8 bits from B40
  raw |= ((uint64_t)((bytes[6]))) << 16; // 8 bits from B48
  raw |= ((uint64_t)((bytes[7]))) << 24; // 8 bits from B56
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Board_Temperature_Low = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Board_Temperature_Low = ((raw * 0.1f));
  }

  message->mia_info.mia_counter = 0;
  return success;
}

/**
 * Decode received message BMS_Load_Power: Sent by BMS
 *   Provided a dbc_message_header_t, this will attempt to decode the received message and return true upon success
 */
static inline bool dbc_decode_BMS_Load_Power(dbc_BMS_Load_Power_s *message, const dbc_message_header_t header, const uint8_t bytes[8]) {
  const bool success = true;

  if ((header.message_id != dbc_header_BMS_Load_Power.message_id) || (header.message_dlc != dbc_header_BMS_Load_Power.message_dlc)) {
    return !success;
  }

  uint64_t raw = 0;
  raw  = ((uint64_t)((bytes[0]))); // 8 bits from B0
  raw |= ((uint64_t)((bytes[1]))) << 8; // 8 bits from B8
  raw |= ((uint64_t)((bytes[2]))) << 16; // 8 bits from B16
  raw |= ((uint64_t)((bytes[3]))) << 24; // 8 bits from B24
  if (raw & (1 << 31)) { // Check signed bit of the raw DBC signal and sign extend from 'raw'
    message->BMS_Load_Power = ((((UINT32_MAX << 31) | raw) * 0.1f));
  } else {
    message->BMS_Load_Power = ((raw * 0.1f));
  }

  message->mia_info.mia_counter = 0;
  return success;
}

// Do not use this function
static inline bool dbc_service_mia_for(dbc_mia_info_t *mia_info, const uint32_t increment_mia_by, const uint32_t threshold) {
  bool message_just_entered_mia = false;

  if (mia_info->mia_counter >= threshold) {
    // Message is already MIA
  } else {
    mia_info->mia_counter += increment_mia_by;
    message_just_entered_mia = (mia_info->mia_counter >= threshold);
  }

  return message_just_entered_mia;
}

static inline bool dbc_service_mia_BMS_Status(dbc_BMS_Status_s *message, const uint32_t increment_mia_by) {
  const bool message_just_entered_mia = dbc_service_mia_for(&(message->mia_info), increment_mia_by, dbc_mia_threshold_BMS_Status);

  if (message_just_entered_mia) {
    const dbc_mia_info_t previous_mia = message->mia_info;
    *message = dbc_mia_replacement_BMS_Status;
    message->mia_info = previous_mia;
  }

  return message_just_entered_mia;
}

static inline bool dbc_service_mia_BMS_Pack_Info(dbc_BMS_Pack_Info_s *message, const uint32_t increment_mia_by) {
  const bool message_just_entered_mia = dbc_service_mia_for(&(message->mia_info), increment_mia_by, dbc_mia_threshold_BMS_Pack_Info);

  if (message_just_entered_mia) {
    const dbc_mia_info_t previous_mia = message->mia_info;
    *message = dbc_mia_replacement_BMS_Pack_Info;
    message->mia_info = previous_mia;
  }

  return message_just_entered_mia;
}

static inline bool dbc_service_mia_BMS_Load_Info(dbc_BMS_Load_Info_s *message, const uint32_t increment_mia_by) {
  const bool message_just_entered_mia = dbc_service_mia_for(&(message->mia_info), increment_mia_by, dbc_mia_threshold_BMS_Load_Info);

  if (message_just_entered_mia) {
    const dbc_mia_info_t previous_mia = message->mia_info;
    *message = dbc_mia_replacement_BMS_Load_Info;
    message->mia_info = previous_mia;
  }

  return message_just_entered_mia;
}

static inline bool dbc_service_mia_BMS_Cell_Voltage_High_Low(dbc_BMS_Cell_Voltage_High_Low_s *message, const uint32_t increment_mia_by) {
  const bool message_just_entered_mia = dbc_service_mia_for(&(message->mia_info), increment_mia_by, dbc_mia_threshold_BMS_Cell_Voltage_High_Low);

  if (message_just_entered_mia) {
    const dbc_mia_info_t previous_mia = message->mia_info;
    *message = dbc_mia_replacement_BMS_Cell_Voltage_High_Low;
    message->mia_info = previous_mia;
  }

  return message_just_entered_mia;
}

static inline bool dbc_service_mia_BMS_Cell_Voltage_Average(dbc_BMS_Cell_Voltage_Average_s *message, const uint32_t increment_mia_by) {
  const bool message_just_entered_mia = dbc_service_mia_for(&(message->mia_info), increment_mia_by, dbc_mia_threshold_BMS_Cell_Voltage_Average);

  if (message_just_entered_mia) {
    const dbc_mia_info_t previous_mia = message->mia_info;
    *message = dbc_mia_replacement_BMS_Cell_Voltage_Average;
    message->mia_info = previous_mia;
  }

  return message_just_entered_mia;
}

static inline bool dbc_service_mia_BMS_Status_Soc(dbc_BMS_Status_Soc_s *message, const uint32_t increment_mia_by) {
  const bool message_just_entered_mia = dbc_service_mia_for(&(message->mia_info), increment_mia_by, dbc_mia_threshold_BMS_Status_Soc);

  if (message_just_entered_mia) {
    const dbc_mia_info_t previous_mia = message->mia_info;
    *message = dbc_mia_replacement_BMS_Status_Soc;
    message->mia_info = previous_mia;
  }

  return message_just_entered_mia;
}

static inline bool dbc_service_mia_BMS_Temperature_Average(dbc_BMS_Temperature_Average_s *message, const uint32_t increment_mia_by) {
  const bool message_just_entered_mia = dbc_service_mia_for(&(message->mia_info), increment_mia_by, dbc_mia_threshold_BMS_Temperature_Average);

  if (message_just_entered_mia) {
    const dbc_mia_info_t previous_mia = message->mia_info;
    *message = dbc_mia_replacement_BMS_Temperature_Average;
    message->mia_info = previous_mia;
  }

  return message_just_entered_mia;
}

static inline bool dbc_service_mia_BMS_Cell_Temperature(dbc_BMS_Cell_Temperature_s *message, const uint32_t increment_mia_by) {
  const bool message_just_entered_mia = dbc_service_mia_for(&(message->mia_info), increment_mia_by, dbc_mia_threshold_BMS_Cell_Temperature);

  if (message_just_entered_mia) {
    const dbc_mia_info_t previous_mia = message->mia_info;
    *message = dbc_mia_replacement_BMS_Cell_Temperature;
    message->mia_info = previous_mia;
  }

  return message_just_entered_mia;
}

static inline bool dbc_service_mia_BMS_Board_Temperature(dbc_BMS_Board_Temperature_s *message, const uint32_t increment_mia_by) {
  const bool message_just_entered_mia = dbc_service_mia_for(&(message->mia_info), increment_mia_by, dbc_mia_threshold_BMS_Board_Temperature);

  if (message_just_entered_mia) {
    const dbc_mia_info_t previous_mia = message->mia_info;
    *message = dbc_mia_replacement_BMS_Board_Temperature;
    message->mia_info = previous_mia;
  }

  return message_just_entered_mia;
}

static inline bool dbc_service_mia_BMS_Load_Power(dbc_BMS_Load_Power_s *message, const uint32_t increment_mia_by) {
  const bool message_just_entered_mia = dbc_service_mia_for(&(message->mia_info), increment_mia_by, dbc_mia_threshold_BMS_Load_Power);

  if (message_just_entered_mia) {
    const dbc_mia_info_t previous_mia = message->mia_info;
    *message = dbc_mia_replacement_BMS_Load_Power;
    message->mia_info = previous_mia;
  }

  return message_just_entered_mia;
}

// clang-format off
