#pragma once
// Zehnder RF frames as sent by an nRF905 (RFZ remote, RPMe/ComfoAir fans), built and checked in software so an
// SX127x in FSK packet mode can send and receive them. No ESPHome dependencies, so it can be tested on a PC.
//
// On air: preamble + sync (sent/matched by the SX127x) followed by 22 bytes, Manchester-coded to 44:
//   4 bytes network address | 16 bytes payload | CRC-16/CCITT-FALSE over address + payload, MSB first.
// Payload: rx_type, rx_id, tx_type, tx_id, ttl, command, parameter count, parameters...

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace esphome {
namespace zehnder_sx127x {

static constexpr size_t FRAME_LEN = 22;
static constexpr size_t AIR_LEN = 2 * FRAME_LEN;

static constexpr uint8_t TYPE_MAIN_UNIT = 0x01;
static constexpr uint8_t TYPE_REMOTE = 0x03;
static constexpr uint8_t TTL = 0xFA;
static constexpr uint8_t CMD_SET_SPEED = 0x02;
static constexpr uint8_t CMD_SET_TIMER = 0x03;     // params: speed, minutes (RFZ clock button: 3, 10 or 30)
static constexpr uint8_t CMD_FAN_SETTINGS = 0x07;  // fan -> remote: count, speed, voltage %, timer, ...
static constexpr uint8_t CMD_QUERY = 0x10;

enum Field : uint8_t { RX_TYPE = 4, RX_ID, TX_TYPE, TX_ID, TTL_BYTE, COMMAND, PARAM_COUNT, PARAMS };

inline uint16_t crc16(const uint8_t *p, size_t len) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    c ^= (uint16_t) p[i] << 8;
    for (int k = 0; k < 8; k++)
      c = (c & 0x8000) ? (uint16_t) ((c << 1) ^ 0x1021) : (uint16_t) (c << 1);
  }
  return c;
}

// Manchester as the nRF905 does it: bit 1 -> chips 01, bit 0 -> chips 10, MSB first.
inline void encode(const uint8_t *frame, uint8_t *air) {
  for (size_t i = 0; i < FRAME_LEN; i++) {
    uint16_t m = 0;
    for (int k = 7; k >= 0; k--)
      m = (m << 2) | (((frame[i] >> k) & 1) ? 0x1 : 0x2);
    air[2 * i] = m >> 8;
    air[2 * i + 1] = m & 0xFF;
  }
}

// False on an invalid chip pair or a CRC mismatch.
inline bool decode(const uint8_t *air, uint8_t *frame) {
  for (size_t i = 0; i < FRAME_LEN; i++) {
    uint16_t m = ((uint16_t) air[2 * i] << 8) | air[2 * i + 1];
    uint8_t b = 0;
    for (int k = 7; k >= 0; k--) {
      uint8_t chips = (m >> (2 * k)) & 0x3;
      if (chips == 0x1)
        b = (b << 1) | 1;
      else if (chips == 0x2)
        b = b << 1;
      else
        return false;
    }
    frame[i] = b;
  }
  return crc16(frame, FRAME_LEN - 2) == (((uint16_t) frame[20] << 8) | frame[21]);
}

// network: the 4 address bytes in air order, e.g. 0x1A2B3C4D is sent as 1A 2B 3C 4D.
inline void build(uint8_t *frame, uint32_t network, uint8_t rx_id, uint8_t tx_id, uint8_t command,
                  const uint8_t *params, uint8_t count) {
  memset(frame, 0, FRAME_LEN);
  for (int i = 0; i < 4; i++)
    frame[i] = network >> (24 - 8 * i);
  frame[RX_TYPE] = TYPE_MAIN_UNIT;
  frame[RX_ID] = rx_id;
  frame[TX_TYPE] = TYPE_REMOTE;
  frame[TX_ID] = tx_id;
  frame[TTL_BYTE] = TTL;
  frame[COMMAND] = command;
  if (count > 9)
    count = 9;
  frame[PARAM_COUNT] = count;
  if (count > 0)
    memcpy(frame + PARAMS, params, count);
  uint16_t c = crc16(frame, FRAME_LEN - 2);
  frame[20] = c >> 8;
  frame[21] = c & 0xFF;
}

inline uint32_t network_of(const uint8_t *frame) {
  return ((uint32_t) frame[0] << 24) | ((uint32_t) frame[1] << 16) | ((uint32_t) frame[2] << 8) | frame[3];
}

}  // namespace zehnder_sx127x
}  // namespace esphome
