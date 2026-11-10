// Host test for protocol.h: g++ -std=c++17 -I components tests/test_protocol.cpp -o /tmp/t && /tmp/t
#include <cassert>
#include <cstdio>
#include "zehnder_sx127x/protocol.h"

using namespace esphome::zehnder_sx127x;

// Frames captured on air (bytes after the 0x55 0x99 sync) from an RFZ remote talking to an RPMe fan on 2026-11-10,
// with the network address and IDs replaced (1A2B3C4D, fan 2C, remote A1). CRC and Manchester coding were
// recomputed for that with a separate Python script; layout and field values are as captured.
static const uint8_t RFZ_SET_SPEED_2[AIR_LEN] = {
    0xA9, 0x66, 0xA6, 0x65, 0xA5, 0x5A, 0x9A, 0x59, 0xAA, 0xA9, 0xAA, 0xAA, 0xAA, 0xA5, 0x66, 0xA9,
    0x55, 0x66, 0xAA, 0xA6, 0xAA, 0xA9, 0xAA, 0xA6, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x66, 0xA9, 0x5A, 0x6A};
static const uint8_t RFZ_ACK[AIR_LEN] = {
    0xA9, 0x66, 0xA6, 0x65, 0xA5, 0x5A, 0x9A, 0x59, 0xAA, 0xA9, 0xA6, 0x5A, 0xAA, 0xA5, 0x66, 0xA9,
    0x55, 0x66, 0xAA, 0x99, 0xAA, 0xA5, 0x9A, 0x55, 0xAA, 0xA6, 0xA6, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x69, 0x95, 0x66, 0x96};

int main() {
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  assert(crc16(check, 9) == 0x29B1);  // CRC-16/CCITT-FALSE check value

  uint8_t f[FRAME_LEN];
  assert(decode(RFZ_SET_SPEED_2, f));
  assert(network_of(f) == 0x1A2B3C4D && f[RX_TYPE] == 0x01 && f[RX_ID] == 0x00 && f[TX_TYPE] == 0x03 &&
         f[TX_ID] == 0xA1 && f[TTL_BYTE] == 0xFA && f[COMMAND] == CMD_SET_SPEED && f[PARAM_COUNT] == 1 &&
         f[PARAMS] == 2);
  assert(decode(RFZ_ACK, f) && f[RX_ID] == 0x2C && f[COMMAND] == 0x05);

  // build + encode must reproduce the RFZ's own frame bit for bit.
  uint8_t g[FRAME_LEN], air[AIR_LEN];
  uint8_t speed = 2;
  build(g, 0x1A2B3C4D, 0x00, 0xA1, CMD_SET_SPEED, &speed, 1);
  encode(g, air);
  for (size_t i = 0; i < AIR_LEN; i++)
    assert(air[i] == RFZ_SET_SPEED_2[i]);

  // Timer frame as the RFZ clock button sends it (seen on air: command 03, params 02 03 0A), and a query without params.
  const uint8_t timer[2] = {3, 10};
  build(g, 0x1A2B3C4D, 0x00, 0xA1, CMD_SET_TIMER, timer, 2);
  encode(g, air);
  assert(decode(air, f) && f[COMMAND] == 0x03 && f[PARAM_COUNT] == 2 && f[PARAMS] == 3 && f[PARAMS + 1] == 10);
  build(g, 0x1A2B3C4D, 0x2C, 0xA1, CMD_QUERY, nullptr, 0);
  encode(g, air);
  assert(decode(air, f) && f[COMMAND] == 0x10 && f[PARAM_COUNT] == 0 && f[PARAMS] == 0);

  // One flipped chip breaks Manchester; one flipped bit pair (valid chips) breaks the CRC.
  uint8_t bad[AIR_LEN];
  for (size_t i = 0; i < AIR_LEN; i++)
    bad[i] = RFZ_SET_SPEED_2[i];
  bad[20] ^= 0x40;
  assert(!decode(bad, f));
  bad[20] ^= 0x40 | 0xC0;  // 10 -> 01: still valid Manchester, other bit value
  assert(!decode(bad, f));

  puts("protocol: all tests passed");
}
