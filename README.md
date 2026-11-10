# esphome-zehnder-sx127x

ESPHome fan component for Zehnder ventilation units with RF control (tested: Zehnder RPMe roof fan with RFZ
remotes), using a cheap **868 MHz SX1276 board** instead of an nRF905 module.

The Zehnder RF protocol runs on a Nordic nRF905 at 868.4 MHz. Most nRF905 modules for sale are tuned for 433 MHz
and only reach 1-2 m at 868 MHz. An SX1276 board built for 868 MHz (for example the LILYGO T3 V1.6.1 868 MHz,
about €22) can speak the same frames in FSK packet mode. The Manchester coding and the CRC are done in software.
It uses ESPHome's official `sx127x` component.

What it does:
- a fan entity with speed 1-3, like the RFZ remote;
- every speed change is resent until the fan confirms it (command 0x07 with the requested speed), then closed
  with 0x05, like the RFZ does;
- the fan is queried every `query_interval`. Speed changes made with an RFZ are picked up as well, because
  the fan's replies to the remote are heard too;
- a `confirmed` binary sensor that goes off when the fan has not answered for 3 query intervals.

No pairing is needed. The fan answered queries and speed commands from a remote ID it had never paired with.

## Tested

10 November 2026: Zehnder RPMe, an RFZ remote, LILYGO T3 V1.6.1 868 MHz with its stock antenna, ESPHome
2026.8.2. The board sits in a garage, two concrete floors and a cavity wall away from the roof fan. The fan's
replies arrive at about -71 dBm.

Seen in tests:
- **Queries:** 10 of 11 single queries (one frame each, no retry) were answered. With this component (resend and
  0x05 ack), every query and speed change in our tests was confirmed at the first attempt.
- **Speed changes:** 1 → 2 → 1, four seconds apart. Before the 0x05 ack was added, a second change within
  seconds was answered with the old speed and ignored.

Not tested: speed 3 from this component, "off" (sends speed 0, like the original firmware), the timer (command
0x03), and ComfoAir units.

## Setup

1. Flash [example.yaml](example.yaml) with logger `level: VERBOSE`. Press a button on your RFZ next to the
   board. Frames like these show up:
   `Frame network 0x1A2B3C4D 01:00 <- 03:A1 ttl FA command 02 ...` (the RFZ)
   `Frame network 0x1A2B3C4D 03:A1 <- 01:2C ttl FA command 07 ...` (the fan's reply)
   That gives you `network_id` (0x1A2B3C4D) and `fan_id` (2C). Pick a `remote_id` your RFZs don't use.
2. **Crystal offset.** Cheap boards can be tens of kHz off. Ours was about 20 kHz low, so every sender looked
   +20 kHz away, and the AFC cannot settle within the 0.2 ms nRF905 preamble. To measure it: let the receiver use
   AFC for a moment and read RegAfc (0x1B/0x1C, 61 Hz steps) after each frame. Or try `frequency` values in 10 kHz
   steps and keep the one where every RFZ press decodes. Set `frequency` to 868.4 MHz plus the offset the senders
   show (ours: +20 kHz, so 868420000). The component turns AFC off.
3. Set `logger` back to `INFO`. Logging every frame over wifi costs milliseconds, and the fan sends its frames
   about 25 ms apart in bursts.

## How it works

On air, after the SX127x preamble and sync (`0x55` + `0x55 0x99`, which is the 10-bit nRF905 preamble
`1111110101` in Manchester), come 44 bytes: 22 bytes Manchester-coded (bit 1 → chips 01, bit 0 → chips 10, MSB
first).

| Bytes | Content |
|---|---|
| 0-3 | network address |
| 4-7 | rx type, rx ID, tx type, tx ID (type 01 = fan, 03 = remote) |
| 8 | TTL (0xFA; every repeat by the fan counts it down) |
| 9 | command: 02 set speed, 05 ack, 07 fan settings, 10 query |
| 10 | parameter count |
| 11-19 | parameters (07: speed, voltage %, timer, ...) |
| 20-21 | CRC-16/CCITT-FALSE over bytes 0-19 |

The component also works around two receiver issues:
- A noise spike above `rx_floor` triggers the SX127x, which then hunts for a sync word and stays deaf until a random
  match. The component restarts the receiver when no sync follows within 3 ms.
- AFC is turned off (see crystal offset above).

`tests/test_protocol.cpp` checks the coding against frames captured from a real RFZ (address and IDs
anonymized):

```bash
g++ -std=c++17 -I components tests/test_protocol.cpp -o test_protocol && ./test_protocol
```

## Legal

868.0-868.6 MHz is a licence-free band in Europe (ERC/REC 70-03): at most 25 mW e.r.p. and 1 % duty cycle. +10
dBm with a 2-5 dBi antenna is 10-19 mW e.r.p. A query every 60 s is far below 1 % (one frame is 3.7 ms).

## Credits

The protocol knowledge comes from [Sanderhuisman/ESPHome-Zehnder-RF](https://github.com/Sanderhuisman/ESPHome-Zehnder-RF)
and [JoooostB/esphome-zehnder-comfoair-rf](https://github.com/JoooostB/esphome-zehnder-comfoair-rf), and the SX1276-as-nRF905 idea from
[lyusupov/arduino-lmic](https://github.com/lyusupov/arduino-lmic). This component is new code.
