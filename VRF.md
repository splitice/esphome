# Midea VRF / Heat Pump Control Protocol Comparison

This document compares the command protocol transmitted by the local ESPHome
`midea_xye` implementation in
[`air_conditioner.cpp`](esphome/components/midea_xye/air_conditioner.cpp) with
[`midea_ac_vrf.ino`](https://github.com/manunited10/midea-vrf-controller/blob/main/src/midea_ac_vrf.ino)
from `manunited10/midea-vrf-controller`.

The focus here is the bytes transmitted for control, not the MQTT or ESPHome
entity layers except where those layers determine the command payload.

Both implementations use a 4800 baud RS485 UART. The local README configures
4800 baud for `midea_xye`; the D1D2 sketch uses `Serial2.begin(4800,
SERIAL_8N1, ...)`.

## Executive Summary

The two implementations control related Midea RS485 systems, but they do not
transmit the same wire protocol.

The local `midea_xye` component transmits fixed 16-byte XYE-style frames:

```text
AA CMD SERVER CLIENT FROM CLIENT DATA... CMD_INV CRC8 55
```

`midea_ac_vrf.ino` transmits variable-length D1D2 VRF frames:

```text
AA 23 RECV_LO RECV_HI SRC_LO SRC_HI PAYLEN PAYLOAD... CRC16_LO CRC16_HI 55 FE
```

The most important differences are:

| Area | Local `midea_xye` | `midea_ac_vrf.ino` |
| --- | --- | --- |
| Frame length | Fixed 16-byte TX frames | Variable, `11 + payload_length` bytes |
| Frame terminator | TX ends at `55`; RX stores through `55` and ignores extra bytes such as trailing `FE` | TX/RX formally ends with `55 FE` |
| Command byte | Multiple commands: `C0`, `C3`, `C4`, `C6` | Single command byte `23` for poll, status, and set payloads |
| Checksum | 8-bit inverted sum over the fixed frame, excluding checksum byte | Modbus CRC16 over `CMD` through final payload byte |
| Addressing | Hardcoded `SERVER_ID = 0`, `CLIENT_ID = 0` | Explicit 16-bit receiver and sender addresses |
| State writes | `C3` sends a complete desired state in fixed byte positions | `23` sends small field-update payloads |
| Setpoint encoding | Celsius integer byte, or optional Fahrenheit encoding | `24.0 deg C = 0x80`, each 0.5 deg C is one count |
| Fan encoding | Auto/high/medium/low only | Auto plus fixed levels `1` through `7` |
| Swing control | One vertical swing flag in the `C3` flags byte | Packed vertical and horizontal swing positions |
| Extra local controls | Follow-me and static pressure via `C6` | Not implemented in the sketch |

## Local `midea_xye` Transmit Protocol

### Fixed TX Frame

The local component builds every transmitted frame with `prepareTXData()`:

| Byte | Name | Meaning |
| --- | --- | --- |
| `0` | Preamble | Always `AA` |
| `1` | Command | `C0`, `C3`, `C4`, `C6`, etc. |
| `2` | Server ID | Hardcoded `00` |
| `3` | Client ID | Hardcoded `00` |
| `4` | Direction | `FROM_CLIENT`, currently `00` |
| `5` | Client ID | Hardcoded `00` |
| `6` | Data 0 | Mode for `C3` |
| `7` | Data 1 | Fan for `C3` |
| `8` | Data 2 | Target temperature for `C3`; static pressure nibble for `C6` |
| `9` | Data 3 | Timer start placeholder for `C3` |
| `10` | Data 4 | Timer stop placeholder for `C3`; subcommand for `C6` |
| `11` | Data 5 | Flags for `C3`; temperature for follow-me `C6` |
| `12` | Data 6 | Currently `00` |
| `13` | Command inverse | `FF - command` |
| `14` | Checksum | `FF - (sum(frame bytes except byte 14) & FF)` |
| `15` | Prologue / end marker | Always `55` |

The checksum includes the `AA` preamble and the final `55` byte, but excludes
the checksum byte itself.

### RX Handling and the Missing Transmitted `FE`

The local implementation defines `RX_LEN = 32` and validates byte `31` as
`55`. `sendRecv()` reads all bytes currently available from UART, but only
stores the first 32 bytes into `RXData`. If more than 32 bytes arrive, it logs a
warning and continues using the first 32.

That means a response shaped like:

```text
AA ... CRC 55 FE
```

can be accepted if the first 32 bytes end at `55` and the trailing `FE` is
available during the same UART drain window; the `FE` is counted as an extra
byte and ignored. Local TX, however, writes exactly `TX_LEN` bytes, so it
transmits:

```text
AA ... CRC 55
```

and does not append `FE`. This is a specific wire-level difference from the
D1D2 sketch. If the far end ignores a trailing `FE`, local control may still
work. If exact controller emulation is desired, appending `FE` after byte `15`
would be the protocol experiment to test; based on the current checksum code,
that byte would sit outside the existing 16-byte checksum calculation.

### Local Command Cycle

The local component uses a small state machine:

| State | Command | Purpose |
| --- | --- | --- |
| `STATE_SEND_C0` | `C0` | Main status query |
| `STATE_SEND_C4` | `C4` | Extended query; also used for Fahrenheit setpoint readback and static pressure readback |
| `STATE_SEND_C3` | `C3` | Set mode, fan, setpoint, timers, and flags |
| `STATE_SEND_C6` | `C6` | Follow-me update or static pressure command |

After `C0`, it schedules `C4`; after `C4`, it schedules `C3`; after `C3`, it
schedules `C6`; after `C6`, it returns to `C0`. A climate control call queues
or immediately schedules `C3`.

`C4` is built by first calling `setACParams()` and then changing byte `1` from
`C3` to `C4` and byte `13` from `3C` to `3B`. The checksum remains valid
because `command + command_inverse` is still `FF`.

## Local `C3` Set Command

`C3` is the main local control command. It sends a complete desired climate
state in one fixed frame.

### `C3` Mode Byte

Local mode is byte `6`:

| ESPHome mode | Byte |
| --- | --- |
| Off | `00` |
| Auto / heat-cool | `80` |
| Fan only | `81` |
| Dry | `82` |
| Heat | `84` |
| Cool | `88` |

Power is not a separate field. `00` means off; any active mode byte implies
the unit should be on in that mode.

### `C3` Fan Byte

Local fan is byte `7`:

| ESPHome fan mode | Byte |
| --- | --- |
| Auto | `80` |
| High | `01` |
| Medium | `02` |
| Low | `03` |

Fan mode is forced to auto when the climate mode is heat-cool / auto.

### `C3` Setpoint Byte

Local target temperature is byte `8`.

For Celsius operation, the transmitted byte is the integer setpoint. Heat mode
uses `ceilf()`, while other modes use `floorf()`. For example:

| Setpoint | Mode | Byte |
| --- | --- | --- |
| 22.0 deg C | Heat | `16` |
| 24.0 deg C | Cool | `18` |

For Fahrenheit operation, the component converts Celsius to Fahrenheit, adds
`0x87`, then applies the same heat/non-heat rounding rule:

```text
encoded = round_policy((celsius * 9 / 5 + 32) + 0x87)
```

### `C3` Flags Byte

Local mode flags are byte `11`:

| Function | Bit / byte value |
| --- | --- |
| Sleep / eco preset | `01` |
| Boost / aux heat preset | `02` |
| Swing enabled | `04` |
| Vent flag constant | `88`, defined but not actively used |

The current code comments say swing setting is not known to work reliably.

### `C3` Timer Bytes

Bytes `9` and `10` are reserved in comments for timer start and timer stop, but
the current `setACParams()` implementation leaves them as `00`.

Timer encoding is bitwise:

| Bit | Duration |
| --- | --- |
| `01` | 15 minutes |
| `02` | 30 minutes |
| `04` | 1 hour |
| `08` | 2 hours |
| `10` | 4 hours |
| `20` | 8 hours |
| `40` | 16 hours |

## Local `C6` Commands

`C6` is used for local controls that are outside the normal `C3` state write.

### Follow-Me

`do_follow_me()` transmits:

| Byte | First follow-me | Later follow-me | Meaning |
| --- | --- | --- | --- |
| `1` | `C6` | `C6` | Command |
| `10` | `06` | `02` | Follow-me subcommand / phase |
| `11` | Temperature | Temperature | Rounded Celsius follow-me temperature |

The component only queues this while the unit is not off. It preserves the last
follow-me temperature and refreshes it after a `C3` mode change if possible.

### Static Pressure

`set_static_pressure()` transmits:

| Byte | Value | Meaning |
| --- | --- | --- |
| `1` | `C6` | Command |
| `8` | `10 | static_pressure` | Static pressure value, low nibble `0` to `15` |
| `10` | `04` | Static pressure subcommand |
| `11` | Last follow-me temperature | Preserved from previous follow-me state |

The local implementation only sends static pressure while the unit is off.

## `midea_ac_vrf.ino` D1D2 Transmit Protocol

### Variable D1D2 Frame

The sketch builds every transmitted frame with `buildAndSend()`:

| Byte | Name | Meaning |
| --- | --- | --- |
| `0` | Start | Always `AA` |
| `1` | Command | Always `23` in this sketch |
| `2` | Receiver low | Target indoor unit ID |
| `3` | Receiver high | Always `00` in the sketch |
| `4` | Sender low | Controller sender ID, `64` |
| `5` | Sender high | `00` |
| `6` | Payload length | Number of payload bytes |
| `7..n` | Payload | Poll or field update |
| `n+1` | CRC low | Modbus CRC16 low byte |
| `n+2` | CRC high | Modbus CRC16 high byte |
| `n+3` | End 1 | `55` |
| `n+4` | End 2 | `FE` |

The CRC covers byte `1` through the final payload byte. It does not cover the
start byte, the CRC bytes, or `55 FE`.

The sketch uses explicit target IDs and is written for multiple indoor units on
the same D1D2 bus. It polls units round-robin and prioritizes queued control
commands over polling.

### D1D2 Poll Command

Polling is a `23` frame with a single-byte payload:

```text
65
```

For target unit `01`, sender `0064`, the transmitted frame is:

```text
AA 23 01 00 64 00 01 65 D6 98 55 FE
```

### D1D2 Field-Update Payloads

All control writes in the sketch use command byte `23`; the payload identifies
which field to change.

| User action | Payload shape | Meaning |
| --- | --- | --- |
| Power / mode | `01 00 VALUE` | `VALUE = power_nibble | mode_nibble` |
| Fan | `01 01 VALUE` | `VALUE = 80` for auto, otherwise `01` to `07` |
| Temperature | `01 03 T 04 T 02 T` | Writes cooling, heating, and active setpoint fields |
| Swing | `01 09 S 0D 0F` | `S = horizontal << 4 | vertical` |

The leading `01` appears to act as a write/update group marker in every command
payload used by the sketch.

## D1D2 Command Semantics

### D1D2 Power and Mode

Power and mode are packed into one payload value:

```text
payload = 01 00 VALUE
VALUE   = power_nibble | mode_nibble
```

Power nibble:

| Power | Nibble |
| --- | --- |
| Off | `00` |
| On | `40` |

Mode nibble:

| Mode | Nibble |
| --- | --- |
| Fan only | `01` |
| Cool | `02` |
| Heat | `03` |
| Dry | `06` |

Turning off retains the last active mode nibble. For example, if the last mode
was cool, off is sent as:

```text
01 00 02
```

and cool on is sent as:

```text
01 00 42
```

### D1D2 Fan

Fan is:

```text
payload = 01 01 VALUE
```

| Fan | Byte |
| --- | --- |
| Auto | `80` |
| Fixed level 1 through 7 | `01` through `07` |

The numeric values overlap partly with the local XYE fan bytes, but the meaning
is not identical. Local XYE only exposes low/medium/high, while the D1D2 sketch
exposes seven discrete fixed levels.

### D1D2 Temperature

Temperature is encoded as:

```text
encoded = (setpoint_deg_c - 24.0) * 2 + 128
decoded = 24.0 + (encoded - 128) / 2
```

Examples:

| Setpoint | Byte |
| --- | --- |
| 17.0 deg C | `72` |
| 22.0 deg C | `7C` |
| 24.0 deg C | `80` |
| 25.5 deg C | `83` |
| 30.0 deg C | `8C` |

The sketch sends the same encoded byte into three fields:

```text
01 03 T 04 T 02 T
```

Its comments identify these as cooling setpoint, heating setpoint, and active
setpoint fields, respectively.

### D1D2 Swing

Swing is:

```text
payload = 01 09 S 0D 0F
S       = horizontal << 4 | vertical
```

Each axis uses:

| Swing position | Nibble |
| --- | --- |
| Auto | `0E` |
| Fixed 1 through 5 | `01` through `05` |

Examples:

| Swing command | Packed `S` |
| --- | --- |
| Horizontal auto, vertical auto | `EE` |
| Horizontal auto, vertical 3 | `E3` |
| Horizontal 2, vertical auto | `2E` |

## Command-by-Command Comparison

### Power and Mode

Local XYE:

```text
C3 byte 6 = complete operating mode
```

| State | Local byte |
| --- | --- |
| Off | `00` |
| Auto | `80` |
| Fan | `81` |
| Dry | `82` |
| Heat | `84` |
| Cool | `88` |

D1D2:

```text
23 payload = 01 00 (power_nibble | mode_nibble)
```

| State | D1D2 value |
| --- | --- |
| Off, retaining cool | `02` |
| Cool on | `42` |
| Heat on | `43` |
| Dry on | `46` |
| Fan on | `41` |

These encodings are incompatible. The only superficial overlap is that both
use a single byte to represent the resulting power/mode state.

### Temperature

Local XYE sends a direct Celsius integer in `C3` byte `8` unless Fahrenheit mode
is enabled. D1D2 sends an offset half-degree byte and repeats it across three
setpoint fields.

| Setpoint | Local Celsius byte | D1D2 byte |
| --- | --- | --- |
| 22.0 deg C | `16` | `7C` |
| 24.0 deg C | `18` | `80` |
| 25.5 deg C | `19` or `1A`, depending local mode/rounding | `83` |

D1D2 supports 0.5 deg C setpoints directly. Local XYE's Celsius transmit path
rounds to an integer before sending.

### Fan

Local XYE:

```text
C3 byte 7 = 80 auto, 01 high, 02 medium, 03 low
```

D1D2:

```text
23 payload = 01 01 VALUE
VALUE = 80 auto, or 01..07 fixed level
```

Although `80` means auto in both, fixed fan levels should not be copied between
implementations without confirming the target system's meaning. Local `01`
means high; D1D2 `01` is just level 1 in the sketch.

### Swing

Local XYE only sends a swing enable bit:

```text
C3 byte 11 bit 04
```

D1D2 sends both axes and position/auto data:

```text
23 payload = 01 09 (horizontal << 4 | vertical) 0D 0F
```

D1D2 swing control is therefore much more specific. The local implementation
does not have an equivalent horizontal swing command and does not encode fixed
louver positions.

### Presets, Aux Heat, Eco, Timers

Local XYE has flag bits for sleep/eco and boost/aux heat in `C3` byte `11`.
The D1D2 sketch does not implement equivalent MQTT commands or payloads.

Local XYE has timer encoding helpers, but the current `C3` transmitter leaves
timer bytes at zero. The D1D2 sketch does not implement timer control.

### Follow-Me and Static Pressure

Local XYE implements both with `C6`:

```text
follow-me first:       C6 byte 10 = 06, byte 11 = room temp
follow-me subsequent:  C6 byte 10 = 02, byte 11 = room temp
static pressure:       C6 byte 8 = 10 | pressure, byte 10 = 04
```

The D1D2 sketch has no corresponding command payloads.

## Example Transmitted Frames

The following examples are generated from the code paths in each implementation.
D1D2 examples assume target unit `01` and sender `0064`.

### Local XYE Examples

Query:

```text
C0 query:
AA C0 00 00 00 00 00 00 00 00 00 00 00 3F 01 55
```

Cool, auto fan, 24 deg C:

```text
C3 set:
AA C3 00 00 00 00 88 80 18 00 00 00 00 3C E1 55
```

Heat, high fan, 22 deg C:

```text
C3 set:
AA C3 00 00 00 00 84 01 16 00 00 00 00 3C 66 55
```

Follow-me first update, 23 deg C:

```text
C6 follow-me:
AA C6 00 00 00 00 00 00 00 00 06 17 00 39 E4 55
```

Static pressure `5`, with last follow-me temperature `23`:

```text
C6 static pressure:
AA C6 00 00 00 00 00 00 15 00 04 17 00 39 D1 55
```

Notice that none of these local TX examples include a trailing `FE`.

### D1D2 Examples

Poll unit `01`:

```text
AA 23 01 00 64 00 01 65 D6 98 55 FE
```

Cool on:

```text
payload: 01 00 42
frame:   AA 23 01 00 64 00 03 01 00 42 9F 9C 55 FE
```

Off while retaining cool as the last active mode:

```text
payload: 01 00 02
frame:   AA 23 01 00 64 00 03 01 00 02 9E 6C 55 FE
```

Set temperature to 24.0 deg C:

```text
payload: 01 03 80 04 80 02 80
frame:   AA 23 01 00 64 00 07 01 03 80 04 80 02 80 96 33 55 FE
```

Fan auto:

```text
payload: 01 01 80
frame:   AA 23 01 00 64 00 03 01 01 80 1F 9D 55 FE
```

Fan level 3:

```text
payload: 01 01 03
frame:   AA 23 01 00 64 00 03 01 01 03 5E 3C 55 FE
```

Swing horizontal auto, vertical 3:

```text
payload: 01 09 E3 0D 0F
frame:   AA 23 01 00 64 00 05 01 09 E3 0D 0F 7F 15 55 FE
```

## Practical Porting Notes

The D1D2 command payloads cannot be dropped into the local XYE `C3` frame. The
frame grammar, checksum, addressing model, and field encodings are all
different.

To make the local component speak the D1D2 VRF protocol, the main work would be:

1. Add a D1D2 frame builder with receiver/source addresses, payload length,
   Modbus CRC16, and `55 FE` termination.
2. Replace local `C3` complete-state writes with `23` field-update payloads.
3. Replace local temperature encoding with the D1D2 half-degree offset encoding.
4. Model power and mode as a packed power/mode byte.
5. Expand fan support from low/medium/high to fixed levels `1` through `7`.
6. Add horizontal and positional vertical swing support if desired.
7. Decide whether local XYE should append a post-frame `FE` for better
   controller emulation, while preserving the existing 16-byte checksum
   behavior unless bus captures show otherwise.

If the goal is only to tighten local XYE compatibility, the lowest-risk protocol
experiment is the terminator difference: transmit the existing 16-byte frame
unchanged, then append `FE` as a 17th byte. The local receiver already tolerates
that style of trailing byte on incoming frames when it is drained along with the
main response, by ignoring bytes beyond `RX_LEN`.
