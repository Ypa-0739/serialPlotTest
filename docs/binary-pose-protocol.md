# Raspberry Pi binary pose protocol v1

This protocol removes the ambiguous completion matching of the legacy ASCII
`POSE SET` flow. Every pose transaction carries a non-zero `goal_id`, and every
terminal event returns that same identifier.

## Transport

USART1 remains at 115200 8-N-1. A frame is:

| Field | Size | Encoding |
| --- | ---: | --- |
| SOF | 2 | `A5 5A` |
| version | 1 | `01` |
| message type | 1 | command `10`, response `11`, event `22` |
| sequence | 1 | wraps modulo 256 |
| payload length | 2 | little-endian, maximum 64 |
| payload | N | message-specific |
| CRC-16/CCITT-FALSE | 2 | little-endian; calculated over version through payload |

CRC parameters are polynomial `0x1021`, initial value `0xFFFF`, no reflection,
and no final xor. The standard `123456789` vector produces `0x29B1`.

The firmware keeps the ASCII parser available for PC/TUNE and RPI startup
self-check. Binary frames cannot claim the host link. The Pi must first complete
the existing `HOST LINK RPI -> STOP -> WORK -> STATUS -> OPS -> CAN -> PID/LIMIT`
self-check and then send the ASCII command `HOST BINARY START`. The firmware
answers `# HOST BINARY READY` only while idle with fresh OPS, healthy CAN and
enabled chassis motors. The serial bridge switches codecs after that reply.

While binary mode is active, `printf` output is suppressed so text cannot corrupt
binary frames. Binary `PING` refreshes the existing host watchdog. Link timeout
stops all motion, clears the old goal and returns USART1 to the ASCII self-check
state; reconnect never resumes a previous goal.

## Commands

The first payload byte is the opcode.

| Opcode | Command data |
| --- | --- |
| `01` | PING: empty |
| `02` | STOP_ALL: empty |
| `80` | SET_POSE_GOAL: `<IiiiI` = goal id, x mm, y mm, yaw mrad, timeout ms |
| `81` | CANCEL_POSE_GOAL: `<I` goal id |
| `82` | QUERY_POSE_GOAL: empty |
| `83` | SET_SPEED_LIMITS: `<ii` = linear µm/s, yaw µrad/s |

Goal timeouts must be 1–60000 ms. Existing OPS freshness, CAN health, 10 m
relative travel, and host-heartbeat safety gates are still enforced by STM32.
Only one goal may move at a time.

The ISR places ordinary frames in a four-entry bounded FIFO. `STOP_ALL` uses a
separate urgent latch, remains admissible when that FIFO is full, and is always
processed before ordinary frames.

A response payload starts with request sequence, opcode, and status. Status is
`00 OK`, `01 UNKNOWN_COMMAND`, `02 INVALID_LENGTH`, `03 INVALID_ARGUMENT`,
`04 BUSY`, or `05 INTERNAL_ERROR`. Query data is `<IBiiiH`: goal id, state,
current x/y/yaw, and fault reason.

## Events

| Code | Event data |
| --- | --- |
| `10` | POSE_STARTED: `<I` goal id |
| `11` | POSE_REACHED: `<Iiiiii` goal id, x, y, yaw, position error, yaw error |
| `12` | POSE_CANCELLED: `<I` goal id |
| `13` | MOTION_FAULT: `<IH` goal id, reason |

The Pi must ignore an event whose `goal_id` is not its currently active goal.
It must keep sending PING while moving and must never automatically replay a
goal after reconnecting.
