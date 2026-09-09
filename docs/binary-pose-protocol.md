# Raspberry Pi binary control protocol v2

The authoritative constants live in `protocol/rpi_binary_protocol.json`.
Run `python tools/generate_rpi_protocol.py` after changing the schema and
`python tools/generate_rpi_protocol.py --check` in CI. Generated C/Python files
are checked in so the STM32 build never depends on Python.
The generated numeric tables are in
[`binary-protocol-reference.generated.md`](binary-protocol-reference.generated.md).

## Negotiation and recovery

The ASCII host protocol is version 4. After the existing
`HOST LINK RPI -> STOP -> WORK -> STATUS -> OPS -> CAN -> PID/LIMIT` self-check,
the Pi sends `HOST BINARY START`. Firmware replies:

```text
# HOST BINARY READY VERSION=2 CAPS=0x0000003F
```

The Pi requires the advertised QUERY, speed-limit, binary-telemetry, async-TX,
session-recovery and atomic-goal capabilities before releasing its motion gate.
On every serial open, the single writer first sends `SESSION_PROBE`. If an old
Pi process left an armed/active binary session, the bridge sends `STOP_ALL`,
does not heartbeat, waits for the firmware session watchdog to expire, and only
then publishes `SerialConnected` for a new ASCII self-check. Old goals and
responses are never replayed.

## Frame transport

USART1 remains 115200 8-N-1. Multi-byte values are little-endian.

| Field | Size | Encoding |
| --- | ---: | --- |
| SOF | 2 | `A5 5A` |
| version | 1 | `02` |
| message type | 1 | command `10`, response `11`, event `22`, telemetry `23` |
| sequence | 1 | modulo 256; never reused while its request is pending |
| payload length | 2 | maximum 64 |
| payload | N | message-specific |
| CRC-16/CCITT-FALSE | 2 | over version through payload |

CRC uses polynomial `0x1021`, initial `0xFFFF`, no reflection/final xor. The
`123456789` vector is `0x29B1`.

Binary `PING` refreshes the existing 1.5 s RPI watchdog. A UART error is latched
in the ISR and the next main-loop pass immediately stops motion and clears the
session. Session timeout also stops motion and returns to ASCII self-check.

## Commands and responses

The first command payload byte is the opcode.

| Opcode | Data after opcode |
| --- | --- |
| `01` | PING: empty |
| `02` | STOP_ALL: empty |
| `03` | SESSION_PROBE: empty |
| `80` | legacy SET_POSE_GOAL: `<IiiiI>` |
| `81` | CANCEL_POSE_GOAL: `<I>` goal id |
| `82` | QUERY_POSE_GOAL: empty |
| `83` | standalone SET_SPEED_LIMITS: `<ii>` |
| `84` | SET_POSE_GOAL_WITH_LIMITS: `<IiiiIii>` |

Goal fields are goal id, x mm, y mm, yaw mrad and timeout ms. Limit fields are
linear µm/s and yaw µrad/s. Runtime mission code uses opcode `84`: STM32 first
validates every target and limit field, then accepts the limits and goal as one
transaction. Rejected limits therefore cannot start a goal using old values.

A response begins with request sequence, opcode and status: `00 OK`,
`01 UNKNOWN_COMMAND`, `02 INVALID_LENGTH`, `03 INVALID_ARGUMENT`, `04 BUSY`, or
`05 INTERNAL_ERROR`.

QUERY data is `<IBiiiHBB>`: goal id, pose state, x/y mm, yaw mrad, fault reason,
robot mode and host-link type. SESSION_PROBE data is active, armed, host link,
pose state, goal id, capability bitmap and wire version (13 bytes).

The Pi keeps a bounded pending table with per-command deadlines. Sequence wrap
skips live requests. Reconnect clears the table. A full table rejects ordinary
commands locally but still permits untracked STOP transmission. Pose commands
are never automatically retried.

Each Navigator QUERY stores the Pi-side owner goal alongside the pending
sequence. A QUERY response is consumed only when that owner is still the active
goal and the Navigator is waiting for a start, motion or cancellation
reconciliation. The firmware-reported goal id alone is not used for ownership,
because a delayed response may legitimately report IDLE after its old goal has
already ended.

## STOP receive barrier

STOP_ALL has a dedicated RX slot. Receiving it discards the ordinary RX FIFO;
ordinary frames arriving while STOP is pending are rejected and counted as
dropped. After STOP is dequeued, new requests can be accepted. Clients must wait
for its response before submitting new work. STOP does not change the wire
version, mode or negotiation state; the Pi emergency latch separately prevents
new motion until explicitly released.

The Pi invalidates commands already removed from its TX queue when an emergency
stop clears that queue, including binary goals and commands held across a later
release. Application shutdown uses the serial thread to write STOP and waits
for an acknowledgement with a bounded deadline before closing. Write completion
and protocol acknowledgement are reported separately; neither is motor feedback.

## Events

| Code | Event data |
| --- | --- |
| `10` | POSE_STARTED: `<I>` goal id |
| `11` | POSE_REACHED: `<Iiiiii>` goal id, pose and final errors |
| `12` | POSE_CANCELLED: `<I>` goal id |
| `13` | MOTION_FAULT: `<IH>` goal id and reason |

The Pi ignores stale goal ids. If a start/cancel event is missing, QUERY may
reconcile the state; it never authorizes replay or restoration of an old goal.

## Telemetry and TX scheduling

The first processed binary frame after negotiation activates the session and
enables alternating 20 Hz wheel/pose samples. Until then, telemetry remains off,
so ASCII `@W`/`@P` lines cannot leak into the binary decoder during the switch.
Wheel and pose payloads are 39 bytes: a one-byte type followed by `<IH8i>`
(tick, sequence and eight scaled values). Wheel RPM uses tenths; pose position
uses mm, yaw uses mrad, and planned velocity uses µm/s or µrad/s. Link
statistics use type `03` plus `<6I>` for tick, RX drops, critical TX drops,
replaced telemetry, CRC errors and UART errors.

STM32 binary TX is a single DMA writer with three bounded classes:

- urgent safety latch for motion faults;
- eight-entry control response/event FIFO (including STOP responses);
- one latest-value telemetry slot.

Urgent traffic is popped first, then control, then telemetry. Telemetry can be
replaced and never blocks safety/control. DMA owns a stable active buffer until
the TX-complete callback releases it. ASCII COM/TUNE output remains unchanged
outside an active binary session.

## Hardware validation still required

The portable tests cover framing, queue policy, reconnect invalidation and
Navigator failure paths, but the following HAL-dependent behavior must be
verified on the STM32 and Raspberry Pi hardware:

- inject USART1 overrun/framing errors and disconnect the USB/UART link during
  motion; verify zero-speed output on the next control pass and a latched fault;
- saturate binary TX while moving; verify fault traffic precedes control traffic,
  control precedes telemetry, and the DMA-owned buffer is not overwritten;
- power-cycle or kill/restart the Pi process during a binary session; verify the
  cold-session probe sends STOP and no old goal is restored.
