# -*- coding: utf-8 -*-
"""STM32/Raspberry Pi binary pose-goal protocol.

This transport is intentionally separate from :mod:`app.protocol`: the legacy
ASCII codec remains available for COM/TUNE and startup self-check sessions.
The Raspberry Pi switches the shared UART only after the firmware explicitly
acknowledges ``HOST BINARY START``. All multi-byte fields are little-endian.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct


SOF = b"\xA5\x5A"
VERSION = 1
MAX_PAYLOAD = 64
_HEADER = struct.Struct("<BBBH")
_CRC = struct.Struct("<H")


class MessageType(IntEnum):
    COMMAND = 0x10
    RESPONSE = 0x11
    EVENT = 0x22


class Command(IntEnum):
    PING = 0x01
    STOP_ALL = 0x02
    SET_POSE_GOAL = 0x80
    CANCEL_POSE_GOAL = 0x81
    QUERY_POSE_GOAL = 0x82
    SET_SPEED_LIMITS = 0x83


class ResponseStatus(IntEnum):
    OK = 0x00
    UNKNOWN_COMMAND = 0x01
    INVALID_LENGTH = 0x02
    INVALID_ARGUMENT = 0x03
    BUSY = 0x04
    INTERNAL_ERROR = 0x05


class EventCode(IntEnum):
    POSE_STARTED = 0x10
    POSE_REACHED = 0x11
    POSE_CANCELLED = 0x12
    MOTION_FAULT = 0x13


class PoseState(IntEnum):
    IDLE = 0x00
    MOVING = 0x02
    REACHED = 0x03
    CANCELLED = 0x04
    FAULT = 0x05


class MotionFault(IntEnum):
    UNSPECIFIED = 0x0000
    OPS9_LOST = 0x0001
    HOST_LOST = 0x0002
    CAN_FAULT = 0x0003
    OUT_OF_BOUNDS = 0x0004
    TIMEOUT = 0x0005
    INTERNAL_ERROR = 0x00FF


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF
                if crc & 0x8000
                else (crc << 1) & 0xFFFF
            )
    return crc


@dataclass(frozen=True)
class Frame:
    message_type: MessageType
    sequence: int
    payload: bytes = b""

    def encode(self) -> bytes:
        if not 0 <= self.sequence <= 0xFF:
            raise ValueError("sequence must fit in uint8")
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError(f"payload exceeds {MAX_PAYLOAD} bytes")
        body = _HEADER.pack(
            VERSION, int(self.message_type), self.sequence, len(self.payload)
        ) + self.payload
        return SOF + body + _CRC.pack(crc16_ccitt(body))


def decode_frame(raw: bytes) -> Frame:
    if len(raw) < 9 or raw[:2] != SOF:
        raise ValueError("invalid binary frame prefix or length")
    version, message_type, sequence, payload_length = _HEADER.unpack_from(raw, 2)
    if version != VERSION or payload_length > MAX_PAYLOAD:
        raise ValueError("unsupported version or payload length")
    expected = 9 + payload_length
    if len(raw) != expected:
        raise ValueError(f"frame length must be {expected} bytes")
    body = raw[2:-2]
    received_crc = _CRC.unpack_from(raw, len(raw) - 2)[0]
    if received_crc != crc16_ccitt(body):
        raise ValueError("CRC mismatch")
    try:
        kind = MessageType(message_type)
    except ValueError as exc:
        raise ValueError(f"unknown message type 0x{message_type:02X}") from exc
    return Frame(kind, sequence, raw[7:-2])


class FrameDecoder:
    """Incremental decoder that recovers after noise and corrupt frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []
        while True:
            marker = self._buffer.find(SOF)
            if marker < 0:
                keep = 1 if self._buffer.endswith(SOF[:1]) else 0
                self.discarded_bytes += len(self._buffer) - keep
                del self._buffer[: len(self._buffer) - keep]
                break
            if marker:
                self.discarded_bytes += marker
                del self._buffer[:marker]
            if len(self._buffer) < 7:
                break
            version, _, _, payload_length = _HEADER.unpack_from(self._buffer, 2)
            if version != VERSION or payload_length > MAX_PAYLOAD:
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            frame_length = 9 + payload_length
            if len(self._buffer) < frame_length:
                break
            candidate = bytes(self._buffer[:frame_length])
            try:
                frames.append(decode_frame(candidate))
                del self._buffer[:frame_length]
            except ValueError as exc:
                if "CRC" in str(exc):
                    self.crc_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
        return frames


@dataclass(frozen=True)
class PoseGoal:
    goal_id: int
    x_mm: int
    y_mm: int
    yaw_mrad: int
    timeout_ms: int

    _STRUCT = struct.Struct("<IiiiI")

    def command_payload(self) -> bytes:
        if not 1 <= self.goal_id <= 0xFFFFFFFF:
            raise ValueError("goal_id must be non-zero uint32")
        if not 1 <= self.timeout_ms <= 60_000:
            raise ValueError("timeout_ms must be in 1..60000")
        return bytes((Command.SET_POSE_GOAL,)) + self._STRUCT.pack(
            self.goal_id, self.x_mm, self.y_mm, self.yaw_mrad, self.timeout_ms
        )


@dataclass(frozen=True)
class Response:
    request_sequence: int
    command: int
    status: ResponseStatus
    data: bytes

    @classmethod
    def decode(cls, payload: bytes) -> "Response":
        if len(payload) < 3:
            raise ValueError("response payload is shorter than 3 bytes")
        return cls(payload[0], payload[1], ResponseStatus(payload[2]), payload[3:])


@dataclass(frozen=True)
class PoseStatus:
    goal_id: int
    state: PoseState
    x_mm: int
    y_mm: int
    yaw_mrad: int
    fault_reason: int

    _STRUCT = struct.Struct("<IBiiiH")

    @classmethod
    def decode(cls, data: bytes) -> "PoseStatus":
        if len(data) != cls._STRUCT.size:
            raise ValueError("pose status must be 19 bytes")
        goal_id, state, x_mm, y_mm, yaw_mrad, fault = cls._STRUCT.unpack(data)
        return cls(goal_id, PoseState(state), x_mm, y_mm, yaw_mrad, fault)


@dataclass(frozen=True)
class PoseEvent:
    code: EventCode
    goal_id: int
    values: tuple[int, ...] = ()


def decode_pose_event(payload: bytes) -> PoseEvent:
    if len(payload) < 5:
        raise ValueError("pose event is shorter than 5 bytes")
    code = EventCode(payload[0])
    goal_id = struct.unpack_from("<I", payload, 1)[0]
    if goal_id == 0:
        raise ValueError("event goal_id cannot be zero")
    if code in (EventCode.POSE_STARTED, EventCode.POSE_CANCELLED):
        if len(payload) != 5:
            raise ValueError("start/cancel event must be 5 bytes")
        return PoseEvent(code, goal_id)
    if code == EventCode.POSE_REACHED:
        if len(payload) != 25:
            raise ValueError("reached event must be 25 bytes")
        return PoseEvent(code, goal_id, struct.unpack_from("<iiiii", payload, 5))
    if code == EventCode.MOTION_FAULT:
        if len(payload) != 7:
            raise ValueError("fault event must be 7 bytes")
        return PoseEvent(code, goal_id, (struct.unpack_from("<H", payload, 5)[0],))
    raise ValueError(f"unsupported pose event {code!r}")


def command_frame(sequence: int, command: Command, data: bytes = b"") -> bytes:
    return Frame(MessageType.COMMAND, sequence, bytes((command,)) + data).encode()


def cancel_goal_data(goal_id: int) -> bytes:
    if not 1 <= goal_id <= 0xFFFFFFFF:
        raise ValueError("goal_id must be non-zero uint32")
    return struct.pack("<I", goal_id)


def speed_limits_data(linear_mps: float, yaw_radps: float) -> bytes:
    """Encode the runtime X/Y linear and yaw speed limits as microunits."""
    if not 0.02 <= linear_mps <= 0.30:
        raise ValueError("linear_mps must be in 0.02..0.30")
    if not 0.02 <= yaw_radps <= 0.80:
        raise ValueError("yaw_radps must be in 0.02..0.80")
    return struct.pack(
        "<ii", round(linear_mps * 1_000_000), round(yaw_radps * 1_000_000)
    )
