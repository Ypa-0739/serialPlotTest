# -*- coding: utf-8 -*-
"""STM32 文本协议封装（纯协议层，不碰串口）。

命令编码与行解析与固件 Core/Src/main.c 保持一致：
  - 命令：一行文本，\\n 结尾（固件按行接收，\\r\\n 亦可）
  - 输出：'#' 开头的状态行 + 非 '#' 开头的 CSV 遥测行
  - POSE SET 目标为 OPS 原始坐标（固件内部换算到车体中心）
  - 安全停止：ROUND STOP <REASON> 或 "# POSE STOP SAFETY"，统一包装成
    SafetyFault(reason=..., raw=...)，上层 SafetySupervisor 无需解析字符串

所有事件都保留 raw 原始行，供日志、协议调试与实车未知输出排查。
"""

from __future__ import annotations

import re
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

# ---------------------------------------------------------------------------
# 命令编码（Pi -> STM32）
# ---------------------------------------------------------------------------


def encode_ping() -> bytes:
    return b"PING\n"


def encode_stop() -> bytes:
    return b"STOP\n"


def encode_status() -> bytes:
    return b"STATUS\n"


def encode_host_link(host: str) -> bytes:
    """申请主机所有权；host 只能是 COM 或 RPI。"""
    normalized = host.strip().upper()
    if normalized not in {"COM", "RPI"}:
        raise ValueError("host must be COM or RPI")
    return f"HOST LINK {normalized}\n".encode("ascii")


def encode_pose_set(x_mm: float, y_mm: float, yaw_deg: float) -> bytes:
    """POSE SET：OPS 原始目标坐标（mm/mm/deg）。"""
    return f"POSE SET {x_mm:.2f} {y_mm:.2f} {yaw_deg:.2f}\n".encode("ascii")


def encode_command(text: str) -> bytes:
    return text.encode("ascii") + b"\n"


@dataclass(frozen=True)
class Command:
    """发往 STM32 的命令。priority 越小越先发出。"""

    text: str
    priority: int = 20


# 优先级约定（PING 由 SerialBridgeThread 直接按 deadline 调度，不进队列）
PRIORITY_STOP = 0
PRIORITY_PING = 10
PRIORITY_MOTION = 20
PRIORITY_CONFIG = 30


# 必须与STM32固件中所有“可能启动运动”的命令同步；新增运动命令时必须更新此表和测试。
_MOTION_COMMAND_PREFIXES = (
    "POSE SET ",
    "MOTOR RUN ",
    "NAV GOTO ",
    "CMD VEL ",
    "SET P:",
    "SET KP:",
    "P:",
)


def is_motion_command(text: str) -> bool:
    """判断命令是否可能启动底盘/电机运动。

    STOP、MOVE STOP、POSE STOP、NAV CANCEL 以及PID配置命令均不属于运动命令，
    因而在急停锁存期间仍可用于安全处置和恢复自检。
    """
    normalized = " ".join(text.strip().upper().split())
    if not normalized:
        return False
    if normalized.startswith("MOVE "):
        return normalized != "MOVE STOP"
    if normalized.startswith("TURN "):
        return True
    if normalized.startswith("PID "):
        return not normalized.startswith(("PID SET ", "PID LIMIT ", "PID STATUS "))
    return normalized.startswith(_MOTION_COMMAND_PREFIXES)


# ---------------------------------------------------------------------------
# 事件（STM32 -> Pi）
# ---------------------------------------------------------------------------


class FaultReason(str, Enum):
    """固件安全停车原因，与 ROUND STOP <REASON> 文本一一对应。"""

    OPS_LOST = "OPS LOST"
    HOST_LOST = "HOST LOST"
    OVERTRAVEL = "OVERTRAVEL"
    CROSS_TRACK = "CROSS TRACK"
    TRANSLATION_LIMIT = "TRANSLATION LIMIT"
    WRONG_DIR = "WRONG DIR"
    YAW_LIMIT = "YAW LIMIT"
    CAN_FAULT = "CAN FAULT"
    POSE_SAFETY_STOP = "POSE SAFETY STOP"
    TIMEOUT = "TIMEOUT"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class Event:
    """事件基类。raw 保留 STM32 原始行。"""

    raw: str = ""

    def __str__(self) -> str:
        return f"<{type(self).__name__} {self.raw!r}>"


@dataclass(frozen=True)
class RawMessage(Event):
    """未识别/未完全解析的设备行。"""

    pass


@dataclass(frozen=True)
class Pong(Event):
    pass


@dataclass(frozen=True)
class HostLinkAcknowledged(Event):
    host: str = ""


@dataclass(frozen=True)
class HostStatus(Event):
    state: str = ""
    owner: str = ""
    motor_enabled: bool = False
    heartbeat: str = ""
    wait_ms: int = 0
    timeout_ms: int = 0


@dataclass(frozen=True)
class Status(Event):
    """STATUS 命令响应。"""

    mode: str = ""
    host_proto: int = 0
    axis: str = ""
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    max_out: float = 0.0
    state: int = 0
    plot: int = 0
    motor_proto: str = ""
    ops_frames: int = 0
    uart_tx_ok: int = 0
    uart_tx_err: int = 0


@dataclass(frozen=True)
class PoseStarted(Event):
    """POSE SET 被固件接受，闭环开始。目标为 OPS 原始坐标。"""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    center_x: float = 0.0
    center_y: float = 0.0
    tol_mm: float = 0.0
    tol_yaw: float = 0.0


@dataclass(frozen=True)
class PoseReached(Event):
    """位姿到达，电机已停。"""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    center_x: float = 0.0
    center_y: float = 0.0
    error_mm: float = 0.0
    error_yaw: float = 0.0


@dataclass(frozen=True)
class PoseStopped(Event):
    """POSE STOP 命令的确认。"""

    pass


@dataclass(frozen=True)
class StopAcknowledged(Event):
    mode: str = ""


@dataclass(frozen=True)
class ModeChanged(Event):
    mode: str = ""
    plot: int = 0
    changed: int = 0


@dataclass(frozen=True)
class PidLoaded(Event):
    axis: str = ""
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


@dataclass(frozen=True)
class PidLimitSet(Event):
    axis: str = ""
    output: float = 0.0
    unit: str = ""


@dataclass(frozen=True)
class PidStatusAll(Event):
    x: tuple[float, float, float] = (0.0, 0.0, 0.0)
    y: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class RoundStarted(Event):
    """调参轮次开始（TUNE 模式）。"""

    round_no: int = 0
    axis: str = ""
    direction: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class RoundStopped(Event):
    """一轮正常结束（TARGET/TIMEOUT/AXIS/HOST/RESET）。"""

    reason: str = "UNKNOWN"
    axis: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class SafetyFault(Event):
    """安全停车事件。reason 映射固件原因，raw 保留原始行。"""

    reason: FaultReason = FaultReason.UNKNOWN


@dataclass(frozen=True)
class OpsStatus(Event):
    """OPS STATUS 响应。"""

    link: str = "UNKNOWN"
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    frames: int = 0


@dataclass(frozen=True)
class CanStatus(Event):
    """CAN STATUS 响应（无错误）。"""

    state: int = 0
    error: int = 0
    tx_ok: int = 0
    tx_err: int = 0


@dataclass(frozen=True)
class CanError(Event):
    """CAN 错误（HAL_CAN_GetError 非零，或 ERROR 消息）。"""

    pass


@dataclass(frozen=True)
class UnknownError(Event):
    """# ERROR 消息。"""

    text: str = ""


@dataclass(frozen=True)
class CsvTelemetry(Event):
    """17 列调参遥测（timestamp,setpoint,input,output,error,p,i,d,x,y,yaw,...）。"""

    timestamp: float = 0.0
    setpoint: float = 0.0
    input: float = 0.0
    output: float = 0.0
    error: float = 0.0
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    cross: float = 0.0
    yaw_delta: float = 0.0
    hold_cross: float = 0.0
    hold_yaw: float = 0.0
    center_x: float = 0.0
    center_y: float = 0.0


@dataclass(frozen=True)
class WheelTelemetry(Event):
    """@W：四轮目标RPM + 四轮反馈RPM，共8通道。"""

    version: int = 0
    tick_ms: int = 0
    sequence: int = 0
    target_rpm: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    actual_rpm: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class PoseTelemetry(Event):
    """@P：OPS、车体中心和POSE规划速度，共8通道。"""

    version: int = 0
    tick_ms: int = 0
    sequence: int = 0
    ops_x_mm: float = 0.0
    ops_y_mm: float = 0.0
    ops_yaw_deg: float = 0.0
    center_x_mm: float = 0.0
    center_y_mm: float = 0.0
    plan_vx_mps: float = 0.0
    plan_vy_mps: float = 0.0
    plan_vz_radps: float = 0.0


# ---------------------------------------------------------------------------
# 行解析（STM32 -> Event）
# ---------------------------------------------------------------------------

_CSV_COLUMNS = (
    "timestamp setpoint input output error kp ki kd x y yaw cross "
    "yaw_delta hold_cross hold_yaw center_x center_y".split()
)

_ROUND_START_RE = re.compile(
    r"# ROUND START (\d+) AXIS=(\w+) DIR\s+(-?\d+(?:\.\d+)?) "
    r"X=(-?\d+(?:\.\d+)?) Y=(-?\d+(?:\.\d+)?) YAW=(-?\d+(?:\.\d+)?)"
)
_ROUND_STOP_HEAD_RE = re.compile(r"# ROUND STOP (.+?) AXIS=(\w+)")
_PID_ALL_RE = re.compile(
    r"# PID ALL X=([^, ]+),([^, ]+),([^ ]+) "
    r"Y=([^, ]+),([^, ]+),([^ ]+) "
    r"YAW=([^, ]+),([^, ]+),([^ ]+)"
)

# 正常结束原因（不构成安全故障）
_NORMAL_REASONS = ("TARGET", "TIMEOUT", "AXIS", "HOST", "RESET")
# 安全原因 -> FaultReason 映射
_FAULT_REASON_MAP = {
    "OPS LOST": FaultReason.OPS_LOST,
    "HOST LOST": FaultReason.HOST_LOST,
    "OVERTRAVEL": FaultReason.OVERTRAVEL,
    "CROSS TRACK": FaultReason.CROSS_TRACK,
    "TRANSLATION LIMIT": FaultReason.TRANSLATION_LIMIT,
    "WRONG DIR": FaultReason.WRONG_DIR,
    "YAW LIMIT": FaultReason.YAW_LIMIT,
    "CAN FAULT": FaultReason.CAN_FAULT,
}

_KV_RE = re.compile(
    r"([A-Z_]+)=(0x[0-9A-Fa-f]+|-?\d+\.?\d*[eE]?[-+]?\d*|-?\d+|[A-Za-z_]+)"
)


def _kv(line: str) -> dict[str, str]:
    return {name: value for name, value in _KV_RE.findall(line)}


def _f(value: Optional[str], default: float = 0.0) -> float:
    try:
        return float(value) if value is not None else default
    except ValueError:
        return default


def _i(value: Optional[str], default: int = 0) -> int:
    try:
        return int(value, 0) if value is not None else default
    except ValueError:
        return default


def _parse_status(text: str) -> Status:
    kv = _kv(text)
    return Status(
        raw=text,
        mode=kv.get("MODE", ""),
        host_proto=_i(kv.get("HOST_PROTO")),
        axis=kv.get("AXIS", ""),
        kp=_f(kv.get("P")),
        ki=_f(kv.get("I")),
        kd=_f(kv.get("D")),
        max_out=_f(kv.get("MAX_OUT")),
        state=_i(kv.get("STATE")),
        plot=_i(kv.get("PLOT")),
        motor_proto=kv.get("MOTOR_PROTO", ""),
        ops_frames=_i(kv.get("OPS_FRAMES")),
        uart_tx_ok=_i(kv.get("UART_TX_OK")),
        uart_tx_err=_i(kv.get("UART_TX_ERR")),
    )


def _parse_pose_start(text: str) -> PoseStarted:
    kv = _kv(text)
    return PoseStarted(
        raw=text,
        x=_f(kv.get("X")),
        y=_f(kv.get("Y")),
        yaw=_f(kv.get("YAW")),
        center_x=_f(kv.get("CENTER_X")),
        center_y=_f(kv.get("CENTER_Y")),
        tol_mm=_f(kv.get("TOL_MM")),
        tol_yaw=_f(kv.get("TOL_YAW")),
    )


def _parse_pose_reached(text: str) -> PoseReached:
    kv = _kv(text)
    return PoseReached(
        raw=text,
        x=_f(kv.get("X")),
        y=_f(kv.get("Y")),
        yaw=_f(kv.get("YAW")),
        center_x=_f(kv.get("CENTER_X")),
        center_y=_f(kv.get("CENTER_Y")),
        error_mm=_f(kv.get("ERROR_MM")),
        error_yaw=_f(kv.get("ERROR_YAW")),
    )


def _parse_round_start(text: str) -> Event:
    m = _ROUND_START_RE.match(text)
    if not m:
        return RawMessage(raw=text)
    return RoundStarted(
        raw=text,
        round_no=int(m.group(1)),
        axis=m.group(2),
        direction=float(m.group(3)),
        x=float(m.group(4)),
        y=float(m.group(5)),
        yaw=float(m.group(6)),
    )


def _parse_round_stop(text: str) -> Event:
    m = _ROUND_STOP_HEAD_RE.match(text)
    if not m:
        return RawMessage(raw=text)
    reason = m.group(1).strip()
    kv = _kv(text)
    x = _f(kv.get("X"))
    y = _f(kv.get("Y"))
    yaw = _f(kv.get("YAW"))
    axis = kv.get("AXIS", "")
    if reason in _FAULT_REASON_MAP:
        return SafetyFault(reason=_FAULT_REASON_MAP[reason], raw=text)
    return RoundStopped(
        raw=text,
        reason=reason if reason in _NORMAL_REASONS else "UNKNOWN",
        axis=axis,
        x=x,
        y=y,
        yaw=yaw,
    )


def _parse_ops_status(text: str) -> OpsStatus:
    kv = _kv(text)
    return OpsStatus(
        raw=text,
        link=kv.get("LINK", "UNKNOWN"),
        x=_f(kv.get("X")),
        y=_f(kv.get("Y")),
        yaw=_f(kv.get("YAW")),
        frames=_i(kv.get("FRAMES")),
    )


def _parse_can(text: str) -> Event:
    kv = _kv(text)
    error = _i(kv.get("ERROR"))
    if error != 0:
        return CanError(raw=text)
    return CanStatus(
        raw=text,
        state=_i(kv.get("STATE")),
        error=error,
        tx_ok=_i(kv.get("TX_OK")),
        tx_err=_i(kv.get("TX_ERR")),
    )


def _parse_pid_all(text: str) -> Event:
    match = _PID_ALL_RE.match(text)
    if not match:
        return RawMessage(raw=text)
    try:
        values = tuple(float(value) for value in match.groups())
    except ValueError:
        return RawMessage(raw=text)
    return PidStatusAll(
        raw=text,
        x=values[0:3],
        y=values[3:6],
        yaw=values[6:9],
    )


def _parse_csv(text: str) -> Event:
    parts = text.split(",")
    if len(parts) < len(_CSV_COLUMNS):
        return RawMessage(raw=text)
    values: list[float] = []
    for part in parts[: len(_CSV_COLUMNS)]:
        try:
            values.append(float(part))
        except ValueError:
            return RawMessage(raw=text)
    fields = dict(zip(_CSV_COLUMNS, values))
    return CsvTelemetry(raw=text, **fields)


def _parse_tagged_telemetry(text: str) -> Event:
    parts = text.split(",")
    if len(parts) != 12 or parts[0] not in {"@W", "@P"}:
        return RawMessage(raw=text)
    try:
        version = int(parts[1])
        tick_ms = int(parts[2])
        sequence = int(parts[3])
        values = tuple(float(value) for value in parts[4:12])
    except ValueError:
        return RawMessage(raw=text)
    if version != 1 or not all(math.isfinite(value) for value in values):
        return RawMessage(raw=text)
    if parts[0] == "@W":
        return WheelTelemetry(
            raw=text,
            version=version,
            tick_ms=tick_ms,
            sequence=sequence,
            target_rpm=values[0:4],
            actual_rpm=values[4:8],
        )
    return PoseTelemetry(
        raw=text,
        version=version,
        tick_ms=tick_ms,
        sequence=sequence,
        ops_x_mm=values[0],
        ops_y_mm=values[1],
        ops_yaw_deg=values[2],
        center_x_mm=values[3],
        center_y_mm=values[4],
        plan_vx_mps=values[5],
        plan_vy_mps=values[6],
        plan_vz_radps=values[7],
    )


def parse_line(line: str) -> Optional[Event]:
    """解析 STM32 输出的一行。无法解析返回 RawMessage；空行返回 None。"""
    text = line.strip()
    if not text:
        return None
    if text.startswith(("@W,", "@P,")):
        return _parse_tagged_telemetry(text)
    if not text.startswith("#"):
        return _parse_csv(text)
    if text == "# PONG":
        return Pong(raw=text)
    if text.startswith("# HOST LINK "):
        parts = text.split()
        requested = parts[3] if len(parts) > 3 else ""
        if len(parts) > 4 and parts[4] == "OK":
            return HostLinkAcknowledged(raw=text, host=requested)
        return RawMessage(raw=text)
    if text.startswith("# HOST STATUS "):
        kv = _kv(text)
        return HostStatus(
            raw=text,
            state=kv.get("STATE", ""),
            owner=kv.get("OWNER", ""),
            motor_enabled=bool(_i(kv.get("MOTOR_EN"))),
            heartbeat=kv.get("HEARTBEAT", ""),
            wait_ms=_i(kv.get("WAIT_MS")),
            timeout_ms=_i(kv.get("TIMEOUT_MS")),
        )
    if text == "# POSE STOP":
        return PoseStopped(raw=text)
    if text.startswith("# STOP MODE="):
        return StopAcknowledged(raw=text, mode=_kv(text).get("MODE", ""))
    if text.startswith("# MODE "):
        parts = text.split()
        kv = _kv(text)
        return ModeChanged(
            raw=text,
            mode=parts[2] if len(parts) > 2 else "",
            plot=_i(kv.get("PLOT")),
            changed=_i(kv.get("CHANGED")),
        )
    if text.startswith("# PID LOADED "):
        kv = _kv(text)
        return PidLoaded(
            raw=text,
            axis=kv.get("AXIS", ""),
            kp=_f(kv.get("P")),
            ki=_f(kv.get("I")),
            kd=_f(kv.get("D")),
        )
    if text.startswith("# PID LIMIT "):
        kv = _kv(text)
        return PidLimitSet(
            raw=text,
            axis=kv.get("AXIS", ""),
            output=_f(kv.get("OUTPUT")),
            unit=kv.get("UNIT", ""),
        )
    if text.startswith("# PID ALL "):
        return _parse_pid_all(text)
    if text.startswith("# POSE STOP SAFETY"):
        return SafetyFault(reason=FaultReason.POSE_SAFETY_STOP, raw=text)
    if text.startswith("# MOTION STOP SAFETY"):
        match = re.search(r"REASON=(.+)$", text)
        reason_text = match.group(1).strip() if match else ""
        reason = _FAULT_REASON_MAP.get(
            reason_text,
            FaultReason.TIMEOUT if reason_text == "TIMEOUT" else FaultReason.UNKNOWN,
        )
        return SafetyFault(reason=reason, raw=text)
    if text.startswith("# POSE START "):
        return _parse_pose_start(text)
    if text.startswith("# POSE TARGET "):
        return _parse_pose_reached(text)
    if text.startswith("# ROUND START "):
        return _parse_round_start(text)
    if text.startswith("# ROUND STOP "):
        return _parse_round_stop(text)
    if text.startswith("# STATUS "):
        return _parse_status(text)
    if text.startswith("# OPS LINK="):
        return _parse_ops_status(text)
    if text.startswith("# CAN "):
        return _parse_can(text)
    if text.startswith("# ERROR "):
        if "CAN" in text:
            return CanError(raw=text)
        return UnknownError(text=text[len("# ERROR ") :].strip(), raw=text)
    return RawMessage(raw=text)
