# -*- coding: utf-8 -*-
"""树莓派启动自检状态机：STOP -> SELF_CHECK -> PID重载 -> READY。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Protocol

from .protocol import (
    CanError,
    CanStatus,
    Event,
    ModeChanged,
    OpsStatus,
    PidLimitSet,
    PidLoaded,
    PidStatusAll,
    RoundStopped,
    SafetyFault,
    Status,
    StopAcknowledged,
    UnknownError,
)
from .serial_bridge import SerialConnected, SerialDisconnected


class BridgePort(Protocol):
    @property
    def is_connected(self) -> bool: ...

    def send(self, command: str, priority: int | None = None) -> None: ...

    def emergency_stop(self) -> None: ...

    def release_emergency_stop(self) -> None: ...


class StartupState(str, Enum):
    BOOT = "BOOT"
    WAIT_CONNECTION = "WAIT_CONNECTION"
    WAIT_STOP = "WAIT_STOP"
    WAIT_MODE_WORK = "WAIT_MODE_WORK"
    WAIT_STATUS = "WAIT_STATUS"
    WAIT_OPS = "WAIT_OPS"
    WAIT_CAN = "WAIT_CAN"
    WAIT_PID_X = "WAIT_PID_X"
    WAIT_PID_Y = "WAIT_PID_Y"
    WAIT_PID_YAW = "WAIT_PID_YAW"
    WAIT_LIMIT_X = "WAIT_LIMIT_X"
    WAIT_LIMIT_Y = "WAIT_LIMIT_Y"
    WAIT_LIMIT_YAW = "WAIT_LIMIT_YAW"
    WAIT_PID_VERIFY = "WAIT_PID_VERIFY"
    READY = "READY"
    FAULT = "FAULT"


@dataclass(frozen=True)
class PidValues:
    kp: float
    ki: float = 0.0
    kd: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.kp, self.ki, self.kd)


@dataclass(frozen=True)
class StartupConfig:
    x: PidValues = PidValues(0.0033)
    y: PidValues = PidValues(0.0033)
    yaw: PidValues = PidValues(0.02)
    limit_x: float = 0.20
    limit_y: float = 0.20
    limit_yaw: float = 0.25
    step_timeout: float = 1.5
    minimum_protocol: int = 2


class StartupStateMachine:
    """非阻塞启动状态机。

    主循环把SerialBridge事件逐个交给handle_event()，并周期调用tick()。
    本类不读写串口；所有命令仍通过SerialBridge唯一写者发送。
    """

    def __init__(
        self,
        bridge: BridgePort,
        config: StartupConfig | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.bridge = bridge
        self.config = config or StartupConfig()
        self._clock = clock
        self.state = StartupState.BOOT
        self.fault_reason = ""
        self._deadline: float | None = None

    @property
    def is_ready(self) -> bool:
        return self.state == StartupState.READY

    def start(self) -> None:
        """开始或重新开始自检；首先锁存急停并清除旧命令。"""
        self.fault_reason = ""
        self.bridge.emergency_stop()
        if self.bridge.is_connected:
            self._enter(StartupState.WAIT_STOP)
        else:
            self._enter(StartupState.WAIT_CONNECTION, timeout=False)

    def tick(self) -> None:
        """检查当前步骤超时；不阻塞主循环。"""
        if self._deadline is not None and self._clock() >= self._deadline:
            self._fail(f"timeout waiting in {self.state.value}")

    def handle_event(self, event: Event) -> None:
        """消费一个协议事件；不相关的PONG/遥测事件会被忽略。"""
        if isinstance(event, SerialDisconnected):
            self._fail("serial disconnected")
            return
        if isinstance(event, (SafetyFault, CanError, UnknownError)):
            self._fail(event.raw or type(event).__name__)
            return
        if isinstance(event, SerialConnected):
            if self.state in (StartupState.BOOT, StartupState.WAIT_CONNECTION, StartupState.FAULT):
                self.start()
            return
        if self.state in (StartupState.READY, StartupState.FAULT, StartupState.BOOT):
            return

        handlers = {
            StartupState.WAIT_STOP: self._on_stop,
            StartupState.WAIT_MODE_WORK: self._on_mode,
            StartupState.WAIT_STATUS: self._on_status,
            StartupState.WAIT_OPS: self._on_ops,
            StartupState.WAIT_CAN: self._on_can,
            StartupState.WAIT_PID_X: lambda ev: self._on_pid(ev, "X", StartupState.WAIT_PID_Y),
            StartupState.WAIT_PID_Y: lambda ev: self._on_pid(ev, "Y", StartupState.WAIT_PID_YAW),
            StartupState.WAIT_PID_YAW: lambda ev: self._on_pid(ev, "YAW", StartupState.WAIT_LIMIT_X),
            StartupState.WAIT_LIMIT_X: lambda ev: self._on_limit(ev, "X", StartupState.WAIT_LIMIT_Y),
            StartupState.WAIT_LIMIT_Y: lambda ev: self._on_limit(ev, "Y", StartupState.WAIT_LIMIT_YAW),
            StartupState.WAIT_LIMIT_YAW: lambda ev: self._on_limit(ev, "YAW", StartupState.WAIT_PID_VERIFY),
            StartupState.WAIT_PID_VERIFY: self._on_pid_verify,
        }
        handler = handlers.get(self.state)
        if handler is not None:
            handler(event)

    def _on_stop(self, event: Event) -> None:
        stopped = isinstance(event, StopAcknowledged) or (
            isinstance(event, RoundStopped) and event.reason == "HOST"
        )
        if stopped:
            self.bridge.send("MODE WORK")
            self._enter(StartupState.WAIT_MODE_WORK)

    def _on_mode(self, event: Event) -> None:
        if not isinstance(event, ModeChanged):
            return
        if event.mode != "WORK" or event.plot != 0:
            self._fail(f"invalid work mode response: {event.raw}")
            return
        self.bridge.send("STATUS")
        self._enter(StartupState.WAIT_STATUS)

    def _on_status(self, event: Event) -> None:
        if not isinstance(event, Status):
            return
        if event.mode != "WORK" or event.plot != 0 or event.state != 0:
            self._fail(f"STM32 not idle in WORK mode: {event.raw}")
            return
        if event.host_proto < self.config.minimum_protocol:
            self._fail(f"host protocol {event.host_proto} is unsupported")
            return
        self.bridge.send("OPS STATUS")
        self._enter(StartupState.WAIT_OPS)

    def _on_ops(self, event: Event) -> None:
        if not isinstance(event, OpsStatus):
            return
        if event.link != "OK" or event.frames <= 0:
            self._fail(f"OPS not ready: {event.raw}")
            return
        self.bridge.send("CAN STATUS")
        self._enter(StartupState.WAIT_CAN)

    def _on_can(self, event: Event) -> None:
        if not isinstance(event, CanStatus):
            return
        # STM32F4 HAL: 2=HAL_CAN_STATE_LISTENING；0/1尚未启动，3/4为睡眠，5为错误。
        if event.error != 0 or event.state != 2:
            self._fail(f"CAN not listening: {event.raw}")
            return
        self._send_pid("X", self.config.x)
        self._enter(StartupState.WAIT_PID_X)

    def _on_pid(self, event: Event, axis: str, next_state: StartupState) -> None:
        if not isinstance(event, PidLoaded) or event.axis != axis:
            return
        expected = self._pid_for_axis(axis)
        if not self._pid_close((event.kp, event.ki, event.kd), expected.as_tuple()):
            self._fail(f"PID {axis} acknowledgement mismatch: {event.raw}")
            return
        if next_state == StartupState.WAIT_PID_Y:
            self._send_pid("Y", self.config.y)
        elif next_state == StartupState.WAIT_PID_YAW:
            self._send_pid("YAW", self.config.yaw)
        else:
            self.bridge.send(f"PID LIMIT X {self.config.limit_x:.2f}")
        self._enter(next_state)

    def _on_limit(self, event: Event, axis: str, next_state: StartupState) -> None:
        if not isinstance(event, PidLimitSet) or event.axis != axis:
            return
        expected = self._limit_for_axis(axis)
        if not math.isclose(event.output, expected, rel_tol=0.0, abs_tol=1e-4):
            self._fail(f"PID limit {axis} acknowledgement mismatch: {event.raw}")
            return
        if next_state == StartupState.WAIT_LIMIT_Y:
            self.bridge.send(f"PID LIMIT Y {self.config.limit_y:.2f}")
        elif next_state == StartupState.WAIT_LIMIT_YAW:
            self.bridge.send(f"PID LIMIT YAW {self.config.limit_yaw:.2f}")
        else:
            self.bridge.send("PID STATUS ALL")
        self._enter(next_state)

    def _on_pid_verify(self, event: Event) -> None:
        if not isinstance(event, PidStatusAll):
            return
        if not (
            self._pid_close(event.x, self.config.x.as_tuple())
            and self._pid_close(event.y, self.config.y.as_tuple())
            and self._pid_close(event.yaw, self.config.yaw.as_tuple())
        ):
            self._fail(f"PID verification mismatch: {event.raw}")
            return
        self.bridge.release_emergency_stop()
        self._deadline = None
        self.state = StartupState.READY

    def _send_pid(self, axis: str, values: PidValues) -> None:
        self.bridge.send(
            f"PID SET {axis} {self._fmt(values.kp)} "
            f"{self._fmt(values.ki)} {self._fmt(values.kd)}"
        )

    def _pid_for_axis(self, axis: str) -> PidValues:
        return {"X": self.config.x, "Y": self.config.y, "YAW": self.config.yaw}[axis]

    def _limit_for_axis(self, axis: str) -> float:
        return {
            "X": self.config.limit_x,
            "Y": self.config.limit_y,
            "YAW": self.config.limit_yaw,
        }[axis]

    @staticmethod
    def _pid_close(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
        return all(
            math.isclose(a, e, rel_tol=0.0, abs_tol=1e-7)
            for a, e in zip(actual, expected)
        )

    @staticmethod
    def _fmt(value: float) -> str:
        return format(value, ".8g")

    def _enter(self, state: StartupState, *, timeout: bool = True) -> None:
        self.state = state
        self._deadline = self._clock() + self.config.step_timeout if timeout else None

    def _fail(self, reason: str) -> None:
        self.fault_reason = reason
        self.state = StartupState.FAULT
        self._deadline = None
        self.bridge.emergency_stop()
