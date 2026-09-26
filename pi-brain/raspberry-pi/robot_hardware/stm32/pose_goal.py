"""带 ``goal_id`` 的 STM32 位姿事务客户端。

本模块不读取串口；它订阅唯一 ``SerialLink`` 的 EVENT 分发，并把 STM32
闭环的接受、启动、到位、取消和故障转换为线程安全快照。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import threading
import secrets
from typing import Callable, Optional

from .messages import (
    Command,
    EventCode,
    MessageType,
    MotionFault,
    MotionFaultReason,
    PoseCancelled,
    PoseGoal,
    PoseGoalState,
    PoseGoalStatus,
    PoseReached,
    PoseStarted,
    decode_motion_event,
    encode_cancel_pose_goal,
    encode_speed_limits,
)
from .protocol import Frame
from .serial_link import SerialLink


class PoseTransactionState(Enum):
    IDLE = auto()
    ACCEPTED = auto()
    MOVING = auto()
    CANCELLING = auto()
    REACHED = auto()
    CANCELLED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class PoseTransactionSnapshot:
    state: PoseTransactionState
    goal: Optional[PoseGoal] = None
    reached: Optional[PoseReached] = None
    fault_reason: int = MotionFaultReason.UNSPECIFIED

    @property
    def terminal(self) -> bool:
        return self.state in {
            PoseTransactionState.REACHED,
            PoseTransactionState.CANCELLED,
            PoseTransactionState.FAULT,
        }


class PoseGoalBusy(RuntimeError):
    pass


class Stm32PoseGoalController:
    """提交一个绝对位姿，并只接受同一 ``goal_id`` 的终态事件。"""

    _ACTIVE_STATES = {
        PoseTransactionState.ACCEPTED,
        PoseTransactionState.MOVING,
        PoseTransactionState.CANCELLING,
    }

    def __init__(
        self,
        link: SerialLink,
        *,
        activity_reader: Optional[Callable[[], bool]] = None,
        maximum_speed_mm_s: float = 350.0,
        maximum_yaw_rate_mrad_s: float = 250.0,
        request_timeout_seconds: float = 0.5,
    ) -> None:
        if maximum_speed_mm_s <= 0:
            raise ValueError("maximum_speed_mm_s 必须大于 0")
        if request_timeout_seconds <= 0:
            raise ValueError("request_timeout_seconds 必须大于 0")
        self.link = link
        self._activity_reader = activity_reader
        # 服从 v2 固件限幅，不用配置中的 350 mm/s 绕过其 300 mm/s 上限。
        self._maximum_speed_mm_s = min(float(maximum_speed_mm_s), 300.0)
        self._speed_limits = encode_speed_limits(
            self._maximum_speed_mm_s, min(float(maximum_yaw_rate_mrad_s), 800.0)
        )
        self._request_timeout = float(request_timeout_seconds)
        self._lock = threading.Lock()
        self._snapshot = PoseTransactionSnapshot(PoseTransactionState.IDLE)
        self._next_goal_id = secrets.randbelow(0xFFFFFFFF) + 1
        self._link_generation = getattr(link, "generation", 0)
        self._attached = False
        self.invalid_events = 0
        self.stale_events = 0

    def attach(self) -> None:
        if self._attached:
            return
        self.link.add_frame_handler(MessageType.EVENT, self._on_frame)
        self._attached = True

    def detach(self) -> None:
        if not self._attached:
            return
        self.link.remove_frame_handler(MessageType.EVENT, self._on_frame)
        self._attached = False

    def submit(
        self,
        x_mm: int,
        y_mm: int,
        yaw_mrad: int,
        *,
        timeout_seconds: float,
    ) -> int:
        timeout_ms = int(round(timeout_seconds * 1000.0))
        if timeout_ms <= 0:
            raise ValueError("timeout_seconds 必须大于 0")
        self.snapshot()  # 先将旧会话的活动事务失效，再决定能否提交新目标
        with self._lock:
            if self._snapshot.state in self._ACTIVE_STATES:
                raise PoseGoalBusy("已有活动位姿目标")
            goal_id = self._next_goal_id
            self._next_goal_id = 1 if goal_id == 0xFFFFFFFF else goal_id + 1
            goal = PoseGoal(goal_id, x_mm, y_mm, yaw_mrad, timeout_ms)
            payload = goal.encode_command_data() + self._speed_limits
            self._snapshot = PoseTransactionSnapshot(
                PoseTransactionState.ACCEPTED,
                goal=goal,
            )
        try:
            self.link.request(
                Command.SET_POSE_GOAL_WITH_LIMITS,
                payload,
                timeout=self._request_timeout,
            )
        except Exception:
            try:
                self.link.send_command(Command.STOP_ALL)
            except Exception:
                pass
            with self._lock:
                if self._snapshot.goal == goal:
                    self._snapshot = PoseTransactionSnapshot(
                        PoseTransactionState.FAULT,
                        goal=goal,
                        fault_reason=MotionFaultReason.INTERNAL_ERROR,
                    )
            raise
        return goal_id

    def cancel(self) -> bool:
        with self._lock:
            snapshot = self._snapshot
            if snapshot.goal is None or snapshot.state not in {
                PoseTransactionState.ACCEPTED,
                PoseTransactionState.MOVING,
            }:
                return False
            goal_id = snapshot.goal.goal_id
        try:
            self.link.request(
                Command.CANCEL_POSE_GOAL,
                encode_cancel_pose_goal(goal_id),
                timeout=self._request_timeout,
            )
        except Exception:
            try:
                self.link.send_command(Command.STOP_ALL)
            except Exception:
                pass
            with self._lock:
                if (
                    self._snapshot.goal is not None
                    and self._snapshot.goal.goal_id == goal_id
                ):
                    self._snapshot = PoseTransactionSnapshot(
                        PoseTransactionState.FAULT,
                        goal=self._snapshot.goal,
                        fault_reason=MotionFaultReason.CANCEL_TIMEOUT,
                    )
            raise
        with self._lock:
            if (
                self._snapshot.goal is not None
                and self._snapshot.goal.goal_id == goal_id
                and self._snapshot.state in self._ACTIVE_STATES
            ):
                self._snapshot = PoseTransactionSnapshot(
                    PoseTransactionState.CANCELLING,
                    goal=self._snapshot.goal,
                )
        return True

    def stop(self) -> None:
        """最高层安全停车；不等待响应，避免阻塞安全状态机。"""

        self.link.send_command(Command.STOP_ALL)
        with self._lock:
            if self._snapshot.state in self._ACTIVE_STATES:
                self._snapshot = PoseTransactionSnapshot(
                    PoseTransactionState.CANCELLING,
                    goal=self._snapshot.goal,
                )

    def query_status(self) -> PoseGoalStatus:
        response = self.link.request(
            Command.QUERY_POSE_GOAL,
            timeout=self._request_timeout,
        )
        return PoseGoalStatus.decode_response_data(response.data)

    def snapshot(self) -> PoseTransactionSnapshot:
        with self._lock:
            generation = getattr(self.link, "generation", 0)
            if generation != self._link_generation:
                self._link_generation = generation
                if self._snapshot.state in self._ACTIVE_STATES:
                    self._snapshot = PoseTransactionSnapshot(
                        PoseTransactionState.FAULT, goal=self._snapshot.goal,
                        fault_reason=MotionFaultReason.HOST_LOST,
                    )
            return self._snapshot

    def clear_terminal(self) -> None:
        with self._lock:
            if self._snapshot.terminal:
                self._snapshot = PoseTransactionSnapshot(PoseTransactionState.IDLE)

    def is_active(self) -> bool:
        return bool(self._activity_reader and self._activity_reader())

    @property
    def commanded_motion_active(self) -> bool:
        return self.snapshot().state in self._ACTIVE_STATES

    @property
    def commanded_speed_mm_s(self) -> float:
        return self._maximum_speed_mm_s if self.commanded_motion_active else 0.0

    def _on_frame(self, frame: Frame) -> bool:
        if not frame.payload or frame.payload[0] not in {
            EventCode.POSE_STARTED,
            EventCode.POSE_REACHED,
            EventCode.POSE_CANCELLED,
            EventCode.MOTION_FAULT,
        }:
            return False
        try:
            event = decode_motion_event(frame.payload)
        except ValueError:
            self.invalid_events += 1
            return True
        with self._lock:
            goal = self._snapshot.goal
            if goal is None or event.goal_id != goal.goal_id:
                self.stale_events += 1
                return True
            if isinstance(event, PoseStarted):
                if self._snapshot.state == PoseTransactionState.ACCEPTED:
                    self._snapshot = PoseTransactionSnapshot(
                        PoseTransactionState.MOVING,
                        goal=goal,
                    )
            elif isinstance(event, PoseReached):
                if self._snapshot.state in self._ACTIVE_STATES:
                    self._snapshot = PoseTransactionSnapshot(
                        PoseTransactionState.REACHED,
                        goal=goal,
                        reached=event,
                    )
            elif isinstance(event, PoseCancelled):
                if self._snapshot.state in self._ACTIVE_STATES:
                    self._snapshot = PoseTransactionSnapshot(
                        PoseTransactionState.CANCELLED,
                        goal=goal,
                    )
            elif isinstance(event, MotionFault):
                if self._snapshot.state in self._ACTIVE_STATES:
                    self._snapshot = PoseTransactionSnapshot(
                        PoseTransactionState.FAULT,
                        goal=goal,
                        fault_reason=event.reason,
                    )
        return True
