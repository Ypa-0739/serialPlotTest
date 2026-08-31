"""视觉观察到本地任务类型的适配，不执行运动也不访问串口。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional, Protocol

from .models import PoseGoal
from .protocol import CanError, ModeChanged, SafetyFault, UnknownError
from .serial_bridge import SerialDisconnected
from .vision.models import VisionObservation
from .vision.observation_adapter import (
    VisionObservationGate,
    VisionRequirement,
)


@dataclass(frozen=True)
class VisionGoalRule:
    """视觉目标只能选择已经人工标定的本地 PoseGoal。"""

    requirement: VisionRequirement
    goal_name: str


class VisionGoalSelector:
    """把有效观察映射到预标定航点；绝不从像素生成任意底盘命令。"""

    def __init__(
        self,
        gate: VisionObservationGate,
        goals: Mapping[str, PoseGoal],
        rules: Mapping[str, VisionGoalRule],
    ) -> None:
        self.gate = gate
        self.goals = dict(goals)
        self.rules = dict(rules)
        missing = {rule.goal_name for rule in self.rules.values()} - set(self.goals)
        if missing:
            raise ValueError(f"vision rules reference unknown goals: {sorted(missing)}")

    def select(self, decision_name: str) -> Optional[PoseGoal]:
        rule = self.rules.get(decision_name)
        if rule is None:
            raise KeyError(f"unknown vision decision: {decision_name}")
        if self.gate.require(rule.requirement) is None:
            return None
        return self.goals[rule.goal_name]


class VisionCache(Protocol):
    def clear(self) -> None: ...


class VisionStateGuard:
    """把本地控制安全边界同步为视觉缓存清理边界。"""

    def __init__(self, cache: VisionCache) -> None:
        self.cache = cache

    def handle_event(self, event) -> None:
        if isinstance(
            event,
            (SerialDisconnected, SafetyFault, CanError, UnknownError, ModeChanged),
        ):
            self.cache.clear()

    def set_control_ready(self, ready: bool) -> None:
        if not ready:
            self.cache.clear()


class TaskCodeObservationReader:
    """把已确认二维码观察转换为本地任务码文本。"""

    def __init__(self, gate: VisionObservationGate) -> None:
        self.gate = gate

    def read_task_code(self) -> Optional[str]:
        observation: Optional[VisionObservation] = self.gate.require(
            VisionRequirement("task_code", 1.0, 0.5, "front")
        )
        if observation is None:
            return None
        text = observation.target_id
        return text if isinstance(text, str) and text else None
