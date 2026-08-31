"""任务层读取视觉结果的窄接口和安全门禁。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional, Protocol

from .models import VisionObservation


class VisionPort(Protocol):
    def latest(self, target_type: Optional[str] = None) -> Optional[VisionObservation]: ...


@dataclass(frozen=True)
class VisionRequirement:
    target_type: str
    minimum_confidence: float = 0.5
    maximum_age_s: float = 0.5
    source: Optional[str] = None
    target_id: Optional[str] = None


class VisionObservationGate:
    """只返回满足类型、来源、置信度和新鲜度要求的观察。"""

    def __init__(
        self,
        port: VisionPort,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.port = port
        self.clock = clock

    def require(self, requirement: VisionRequirement) -> Optional[VisionObservation]:
        observation = self.port.latest(requirement.target_type)
        if observation is None:
            return None
        if requirement.source is not None and observation.source != requirement.source:
            return None
        if (
            requirement.target_id is not None
            and observation.target_id != requirement.target_id
        ):
            return None
        if observation.confidence < requirement.minimum_confidence:
            return None
        if not observation.is_fresh(self.clock(), requirement.maximum_age_s):
            return None
        return observation
