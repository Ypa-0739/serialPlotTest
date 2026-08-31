"""视觉层公共数据模型；全部与外部仓库类型解耦。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass(frozen=True)
class CameraFrame:
    """一次采集结果；timestamp 使用单调时钟，image 为 OpenCV/NumPy 图像。"""

    timestamp: float
    frame_id: int
    source: str
    image: Any = field(repr=False, compare=False)


@dataclass(frozen=True)
class VisionObservation:
    """任务层唯一依赖的通用视觉观察。

    x/y/yaw 只有在对应相机已完成坐标标定时才允许填写；像素坐标不得冒充
    OPS 毫米坐标。confidence 是 0~1 的检测证据分数，不声明为统计概率。
    """

    timestamp: float
    frame_id: int
    target_type: str
    confidence: float
    x_mm: Optional[float]
    y_mm: Optional[float]
    yaw_deg: Optional[float]
    valid: bool
    source: str
    target_id: Optional[str] = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.timestamp) or self.timestamp < 0.0:
            raise ValueError("timestamp must be a non-negative finite monotonic value")
        if self.frame_id < 0:
            raise ValueError("frame_id must be non-negative")
        if not self.target_type:
            raise ValueError("target_type must not be empty")
        if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be finite and within [0, 1]")
        if not self.source:
            raise ValueError("source must not be empty")
        if self.target_id is not None and not self.target_id:
            raise ValueError("target_id must be non-empty or None")
        for name, value in (
            ("x_mm", self.x_mm),
            ("y_mm", self.y_mm),
            ("yaw_deg", self.yaw_deg),
        ):
            if value is not None and not math.isfinite(value):
                raise ValueError(f"{name} must be finite or None")

    def is_fresh(self, now: float, max_age_s: float) -> bool:
        return (
            self.valid
            and max_age_s > 0.0
            and now >= self.timestamp
            and now - self.timestamp <= max_age_s
        )


@dataclass(frozen=True)
class MaterialObservation:
    timestamp: float
    frame_id: int
    material_code: int
    color_name: str
    color_cn_name: str
    center_px: tuple[int, int]
    offset_px: tuple[float, float]
    normalized_offset: tuple[float, float]
    box_px: tuple[int, int, int, int]
    area_px: float
    confidence: float
    confirmed: bool
    aligned: bool
    safe_to_pick: bool
    source: str = "gripper"


@dataclass(frozen=True)
class QrObservation:
    timestamp: float
    frame_id: int
    text: str
    confidence: float
    confirmed: bool
    source: str = "front"


@dataclass(frozen=True)
class LineObservation:
    timestamp: float
    frame_id: int
    found: bool
    normalized_error: Optional[float]
    center_px: Optional[tuple[int, int]]
    contour_area_px: float
    confidence: float
    source: str = "front"


@dataclass(frozen=True)
class RoadObservation:
    timestamp: float
    frame_id: int
    drivable_fraction: float
    forbidden_fraction: float
    central_drivable_fraction: float
    boundary_safe: bool
    confidence: float
    source: str = "front"


@dataclass(frozen=True)
class ObstacleObservation:
    timestamp: float
    frame_id: int
    x_mm: float
    y_mm: float
    radius_mm: float
    confidence: float
    confirmed: bool
    source: str = "front"


@dataclass(frozen=True)
class VisionHealth:
    timestamp: float
    running: bool
    healthy: bool
    last_frame_age_s: Optional[float]
    error: str = ""
