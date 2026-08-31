"""固定场地图坐标到本轮 OPS 原始坐标的启动锚定。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .models import Pose, PoseGoal


def _rotate(x: float, y: float, yaw_deg: float) -> tuple[float, float]:
    yaw = math.radians(yaw_deg)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def _wrap_yaw(yaw_deg: float) -> float:
    return (yaw_deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class MapGoal:
    """固定场地图纸中的车体中心航点。"""

    name: str
    x_mm: float
    y_mm: float
    yaw_deg: float
    timeout_s: float = 30.0
    profile: str = "precise"


@dataclass(frozen=True)
class StartAnchoredMapFrame:
    """假定实际放置位姿对应地图标准起点，转换地图中心航点。

    `actual_start_ops` 是启动自检读到的 OPS 传感器原始位姿；地图坐标描述
    车体中心。放置误差会成为本轮地图的整体误差，粗导航后应由视觉精对准。
    """

    nominal_start_center: Pose
    actual_start_ops: Pose
    ops_offset_x_mm: float = 0.0
    ops_offset_y_mm: float = 25.0

    def __post_init__(self) -> None:
        values = (
            self.nominal_start_center.x_mm,
            self.nominal_start_center.y_mm,
            self.nominal_start_center.yaw_deg,
            self.actual_start_ops.x_mm,
            self.actual_start_ops.y_mm,
            self.actual_start_ops.yaw_deg,
            self.ops_offset_x_mm,
            self.ops_offset_y_mm,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("map frame values must be finite")

    def to_ops_goal(self, map_center_goal: MapGoal) -> PoseGoal:
        """把地图车体中心航点转换为 `POSE SET` 所需的 OPS 原始航点。"""
        if not all(
            math.isfinite(value)
            for value in (
                map_center_goal.x_mm,
                map_center_goal.y_mm,
                map_center_goal.yaw_deg,
            )
        ):
            raise ValueError("map goal values must be finite")

        start_offset = _rotate(
            self.ops_offset_x_mm,
            self.ops_offset_y_mm,
            self.actual_start_ops.yaw_deg,
        )
        actual_center_x = self.actual_start_ops.x_mm - start_offset[0]
        actual_center_y = self.actual_start_ops.y_mm - start_offset[1]
        yaw_delta = self.actual_start_ops.yaw_deg - self.nominal_start_center.yaw_deg
        delta_x = map_center_goal.x_mm - self.nominal_start_center.x_mm
        delta_y = map_center_goal.y_mm - self.nominal_start_center.y_mm
        rotated_delta = _rotate(delta_x, delta_y, yaw_delta)
        target_center_x = actual_center_x + rotated_delta[0]
        target_center_y = actual_center_y + rotated_delta[1]
        target_yaw = _wrap_yaw(
            self.actual_start_ops.yaw_deg
            + map_center_goal.yaw_deg
            - self.nominal_start_center.yaw_deg
        )
        target_offset = _rotate(
            self.ops_offset_x_mm,
            self.ops_offset_y_mm,
            target_yaw,
        )
        return PoseGoal(
            name=map_center_goal.name,
            x_mm=target_center_x + target_offset[0],
            y_mm=target_center_y + target_offset[1],
            yaw_deg=target_yaw,
            timeout_s=map_center_goal.timeout_s,
            profile=map_center_goal.profile,
        )
