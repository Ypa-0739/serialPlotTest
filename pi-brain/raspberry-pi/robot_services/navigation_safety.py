"""导航相关的独立安全监控和动态制动包络。"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Callable, Iterable, Optional

from robot_control.navigation_map import NavigationMap, Pose2D
from robot_runtime.models import ActionResult, SafetyReport


@dataclass(frozen=True)
class NavigationSafetyConfig:
    camera_stale_after_seconds: float
    road_observation_stale_after_seconds: float
    camera_blind_distance_mm: float
    total_reaction_seconds: float
    minimum_braking_deceleration_mm_s2: float
    braking_margin_mm: float

    @classmethod
    def load(cls, path: str | Path) -> "NavigationSafetyConfig":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        result = cls(**{key: float(value) for key, value in data.items()})
        if any(value <= 0 for value in vars(result).values()):
            raise ValueError("导航安全配置的所有数值都必须大于 0")
        return result

    def stopping_distance_mm(self, speed_mm_s: float) -> float:
        speed = max(0.0, float(speed_mm_s))
        return (
            self.camera_blind_distance_mm
            + speed * self.total_reaction_seconds
            + speed * speed / (2.0 * self.minimum_braking_deceleration_mm_s2)
            + self.braking_margin_mm
        )


class NavigationSafetyMonitor:
    """汇总串口、OPS9、相机、道路边界和近距离候选障碍。"""

    def __init__(
        self,
        navigation_map: NavigationMap,
        pose_reader: Callable[[], Optional[Pose2D]],
        chassis,
        *,
        link_health: Callable[[], bool],
        camera_health: Callable[[float], bool],
        road_observation_reader: Callable[[], object | None],
        obstacle_candidate_reader: Callable[[], Iterable[object]],
        config: NavigationSafetyConfig,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.map = navigation_map
        self.pose_reader = pose_reader
        self.chassis = chassis
        self.link_health = link_health
        self.camera_health = camera_health
        self.road_observation_reader = road_observation_reader
        self.obstacle_candidate_reader = obstacle_candidate_reader
        self.config = config
        self.clock = clock

    def self_check(self) -> ActionResult:
        if not self.link_health():
            return ActionResult.retryable("STM32 链路未连接")
        if not self.camera_health(self.config.camera_stale_after_seconds):
            return ActionResult.retryable("前视摄像头未就绪")
        if self.pose_reader() is None:
            return ActionResult.running("等待 OPS9 有效位姿", activity=False)
        return ActionResult.done("导航安全传感器自检通过", activity=False)

    def check(self) -> SafetyReport:
        if not self.link_health():
            return SafetyReport(False, "STM32 通信断开", emergency_stop=True)
        if not self.camera_health(self.config.camera_stale_after_seconds):
            return SafetyReport(False, "前视摄像头失效或画面冻结", emergency_stop=True)

        moving_command = bool(self.chassis.commanded_motion_active)
        pose = self.pose_reader()
        if pose is None:
            if moving_command:
                return SafetyReport(False, "OPS9 位姿失效或超时", emergency_stop=True)
            return SafetyReport()
        if not self.map.contains_footprint(pose):
            return SafetyReport(
                False,
                "车体安全包络接近或越过场地边界",
                boundary_ok=False,
                emergency_stop=True,
            )
        if not moving_command:
            return SafetyReport()

        road = self.road_observation_reader()
        if road is None:
            return SafetyReport(False, "缺少前方道路安全观测", emergency_stop=True)
        if self.clock() - road.observed_at > self.config.road_observation_stale_after_seconds:
            return SafetyReport(False, "道路安全观测已超时", emergency_stop=True)
        if not road.boundary_safe:
            return SafetyReport(
                False,
                "视觉发现灰色车道不足或黄白禁入区域进入车体走廊",
                boundary_ok=False,
                emergency_stop=True,
            )

        stopping_distance = self.config.stopping_distance_mm(
            self.chassis.commanded_speed_mm_s
        )
        nearest = self._nearest_forward_clearance(pose)
        if nearest is not None and nearest <= stopping_distance:
            return SafetyReport(
                False,
                f"前方候选障碍进入动态制动包络：{nearest:.0f} mm",
                emergency_stop=True,
            )
        return SafetyReport()

    def _nearest_forward_clearance(self, pose: Pose2D) -> Optional[float]:
        nearest: Optional[float] = None
        cosine, sine = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        for obstacle in self.obstacle_candidate_reader():
            dx = obstacle.x_mm - pose.x_mm
            dy = obstacle.y_mm - pose.y_mm
            body_x = cosine * dx + sine * dy
            if body_x < 0:
                continue
            centre_distance = math.hypot(dx, dy)
            clearance = max(
                0.0,
                centre_distance - self.map.clearance_mm - obstacle.radius_mm,
            )
            nearest = clearance if nearest is None else min(nearest, clearance)
        return nearest
