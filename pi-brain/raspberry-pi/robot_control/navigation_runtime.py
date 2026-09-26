"""真实前摄像头、STM32 OPS9、规划器和安全监控的一体化装配。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from robot_hardware.stm32 import SerialLink
from robot_runtime.models import ActionResult, ActionStatus
from robot_services.navigation_safety import (
    NavigationSafetyConfig,
    NavigationSafetyMonitor,
)

from .dual_camera_vision import DualCameraVisionController
from .navigation_factory import PROJECT_ROOT, Stm32NavigationStack, build_stm32_navigation


@dataclass
class NavigationRuntime:
    """可直接作为 ``ComponentBundle.navigator`` 使用。"""

    stack: Stm32NavigationStack
    vision: DualCameraVisionController
    safety: NavigationSafetyMonitor

    def initialize(self) -> None:
        self.vision.start()
        try:
            self.stack.start()
        except Exception:
            self.vision.close()
            raise

    def self_check(self) -> ActionResult:
        stack_result = self.stack.self_check()
        if stack_result.status is not ActionStatus.DONE:
            return stack_result
        return self.safety.self_check()

    def navigate_to(self, target):
        return self.stack.navigate_to(target)

    def cancel(self) -> None:
        self.stack.cancel()

    def shutdown(self) -> None:
        try:
            self.stack.close()
        finally:
            self.vision.close()


def build_navigation_runtime(
    link: SerialLink,
    vision: DualCameraVisionController,
    *,
    navigation_config: str | Path = PROJECT_ROOT / "config" / "navigation.json",
    ops9_config: str | Path = PROJECT_ROOT / "config" / "ops9.json",
    safety_config: str | Path = PROJECT_ROOT / "config" / "navigation_safety.json",
) -> NavigationRuntime:
    if vision.obstacle_source is None or vision.road_detector is None:
        raise ValueError(
            "视觉控制器未启用导航感知；构建时需要 "
            "enable_navigation_perception=True 且完成现场标定"
        )

    stack = build_stm32_navigation(
        link,
        navigation_config=navigation_config,
        ops9_config=ops9_config,
        obstacle_reader=vision.obstacle_source.obstacles,
        perception_updater=vision.observe_navigation,
    )

    def latest_road_observation():
        result = vision.latest_navigation_result
        return None if result is None else result.road_observation

    safety = NavigationSafetyMonitor(
        stack.navigator.map,
        stack.pose_reader,
        stack.chassis,
        link_health=lambda: link.connected,
        camera_health=vision.is_healthy,
        road_observation_reader=latest_road_observation,
        obstacle_candidate_reader=vision.obstacle_source.candidates,
        config=NavigationSafetyConfig.load(safety_config),
    )
    return NavigationRuntime(stack, vision, safety)
