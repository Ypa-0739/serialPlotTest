"""从本地配置创建视觉流水线；硬件模块在调用时延迟导入。"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional

from .camera import DualCameraManager
from .color_detector import (
    ColorDetector,
    apply_white_balance,
    build_white_balance_luts,
    estimate_white_balance,
)
from .config import load_json_config
from .line_detector import LineDetector
from .material_detector import MaterialDetector
from .obstacle_detector import MapPose, ObstacleDetector, ObstacleTracker
from .pipeline import VisionPipeline
from .qr_scanner import QrScanner
from .road_detector import RoadDetector


def build_vision_pipeline(
    *,
    enable_material: bool = True,
    enable_qr: bool = True,
    enable_line: bool = False,
    enable_road: bool = False,
    enable_obstacle: bool = False,
    pose_provider: Optional[Callable[[], Optional[MapPose]]] = None,
    target_material_provider: Optional[Callable[[], Optional[int]]] = None,
    config_dir: Optional[str | Path] = None,
) -> VisionPipeline:
    def load(name: str):
        return load_json_config(name, Path(config_dir) / name if config_dir else None)

    camera_config = load("cameras.json")
    cameras = DualCameraManager(camera_config)
    material = None
    gripper_calibrator = None
    if enable_material:
        color_config = load("color.json")
        material = MaterialDetector(ColorDetector(color_config), camera_config["gripper"])
        white_balance = dict(color_config.get("white_balance", {}))

        def calibrate(camera):
            import numpy as np

            if not bool(white_balance.get("enabled", False)):
                return lambda image: image
            estimates = []
            for _ in range(int(white_balance.get("calibration_frames", 8))):
                estimate = estimate_white_balance(camera.capture().image, white_balance)
                if estimate is not None:
                    estimates.append(estimate)
            if not estimates:
                return lambda image: image
            luts = build_white_balance_luts(np.median(np.stack(estimates), axis=0))
            return lambda image: apply_white_balance(image, luts)

        gripper_calibrator = calibrate
    qr = QrScanner(confirmations=int(camera_config["front"].get("qr_confirmations", 2))) if enable_qr else None
    line = LineDetector(load("line.json")) if enable_line else None
    road = RoadDetector(load("road.json")) if enable_road else None
    obstacle = tracker = None
    if enable_obstacle:
        obstacle_config = load("obstacle.json")
        obstacle = ObstacleDetector(obstacle_config)
        tracker = ObstacleTracker(**obstacle_config.get("tracking", {}))
    return VisionPipeline(
        cameras,
        material=material,
        qr=qr,
        line=line,
        road=road,
        obstacle=obstacle,
        obstacle_tracker=tracker,
        pose_provider=pose_provider,
        target_material_provider=target_material_provider,
        gripper_calibrator=gripper_calibrator,
    )
