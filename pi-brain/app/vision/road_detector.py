"""前视灰色可行区与黄/白禁入区检测。"""

from __future__ import annotations

from typing import Any, Mapping

from .models import CameraFrame, RoadObservation, VisionObservation


class RoadDetector:
    def __init__(self, config: Mapping[str, Any]) -> None:
        self.config = dict(config)
        if bool(self.config.get("calibration_required", True)):
            raise ValueError("road thresholds are not calibrated")
        self.input_color_order = str(self.config.get("input_color_order", "RGB")).upper()
        if self.input_color_order not in {"RGB", "BGR"}:
            raise ValueError("input_color_order must be RGB or BGR")

    def detect(self, frame: CameraFrame) -> RoadObservation:
        import cv2
        import numpy as np

        image = np.asarray(frame.image)
        if image.ndim != 3:
            raise ValueError("road detector requires a three-channel image")
        conversion = cv2.COLOR_RGB2HSV if self.input_color_order == "RGB" else cv2.COLOR_BGR2HSV
        hsv = cv2.cvtColor(image, conversion)
        height, width = hsv.shape[:2]
        roi_top = int(height * float(self.config["roi_top_fraction"]))
        roi = hsv[roi_top:, :]
        if roi.size == 0:
            raise ValueError("road ROI is empty")

        def mask(name: str):
            section = self.config[name]
            return cv2.inRange(
                roi,
                np.asarray(section["lower"], dtype=np.uint8),
                np.asarray(section["upper"], dtype=np.uint8),
            )

        gray = mask("gray_hsv")
        gray = cv2.morphologyEx(
            gray, cv2.MORPH_CLOSE, np.ones((5, 5), dtype=np.uint8)
        )
        forbidden = cv2.bitwise_or(mask("yellow_hsv"), mask("white_hsv"))
        drivable = float(cv2.countNonZero(gray)) / float(gray.size)
        corridor_width = max(1, int(width * float(self.config["central_corridor_fraction"])))
        left = (width - corridor_width) // 2
        corridor = gray[:, left : left + corridor_width]
        forbidden_corridor = forbidden[:, left : left + corridor_width]
        central = float(cv2.countNonZero(corridor)) / float(corridor.size)
        forbidden_fraction = float(cv2.countNonZero(forbidden_corridor)) / float(
            forbidden_corridor.size
        )
        safe = (
            drivable >= float(self.config["minimum_drivable_fraction"])
            and central >= float(self.config["minimum_central_drivable_fraction"])
            and forbidden_fraction <= float(self.config["maximum_forbidden_fraction"])
        )
        confidence = max(0.0, min(1.0, 0.6 * central + 0.4 * drivable - forbidden_fraction))
        return RoadObservation(
            frame.timestamp,
            frame.frame_id,
            drivable,
            forbidden_fraction,
            central,
            safe,
            confidence,
            frame.source,
        )

    @staticmethod
    def as_common(observation: RoadObservation) -> VisionObservation:
        return VisionObservation(
            observation.timestamp,
            observation.frame_id,
            "road_boundary",
            observation.confidence,
            None,
            None,
            None,
            observation.boundary_safe,
            observation.source,
        )
