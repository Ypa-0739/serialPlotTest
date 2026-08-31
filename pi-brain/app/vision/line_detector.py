"""图像巡线检测器；不包含方向决策或速度输出。"""

from __future__ import annotations

from typing import Any, Mapping

from .models import CameraFrame, LineObservation, VisionObservation


class LineDetector:
    def __init__(self, config: Mapping[str, Any]) -> None:
        self.roi_top_ratio = float(config.get("roi_top_ratio", 0.55))
        self.line_is_dark = bool(config.get("line_is_dark", True))
        self.min_area = float(config.get("min_line_area", 300))
        self.max_area_ratio = float(config.get("max_line_area_ratio", 0.8))
        kernel_size = int(config.get("morph_kernel_size", 5))
        if not 0.0 < self.roi_top_ratio < 1.0:
            raise ValueError("roi_top_ratio must be within (0, 1)")
        if kernel_size <= 0 or kernel_size % 2 == 0:
            raise ValueError("morph_kernel_size must be a positive odd integer")
        import numpy as np

        self._kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

    def detect(self, frame: CameraFrame) -> LineObservation:
        import cv2

        image = frame.image
        height, width = image.shape[:2]
        roi_top = int(height * self.roi_top_ratio)
        roi = image[roi_top:, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        mode = cv2.THRESH_BINARY_INV if self.line_is_dark else cv2.THRESH_BINARY
        _, mask = cv2.threshold(blurred, 0, 255, mode | cv2.THRESH_OTSU)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_area = width * max(1, height - roi_top)
        valid = [
            contour
            for contour in contours
            if self.min_area <= cv2.contourArea(contour) <= roi_area * self.max_area_ratio
        ]
        if not valid:
            return LineObservation(frame.timestamp, frame.frame_id, False, None, None, 0.0, 0.0, frame.source)
        contour = max(valid, key=cv2.contourArea)
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return LineObservation(frame.timestamp, frame.frame_id, False, None, None, 0.0, 0.0, frame.source)
        center = (
            int(moments["m10"] / moments["m00"]),
            int(moments["m01"] / moments["m00"]) + roi_top,
        )
        area = float(cv2.contourArea(contour))
        return LineObservation(
            frame.timestamp,
            frame.frame_id,
            True,
            float((center[0] - width / 2.0) / max(width / 2.0, 1.0)),
            center,
            area,
            min(1.0, area / max(self.min_area * 2.0, 1.0)),
            frame.source,
        )

    @staticmethod
    def as_common(observation: LineObservation) -> VisionObservation:
        return VisionObservation(
            observation.timestamp, observation.frame_id, "guide_line",
            observation.confidence, None, None, None, observation.found,
            observation.source,
        )
