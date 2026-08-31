"""黑色圆柱检测、像素到地图投影和多帧确认。"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from .models import CameraFrame, ObstacleObservation, VisionObservation


@dataclass(frozen=True)
class MapPose:
    x_mm: float
    y_mm: float
    yaw_deg: float


class ObstacleDetector:
    def __init__(self, config: Mapping[str, Any]) -> None:
        self.config = dict(config)
        if bool(self.config.get("calibration_required", True)):
            raise ValueError("obstacle homography is not calibrated")
        matrix = self.config.get("homography_image_to_base")
        if not isinstance(matrix, list) or len(matrix) != 3 or any(
            not isinstance(row, list) or len(row) != 3 for row in matrix
        ):
            raise ValueError("homography_image_to_base must be 3x3")
        self.h = tuple(float(value) for row in matrix for value in row)
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("obstacle detection requires OpenCV and numpy") from exc
        # 缓存模块引用，避免在每个视频帧中重复执行导入路径。
        self._cv2 = cv2
        self._np = np

    def detect(self, frame: CameraFrame, pose: MapPose) -> tuple[ObstacleObservation, ...]:
        cv2 = self._cv2
        np = self._np
        image = np.asarray(frame.image)
        order = str(self.config.get("input_color_order", "RGB")).upper()
        if image.ndim == 3:
            gray = cv2.cvtColor(
                image, cv2.COLOR_RGB2GRAY if order == "RGB" else cv2.COLOR_BGR2GRAY
            )
        elif image.ndim == 2:
            gray = image
        else:
            raise ValueError("obstacle frame must be grayscale or three-channel")
        mask = cv2.inRange(gray, 0, int(self.config["black_threshold"]))
        mask[: int(gray.shape[0] * float(self.config["roi_top_fraction"])), :] = 0
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        yaw = math.radians(pose.yaw_deg)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        found: list[ObstacleObservation] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not float(self.config["minimum_area_px"]) <= area <= float(
                self.config["maximum_area_px"]
            ):
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if not width or not height:
                continue
            aspect = width / height
            if not float(self.config["minimum_aspect_ratio"]) <= aspect <= float(
                self.config["maximum_aspect_ratio"]
            ):
                continue
            base_x, base_y = self._project(x + width / 2.0, y + height)
            map_x = pose.x_mm + cosine * base_x - sine * base_y
            map_y = pose.y_mm + sine * base_x + cosine * base_y
            found.append(
                ObstacleObservation(
                    frame.timestamp,
                    frame.frame_id,
                    map_x,
                    map_y,
                    float(self.config["physical_radius_mm"])
                    + float(self.config["projection_uncertainty_mm"]),
                    min(1.0, area / float(width * height)),
                    False,
                    frame.source,
                )
            )
        return tuple(found)

    def _project(self, x: float, y: float) -> tuple[float, float]:
        h = self.h
        denominator = h[6] * x + h[7] * y + h[8]
        if abs(denominator) < 1e-9:
            raise ValueError("homography projects point to infinity")
        return (
            (h[0] * x + h[1] * y + h[2]) / denominator,
            (h[3] * x + h[4] * y + h[5]) / denominator,
        )


@dataclass
class _Track:
    observation: ObstacleObservation
    confirmations: int


class ObstacleTracker:
    """多帧确认与短时保留。

    暂时离开画面的轨迹会保留原始 timestamp，绝不伪装成新观察；任务层仍须
    通过 VisionObservationGate 检查新鲜度。默认保留期与 0.5s 门禁一致。
    """

    def __init__(
        self,
        *,
        confirmations_required: int = 3,
        matching_distance_mm: float = 120.0,
        retention_seconds: float = 0.5,
    ) -> None:
        self.confirmations_required = confirmations_required
        self.matching_distance_mm = matching_distance_mm
        self.retention_seconds = retention_seconds
        self._tracks: list[_Track] = []
        self._lock = threading.Lock()

    def update(
        self, observations: Sequence[ObstacleObservation], now: float
    ) -> tuple[ObstacleObservation, ...]:
        with self._lock:
            self._tracks = [
                track
                for track in self._tracks
                if now - track.observation.timestamp <= self.retention_seconds
            ]
            matched: set[int] = set()
            for observation in observations:
                best = None
                distance_limit = self.matching_distance_mm
                for index, track in enumerate(self._tracks):
                    if index in matched:
                        continue
                    distance = math.hypot(
                        observation.x_mm - track.observation.x_mm,
                        observation.y_mm - track.observation.y_mm,
                    )
                    if distance <= distance_limit:
                        best, distance_limit = index, distance
                if best is None:
                    self._tracks.append(_Track(observation, 1))
                    matched.add(len(self._tracks) - 1)
                else:
                    self._tracks[best].observation = observation
                    self._tracks[best].confirmations += 1
                    matched.add(best)
            return tuple(
                ObstacleObservation(
                    track.observation.timestamp,
                    track.observation.frame_id,
                    track.observation.x_mm,
                    track.observation.y_mm,
                    track.observation.radius_mm,
                    track.observation.confidence,
                    True,
                    track.observation.source,
                )
                for track in self._tracks
                if track.confirmations >= self.confirmations_required
            )

    @staticmethod
    def as_common(observation: ObstacleObservation) -> VisionObservation:
        return VisionObservation(
            observation.timestamp,
            observation.frame_id,
            "obstacle",
            observation.confidence,
            observation.x_mm,
            observation.y_mm,
            None,
            observation.confirmed,
            observation.source,
        )
