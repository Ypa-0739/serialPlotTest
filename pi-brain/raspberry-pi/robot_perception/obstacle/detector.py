"""已知黑色圆柱的可解释视觉检测。

相机单应矩阵把轮廓底部接地点从像素投影到车体坐标，再利用 OPS9 地图位姿
转到全局坐标。正式上车前必须用实际相机重新标定配置中的矩阵。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
from typing import Sequence


@dataclass(frozen=True)
class DetectedObstacle:
    x_mm: float
    y_mm: float
    radius_mm: float
    observed_at: float
    confidence: float


@dataclass(frozen=True)
class ObstacleDetectorConfig:
    input_color_order: str
    black_threshold: int
    minimum_area_px: float
    maximum_area_px: float
    minimum_aspect_ratio: float
    maximum_aspect_ratio: float
    roi_top_fraction: float
    physical_radius_mm: float
    projection_uncertainty_mm: float
    homography_image_to_base: tuple[float, ...]
    calibration_required: bool = True

    @classmethod
    def load(cls, path: str | Path) -> "ObstacleDetectorConfig":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        matrix = tuple(float(value) for row in data["homography_image_to_base"] for value in row)
        if len(matrix) != 9:
            raise ValueError("homography_image_to_base 必须是 3x3 矩阵")
        return cls(
            input_color_order=str(data.get("input_color_order", "RGB")).upper(),
            black_threshold=int(data["black_threshold"]),
            minimum_area_px=float(data["minimum_area_px"]),
            maximum_area_px=float(data["maximum_area_px"]),
            minimum_aspect_ratio=float(data["minimum_aspect_ratio"]),
            maximum_aspect_ratio=float(data["maximum_aspect_ratio"]),
            roi_top_fraction=float(data["roi_top_fraction"]),
            physical_radius_mm=float(data["physical_radius_mm"]),
            projection_uncertainty_mm=float(data["projection_uncertainty_mm"]),
            homography_image_to_base=matrix,
            calibration_required=bool(data.get("calibration_required", True)),
        )


class CameraObstacleDetector:
    def __init__(self, config: ObstacleDetectorConfig) -> None:
        if config.calibration_required:
            raise ValueError(
                "前视相机单应矩阵仍是占位值；完成实车标定并将 "
                "calibration_required 设为 false 后才能启用障碍检测"
            )
        self.config = config

    def detect(
        self,
        frame: object,
        robot_pose: object,
        *,
        observed_at: float,
    ) -> list[DetectedObstacle]:
        """返回地图坐标障碍；robot_pose 需提供 x_mm、y_mm、yaw_rad。"""

        try:
            import cv2  # type: ignore[import-not-found]
            import numpy as np  # type: ignore[import-not-found]
        except ImportError as error:
            raise RuntimeError("障碍检测需要安装 numpy 和 opencv-python") from error
        image = np.asarray(frame)
        if image.ndim == 3:
            conversion = (
                cv2.COLOR_RGB2GRAY
                if self.config.input_color_order == "RGB"
                else cv2.COLOR_BGR2GRAY
            )
            gray = cv2.cvtColor(image, conversion)
        elif image.ndim == 2:
            gray = image
        else:
            raise ValueError("frame 必须是灰度或 BGR 图像")
        height, width = gray.shape[:2]
        roi_top = int(height * self.config.roi_top_fraction)
        mask = cv2.inRange(gray, 0, self.config.black_threshold)
        mask[:roi_top, :] = 0
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result: list[DetectedObstacle] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not self.config.minimum_area_px <= area <= self.config.maximum_area_px:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_width == 0 or box_height == 0:
                continue
            aspect = box_width / box_height
            if not self.config.minimum_aspect_ratio <= aspect <= self.config.maximum_aspect_ratio:
                continue
            base_x, base_y = self._project(x + box_width / 2.0, y + box_height)
            cosine, sine = math.cos(robot_pose.yaw_rad), math.sin(robot_pose.yaw_rad)
            map_x = robot_pose.x_mm + cosine * base_x - sine * base_y
            map_y = robot_pose.y_mm + sine * base_x + cosine * base_y
            fill_ratio = min(1.0, area / float(box_width * box_height))
            result.append(
                DetectedObstacle(
                    map_x,
                    map_y,
                    self.config.physical_radius_mm + self.config.projection_uncertainty_mm,
                    observed_at,
                    fill_ratio,
                )
            )
        return result

    def _project(self, pixel_x: float, pixel_y: float) -> tuple[float, float]:
        h = self.config.homography_image_to_base
        denominator = h[6] * pixel_x + h[7] * pixel_y + h[8]
        if abs(denominator) < 1e-9:
            raise ValueError("障碍接地点落在单应矩阵无穷远处")
        return (
            (h[0] * pixel_x + h[1] * pixel_y + h[2]) / denominator,
            (h[3] * pixel_x + h[4] * pixel_y + h[5]) / denominator,
        )


@dataclass
class _Track:
    obstacle: DetectedObstacle
    confirmations: int


class ConfirmedObstacleTracker:
    """近邻多帧确认；未确认候选应由近距离制动层另行处理。"""

    def __init__(
        self,
        *,
        confirmations_required: int = 3,
        matching_distance_mm: float = 120.0,
        retention_seconds: float = 1.0,
    ) -> None:
        self.confirmations_required = confirmations_required
        self.matching_distance_mm = matching_distance_mm
        self.retention_seconds = retention_seconds
        self._tracks: list[_Track] = []

    def update(
        self,
        detections: Sequence[DetectedObstacle],
        now: float,
    ) -> tuple[DetectedObstacle, ...]:
        self._tracks = [
            track
            for track in self._tracks
            if now - track.obstacle.observed_at <= self.retention_seconds
        ]
        matched: set[int] = set()
        for detection in detections:
            best_index = None
            best_distance = self.matching_distance_mm
            for index, track in enumerate(self._tracks):
                if index in matched:
                    continue
                distance = math.hypot(
                    detection.x_mm - track.obstacle.x_mm,
                    detection.y_mm - track.obstacle.y_mm,
                )
                if distance <= best_distance:
                    best_index, best_distance = index, distance
            if best_index is None:
                self._tracks.append(_Track(detection, 1))
                matched.add(len(self._tracks) - 1)
            else:
                track = self._tracks[best_index]
                track.obstacle = detection
                track.confirmations += 1
                matched.add(best_index)
        return tuple(
            track.obstacle
            for track in self._tracks
            if track.confirmations >= self.confirmations_required
        )


class FrontCameraObstacleSource:
    """连接相机循环与导航循环的线程安全动态障碍源。"""

    def __init__(
        self,
        detector: CameraObstacleDetector,
        tracker: ConfirmedObstacleTracker,
    ) -> None:
        self.detector = detector
        self.tracker = tracker
        self._lock = threading.Lock()
        self._confirmed: tuple[DetectedObstacle, ...] = ()
        self.latest_candidates: tuple[DetectedObstacle, ...] = ()

    def update(self, frame: object, robot_pose: object, *, observed_at: float) -> None:
        candidates = tuple(
            self.detector.detect(frame, robot_pose, observed_at=observed_at)
        )
        confirmed = self.tracker.update(candidates, observed_at)
        with self._lock:
            self.latest_candidates = candidates
            self._confirmed = confirmed

    def obstacles(self) -> tuple[DetectedObstacle, ...]:
        with self._lock:
            return self._confirmed

    def candidates(self) -> tuple[DetectedObstacle, ...]:
        with self._lock:
            return self.latest_candidates
