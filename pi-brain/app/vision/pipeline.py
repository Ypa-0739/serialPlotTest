"""双摄像头与纯视觉检测器的组合层；不包含任何运动控制。"""

from __future__ import annotations

from typing import Any, Callable, Iterable, Optional

from .camera import DualCameraManager
from .line_detector import LineDetector
from .material_detector import MaterialDetector
from .models import CameraFrame, VisionObservation
from .obstacle_detector import MapPose, ObstacleDetector, ObstacleTracker
from .qr_scanner import QrScanner
from .road_detector import RoadDetector


class VisionPipeline:
    """每次 poll 最多各采一帧；前摄像头上的算法共享同一帧。"""

    def __init__(
        self,
        cameras: DualCameraManager,
        *,
        material: Optional[MaterialDetector] = None,
        qr: Optional[QrScanner] = None,
        line: Optional[LineDetector] = None,
        road: Optional[RoadDetector] = None,
        obstacle: Optional[ObstacleDetector] = None,
        obstacle_tracker: Optional[ObstacleTracker] = None,
        pose_provider: Optional[Callable[[], Optional[MapPose]]] = None,
        target_material_provider: Optional[Callable[[], Optional[int]]] = None,
        gripper_calibrator: Optional[Callable[[object], Callable[[Any], Any]]] = None,
    ) -> None:
        self.cameras = cameras
        self.material = material
        self.qr = qr
        self.line = line
        self.road = road
        self.obstacle = obstacle
        self.obstacle_tracker = obstacle_tracker
        self.pose_provider = pose_provider
        self.target_material_provider = target_material_provider
        self.gripper_calibrator = gripper_calibrator
        self._gripper_transform: Callable[[Any], Any] = lambda image: image
        self._roles: tuple[str, ...] = ()

    def start(self) -> None:
        roles: list[str] = []
        if any((self.qr, self.line, self.road, self.obstacle)):
            roles.append("front")
        if self.material is not None:
            roles.append("gripper")
        if not roles:
            raise ValueError("vision pipeline has no enabled detector")
        self._roles = tuple(roles)
        self.cameras.start(self._roles)
        if "gripper" in self._roles and self.gripper_calibrator is not None:
            # 标定发生在视觉工作线程的 producer.start() 中，不阻塞主事件泵。
            self._gripper_transform = self.gripper_calibrator(self.cameras.gripper)

    def poll(self) -> Iterable[VisionObservation]:
        result: list[VisionObservation] = []
        if "front" in self._roles:
            frame = self.cameras.front.capture()
            if self.qr is not None:
                qr = self.qr.detect(frame)
                if qr is not None:
                    result.append(self.qr.as_common(qr))
            if self.line is not None:
                result.append(self.line.as_common(self.line.detect(frame)))
            if self.road is not None:
                result.append(self.road.as_common(self.road.detect(frame)))
            if self.obstacle is not None:
                pose = self.pose_provider() if self.pose_provider is not None else None
                if pose is not None:
                    candidates = self.obstacle.detect(frame, pose)
                    confirmed = (
                        self.obstacle_tracker.update(candidates, frame.timestamp)
                        if self.obstacle_tracker is not None
                        else candidates
                    )
                    result.extend(
                        ObstacleTracker.as_common(item) for item in confirmed
                    )
        if "gripper" in self._roles and self.material is not None:
            frame = self.cameras.gripper.capture()
            frame = CameraFrame(
                frame.timestamp,
                frame.frame_id,
                frame.source,
                self._gripper_transform(frame.image),
            )
            target = (
                self.target_material_provider()
                if self.target_material_provider is not None
                else None
            )
            material = self.material.detect(frame, target)
            if material is not None:
                result.append(self.material.as_common(material))
        return tuple(result)

    def close(self) -> None:
        self.cameras.close()
        self._roles = ()
        self._gripper_transform = lambda image: image
