"""树莓派 5 双摄像头的唯一生命周期持有者。"""

from __future__ import annotations

import itertools
import threading
import time
from typing import Any, Callable, Mapping, Optional

from .models import CameraFrame


class CameraError(RuntimeError):
    pass


class CameraConfigError(ValueError):
    pass


def validate_camera_config(config: Mapping[str, Any]) -> dict[str, Any]:
    if config.get("platform") != "raspberry_pi_5":
        raise CameraConfigError("platform must be raspberry_pi_5")
    result = dict(config)
    for role in ("front", "gripper"):
        section = result.get(role)
        if not isinstance(section, Mapping):
            raise CameraConfigError(f"{role} camera config must be an object")
        section = dict(section)
        if (
            not isinstance(section.get("camera_num"), int)
            or isinstance(section["camera_num"], bool)
            or section["camera_num"] < 0
        ):
            raise CameraConfigError(f"{role}.camera_num must be a non-negative integer")
        size = section.get("frame_size")
        if not isinstance(size, list) or len(size) != 2 or any(
            not isinstance(value, int) or value <= 0 for value in size
        ):
            raise CameraConfigError(f"{role}.frame_size must contain two positive integers")
        if float(section.get("fps", 0)) <= 0:
            raise CameraConfigError(f"{role}.fps must be positive")
        if (
            not isinstance(section.get("buffer_count"), int)
            or isinstance(section["buffer_count"], bool)
            or section["buffer_count"] <= 0
        ):
            raise CameraConfigError(f"{role}.buffer_count must be positive")
        result[role] = section
    if result["front"]["camera_num"] == result["gripper"]["camera_num"]:
        raise CameraConfigError("front and gripper camera_num must differ")
    return result


class PiCamera:
    """Picamera2 的同步采集封装；调用者负责把采集放在视觉工作线程。"""

    def __init__(
        self,
        role: str,
        config: Mapping[str, Any],
        *,
        device_factory: Optional[Callable[[int], Any]] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.role = role
        self.config = dict(config)
        self.camera_num = int(self.config["camera_num"])
        self._device_factory = device_factory
        self._clock = clock
        self._device: Optional[Any] = None
        self._started = False
        self._last_frame_time: Optional[float] = None
        self._frames = itertools.count()
        self._lock = threading.RLock()

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            factory = self._device_factory
            if factory is None:
                try:
                    from picamera2 import Picamera2
                except ImportError as exc:
                    raise CameraError("picamera2 is not installed") from exc
                factory = Picamera2
            device = None
            try:
                device = factory(self.camera_num)
                camera_config = device.create_preview_configuration(
                    main={
                        "size": tuple(self.config["frame_size"]),
                        "format": str(self.config.get("format", "RGB888")),
                    },
                    raw=None,
                    buffer_count=int(self.config.get("buffer_count", 3)),
                    controls={
                        **dict(self.config.get("controls", {})),
                        "FrameRate": float(self.config["fps"]),
                    },
                    display=None,
                    encode=None,
                    queue=False,
                )
                device.configure(camera_config)
                device.start()
            except Exception as exc:
                if device is not None:
                    try:
                        device.close()
                    except Exception:
                        pass
                raise CameraError(
                    f"failed to start {self.role} camera #{self.camera_num}: {exc}"
                ) from exc
            self._device = device
            self._started = True
            self._last_frame_time = None

    def capture(self, stream: str = "main") -> CameraFrame:
        with self._lock:
            if not self._started or self._device is None:
                raise CameraError(f"{self.role} camera is not started")
            try:
                image = self._device.capture_array(stream)
            except Exception as exc:
                raise CameraError(f"{self.role} camera capture failed: {exc}") from exc
            captured_at = self._clock()
            self._last_frame_time = captured_at
            return CameraFrame(captured_at, next(self._frames), self.role, image)

    def is_healthy(self, stale_after_s: float = 1.0) -> bool:
        with self._lock:
            return (
                self._started
                and self._last_frame_time is not None
                and stale_after_s > 0.0
                and self._clock() - self._last_frame_time <= stale_after_s
            )

    def close(self) -> None:
        with self._lock:
            device, self._device = self._device, None
            self._started = False
            self._last_frame_time = None
            if device is not None:
                try:
                    device.stop()
                finally:
                    device.close()


class DualCameraManager:
    ROLES = ("front", "gripper")

    def __init__(self, config: Mapping[str, Any], camera_factory=PiCamera) -> None:
        checked = validate_camera_config(config)
        self.front = camera_factory("front", checked["front"])
        self.gripper = camera_factory("gripper", checked["gripper"])
        self._active: set[str] = set()

    def camera(self, role: str) -> PiCamera:
        if role not in self.ROLES:
            raise KeyError(f"unknown camera role: {role}")
        return getattr(self, role)

    def start(self, roles: tuple[str, ...] = ROLES) -> None:
        unknown = set(roles) - set(self.ROLES)
        if not roles or unknown:
            raise ValueError(f"invalid camera roles: {sorted(unknown)}")
        started: list[str] = []
        try:
            for role in roles:
                if role in self._active:
                    continue
                self.camera(role).start()
                self._active.add(role)
                started.append(role)
        except Exception:
            for role in reversed(started):
                self.camera(role).close()
                self._active.discard(role)
            raise

    def is_healthy(self, stale_after_s: float = 1.0) -> bool:
        return bool(self._active) and all(
            self.camera(role).is_healthy(stale_after_s) for role in self._active
        )

    def close(self) -> None:
        first_error: Optional[Exception] = None
        for role in reversed(self.ROLES):
            try:
                self.camera(role).close()
            except Exception as exc:
                first_error = first_error or exc
        self._active.clear()
        if first_error is not None:
            raise CameraError(f"failed to close cameras: {first_error}")
