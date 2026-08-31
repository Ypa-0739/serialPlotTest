"""OpenCV 二维码解码和连续帧确认。"""

from __future__ import annotations

from collections import OrderedDict
from typing import Any, Optional

from .models import CameraFrame, QrObservation, VisionObservation


class QrScanner:
    def __init__(self, detector: Optional[Any] = None, *, confirmations: int = 2) -> None:
        if confirmations < 1:
            raise ValueError("confirmations must be positive")
        if detector is None:
            try:
                import cv2
            except ImportError as exc:
                raise RuntimeError("QR scanning requires OpenCV") from exc
            detector = cv2.QRCodeDetector()
        self.detector = detector
        self.confirmations = confirmations
        self._candidate: Optional[str] = None
        self._count = 0

    def reset(self) -> None:
        self._candidate = None
        self._count = 0

    def decode(self, image: Any) -> tuple[str, ...]:
        if image is None or not hasattr(image, "shape"):
            raise ValueError("image must be an OpenCV-compatible frame")
        decoded: list[str] = []
        multi = getattr(self.detector, "detectAndDecodeMulti", None)
        if multi is not None:
            result = multi(image)
            if isinstance(result, tuple) and len(result) >= 2 and result[0]:
                decoded.extend(result[1] or ())
        if not decoded:
            result = self.detector.detectAndDecode(image)
            if isinstance(result, tuple) and result and result[0]:
                decoded.append(result[0])
        unique: OrderedDict[str, None] = OrderedDict()
        for value in decoded:
            text = value.strip() if isinstance(value, str) else ""
            if text:
                unique.setdefault(text, None)
        return tuple(unique)

    def detect(self, frame: CameraFrame) -> Optional[QrObservation]:
        values = self.decode(frame.image)
        if len(values) != 1:
            self.reset()
            return None
        value = values[0]
        if value == self._candidate:
            self._count += 1
        else:
            self._candidate = value
            self._count = 1
        confirmed = self._count >= self.confirmations
        return QrObservation(
            timestamp=frame.timestamp,
            frame_id=frame.frame_id,
            text=value,
            confidence=min(1.0, self._count / self.confirmations),
            confirmed=confirmed,
            source=frame.source,
        )

    @staticmethod
    def as_common(observation: QrObservation) -> VisionObservation:
        return VisionObservation(
            timestamp=observation.timestamp,
            frame_id=observation.frame_id,
            target_type="task_code",
            confidence=observation.confidence,
            x_mm=None,
            y_mm=None,
            yaw_deg=None,
            valid=observation.confirmed,
            source=observation.source,
            target_id=observation.text,
        )
