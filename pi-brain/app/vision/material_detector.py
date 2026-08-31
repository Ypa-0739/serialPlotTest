"""物料颜色选择和夹爪像素对准。"""

from __future__ import annotations

from math import hypot
from typing import Any, Mapping, Optional

from .color_detector import ColorDetector
from .models import CameraFrame, MaterialObservation, VisionObservation


class MaterialDetector:
    def __init__(self, color: ColorDetector, config: Mapping[str, Any]) -> None:
        self.color = color
        center = config.get("grip_center")
        tolerance = config.get("alignment_tolerance_pixels", (18, 18))
        if not isinstance(center, (list, tuple)) or len(center) != 2:
            raise ValueError("grip_center must contain two values")
        if not isinstance(tolerance, (list, tuple)) or len(tolerance) != 2:
            raise ValueError("alignment_tolerance_pixels must contain two values")
        self.center = (float(center[0]), float(center[1]))
        self.tolerance = (float(tolerance[0]), float(tolerance[1]))
        self.require_global_ready = bool(config.get("require_global_ready", False))

    def detect(
        self, frame: CameraFrame, target_code: Optional[int] = None
    ) -> Optional[MaterialObservation]:
        result = self.color.detect(frame.image)
        candidates = [
            item for item in result.candidates if target_code is None or item.code == target_code
        ]
        if not candidates:
            return None
        candidates.sort(
            key=lambda item: (
                not item.confirmed,
                hypot(item.center_px[0] - self.center[0], item.center_px[1] - self.center[1]),
                -item.area_px,
            )
        )
        item = candidates[0]
        offset = (item.center_px[0] - self.center[0], item.center_px[1] - self.center[1])
        height, width = frame.image.shape[:2]
        normalized = (
            offset[0] / max(width / 2.0, 1.0),
            offset[1] / max(height / 2.0, 1.0),
        )
        aligned = (
            abs(offset[0]) <= self.tolerance[0]
            and abs(offset[1]) <= self.tolerance[1]
        )
        color_allowed = result.safe_to_pick if self.require_global_ready else result.status != "AMBIGUOUS"
        safe = item.confirmed and aligned and color_allowed
        return MaterialObservation(
            timestamp=frame.timestamp,
            frame_id=frame.frame_id,
            material_code=item.code,
            color_name=item.name,
            color_cn_name=item.cn_name,
            center_px=item.center_px,
            offset_px=offset,
            normalized_offset=normalized,
            box_px=item.box_px,
            area_px=item.area_px,
            confidence=item.confidence,
            confirmed=item.confirmed,
            aligned=aligned,
            safe_to_pick=safe,
            source=frame.source,
        )

    @staticmethod
    def as_common(observation: MaterialObservation) -> VisionObservation:
        return VisionObservation(
            timestamp=observation.timestamp,
            frame_id=observation.frame_id,
            target_type=f"material:{observation.material_code}",
            confidence=observation.confidence,
            x_mm=None,
            y_mm=None,
            yaw_deg=None,
            # 任务层的 common.valid 表示可用于决策，不只是“看见过”。夹爪物料
            # 必须同时满足多帧确认、像素对准和当前画面无歧义。
            valid=observation.safe_to_pick,
            source=observation.source,
            target_id=str(observation.material_code),
        )
