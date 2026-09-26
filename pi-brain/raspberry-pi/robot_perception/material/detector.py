"""抓取摄像头的物料颜色识别与二维对准结果。"""

from dataclasses import dataclass
from math import hypot
from typing import Any, Mapping, Optional, Tuple


@dataclass(frozen=True)
class MaterialObservation:
    """一件物料相对夹爪抓取中心的位置与颜色信息。"""

    material_code: int
    color_name: str
    color_cn_name: str
    center: Tuple[int, int]
    offset_pixels: Tuple[float, float]
    normalized_offset: Tuple[float, float]
    box: Tuple[int, int, int, int]
    area: float
    confirmed: bool


@dataclass(frozen=True)
class GripperDetection:
    """抓取相机单次识别结果。"""

    status: str
    target_material_code: Optional[int]
    observation: Optional[MaterialObservation]
    aligned: bool
    safe_to_pick: bool
    detected_color_codes: Tuple[int, ...]
    masks: Any = None


class GripperMaterialDetector:
    """在颜色检测结果中选择目标，并计算相对夹爪中心的像素偏差。"""

    def __init__(self, color_detector, config: Mapping[str, Any]):
        self.color_detector = color_detector
        grip_center = config.get("grip_center")
        tolerance = config.get("alignment_tolerance_pixels", (18, 18))
        if (
            not isinstance(grip_center, (list, tuple))
            or len(grip_center) != 2
        ):
            raise ValueError("grip_center 必须是两个数字")
        if not isinstance(tolerance, (list, tuple)) or len(tolerance) != 2:
            raise ValueError("alignment_tolerance_pixels 必须是两个数字")
        if any(
            not isinstance(value, (int, float)) or isinstance(value, bool)
            for value in (*grip_center, *tolerance)
        ):
            raise ValueError("抓取中心和对准容差必须是数字")
        if any(value < 0 for value in tolerance):
            raise ValueError("对准容差不能小于0")

        self.grip_center = (float(grip_center[0]), float(grip_center[1]))
        self.tolerance = (float(tolerance[0]), float(tolerance[1]))
        self.require_global_ready = bool(config.get("require_global_ready", False))

    def detect(
        self,
        frame,
        target_material_code: Optional[int] = None,
        collect_masks: bool = False,
    ) -> GripperDetection:
        if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 2:
            raise ValueError("frame 必须是有效图像")
        if target_material_code is not None and (
            not isinstance(target_material_code, int)
            or isinstance(target_material_code, bool)
            or target_material_code <= 0
        ):
            raise ValueError("target_material_code 必须是正整数或None")

        state, masks = self.color_detector.detect(
            frame,
            collect_masks=collect_masks,
        )
        detections = tuple(state.get("detections", ()))
        detected_codes = tuple(
            sorted({int(item["code"]) for item in detections})
        )
        candidates = [
            item
            for item in detections
            if target_material_code is None
            or int(item["code"]) == target_material_code
        ]
        if not candidates:
            return GripperDetection(
                status="TARGET_NOT_FOUND" if target_material_code else "SEARCHING",
                target_material_code=target_material_code,
                observation=None,
                aligned=False,
                safe_to_pick=False,
                detected_color_codes=detected_codes,
                masks=masks if collect_masks else None,
            )

        center_x, center_y = self.grip_center
        candidates.sort(
            key=lambda item: (
                not bool(item.get("confirmed", False)),
                hypot(
                    float(item["center"][0]) - center_x,
                    float(item["center"][1]) - center_y,
                ),
                -float(item.get("area", 0.0)),
            )
        )
        selected = candidates[0]
        object_x = int(selected["center"][0])
        object_y = int(selected["center"][1])
        offset_x = object_x - center_x
        offset_y = object_y - center_y
        frame_height, frame_width = frame.shape[:2]
        normalized_x = offset_x / max(frame_width / 2.0, 1.0)
        normalized_y = offset_y / max(frame_height / 2.0, 1.0)
        confirmed = bool(selected.get("confirmed", False))
        aligned = (
            abs(offset_x) <= self.tolerance[0]
            and abs(offset_y) <= self.tolerance[1]
        )
        observation = MaterialObservation(
            material_code=int(selected["code"]),
            color_name=str(selected.get("name", "")),
            color_cn_name=str(selected.get("cn_name", "")),
            center=(object_x, object_y),
            offset_pixels=(float(offset_x), float(offset_y)),
            normalized_offset=(float(normalized_x), float(normalized_y)),
            box=tuple(int(value) for value in selected["box"]),
            area=float(selected.get("area", 0.0)),
            confirmed=confirmed,
        )
        global_status = str(state.get("status", "HOLD"))
        global_ready = bool(state.get("safe_to_pick", False))
        color_state_allows_pick = (
            global_ready
            if self.require_global_ready
            else global_status != "AMBIGUOUS"
        )
        safe_to_pick = confirmed and aligned and color_state_allows_pick
        if not confirmed:
            status = "CONFIRMING_COLOR"
        elif not aligned:
            status = "ALIGNING"
        elif safe_to_pick:
            status = "READY"
        else:
            status = global_status

        return GripperDetection(
            status=status,
            target_material_code=target_material_code,
            observation=observation,
            aligned=aligned,
            safe_to_pick=safe_to_pick,
            detected_color_codes=detected_codes,
            masks=masks if collect_masks else None,
        )
