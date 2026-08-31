"""可离线测试的 OpenCV 颜色识别核心。"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
from typing import Any, Mapping, Optional


@dataclass(frozen=True)
class ColorCandidate:
    code: int
    name: str
    cn_name: str
    center_px: tuple[int, int]
    box_px: tuple[int, int, int, int]
    area_px: float
    fill_ratio: float
    solidity: float
    confidence: float
    confirmed: bool


@dataclass(frozen=True)
class ColorDetection:
    status: str
    safe_to_pick: bool
    confirmed_codes: tuple[int, ...]
    candidates: tuple[ColorCandidate, ...]
    masks: tuple[Any, ...] = ()


class ColorDetector:
    """HSV 阈值、轮廓质量和多帧投票组合的颜色检测器。"""

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.config = dict(config)
        self.detection = dict(self.config["detection"])
        self.input_color_order = str(
            self.config.get("input_color_order", "RGB")
        ).upper()
        if self.input_color_order not in {"RGB", "BGR"}:
            raise ValueError("input_color_order must be RGB or BGR")
        self.rules = tuple(dict(rule) for rule in self.config["colors"])
        self._history: deque[set[int]] = deque(
            maxlen=int(self.detection["history_length"])
        )
        kernel_size = int(self.detection["morph_kernel_size"])
        if kernel_size <= 0 or kernel_size % 2 == 0:
            raise ValueError("morph_kernel_size must be a positive odd integer")
        try:
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("color detection requires numpy") from exc
        self._kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

    def reset(self) -> None:
        self._history.clear()

    def detect(self, frame: Any, *, collect_masks: bool = False) -> ColorDetection:
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("color detection requires OpenCV and numpy") from exc
        image = np.asarray(frame)
        if image.ndim != 3 or image.shape[2] < 3:
            raise ValueError("frame must be a three-channel image")
        conversion = (
            cv2.COLOR_RGB2HSV
            if self.input_color_order == "RGB"
            else cv2.COLOR_BGR2HSV
        )
        hsv = cv2.cvtColor(image, conversion)
        height, width = hsv.shape[:2]
        max_area = height * width * float(self.detection["max_object_area_ratio"])
        raw: list[dict[str, Any]] = []
        masks: list[Any] = []
        for rule in self.rules:
            mask = None
            for lower, upper in rule["ranges"]:
                part = cv2.inRange(
                    hsv,
                    np.asarray(lower, dtype=np.uint8),
                    np.asarray(upper, dtype=np.uint8),
                )
                mask = part if mask is None else cv2.bitwise_or(mask, part)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
            if collect_masks:
                masks.append(mask)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            accepted: list[dict[str, Any]] = []
            strict = bool(rule.get("strict_shape_filter", False))
            min_area = float(
                self.detection[
                    "strict_min_object_area" if strict else "min_object_area"
                ]
            )
            min_fill = float(
                self.detection[
                    "strict_min_fill_ratio" if strict else "min_fill_ratio"
                ]
            )
            min_solidity = float(
                self.detection[
                    "strict_min_solidity" if strict else "min_solidity"
                ]
            )
            edge = int(self.detection["edge_margin"])
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if not min_area <= area <= max_area:
                    continue
                x, y, box_w, box_h = cv2.boundingRect(contour)
                if (
                    x <= edge
                    or y <= edge
                    or x + box_w >= width - edge
                    or y + box_h >= height - edge
                ):
                    continue
                aspect = box_w / max(box_h, 1)
                if not float(self.detection["min_aspect_ratio"]) <= aspect <= float(
                    self.detection["max_aspect_ratio"]
                ):
                    continue
                fill = area / max(float(box_w * box_h), 1.0)
                hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
                solidity = area / max(hull_area, 1.0)
                if fill < min_fill or solidity < min_solidity:
                    continue
                # 证据分数表达相对阈值余量，不把它解释为统计概率。
                fill_score = (fill - min_fill) / max(1.0 - min_fill, 1e-6)
                solidity_score = (solidity - min_solidity) / max(1.0 - min_solidity, 1e-6)
                shape_score = min(
                    1.0,
                    0.5 + 0.25 * max(0.0, fill_score)
                    + 0.25 * max(0.0, solidity_score),
                )
                accepted.append(
                    {
                        "code": int(rule["code"]),
                        "name": str(rule["name"]),
                        "cn_name": str(rule.get("cn_name", "")),
                        "center_px": (x + box_w // 2, y + box_h // 2),
                        "box_px": (x, y, box_w, box_h),
                        "area_px": area,
                        "fill_ratio": fill,
                        "solidity": solidity,
                        "shape_score": shape_score,
                    }
                )
            accepted.sort(key=lambda item: item["area_px"], reverse=True)
            raw.extend(accepted[: int(self.detection["max_objects_per_color"])])

        current_codes = {item["code"] for item in raw}
        self._history.append(current_codes)
        votes = Counter(code for codes in self._history for code in codes)
        required = int(self.detection["min_confirmations"])
        confirmed = tuple(sorted(code for code, count in votes.items() if count >= required))
        expected = int(self.detection["expected_color_count"])
        if len(confirmed) > expected:
            status = "AMBIGUOUS"
        elif len(confirmed) < expected:
            status = "SEARCHING"
        elif current_codes == set(confirmed):
            status = "READY"
        else:
            status = "HOLD"
        candidates = tuple(
            ColorCandidate(
                code=item["code"],
                name=item["name"],
                cn_name=item["cn_name"],
                center_px=item["center_px"],
                box_px=item["box_px"],
                area_px=item["area_px"],
                fill_ratio=item["fill_ratio"],
                solidity=item["solidity"],
                confidence=min(
                    1.0,
                    item["shape_score"] * min(1.0, votes[item["code"]] / required),
                ),
                confirmed=item["code"] in confirmed,
            )
            for item in sorted(raw, key=lambda value: value["center_px"][0])
        )
        return ColorDetection(
            status=status,
            safe_to_pick=status == "READY",
            confirmed_codes=confirmed,
            candidates=candidates,
            masks=tuple(masks),
        )


def estimate_white_balance(frame: Any, config: Mapping[str, Any]) -> Optional[Any]:
    """从亮色低色差像素估计三通道软件白平衡增益。"""
    import numpy as np

    step = int(config["sample_step"])
    sample = np.asarray(frame)[::step, ::step].astype(np.float32)
    channel_max = sample.max(axis=2)
    channel_min = sample.min(axis=2)
    brightness = sample.mean(axis=2)
    neutral = (
        (brightness >= float(config["min_brightness"]))
        & (channel_max <= float(config["max_channel_value"]))
        & (channel_max - channel_min <= float(config["max_channel_difference"]))
    )
    if np.count_nonzero(neutral) < int(config["min_strict_pixels"]):
        neutral = (
            (brightness >= float(config["fallback_min_brightness"]))
            & (channel_max <= float(config["max_channel_value"]))
        )
    if np.count_nonzero(neutral) < int(config["min_pixels"]):
        return None
    means = sample[neutral].mean(axis=0)
    target = float(means.mean())
    return np.clip(
        target / np.maximum(means, 1.0),
        float(config["gain_min"]),
        float(config["gain_max"]),
    ).astype(np.float32)


def build_white_balance_luts(gains: Any) -> tuple[Any, ...]:
    import numpy as np

    values = np.arange(256, dtype=np.float32)
    return tuple(
        np.clip(values * float(gain), 0, 255).astype(np.uint8) for gain in gains
    )


def apply_white_balance(frame: Any, luts: tuple[Any, ...]) -> Any:
    import cv2

    return cv2.merge(
        tuple(cv2.LUT(channel, lut) for channel, lut in zip(cv2.split(frame), luts))
    )
