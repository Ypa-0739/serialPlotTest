"""视觉 JSON 配置读取和路径管理。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Optional


VISION_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config" / "vision"


class VisionConfigError(ValueError):
    pass


def load_json_config(name: str, path: Optional[str | Path] = None) -> dict[str, Any]:
    selected = Path(path) if path is not None else VISION_CONFIG_DIR / name
    try:
        value = json.loads(selected.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise VisionConfigError(f"vision config not found: {selected}") from exc
    except json.JSONDecodeError as exc:
        raise VisionConfigError(
            f"invalid vision JSON {selected}: line {exc.lineno}, column {exc.colno}"
        ) from exc
    if not isinstance(value, dict):
        raise VisionConfigError(f"vision config root must be an object: {selected}")
    return value
