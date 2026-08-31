"""树莓派视觉只读调试入口；不连接 STM32，不产生运动命令。"""

from __future__ import annotations

import argparse
import dataclasses
import json
import time
from pathlib import Path
from typing import Optional

from app.vision import VisionService, build_vision_pipeline


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pi 5 OpenCV视觉只读调试")
    parser.add_argument("--config-dir", type=Path)
    parser.add_argument("--material", action="store_true", help="启用夹爪物料识别")
    parser.add_argument("--qr", action="store_true", help="启用前摄像头二维码识别")
    parser.add_argument("--line", action="store_true", help="启用巡线图像检测（不控制底盘）")
    parser.add_argument("--road", action="store_true", help="启用道路检测（必须先完成标定）")
    parser.add_argument("--target-code", type=int, help="只选择指定物料编号")
    parser.add_argument("--interval", type=float, default=0.02)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    selected = any((args.material, args.qr, args.line, args.road))
    enable_material = args.material or not selected
    enable_qr = args.qr or not selected
    pipeline = build_vision_pipeline(
        enable_material=enable_material,
        enable_qr=enable_qr,
        enable_line=args.line,
        enable_road=args.road,
        target_material_provider=lambda: args.target_code,
        config_dir=args.config_dir,
    )
    service = VisionService(pipeline, poll_interval_s=args.interval)
    service.start()
    try:
        while True:
            observation = service.get(timeout=0.5)
            if observation is not None:
                print(json.dumps(dataclasses.asdict(observation), ensure_ascii=False))
                continue
            health = service.health()
            if health.error:
                print(json.dumps(dataclasses.asdict(health), ensure_ascii=False))
                return 2
    except KeyboardInterrupt:
        return 0
    finally:
        if not service.stop(timeout=2.0):
            print("vision worker did not stop within 2 seconds")


if __name__ == "__main__":
    raise SystemExit(main())
