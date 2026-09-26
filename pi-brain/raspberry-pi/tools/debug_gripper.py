"""抓取摄像头的物料颜色、中心偏差与对准调试入口。"""

from argparse import ArgumentParser
import os
import time

import cv2

from robot_hardware.camera import (
    DEFAULT_CAMERA_CONFIG_PATH,
    CameraConfigError,
    PiCamera,
    load_camera_config,
)
from robot_perception.color import ConfigError, load_config as load_color_config
from robot_perception.color.detector import (
    CompetitionColorDetector,
    apply_white_balance,
    build_white_balance_luts,
    calibrate_white_balance,
)
from robot_perception.material import GripperMaterialDetector


def parse_arguments():
    parser = ArgumentParser(description="抓取摄像头颜色和对准调试")
    parser.add_argument("--camera-config", default=str(DEFAULT_CAMERA_CONFIG_PATH))
    parser.add_argument("--color-config")
    parser.add_argument(
        "--target-code",
        type=int,
        choices=range(1, 7),
        help="只跟踪指定物料编号；不填时选择最接近夹爪中心的物料",
    )
    parser.add_argument("--no-preview", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    try:
        camera_config = load_camera_config(args.camera_config)["gripper"]
        color_config = load_color_config(args.color_config)
        camera = PiCamera(camera_config)
        color_detector = CompetitionColorDetector(color_config)
        detector = GripperMaterialDetector(color_detector, camera_config)
    except (CameraConfigError, ConfigError, ValueError) as error:
        print(f"抓取视觉配置错误：{error}")
        return 2

    preview_enabled = not args.no_preview and bool(
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    )
    if not args.no_preview and not preview_enabled:
        print("当前没有图形桌面，已关闭预览窗口。")

    cv2.setNumThreads(1)
    last_summary = None
    last_print_time = 0.0
    try:
        camera.start()
        time.sleep(float(camera_config.get("settle_seconds", 1.0)))
        gains = calibrate_white_balance(camera, color_config["white_balance"])
        lookup_tables = build_white_balance_luts(gains)
        grip_x, grip_y = (int(value) for value in camera_config["grip_center"])

        while True:
            raw_frame = camera.capture_array("main")
            frame = apply_white_balance(raw_frame, lookup_tables)
            result = detector.detect(
                frame,
                target_material_code=args.target_code,
            )
            observation = result.observation
            summary = (
                result.status,
                observation.material_code if observation else None,
                tuple(round(value, 1) for value in observation.offset_pixels)
                if observation
                else None,
            )
            now = time.monotonic()
            if summary != last_summary or now - last_print_time >= 1.0:
                if observation is None:
                    print(
                        f"状态={result.status} 目标={args.target_code or '自动'} "
                        "未找到物料 可抓取=False"
                    )
                else:
                    print(
                        f"状态={result.status} 编号={observation.material_code} "
                        f"颜色={observation.color_cn_name} "
                        f"偏差=({observation.offset_pixels[0]:+.1f},"
                        f"{observation.offset_pixels[1]:+.1f}) "
                        f"已对准={result.aligned} 可抓取={result.safe_to_pick}"
                    )
                last_summary = summary
                last_print_time = now

            if preview_enabled:
                cv2.drawMarker(
                    frame,
                    (grip_x, grip_y),
                    (255, 255, 255),
                    cv2.MARKER_CROSS,
                    24,
                    2,
                )
                if observation is not None:
                    x, y, width, height = observation.box
                    color = (0, 255, 0) if result.safe_to_pick else (0, 165, 255)
                    cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
                    cv2.line(frame, (grip_x, grip_y), observation.center, color, 2)
                    cv2.putText(
                        frame,
                        f"{observation.material_code} {observation.color_name}",
                        (x, max(18, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        color,
                        2,
                    )
                cv2.putText(
                    frame,
                    f"{result.status} PICK={result.safe_to_pick}",
                    (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                )
                cv2.imshow("Gripper Material Alignment", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\n抓取视觉调试已停止")
    except Exception as error:
        print(f"抓取视觉错误：{type(error).__name__}: {error}")
        return 1
    finally:
        camera.close()
        if preview_enabled:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
