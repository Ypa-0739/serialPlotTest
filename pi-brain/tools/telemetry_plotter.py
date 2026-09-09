# -*- coding: utf-8 -*-
"""STM32两组8通道实时曲线工具。

本进程是串口唯一所有者：RX、命令、STOP和WORK/PLOT心跳全部经
SerialBridgeThread串行执行，避免绘图线程与心跳线程同时写串口。
"""

from __future__ import annotations

import argparse
import csv
import threading
import time
from collections import deque
from pathlib import Path
from typing import Iterable

import serial

from app.protocol import (
    HostLinkAcknowledged,
    ModeChanged,
    Pong,
    PoseTelemetry,
    WheelTelemetry,
)
from app.serial_bridge import SerialBridgeThread, SerialConnected, SerialDisconnected


class TelemetryBuffer:
    """线程安全的定长曲线缓存；横轴使用STM32单调毫秒时间。"""

    def __init__(self, max_samples: int) -> None:
        self.lock = threading.Lock()
        self.wheel = deque(maxlen=max_samples)
        self.pose = deque(maxlen=max_samples)

    def add(self, event: WheelTelemetry | PoseTelemetry) -> None:
        with self.lock:
            (self.wheel if isinstance(event, WheelTelemetry) else self.pose).append(event)

    def snapshots(self):
        with self.lock:
            return list(self.wheel), list(self.pose)


class CsvRecorder:
    """分别保存W8和P8，避免把不同单位硬塞进同一列。"""

    def __init__(self, output_dir: Path) -> None:
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self._wheel_file = (output_dir / f"wheel-{stamp}.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._pose_file = (output_dir / f"pose-{stamp}.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._wheel = csv.writer(self._wheel_file)
        self._pose = csv.writer(self._pose_file)
        self._wheel.writerow(
            ["tick_ms", "sequence"]
            + [f"target_rpm_{i}" for i in range(1, 5)]
            + [f"actual_rpm_{i}" for i in range(1, 5)]
        )
        self._pose.writerow(
            [
                "tick_ms", "sequence", "ops_x_mm", "ops_y_mm", "ops_yaw_deg",
                "center_x_mm", "center_y_mm", "plan_vx_mps", "plan_vy_mps",
                "plan_vz_radps",
            ]
        )

    def write(self, event: WheelTelemetry | PoseTelemetry) -> None:
        if isinstance(event, WheelTelemetry):
            self._wheel.writerow(
                [event.tick_ms, event.sequence, *event.target_rpm, *event.actual_rpm]
            )
            self._wheel_file.flush()
        else:
            self._pose.writerow(
                [
                    event.tick_ms, event.sequence, event.ops_x_mm, event.ops_y_mm,
                    event.ops_yaw_deg, event.center_x_mm, event.center_y_mm,
                    event.plan_vx_mps, event.plan_vy_mps, event.plan_vz_radps,
                ]
            )
            self._pose_file.flush()

    def close(self) -> None:
        self._wheel_file.close()
        self._pose_file.close()


def _seconds(events: Iterable[WheelTelemetry | PoseTelemetry]) -> list[float]:
    values = list(events)
    if not values:
        return []
    origin = values[0].tick_ms
    return [((item.tick_ms - origin) & 0xFFFFFFFF) / 1000.0 for item in values]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="STM32 W8/P8实时曲线与CSV记录")
    parser.add_argument("--port", required=True, help="COM3 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mode", choices=("work", "tune", "plot"), default="work")
    parser.add_argument("--history", type=float, default=20.0, help="显示窗口秒数")
    parser.add_argument("--csv-dir", type=Path, default=Path("telemetry-logs"))
    return parser


def main() -> int:
    args = build_parser().parse_args()
    # 两组各20Hz；至少保留10秒，防止错误参数创建空deque。
    sample_count = max(200, int(max(1.0, args.history) * 20.0))
    buffer = TelemetryBuffer(sample_count)
    recorder = CsvRecorder(args.csv_dir)
    running = threading.Event()
    running.set()
    host_owned = threading.Event()
    host_ready = threading.Event()

    def serial_factory():
        return serial.Serial(
            args.port, args.baud, timeout=0.05, write_timeout=0.1
        )

    # HOST LINK COM 明确免心跳；PC 工具不发送周期 PING。
    bridge = SerialBridgeThread(
        serial_factory,
        heartbeat_enabled=False,
        ping_interval=0.4,
    )

    def event_worker() -> None:
        while running.is_set():
            # 控制/安全事件优先；遥测使用独立有界队列并一次排空，不能反压心跳。
            event = bridge.get_event(timeout=0.01)
            if isinstance(event, SerialConnected):
                host_owned.clear()
                host_ready.clear()
                bridge.emergency_stop()
                bridge.send("HOST LINK COM", priority=30)
                print(f"[LINK] connected {args.port}; requesting COM ownership")
            elif isinstance(event, HostLinkAcknowledged) and event.host == "COM":
                # 取得所有权后再次 STOP，再切换模式；ModeChanged 前不开放控制台。
                host_owned.set()
                bridge.emergency_stop()
                bridge.send(f"MODE {args.mode.upper()}", priority=30)
            elif (
                isinstance(event, ModeChanged)
                and host_owned.is_set()
                and event.mode == args.mode.upper()
            ):
                bridge.send("TELEM BOTH", priority=30)
                bridge.release_emergency_stop()
                host_ready.set()
                print(f"[LINK] COM ownership confirmed; mode={args.mode.upper()}")
            elif isinstance(event, SerialDisconnected):
                host_owned.clear()
                host_ready.clear()
                print("[LINK] disconnected; motion gate is latched")
            elif event is not None and not isinstance(event, Pong):
                print(event.raw or str(event))

            while True:
                telemetry = bridge.get_telemetry(timeout=0.0)
                if not isinstance(telemetry, (WheelTelemetry, PoseTelemetry)):
                    break
                buffer.add(telemetry)
                recorder.write(telemetry)

    def command_worker() -> None:
        while running.is_set():
            try:
                command = input("cmd> ").strip()
            except (EOFError, KeyboardInterrupt):
                running.clear()
                return
            if not command:
                continue
            if command.upper() in {"QUIT", "EXIT"}:
                running.clear()
                return
            if command.upper() == "STOP":
                bridge.emergency_stop()
            elif not host_ready.is_set():
                print("[SEND] HOST LINK COM and mode confirmation not complete")
            else:
                try:
                    bridge.send(command)
                except Exception as exc:
                    print(f"[SEND] {exc}")

    bridge.start()
    event_thread = threading.Thread(
        target=event_worker, daemon=True, name="telemetry-events"
    )
    event_thread.start()
    threading.Thread(target=command_worker, daemon=True, name="telemetry-console").start()

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    figure, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=False)
    figure.canvas.manager.set_window_title("STM32 Mecanum Telemetry W8 + P8")

    def redraw(_frame):
        wheel, pose = buffer.snapshots()
        for axis in axes:
            axis.clear()

        wheel_time = _seconds(wheel)
        colors = ("tab:blue", "tab:orange", "tab:green", "tab:red")
        for index, color in enumerate(colors):
            axes[0].plot(
                wheel_time, [item.target_rpm[index] for item in wheel],
                color=color, linestyle="--", label=f"M{index + 1} target",
            )
            axes[0].plot(
                wheel_time, [item.actual_rpm[index] for item in wheel],
                color=color, label=f"M{index + 1} actual",
            )
        axes[0].set_ylabel("wheel rpm")
        axes[0].legend(ncol=4, fontsize=8)
        axes[0].grid(True)

        pose_time = _seconds(pose)
        for name, getter in (
            ("OPS X", lambda item: item.ops_x_mm),
            ("OPS Y", lambda item: item.ops_y_mm),
            ("CENTER X", lambda item: item.center_x_mm),
            ("CENTER Y", lambda item: item.center_y_mm),
        ):
            axes[1].plot(pose_time, [getter(item) for item in pose], label=name)
        axes[1].set_ylabel("position mm")
        axes[1].legend(ncol=4, fontsize=8)
        axes[1].grid(True)

        axes[2].plot(pose_time, [item.ops_yaw_deg for item in pose], label="OPS YAW")
        axes[2].set_ylabel("yaw deg")
        axes[2].legend()
        axes[2].grid(True)

        axes[3].plot(pose_time, [item.plan_vx_mps for item in pose], label="Vx m/s")
        axes[3].plot(pose_time, [item.plan_vy_mps for item in pose], label="Vy m/s")
        axes[3].plot(pose_time, [item.plan_vz_radps for item in pose], label="Vz rad/s")
        axes[3].set_ylabel("planned speed")
        axes[3].set_xlabel("STM32 elapsed s")
        axes[3].legend(ncol=3)
        axes[3].grid(True)
        figure.tight_layout()

    animation = FuncAnimation(figure, redraw, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        running.clear()
        # 退出绘图工具等同操作员离开：先锁存STOP，再关闭唯一串口写者。
        result = bridge.shutdown()
        print(f"[EXIT] STOP written={result.stop_written} "
              f"acknowledged={result.stop_acknowledged} "
              f"thread_stopped={result.thread_stopped}")
        event_thread.join(timeout=0.5)
        recorder.close()
        _ = animation
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
