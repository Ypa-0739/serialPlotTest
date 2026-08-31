"""SSH 下使用的视觉—底盘人工联调入口。"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from app.main import build_auto_serial_factory, build_serial_factory, run_loop
from app.map_frame import MapGoal, StartAnchoredMapFrame
from app.models import Pose
from app.navigator import Navigator
from app.route_runner import RouteRunner, RouteState
from app.serial_bridge import SerialBridgeThread
from app.speed_profile import SpeedProfileController
from app.state_machine import StartupStateMachine
from app.vision import VisionService, build_vision_pipeline
from app.vision.observation_adapter import VisionObservationGate, VisionRequirement
from app.vision_integration import VisionStateGuard


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "vision_motion_debug.json"


@dataclass(frozen=True)
class DebugConfig:
    nominal_start: Pose
    map_goal: MapGoal
    ops_offset: tuple[float, float]
    requirement: VisionRequirement


def _pose(values, name: str) -> Pose:
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"{name} must be [x_mm, y_mm, yaw_deg]")
    result = Pose(*(float(value) for value in values))
    if not all(
        math.isfinite(value)
        for value in (result.x_mm, result.y_mm, result.yaw_deg)
    ):
        raise ValueError(f"{name} must contain finite values")
    return result


def load_config(path: Path) -> DebugConfig:
    raw = json.loads(path.read_text(encoding="utf-8"))
    nominal = _pose(raw["nominal_start_center"], "nominal_start_center")
    goal_raw = raw["map_goal_center"]
    goal_pose = _pose(goal_raw["pose"], "map_goal_center.pose")
    goal = MapGoal(
        str(goal_raw["name"]),
        goal_pose.x_mm,
        goal_pose.y_mm,
        goal_pose.yaw_deg,
        float(goal_raw.get("timeout_s", 15.0)),
        str(goal_raw.get("profile", "precise")),
    )
    offset = tuple(float(value) for value in raw.get("ops_offset_body_mm", [0, 25]))
    vision = raw["vision"]
    target_type = vision["target_type"]
    source = vision["source"]
    target_id = vision["target_id"]
    if not all(isinstance(value, str) and value for value in (target_type, source, target_id)):
        raise ValueError("vision target_type/source/target_id must be non-empty strings")
    requirement = VisionRequirement(
        target_type,
        float(vision.get("minimum_confidence", 0.8)),
        float(vision.get("maximum_age_s", 0.5)),
        source,
        target_id,
    )
    if len(offset) != 2 or not all(math.isfinite(value) for value in offset):
        raise ValueError("ops_offset_body_mm must contain two finite values")
    if not goal.name or not goal.profile:
        raise ValueError("map goal name/profile must be non-empty")
    if not math.isfinite(goal.timeout_s) or goal.timeout_s <= 0:
        raise ValueError("goal timeout must be a positive finite number")
    if not math.isfinite(requirement.minimum_confidence) or not (
        0 <= requirement.minimum_confidence <= 1
    ):
        raise ValueError("invalid timeout or confidence")
    if not math.isfinite(requirement.maximum_age_s) or requirement.maximum_age_s <= 0:
        raise ValueError("maximum_age_s must be positive")
    if target_type.startswith("material:"):
        material_code = target_type.split(":", 1)[1]
        if not material_code.isdigit() or target_id != material_code:
            raise ValueError("material target_id must match material:<code>")
    elif target_type != "task_code":
        raise ValueError("motion debug supports material:<code> or task_code")
    return DebugConfig(nominal, goal, offset, requirement)


class CommandReader(threading.Thread):
    def __init__(self) -> None:
        super().__init__(daemon=True, name="vision-debug-console")
        self.commands: queue.Queue[str] = queue.Queue()

    def run(self) -> None:
        for line in sys.stdin:
            if line.strip():
                self.commands.put(line.strip().lower())
        self.commands.put("quit")


class VisionDebugRuntime:
    """复用主事件泵，只补充视觉、地图锚定和人工命令。"""

    def __init__(self, bridge, startup, navigator, vision, config, *, armed, print_interval):
        self.bridge, self.startup, self.navigator = bridge, startup, navigator
        self.vision, self.config, self.armed = vision, config, armed
        self.profiles = SpeedProfileController(bridge)
        self.route = RouteRunner(navigator, profiles=self.profiles)
        self.guard = VisionStateGuard(vision)
        self.gate = VisionObservationGate(vision)
        self.commands = CommandReader()
        self.commands.start()
        self.ready = False
        self.frame = None
        self.latest = None
        self.next_print = 0.0
        self.print_interval = print_interval
        self.exit_code = 0

    def handle_event(self, event) -> None:
        self.guard.handle_event(event)

    def set_control_ready(self, ready: bool) -> None:
        self.ready = ready
        self.guard.set_control_ready(ready)
        self.vision.clear()
        if not ready:
            self.frame = None
            self.profiles.mark_unknown()
            return
        if self.startup.ops_pose is None:
            self.bridge.emergency_stop()
            self.ready = False
            print("[FAULT] READY without an OPS start pose")
            return
        self.frame = StartAnchoredMapFrame(
            self.config.nominal_start,
            self.startup.ops_pose,
            *self.config.ops_offset,
        )
        goal = self.frame.to_ops_goal(self.config.map_goal)
        print(
            f"[MAP] OPS start={self.startup.ops_pose} -> "
            f"goal=({goal.x_mm:.1f}, {goal.y_mm:.1f}, {goal.yaw_deg:.1f})"
        )

    def tick(self) -> bool:
        self.route.tick()
        while True:
            observation = self.vision.get()
            if observation is None:
                break
            self.latest = observation
        now = time.monotonic()
        if self.latest is not None and now >= self.next_print:
            print("[VISION] " + json.dumps(dataclasses.asdict(self.latest), ensure_ascii=False))
            self.next_print = now + self.print_interval
        health = self.vision.health()
        if health.error:
            print(f"[VISION FAULT] {health.error}", file=sys.stderr)
            self.exit_code = 2
            return False
        while not self.commands.commands.empty():
            command = self.commands.commands.get_nowait()
            if command == "go":
                self._go()
            elif command == "cancel":
                print("[CANCEL] requested" if self.route.cancel() else "[CANCEL] no active route")
            elif command == "estop":
                self.bridge.emergency_stop()
                self.ready = False
                self.vision.clear()
                print("[ESTOP] latched; restart for a new self-check")
            elif command == "status":
                print(
                    f"[STATUS] startup={self.startup.state.value} "
                    f"route={self.route.state.value} armed={self.armed} "
                    f"vision_healthy={health.healthy}"
                )
            elif command in ("quit", "exit", "q"):
                return False
            elif command == "help":
                print("[COMMANDS] status | go | cancel | estop | help | quit")
            else:
                print(f"[COMMAND ERROR] {command}")
        return True

    def _go(self) -> None:
        if not self.armed or not self.ready or self.frame is None:
            print("[GO REJECTED] requires --armed and READY")
            return
        if not self.vision.health().healthy:
            print("[GO REJECTED] vision is not healthy")
            return
        if self.route.state == RouteState.RUNNING:
            print("[GO REJECTED] route already running")
            return
        if self.gate.require(self.config.requirement) is None:
            print("[GO REJECTED] no fresh matching observation")
            return
        goal = self.frame.to_ops_goal(self.config.map_goal)
        self.route.start_route((goal,))
        if self.route.state == RouteState.RUNNING:
            print(f"[GO ACCEPTED] {goal.name}")
        else:
            print(f"[GO REJECTED] {self.route.last_reason}")

    def close(self) -> None:
        if not self.vision.stop(timeout=2.0):
            print("[WARN] vision worker did not stop within 2 seconds", file=sys.stderr)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="固定地图 + 启动 OPS 锚定的视觉联调")
    parser.add_argument("--port", help="STM32 串口；缺省自动枚举")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--config-dir", type=Path, help="视觉检测配置目录")
    parser.add_argument("--motion-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--armed", action="store_true", help="允许输入 go 发车")
    parser.add_argument("--interval", type=float, default=0.02)
    parser.add_argument("--print-interval", type=float, default=0.5)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_config(args.motion_config)
        target_type = config.requirement.target_type
        code = int(target_type.split(":", 1)[1]) if target_type.startswith("material:") else None
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        print(f"[CONFIG ERROR] {exc}", file=sys.stderr)
        return 2
    if args.interval < 0 or args.print_interval <= 0:
        print("[CONFIG ERROR] invalid interval", file=sys.stderr)
        return 2

    vision = VisionService(
        build_vision_pipeline(
            enable_material=code is not None,
            enable_qr=target_type == "task_code",
            target_material_provider=(lambda: code) if code is not None else None,
            config_dir=args.config_dir,
        ),
        poll_interval_s=args.interval,
    )
    factory = (
        build_serial_factory(args.port, args.baud)
        if args.port
        else build_auto_serial_factory(args.baud)
    )
    bridge = SerialBridgeThread(factory)
    startup = StartupStateMachine(bridge)
    navigator = Navigator(bridge)
    runtime = VisionDebugRuntime(
        bridge,
        startup,
        navigator,
        vision,
        config,
        armed=args.armed,
        print_interval=args.print_interval,
    )
    print(
        f"[INFO] {'ARMED' if args.armed else 'OBSERVE_ONLY'}; "
        "commands: status/go/cancel/estop/quit"
    )
    bridge.start()
    vision.start()
    run_loop(bridge, startup, navigator, runtime=runtime)
    return runtime.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
