# -*- coding: utf-8 -*-
"""Brain 主入口：串口桥 + 启动自检 + 单航点导航 + 事件泵（第四阶段）。

主循环按事件驱动设计：
  - bridge.get_event(timeout=0.05)：短超时取事件，循环天然定期醒来
  - EventRouter 广播：主循环唯一事件源，依次喂给启动状态机、导航器，
    各模块绝不自己调用 get_event()（否则事件可能被“抢走”）
  - startup.tick() / navigator.tick()：检查各自的超时
  - READY 后门禁打开（navigator.set_ready(True)），demo 模式自动提交
    一个演示航点；FAULT 后门禁关闭，等待人工干预或桥自动重连重自检

运行方式：
  python -m app.main --port COM3            # 真串口
  python -m app.main --demo                 # FakeFirmware 演示，无需硬件
  python -m app.main                         # 自动枚举第一个可用串口
  python -m app.main --demo --log-jsonl run.jsonl   # 同时启用 JSONL 结构化日志
"""

from __future__ import annotations

import argparse
import time
from typing import Optional

from .event_router import EventRouter
from .jsonl_logger import JsonlLogger
from .navigator import NavigationRejected, Navigator
from .protocol import (
    CanError,
    Event,
    HostLinkAcknowledged,
    SafetyFault,
    UnknownError,
)
from .serial_bridge import (
    SerialBridgeThread,
    SerialConnected,
    SerialDisconnected,
)
from .state_machine import StartupState, StartupStateMachine
from .vision_integration import VisionStateGuard


def build_serial_factory(port: str, baud: int):
    """真串口工厂：每次（重）连接都新建 serial 对象。"""
    import serial

    def factory():
        return serial.Serial(port, baud, timeout=0.05, write_timeout=0.1)

    return factory


def build_auto_serial_factory(baud: int):
    """每次重连都重新枚举，支持树莓派启动时 STM32 尚未出现。"""
    import serial
    import serial.tools.list_ports

    def factory():
        ports = sorted(
            serial.tools.list_ports.comports(), key=lambda item: item.device
        )
        if not ports:
            raise OSError("no serial device found")
        return serial.Serial(
            ports[0].device, baud, timeout=0.05, write_timeout=0.1
        )

    return factory


def build_demo_factory():
    """演示工厂：FakeFirmware 响应式仿真，验证完整自检流程。"""
    from .demo import FakeFirmware

    firmware = FakeFirmware(host_owner=None)

    def factory():
        return firmware

    return factory


def _event_label(event: Event) -> str:
    if isinstance(event, SerialConnected):
        return "LINK UP"
    if isinstance(event, SerialDisconnected):
        return "LINK DOWN"
    if isinstance(event, HostLinkAcknowledged):
        return f"HOST {event.host} OWNED"
    if isinstance(event, SafetyFault):
        return f"SAFETY FAULT {event.reason.value}"
    if isinstance(event, (CanError, UnknownError)):
        return f"{type(event).__name__}"
    return type(event).__name__


def run_loop(
    bridge: SerialBridgeThread,
    startup: StartupStateMachine,
    navigator: Navigator,
    *,
    demo: bool = False,
    logger: Optional[JsonlLogger] = None,
    vision_cache=None,
    runtime=None,
) -> None:
    """事件泵主循环：统一取事件 -> 广播给状态机与导航器，直到中断。"""
    router = EventRouter()
    router.subscribe(startup.handle_event)
    router.subscribe(navigator.handle_event)
    # 日志是纯旁路消费者，放在控制模块之后订阅；写盘在独立线程，不阻塞本循环
    if logger is not None:
        router.subscribe(logger.handle_event)
    vision_guard = VisionStateGuard(vision_cache) if vision_cache is not None else None
    if vision_guard is not None:
        # 只同步清缓存语义；视觉层仍不接触串口、Navigator 或运动命令。
        router.subscribe(vision_guard.handle_event)
    if runtime is not None:
        router.subscribe(runtime.handle_event)

    last_startup: Optional[StartupState] = None
    last_nav = None
    last_result_id: Optional[int] = None
    demo_goal_sent = False
    try:
        startup.start()
        while True:
            event = bridge.get_event(timeout=0.05)
            if event is not None:
                router.publish(event)
                print(f"[RX] {_event_label(event)}: {event.raw}")
            startup.tick()
            navigator.tick()

            # 启动状态变化：READY 打开导航门禁，离开 READY 关闭
            if startup.state != last_startup:
                if vision_guard is not None:
                    vision_guard.set_control_ready(startup.state == StartupState.READY)
                if startup.state == StartupState.FAULT:
                    print(f"[FAULT] {startup.fault_reason}")
                    navigator.set_ready(False)
                elif startup.state == StartupState.READY:
                    print("[READY] 自检全部通过，允许进入 WAIT_START")
                    navigator.set_ready(True)
                else:
                    print(f"[STATE] {startup.state.value}")
                if runtime is not None:
                    runtime.set_control_ready(startup.state == StartupState.READY)
                last_startup = startup.state

            if navigator.state != last_nav:
                print(f"[NAV] {navigator.state.value}")
                last_nav = navigator.state

            if (
                navigator.last_result is not None
                and navigator.last_result.goal_id != last_result_id
            ):
                r = navigator.last_result
                print(
                    f"[NAV RESULT] goal #{r.goal_id} {r.reason.value} "
                    f"success={r.success} err_mm={r.position_error_mm} "
                    f"err_yaw={r.yaw_error_deg} elapsed={r.elapsed_s:.2f}s"
                )
                last_result_id = r.goal_id

            if runtime is not None and not runtime.tick():
                break

            if startup.is_ready:
                # demo 模式：READY 后自动提交一个演示航点，展示导航闭环
                if demo and not demo_goal_sent:
                    try:
                        gid = navigator.goto_pose(1000, 500, 90, motion_timeout_s=8.0)
                        print(f"[NAV] demo 目标 #{gid} 已提交: POSE SET 1000 500 90")
                    except NavigationRejected as exc:
                        print(f"[NAV] demo 目标被拒绝: {exc}")
                    demo_goal_sent = True
                time.sleep(0.1)
            elif startup.state == StartupState.FAULT:
                # 故障后不自动重试；串口断开时 bridge 自动重连并触发重新自检，
                # 若串口仍在但 STM32 异常，等待人工干预（Ctrl+C 退出）。
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[EXIT] 正在停止...")
    finally:
        # 任何入口退出都先锁存并排队 STOP；外部运行时关闭相机时，串口线程
        # 仍保持运行，能够把 STOP 实际写出后再关闭端口。
        bridge.emergency_stop()
        try:
            if runtime is not None:
                runtime.close()
        finally:
            bridge.stop()
            if logger is not None:
                logger.close()


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Brain 主入口：启动自检 + 事件泵")
    parser.add_argument("--port", help="串口（如 COM3 或 /dev/ttyUSB0）；缺省自动枚举")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--demo", action="store_true", help="FakeFirmware 演示模式")
    parser.add_argument(
        "--log-jsonl",
        metavar="PATH",
        help="启用 JSONL 结构化日志并写入指定文件（独立线程，纯旁路）",
    )
    args = parser.parse_args(argv)

    if args.demo:
        factory = build_demo_factory()
        print("[INFO] demo 模式：使用 FakeFirmware，不连接真实串口")
    elif args.port:
        factory = build_serial_factory(args.port, args.baud)
    else:
        print("[INFO] 自动等待串口设备；建议实车使用 /dev/serial/by-id/... 固定路径")
        factory = build_auto_serial_factory(args.baud)

    bridge = SerialBridgeThread(factory)
    startup = StartupStateMachine(bridge)
    navigator = Navigator(bridge)
    logger = None
    if args.log_jsonl:
        # 日志上下文只读快照：任务/导航状态 + 活动航点；不参与任何控制决策
        def _log_context() -> dict:
            return {
                "task_state": startup.state.value,
                "nav_state": navigator.state.value,
                "goal_id": navigator.active_goal_id,
                "target": navigator.active_target,
            }

        logger = JsonlLogger(args.log_jsonl, context_provider=_log_context)
        print(f"[INFO] JSONL 日志: {args.log_jsonl}")
    bridge.start()
    run_loop(bridge, startup, navigator, demo=args.demo, logger=logger)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
