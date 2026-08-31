# -*- coding: utf-8 -*-
"""端到端集成测试：串口桥 + 启动状态机 + FakeFirmware 协作。

覆盖用户验收要点：
  - 完整自检成功路径：STOP -> HOST LINK RPI -> STOP -> MODE WORK -> STATUS -> OPS -> CAN ->
    PID SET X/Y/YAW -> PID LIMIT X/Y/YAW -> PID STATUS ALL -> READY
  - OPS 异常（LINK=STALE）-> FAULT
  - 运行中断线 -> FAULT
  - FAULT 后桥自动重连 -> 重新自检
"""

import time
import unittest

from app.demo import FakeFirmware
from app.serial_bridge import (
    LinkState,
    SerialBridgeThread,
    SerialConnected,
    SerialDisconnected,
)
from app.state_machine import StartupConfig, StartupStateMachine

# 自检成功路径的期望命令序列（固件真实命令格式）
EXPECTED_SEQUENCE = [
    "STOP",
    "HOST LINK RPI",
    "STOP",
    "MODE WORK",
    "STATUS",
    "OPS STATUS",
    "CAN STATUS",
    "PID SET X 0.0033 0 0",
    "PID SET Y 0.0033 0 0",
    "PID SET YAW 0.02 0 0",
    "PID LIMIT X 0.20",
    "PID LIMIT Y 0.20",
    "PID LIMIT YAW 0.25",
    "PID STATUS ALL",
]


def pump(bridge, startup, *, until, timeout=3.0, step=0.02):
    """事件泵：把 bridge 事件喂给状态机，直到 predicate 为真或超时。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        event = bridge.get_event(timeout=step)
        if event is not None:
            startup.handle_event(event)
        startup.tick()
        if until():
            return True
    return until()


class StartupIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.firmware = FakeFirmware(host_owner=None)
        self.bridge = SerialBridgeThread(
            lambda: self.firmware, ping_interval=0.05, backoff_seconds=0.05
        )
        self.startup = StartupStateMachine(self.bridge)
        self.bridge.start()

    def tearDown(self):
        self.bridge.stop()

    def test_full_startup_success(self):
        """完整自检成功：命令顺序正确，进入 READY 且解除急停。"""
        self.startup.start()
        ok = pump(self.bridge, self.startup, until=lambda: self.startup.is_ready)
        self.assertTrue(ok, f"自检未完成，状态={self.startup.state} 故障={self.startup.fault_reason}")
        self.assertEqual(self.startup.state.value, "READY")
        # 命令按序发出（过滤掉桥线程周期性的 PING 心跳）
        commands = [c for c in self.firmware.received if c != "PING"]
        self.assertEqual(commands, EXPECTED_SEQUENCE)
        # 急停已解除（bridge 不再拒绝运动命令）
        self.assertFalse(self.bridge.is_emergency_stopped)
        self.bridge.send("POSE SET 100 200 0")  # 不抛异常即通过

    def test_ready_after_release_allows_motion(self):
        """READY 后运动命令不再被拒绝。"""
        self.startup.start()
        self.assertTrue(
            pump(self.bridge, self.startup, until=lambda: self.startup.is_ready),
            f"自检未完成: {self.startup.fault_reason}",
        )
        try:
            self.bridge.send("MOVE FWD 0.1 500")
        except Exception as exc:  # noqa: BLE001
            self.fail(f"READY 后运动命令仍被拒绝: {exc}")

    def test_fault_when_ops_stale(self):
        """OPS LINK=STALE 时进入 FAULT 并再次锁存急停。"""
        self.firmware.ops_link = "STALE"
        self.startup.start()
        ok = pump(
            self.bridge,
            self.startup,
            until=lambda: self.startup.state.value == "FAULT",
        )
        self.assertTrue(ok, "应进入 FAULT")
        self.assertIn("OPS", self.startup.fault_reason)
        # FAULT 里 emergency_stop 会再入队一个 STOP；等待线程异步写出
        self.assertTrue(
            pump(
                self.bridge,
                self.startup,
                until=lambda: self.firmware.received.count("STOP") >= 2,
                timeout=1.0,
            ),
            f"FAULT 后应再次 STOP，收到: {self.firmware.received}",
        )
        self.assertTrue(self.bridge.is_emergency_stopped)

    def test_fault_when_can_error(self):
        """CAN 错误（ERROR 非零）时进入 FAULT。"""
        self.firmware.can_error = 0x00000004
        self.startup.start()
        ok = pump(
            self.bridge,
            self.startup,
            until=lambda: self.startup.state.value == "FAULT",
        )
        self.assertTrue(ok, "应进入 FAULT")
        self.assertIn("CAN", self.startup.fault_reason)

    def test_rpi_handshake_safely_replaces_previous_com_declaration(self):
        """单根 USB 线重新接入 RPI 时允许切换，但仍须从完整自检开始。"""
        self.firmware.host_owner = "COM"
        self.startup.start()
        ok = pump(
            self.bridge,
            self.startup,
            until=lambda: self.startup.is_ready,
        )
        self.assertTrue(ok, f"RPI 切换后应完成自检: {self.startup.fault_reason}")
        self.assertEqual(self.firmware.host_owner, "RPI")

    def test_fault_on_disconnect_then_recover(self):
        """运行中断线 -> FAULT；桥自动重连 -> 从 STOP 重新自检 -> READY。"""
        self.startup.start()
        # 走到中间某步后注入断线
        self.assertTrue(
            pump(self.bridge, self.startup, until=lambda: len(self.firmware.received) >= 5)
        )
        self.firmware.fail()
        self.assertTrue(
            pump(
                self.bridge,
                self.startup,
                until=lambda: self.startup.state.value == "FAULT",
                timeout=3.0,
            ),
            "断线应进入 FAULT",
        )
        self.assertIn("disconnected", self.startup.fault_reason)

        # 桥退避后自动重连（factory 重新调用返回同一 firmware，恢复 is_open）
        self.firmware.is_open = True
        self.firmware._fail = False
        # 清除旧响应，避免干扰
        self.firmware._responses.clear()
        ok = pump(
            self.bridge,
            self.startup,
            until=lambda: self.startup.is_ready,
            timeout=3.0,
        )
        self.assertTrue(ok, f"重连后重新自检未完成: {self.startup.fault_reason}")
        # 重连后从 STOP 重新开始（完整序列再次出现，过滤 PING 心跳）
        commands = [c for c in self.firmware.received if c != "PING"]
        self.assertGreaterEqual(commands.count("STOP"), 2)
        self.assertEqual(commands[-1], "PID STATUS ALL")


class StartupTimeoutTests(unittest.TestCase):
    def test_step_timeout_enters_fault(self):
        """固件无响应时步骤超时 -> FAULT。"""
        firmware = FakeFirmware()
        bridge = SerialBridgeThread(
            lambda: firmware, ping_interval=0.05, backoff_seconds=0.05
        )
        startup = StartupStateMachine(
            bridge, StartupConfig(step_timeout=0.15)  # 加速：150ms 超时
        )
        bridge.start()
        try:
            startup.start()
            # 不喂任何事件，直接 tick 推进时间
            fake_clock = [time.monotonic()]
            startup._clock = lambda: fake_clock[0]
            fake_clock[0] += 0.2
            startup.tick()
            self.assertEqual(startup.state.value, "FAULT")
            self.assertIn("timeout", startup.fault_reason)
        finally:
            bridge.stop()


if __name__ == "__main__":
    unittest.main()
