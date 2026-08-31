# -*- coding: utf-8 -*-
"""SerialBridgeThread 单测：唯一写者 / PING 心跳 / STOP 抢占 / 断线重连。"""

import time
import unittest

from app.binary_protocol import Command as BinaryCommand, decode_frame

from app.serial_bridge import (
    LinkState,
    MotionCommandRejected,
    SerialBridgeThread,
    SerialConnected,
    SerialDisconnected,
)
from app.protocol import Pong, PoseReached, WheelTelemetry, parse_line
from tests.fakes import FakeSerial

PING = b"PING\n"


def _factory_tracked(serials, **kwargs):
    """记录每次 factory 调用返回的实例，便于断言重连行为。"""

    def factory():
        ser = FakeSerial(**kwargs)
        serials.append(ser)
        return ser

    return factory


class BridgeTestCase(unittest.TestCase):
    def start_bridge(self, factory, **kwargs):
        kwargs.setdefault("ping_interval", 0.05)
        kwargs.setdefault("backoff_seconds", 0.05)
        bridge = SerialBridgeThread(factory, **kwargs)
        bridge.start()
        return bridge

    def collect_events(self, bridge, until, timeout=1.0):
        """收集事件直到 predicate(ev) 为真或超时。返回事件列表。"""
        events = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ev = bridge.get_event(timeout=0.05)
            if ev is None:
                continue
            events.append(ev)
            if until(ev):
                break
        return events

    def wait_until(self, predicate, timeout=2.0, step=0.02):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(step)
        return predicate()


class PingTests(BridgeTestCase):
    def test_ping_sent_without_rx(self):
        """用户验收第 6 项：无 STM32 数据时，PING 仍按周期发送。"""
        serials = []
        bridge = self.start_bridge(_factory_tracked(serials))
        time.sleep(0.45)  # ping_interval=0.05，预期约 8 个
        bridge.stop()
        self.assertTrue(serials)
        pings = sum(1 for w in serials[0].writes if w == PING)
        self.assertGreaterEqual(pings, 6)

    def test_ping_interval_measured(self):
        """真实节奏（400ms）下 1.2s 内至少 2 个 PING。"""
        serials = []
        bridge = SerialBridgeThread(
            _factory_tracked(serials), ping_interval=0.4, backoff_seconds=0.05
        )
        bridge.start()
        time.sleep(1.25)
        bridge.stop()
        pings = sum(1 for w in serials[0].writes if w == PING)
        self.assertGreaterEqual(pings, 2)

    def test_tune_client_can_disable_heartbeat_without_disabling_commands(self):
        serials = []
        bridge = SerialBridgeThread(
            _factory_tracked(serials),
            ping_interval=0.05,
            heartbeat_enabled=False,
            backoff_seconds=0.05,
        )
        bridge.start()
        bridge.send("MODE TUNE")
        time.sleep(0.2)
        bridge.stop()
        self.assertIn(b"MODE TUNE\n", serials[0].writes)
        self.assertNotIn(PING, serials[0].writes)


class TelemetryQueueTests(BridgeTestCase):
    def test_telemetry_is_separated_from_control_events(self):
        serials = []
        bridge = self.start_bridge(
            _factory_tracked(
                serials,
                lines=[
                    "@W,1,100,1,1,2,3,4,5,6,7,8\n",
                    "# PONG\n",
                ],
            )
        )
        self.assertTrue(
            self.wait_until(lambda: bridge.get_telemetry() is not None, timeout=0.5)
        )
        # 再注入一帧用于类型断言，避免wait_until已经消费首帧。
        serials[0].lines.append("@W,1,150,2,1,2,3,4,5,6,7,8\n")
        telemetry = None
        deadline = time.monotonic() + 0.5
        while telemetry is None and time.monotonic() < deadline:
            telemetry = bridge.get_telemetry(timeout=0.02)
        events = []
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            event = bridge.get_event(timeout=0.02)
            if event is not None:
                events.append(event)
            if any(isinstance(item, Pong) for item in events):
                break
        bridge.stop()
        self.assertIsInstance(telemetry, WheelTelemetry)
        self.assertTrue(any(isinstance(item, Pong) for item in events))

    def test_full_telemetry_queue_drops_oldest_but_keeps_control_event(self):
        bridge = SerialBridgeThread(
            lambda: FakeSerial(), heartbeat_enabled=False, telemetry_queue_size=2
        )
        bridge._put_latest_telemetry(parse_line("@W,1,100,1,1,2,3,4,5,6,7,8"))
        bridge._put_latest_telemetry(parse_line("@W,1,150,2,1,2,3,4,5,6,7,8"))
        bridge._put_latest_telemetry(parse_line("@W,1,200,3,1,2,3,4,5,6,7,8"))
        bridge._events.put(Pong(raw="# PONG"))

        first = bridge.get_telemetry()
        second = bridge.get_telemetry()
        control = bridge.get_event()
        self.assertEqual((first.sequence, second.sequence), (2, 3))
        self.assertIsInstance(control, Pong)


class SingleWriterTests(BridgeTestCase):
    def test_only_bridge_thread_writes(self):
        """用户验收第 7 项：普通 TX 命令只能从 SerialBridgeThread 写出。"""
        serials = []
        bridge = self.start_bridge(_factory_tracked(serials))
        for i in range(20):
            bridge.send(f"MOVE {i} 0.1 100")
        bridge.emergency_stop()
        time.sleep(0.3)
        bridge.stop()
        self.assertTrue(serials)
        self.assertTrue(serials[0].writes)
        # 所有写入必须发生在线程内
        self.assertTrue(
            all(name == "serial-bridge" for name in serials[0].write_threads),
            f"发现外部线程写入: {set(serials[0].write_threads)}",
        )


class PriorityTests(BridgeTestCase):
    def test_stop_preempts_regular_commands(self):
        """STOP 清除旧队列；停车后不能继续发送旧命令。"""
        serials = []
        bridge = SerialBridgeThread(
            _factory_tracked(serials), ping_interval=0.05, backoff_seconds=0.05
        )
        for i in range(50):
            bridge.send(f"CONFIG {i}")
        bridge.emergency_stop()
        bridge.start()
        time.sleep(0.4)
        bridge.stop()
        non_ping = [w for w in serials[0].writes if w != PING]
        self.assertEqual(non_ping, [b"STOP\n"], "STOP 后不得发送任何旧队列命令")

    def test_latched_stop_rejects_motion_but_allows_safe_commands(self):
        serials = []
        bridge = self.start_bridge(_factory_tracked(serials))
        bridge.emergency_stop()
        with self.assertRaises(MotionCommandRejected):
            bridge.send("POSE SET 100 200 0")
        with self.assertRaises(MotionCommandRejected):
            bridge.send("MOVE FWD 0.1 500")
        with self.assertRaises(MotionCommandRejected):
            bridge.send("SET P:0.003 I:0 D:0")
        bridge.send("STATUS")
        bridge.send("PID SET X 0.0033 0 0")
        time.sleep(0.2)
        bridge.stop()
        writes = serials[0].writes
        self.assertIn(b"STOP\n", writes)
        self.assertIn(b"STATUS\n", writes)
        self.assertIn(b"PID SET X 0.0033 0 0\n", writes)
        self.assertNotIn(b"POSE SET 100 200 0\n", writes)

    def test_release_does_not_replay_and_allows_new_motion(self):
        serials = []
        bridge = self.start_bridge(_factory_tracked(serials))
        bridge.send("POSE SET 10 20 0")
        bridge.emergency_stop()
        bridge.release_emergency_stop()
        bridge.send("POSE SET 30 40 0")
        time.sleep(0.2)
        bridge.stop()
        writes = serials[0].writes
        self.assertNotIn(b"POSE SET 10 20 0\n", writes)
        self.assertIn(b"POSE SET 30 40 0\n", writes)


class BinaryModeTests(BridgeTestCase):
    def test_runtime_commands_use_single_writer_binary_frames(self):
        serials = []
        bridge = self.start_bridge(
            _factory_tracked(serials), heartbeat_enabled=False
        )
        self.assertTrue(self.wait_until(lambda: bool(serials)))
        bridge.enable_binary_mode()
        bridge.release_emergency_stop()
        bridge.set_speed_limits(0.25, 0.12, priority=10)
        bridge.send_pose_goal(42, 1000, 500, 90, 5.0, priority=10)
        self.assertTrue(self.wait_until(lambda: len(serials[0].writes) >= 2))
        bridge.emergency_stop()
        self.assertTrue(self.wait_until(lambda: len(serials[0].writes) >= 3))
        bridge.stop()

        frames = [decode_frame(wire) for wire in serials[0].writes]
        commands = [frame.payload[0] for frame in frames]
        self.assertEqual(
            commands,
            [
                BinaryCommand.SET_SPEED_LIMITS,
                BinaryCommand.SET_POSE_GOAL,
                BinaryCommand.STOP_ALL,
            ],
        )
        self.assertTrue(
            all(name == "serial-bridge" for name in serials[0].write_threads)
        )


class DisconnectTests(BridgeTestCase):
    def test_serial_exception_produces_disconnected(self):
        """用户验收第 9 项：模拟 SerialException 后产生 SerialDisconnected。"""
        serials = []
        bridge = self.start_bridge(_factory_tracked(serials, fail_after=3))
        events = self.collect_events(
            bridge, until=lambda ev: isinstance(ev, SerialDisconnected), timeout=3.0
        )
        bridge.stop()
        self.assertTrue(
            any(isinstance(ev, SerialDisconnected) for ev in events),
            f"未收到 SerialDisconnected, 收到: {events}",
        )

    def test_manual_fail_also_disconnects(self):
        serials = []
        bridge = self.start_bridge(_factory_tracked(serials))
        time.sleep(0.15)
        serials[0].fail()
        events = self.collect_events(
            bridge, until=lambda ev: isinstance(ev, SerialDisconnected), timeout=3.0
        )
        bridge.stop()
        self.assertTrue(any(isinstance(ev, SerialDisconnected) for ev in events))

    def test_open_failure_backoff_then_connected(self):
        """factory 抛异常（端口打不开）时进入退避，不崩溃。"""
        calls = []

        def flaky_factory():
            calls.append(1)
            if len(calls) <= 2:
                raise OSError("port busy")
            return FakeSerial()

        bridge = SerialBridgeThread(flaky_factory, ping_interval=0.05, backoff_seconds=0.05)
        bridge.start()
        events = self.collect_events(
            bridge, until=lambda ev: isinstance(ev, SerialConnected), timeout=3.0
        )
        bridge.stop()
        self.assertTrue(any(isinstance(ev, SerialConnected) for ev in events))


class ReconnectTests(BridgeTestCase):
    def test_reconnect_publishes_connected_no_retransmit(self):
        """用户验收第 10 项：重连后产生 SerialConnected，旧运动命令不被自动重发。"""
        serials = []

        def factory():
            ser = FakeSerial()
            if len(serials) == 0:
                ser.fail_after = 2  # 第一个实例写 2 次后断线
            serials.append(ser)
            return ser

        bridge = self.start_bridge(factory)
        bridge.send("MOVE 1 0.1 500")  # 断线前投递的运动命令

        # 等待第二次连接（重连成功）
        connected = self.wait_until(
            lambda: (
                len(serials) >= 2
                and bridge.link_state == LinkState.CONNECTED
            ),
            timeout=3.0,
        )
        self.assertTrue(connected, "重连未成功")
        bridge.send("STATUS")  # 重连后的新命令
        time.sleep(0.3)
        bridge.stop()

        self.assertGreaterEqual(len(serials), 2)
        first, second = serials[0], serials[1]
        # 旧运动命令绝不重发到新串口
        self.assertNotIn(b"MOVE", b"".join(second.writes), "重连后旧命令被重发")
        self.assertTrue(bridge.is_emergency_stopped, "断线后应锁存运动门禁")
        # 重连后的新命令正常发出
        self.assertIn(b"STATUS\n", second.writes)


class EventFlowTests(BridgeTestCase):
    def test_incoming_lines_parsed_into_events(self):
        """固件行正确解析为事件并进入事件队列。"""
        incoming = [
            "# PONG",
            "# POSE TARGET X=100.00 Y=200.00 YAW=30.00 CENTER_X=98.75 "
            "CENTER_Y=201.25 ERROR_MM=2.10 ERROR_YAW=0.30",
        ]
        serials = []

        def factory():
            ser = FakeSerial(lines=incoming)
            serials.append(ser)
            return ser

        bridge = self.start_bridge(factory)
        events = self.collect_events(
            bridge, until=lambda ev: isinstance(ev, PoseReached), timeout=2.0
        )
        bridge.stop()
        self.assertTrue(any(isinstance(ev, Pong) for ev in events))
        reached = [ev for ev in events if isinstance(ev, PoseReached)]
        self.assertTrue(reached)
        self.assertAlmostEqual(reached[0].error_mm, 2.1)


if __name__ == "__main__":
    unittest.main()
