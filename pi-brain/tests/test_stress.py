# -*- coding: utf-8 -*-
"""压力测试：长时间无 RX 心跳不断；大量命令下 PING/STOP 不被饿死。"""

import time
import unittest

from app.serial_bridge import SerialBridgeThread
from app.protocol import RawMessage
from tests.fakes import FakeSerial

PING = b"PING\n"


class SilentRxStressTests(unittest.TestCase):
    def test_ping_never_stops_without_rx(self):
        """连续 3 秒无任何 RX，PING 必须持续（0.05s 间隔 → 预期约 60 个）。"""
        serials = []

        def factory():
            ser = FakeSerial()  # 无预置行 -> readline 永远超时
            serials.append(ser)
            return ser

        bridge = SerialBridgeThread(factory, ping_interval=0.05, backoff_seconds=0.05)
        bridge.start()
        time.sleep(3.0)
        bridge.stop()
        pings = sum(1 for w in serials[0].writes if w == PING)
        self.assertGreaterEqual(pings, 40, f"仅收到 {pings} 个 PING，心跳被中断")


class TxStormStressTests(unittest.TestCase):
    def test_ping_and_stop_not_starved_by_command_flood(self):
        """快速塞入 2000 条普通命令 + STOP，PING 与 STOP 都不能被饿死。"""
        serials = []

        def factory():
            ser = FakeSerial()
            serials.append(ser)
            return ser

        # tx_drain_limit=4：制造队列拥堵（每轮最多发 4 条普通命令）
        bridge = SerialBridgeThread(
            factory, ping_interval=0.05, backoff_seconds=0.05, tx_drain_limit=4
        )
        bridge.start()
        for i in range(2000):
            bridge.send(f"CONFIG {i}")
        bridge.emergency_stop()
        time.sleep(1.0)
        bridge.stop()

        writes = serials[0].writes
        pings = sum(1 for w in writes if w == PING)
        self.assertGreaterEqual(pings, 15, f"仅收到 {pings} 个 PING，心跳被饿死")

        non_ping = [w for w in writes if w != PING]
        self.assertIn(b"STOP\n", non_ping)
        stop_index = writes.index(b"STOP\n")
        self.assertTrue(
            all(w == PING for w in writes[stop_index + 1 :]),
            "STOP 发出后仍发送了旧队列命令",
        )


class LoopbackIntegrationTests(unittest.TestCase):
    """pySerial loop:// 回环集成测试：写-读自闭环，验证线程自驱动。"""

    @classmethod
    def setUpClass(cls):
        try:
            import serial  # noqa: F401

            cls.available = True
        except ImportError:
            cls.available = False

    def _make_bridge(self, ping_interval):
        import serial

        def factory():
            return serial.serial_for_url("loop://", timeout=0.05, write_timeout=0.1)

        return SerialBridgeThread(factory, ping_interval=ping_interval, backoff_seconds=0.05)

    def test_loopback_self_drives_events(self):
        """回环中 PING 写-读闭环：事件流出现自己回环的 RawMessage。"""
        if not self.available:
            self.skipTest("pyserial 未安装")
        bridge = self._make_bridge(ping_interval=0.05)
        bridge.start()
        raw_messages = []
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            ev = bridge.get_event(timeout=0.1)
            if isinstance(ev, RawMessage):
                raw_messages.append(ev)
        bridge.stop()
        # 回环读到的是自己发的 "PING"（非 # 开头、不足 17 列 -> RawMessage）
        self.assertGreaterEqual(
            len(raw_messages), 3, f"回环只读到 {len(raw_messages)} 条 RawMessage"
        )


if __name__ == "__main__":
    unittest.main()
