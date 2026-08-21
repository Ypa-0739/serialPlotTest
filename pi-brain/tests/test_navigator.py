# -*- coding: utf-8 -*-
"""导航器单测：正常到达 / 超时 / 取消 / 故障 / 门禁 / 串行 / 心跳。

覆盖第四阶段验收要点：
  - 正常到达（含注入最终误差的回读）
  - 未收到 POSE START -> 锁存急停并失败
  - 运动总超时 -> emergency_stop 并失败
  - 用户取消成功 / 取消确认超时升级 STOP / 升级后仍无确认 -> 失败
  - 运动途中 OPS 丢失（# POSE STOP SAFETY）与串口断开
  - READY 前拒绝导航、急停锁存期间拒绝、活动航点期间拒绝第二个目标
  - 故障后旧目标不自动重发
  - 连续三个航点严格串行执行
  - 心跳在整个运动期间持续
  - 未确认启动就收到到位（协议异常）-> FAILED(UNEXPECTED_REACHED)
"""

import time
import unittest

from app.demo import FakeFirmware
from app.models import GotoReason, NavState
from app.navigator import NavigationRejected, Navigator
from app.serial_bridge import SerialBridgeThread


def pump_nav(bridge, nav, *, until, timeout=3.0, step=0.01):
    """事件泵：把 bridge 事件喂给导航器，直到 predicate 为真或超时。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        event = bridge.get_event(timeout=step)
        if event is not None:
            nav.handle_event(event)
        nav.tick()
        if until():
            return True
    return until()


class NavigatorTests(unittest.TestCase):
    def setUp(self):
        self.firmware = FakeFirmware()
        self.bridge = SerialBridgeThread(
            lambda: self.firmware, ping_interval=0.05, backoff_seconds=0.05
        )
        self.nav = Navigator(
            self.bridge,
            start_timeout_s=0.5,
            motion_timeout_s=1.0,
            cancel_confirm_timeout_s=0.3,
        )
        self.bridge.start()
        self.firmware.pose_reached_delay_s = 0.05  # 测试尽量快

    def tearDown(self):
        self.bridge.stop()

    def ready(self):
        """模拟启动自检完成：门禁打开、急停未锁存。"""
        self.nav.set_ready(True)

    # ------------------------------------------------------------------
    # 正常到达
    # ------------------------------------------------------------------

    def test_normal_reach(self):
        """正常到达：REACHED，回读注入的最终误差。"""
        self.firmware.pose_error_mm = 3.1
        self.firmware.pose_error_yaw = 0.6
        self.ready()
        gid = self.nav.goto_pose(1000, 500, 90)
        self.assertEqual(gid, 1)
        self.assertEqual(self.nav.state, NavState.WAIT_POSE_START)
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.REACHED)
        self.assertTrue(ok, f"未到达，state={self.nav.state}")
        r = self.nav.last_result
        self.assertTrue(r.success)
        self.assertEqual(r.reason, GotoReason.REACHED)
        self.assertAlmostEqual(r.position_error_mm, 3.1)
        self.assertAlmostEqual(r.yaw_error_deg, 0.6)
        self.assertAlmostEqual(r.target.x_mm, 1000)
        self.assertAlmostEqual(r.final_pose.x_mm, 1000)
        self.assertEqual(self.nav.state, NavState.REACHED)  # 终态保持可观察
        self.assertGreaterEqual(r.elapsed_s, 0.0)

    # ------------------------------------------------------------------
    # 超时
    # ------------------------------------------------------------------

    def test_start_timeout_latches_emergency_stop(self):
        """未收到POSE START也可能只是回复丢失：必须锁存急停并失败。"""
        self.firmware.pose_start_ok = False
        self.firmware.pose_reached_ok = False
        self.ready()
        self.nav.goto_pose(100, 100, 0)
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.FAILED, timeout=2.0)
        self.assertTrue(ok, f"未失败，state={self.nav.state}")
        self.assertEqual(self.nav.last_result.reason, GotoReason.START_TIMEOUT)
        self.assertFalse(self.nav.last_result.success)
        # 等待 STOP 被桥线程实际写出（入队与写出异步）
        self.assertTrue(
            pump_nav(self.bridge, self.nav, until=lambda: "STOP" in self.firmware.received, timeout=1.0),
            f"STOP 未写出，received={self.firmware.received}",
        )
        self.assertTrue(self.bridge.is_emergency_stopped)

    def test_motion_timeout_emergency_stop(self):
        """运动总超时：emergency_stop 锁存并失败。"""
        self.firmware.pose_reached_ok = False
        self.ready()
        self.nav.goto_pose(100, 100, 0)
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.FAILED, timeout=3.0)
        self.assertTrue(ok, f"未失败，state={self.nav.state}")
        self.assertEqual(self.nav.last_result.reason, GotoReason.MOTION_TIMEOUT)
        self.assertTrue(self.bridge.is_emergency_stopped)  # 运动超时必须锁存

    def test_per_goal_motion_timeout_override_is_used(self):
        """单航点传入的超时必须覆盖Navigator默认值。"""
        self.firmware.pose_reached_ok = False
        self.ready()
        self.nav.goto_pose(100, 100, 0, motion_timeout_s=0.2)
        ok = pump_nav(
            self.bridge,
            self.nav,
            until=lambda: self.nav.state == NavState.FAILED,
            timeout=0.7,
        )
        self.assertTrue(ok, f"单航点超时未生效，state={self.nav.state}")
        self.assertEqual(self.nav.last_result.reason, GotoReason.MOTION_TIMEOUT)

    # ------------------------------------------------------------------
    # 取消
    # ------------------------------------------------------------------

    def test_cancel_success(self):
        """用户取消：发 POSE STOP，收到 PoseStopped -> CANCELLED。"""
        self.firmware.pose_reached_delay_s = 5.0  # 运动迟迟不到达
        self.ready()
        gid = self.nav.goto_pose(100, 100, 0)
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.MOVING)
        self.assertTrue(ok, f"未进入 MOVING，state={self.nav.state}")
        self.assertTrue(self.nav.cancel(gid))
        self.assertEqual(self.nav.state, NavState.CANCELLING)
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.CANCELLED)
        self.assertTrue(ok, f"取消未完成，state={self.nav.state}")
        self.assertEqual(self.nav.last_result.reason, GotoReason.CANCELLED)
        self.assertFalse(self.nav.last_result.success)
        self.assertIn("POSE STOP", self.firmware.received)
        # POSE STOP 清除了未到期的到达响应：之后不会再收到迟到的 POSE TARGET
        pending = [text for _, text in self.firmware._pose_pending]
        self.assertNotIn("# POSE TARGET", pending)
        self.assertNotIn("# POSE START", pending)

    def test_cancel_wait_pose_start(self):
        """在 WAIT_POSE_START 阶段取消：POSE STOP 清除未到期的 POSE START。"""
        self.firmware.pose_start_delay_s = 0.5
        self.firmware.pose_reached_delay_s = 5.0
        self.ready()
        gid = self.nav.goto_pose(100, 100, 0)
        self.assertEqual(self.nav.state, NavState.WAIT_POSE_START)
        self.assertTrue(self.nav.cancel(gid))
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.CANCELLED, timeout=2.0)
        self.assertTrue(ok, f"取消未完成，state={self.nav.state}")
        # POSE STOP 已清除未到期响应，迟到的 POSE START 不会打扰后续新目标
        self.firmware.pose_start_delay_s = 0.0
        self.firmware.pose_reached_delay_s = 0.05
        self.ready()
        gid2 = self.nav.goto_pose(200, 200, 0)
        self.assertEqual(gid2, 2)
        self.assertTrue(pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.REACHED, timeout=2.0))
        self.assertTrue(self.nav.last_result.success)

    def test_cancel_confirm_timeout_escalates_to_stop(self):
        """取消确认超时：升级为 STOP，收到 # STOP MODE=WORK -> CANCELLED。"""
        self.firmware.pose_reached_delay_s = 5.0
        self.firmware.pose_stopped_delay_s = 5.0  # POSE STOP 确认丢失
        self.ready()
        gid = self.nav.goto_pose(100, 100, 0)
        pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.MOVING)
        self.assertTrue(self.nav.cancel(gid))
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.CANCELLED, timeout=2.0)
        self.assertTrue(ok, f"升级 STOP 后未取消，state={self.nav.state}")
        cmds = self.firmware.received
        self.assertLess(cmds.index("POSE STOP"), cmds.index("STOP"))  # 先 POSE STOP 后 STOP
        self.assertEqual(self.nav.last_result.reason, GotoReason.CANCELLED)

    def test_cancel_unconfirmed_fails(self):
        """升级 STOP 后仍无确认：emergency_stop 并 FAILED(CANCEL_UNCONFIRMED)。"""
        self.firmware.pose_reached_delay_s = 5.0
        self.firmware.pose_stopped_delay_s = 5.0
        self.firmware.ack_stop = False  # 对 STOP 也不确认
        self.ready()
        gid = self.nav.goto_pose(100, 100, 0)
        pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.MOVING)
        self.assertTrue(self.nav.cancel(gid))
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.FAILED, timeout=2.0)
        self.assertTrue(ok, f"未失败，state={self.nav.state}")
        self.assertEqual(self.nav.last_result.reason, GotoReason.CANCEL_UNCONFIRMED)
        self.assertTrue(self.bridge.is_emergency_stopped)

    def test_cancel_wrong_goal_id_ignored(self):
        """取消不匹配的 goal_id：无效果，活动目标不受影响。"""
        self.firmware.pose_reached_delay_s = 5.0
        self.ready()
        gid = self.nav.goto_pose(100, 100, 0)
        pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.MOVING)
        self.assertFalse(self.nav.cancel(gid + 1))
        self.assertEqual(self.nav.state, NavState.MOVING)
        self.assertNotIn("POSE STOP", self.firmware.received)

    # ------------------------------------------------------------------
    # 故障
    # ------------------------------------------------------------------

    def test_ops_lost_during_motion(self):
        """运动途中 OPS 丢失：SafetyFault -> FAILED 并锁存急停。"""
        self.firmware.pose_reached_delay_s = 5.0
        self.firmware.pose_safety_after_s = 0.2  # 运动 0.2s 后 # POSE STOP SAFETY
        self.ready()
        self.nav.goto_pose(100, 100, 0)
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.FAILED, timeout=2.0)
        self.assertTrue(ok, f"未失败，state={self.nav.state}")
        self.assertEqual(self.nav.last_result.reason, GotoReason.SAFETY_FAULT)
        self.assertTrue(self.bridge.is_emergency_stopped)

    def test_disconnect_during_motion(self):
        """运动途中串口断开：FAILED(DISCONNECTED)。"""
        self.firmware.pose_reached_delay_s = 5.0
        self.ready()
        self.nav.goto_pose(100, 100, 0)
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.MOVING)
        self.assertTrue(ok, f"未进入 MOVING，state={self.nav.state}")
        self.firmware.fail()
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.FAILED, timeout=2.0)
        self.assertTrue(ok, f"断线未失败，state={self.nav.state}")
        self.assertEqual(self.nav.last_result.reason, GotoReason.DISCONNECTED)

    def test_unexpected_reached_before_start(self):
        """未确认启动就收到到位（协议异常）：FAILED(UNEXPECTED_REACHED) 并急停。"""
        self.firmware.pose_start_ok = False
        self.ready()
        self.nav.goto_pose(100, 100, 0)
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.FAILED, timeout=2.0)
        self.assertTrue(ok, f"未失败，state={self.nav.state}")
        self.assertEqual(self.nav.last_result.reason, GotoReason.UNEXPECTED_REACHED)
        self.assertTrue(self.bridge.is_emergency_stopped)

    # ------------------------------------------------------------------
    # 门禁与单航点
    # ------------------------------------------------------------------

    def test_reject_before_ready(self):
        """READY 前（门禁关闭）拒绝导航。"""
        with self.assertRaises(NavigationRejected):
            self.nav.goto_pose(100, 100, 0)
        self.assertEqual(self.nav.state, NavState.IDLE)
        self.assertIsNone(self.nav.last_result)

    def test_reject_when_emergency_latched(self):
        """急停锁存期间拒绝；解除后可导航。"""
        self.ready()
        self.bridge.emergency_stop()
        with self.assertRaises(NavigationRejected):
            self.nav.goto_pose(100, 100, 0)
        self.bridge.release_emergency_stop()
        gid = self.nav.goto_pose(100, 100, 0)
        self.assertEqual(gid, 1)

    def test_reject_second_goal_while_active(self):
        """活动航点期间第二个目标被拒绝；取消后可立即提交。"""
        self.firmware.pose_reached_delay_s = 5.0
        self.ready()
        gid1 = self.nav.goto_pose(100, 100, 0)
        pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.MOVING)
        with self.assertRaises(NavigationRejected):
            self.nav.goto_pose(200, 200, 0)
        self.assertTrue(self.nav.cancel(gid1))
        pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.CANCELLED)
        gid2 = self.nav.goto_pose(200, 200, 0)
        self.assertEqual(gid2, 2)

    def test_no_auto_retry_after_failure(self):
        """故障后旧目标不自动重发。"""
        self.firmware.pose_reached_ok = False
        self.ready()
        self.nav.goto_pose(100, 100, 0)
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.FAILED, timeout=3.0)
        self.assertTrue(ok, f"未失败，state={self.nav.state}")
        pose_sets = lambda: self.firmware.received.count("POSE SET")  # noqa: E731
        before = pose_sets()
        # 继续泵一段时间，不应出现自动重发
        pump_nav(self.bridge, self.nav, until=lambda: False, timeout=0.4)
        self.assertEqual(pose_sets(), before)
        self.assertEqual(self.nav.state, NavState.FAILED)  # 终态保持

    def test_gate_closes_after_fault(self):
        """故障后重新自检完成前不能导航；自检完成后恢复。"""
        self.firmware.pose_reached_delay_s = 5.0
        self.firmware.pose_safety_after_s = 0.2
        self.ready()
        self.nav.goto_pose(100, 100, 0)
        pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.FAILED)
        # Brain 流程：进入 FAULT 关闭门禁（主循环在 startup 离开 READY 时调用）
        self.nav.set_ready(False)
        with self.assertRaises(NavigationRejected):
            self.nav.goto_pose(200, 200, 0)
        # 重新自检完成：急停解除 + 门禁打开，新目标被接受
        self.bridge.release_emergency_stop()
        self.nav.set_ready(True)
        gid = self.nav.goto_pose(200, 200, 0)
        self.assertEqual(gid, 2)
        self.assertEqual(self.nav.state, NavState.WAIT_POSE_START)

    # ------------------------------------------------------------------
    # 串行执行与心跳
    # ------------------------------------------------------------------

    def test_three_goals_strictly_serial(self):
        """连续三个航点严格串行执行：每个目标恰好一次 POSE SET。"""
        self.ready()
        goals = [(800, 300, 0), (950, 300, 0), (1500, 900, 90)]
        for i, (x, y, yaw) in enumerate(goals, start=1):
            gid = self.nav.goto_pose(x, y, yaw)
            self.assertEqual(gid, i)
            ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.REACHED, timeout=2.0)
            self.assertTrue(ok, f"目标 {i} 未到达，state={self.nav.state}")
            self.assertTrue(self.nav.last_result.success)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(pose_sets, [
            "POSE SET 800.00 300.00 0.00",
            "POSE SET 950.00 300.00 0.00",
            "POSE SET 1500.00 900.00 90.00",
        ])

    def test_heartbeat_during_motion(self):
        """心跳在整个运动期间持续（PING 由桥线程自行调度）。"""
        self.firmware.pose_reached_delay_s = 0.5  # 运动持续约 0.5s
        self.ready()
        self.nav.goto_pose(100, 100, 0)
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.MOVING)
        self.assertTrue(ok, f"未进入 MOVING，state={self.nav.state}")
        # 等待首个 PING 实际写出（PING 由桥线程按 deadline 调度）
        self.assertTrue(
            pump_nav(self.bridge, self.nav, until=lambda: self.firmware.received.count("PING") > 0, timeout=1.0),
            "心跳未开始",
        )
        pings_before = self.firmware.received.count("PING")
        ok = pump_nav(self.bridge, self.nav, until=lambda: self.nav.state == NavState.REACHED, timeout=2.0)
        self.assertTrue(ok, f"未到达，state={self.nav.state}")
        pings_after = self.firmware.received.count("PING")
        self.assertGreater(pings_before, 0)
        self.assertGreater(pings_after, pings_before, "运动期间心跳应持续")

    # ------------------------------------------------------------------
    # FakeFirmware 覆盖语义
    # ------------------------------------------------------------------

    def test_fake_override_old_goal(self):
        """固件侧覆盖：新 POSE SET 丢弃旧目标未到期的响应。

        使用独立 firmware（不经桥线程），避免后台 readline 提前消费到期响应。
        """
        firmware = FakeFirmware(pose_start_delay_s=1.0, pose_reached_delay_s=1.0)
        firmware.write(b"POSE SET 100 100 0\n")
        firmware.write(b"POSE SET 200 200 90\n")  # 覆盖旧目标
        # 旧目标的响应被清空，只剩第二个目标的 START + TARGET
        pending = [text for _, text in firmware._pose_pending]
        self.assertEqual(len(pending), 2)
        self.assertTrue(all("X=200.00 Y=200.00 YAW=90.00" in text for text in pending))


if __name__ == "__main__":
    unittest.main()
