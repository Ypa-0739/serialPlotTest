# -*- coding: utf-8 -*-
"""路由调度器单测：严格串行 / 中途失败中止 / 取消 / 拒绝条件。

覆盖验收要点：
  - 三个航点严格串行完成：每个目标恰好一次 POSE SET，顺序与路线一致
  - 任一航点失败 -> 剩余航点不再提交（ABORTED）
  - 运动中取消 -> 当前航点 POSE STOP，剩余航点不再执行
  - 空路线 / 路线运行中重复 start / 导航器忙 -> RouteRejected
  - 门禁未开（未 READY）时第一个航点被拒 -> ABORTED，不重试
  - 完成后可再次 start_route 执行新路线
"""

import time
import unittest

from app.demo import FakeFirmware
from app.models import NavState, PoseGoal
from app.navigator import Navigator
from app.route_runner import RouteRejected, RouteRunner, RouteState
from app.serial_bridge import SerialBridgeThread


def pump(bridge, nav, route, *, until, timeout=3.0, step=0.01):
    """事件泵：bridge 事件喂导航器；导航器与路由各 tick 一次。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        event = bridge.get_event(timeout=step)
        if event is not None:
            nav.handle_event(event)
        nav.tick()
        route.tick()
        if until():
            return True
    return until()


class RouteRunnerTests(unittest.TestCase):
    def setUp(self):
        self.firmware = FakeFirmware()
        self.firmware.pose_reached_delay_s = 0.05
        self.bridge = SerialBridgeThread(
            lambda: self.firmware, ping_interval=0.05, backoff_seconds=0.05
        )
        self.nav = Navigator(
            self.bridge,
            start_timeout_s=0.5,
            motion_timeout_s=2.0,
            cancel_confirm_timeout_s=0.3,
        )
        self.route = RouteRunner(self.nav)
        self.bridge.start()

    def tearDown(self):
        self.bridge.stop()

    def ready(self):
        self.nav.set_ready(True)

    # ------------------------------------------------------------------
    # 正常串行完成
    # ------------------------------------------------------------------

    def test_three_waypoint_route_completes_serially(self):
        """三个航点严格串行：POSE SET 顺序与路线一致，最终 COMPLETED。"""
        self.ready()
        goals = [
            PoseGoal("a", 800, 300, 0),
            PoseGoal("b", 950, 300, 0),
            PoseGoal("c", 1500, 900, 90),
        ]
        self.assertEqual(self.route.start_route(goals), 3)
        self.assertEqual(self.route.state, RouteState.RUNNING)
        ok = pump(
            self.bridge, self.nav, self.route,
            until=lambda: self.route.state == RouteState.COMPLETED,
            timeout=5.0,
        )
        self.assertTrue(ok, f"路线未完成，state={self.route.state}")
        self.assertEqual(self.route.index, 3)
        self.assertIsNone(self.route.current_goal)
        self.assertEqual(self.route.last_reason, "REACHED")
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(pose_sets, [
            "POSE SET 800.00 300.00 0.00",
            "POSE SET 950.00 300.00 0.00",
            "POSE SET 1500.00 900.00 90.00",
        ])
        # 串口上从未同时存在两个未完结目标：每次 REACHED 后才发下一条
        self.assertEqual(self.nav.state, NavState.REACHED)

    def test_rerun_after_completion(self):
        """上一条路线完成后允许开始新路线。"""
        self.ready()
        self.route.start_route([PoseGoal("a", 100, 100, 0)])
        self.assertTrue(pump(
            self.bridge, self.nav, self.route,
            until=lambda: self.route.state == RouteState.COMPLETED,
        ))
        self.route.start_route([PoseGoal("b", 200, 200, 0)])
        self.assertTrue(pump(
            self.bridge, self.nav, self.route,
            until=lambda: self.route.state == RouteState.COMPLETED,
        ))
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(len(pose_sets), 2)

    # ------------------------------------------------------------------
    # 失败中止
    # ------------------------------------------------------------------

    def test_failure_mid_route_aborts_remaining(self):
        """第二个航点安全故障：ABORTED，第三个航点绝不提交。"""
        self.ready()
        self.route.start_route([
            PoseGoal("a", 100, 100, 0),
            PoseGoal("b", 200, 200, 0),
            PoseGoal("c", 300, 300, 0),
        ])
        # 等第一个航点完成、第二个航点已提交后注入“OPS 丢失且无到位”
        ok = pump(self.bridge, self.nav, self.route, until=lambda: self.route.index == 1)
        self.assertTrue(ok, f"第一航点未完成，state={self.route.state}")
        self.firmware.pose_reached_ok = False
        self.firmware.pose_safety_after_s = 0.15
        ok = pump(
            self.bridge, self.nav, self.route,
            until=lambda: self.route.state == RouteState.ABORTED,
            timeout=3.0,
        )
        self.assertTrue(ok, f"未中止，state={self.route.state}")
        self.assertIn("SAFETY_FAULT", self.route.last_reason)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(len(pose_sets), 2)  # 第三个航点绝不出现在串口上
        self.assertTrue(self.bridge.is_emergency_stopped)  # 固件安全故障已锁存急停

    def test_cancel_mid_route_aborts_remaining(self):
        """运动中取消：当前航点 POSE STOP，剩余航点不执行。"""
        self.firmware.pose_reached_delay_s = 5.0  # 第一航点迟迟不到
        self.ready()
        self.route.start_route([
            PoseGoal("a", 100, 100, 0),
            PoseGoal("b", 200, 200, 0),
        ])
        ok = pump(self.bridge, self.nav, self.route, until=lambda: self.nav.state == NavState.MOVING)
        self.assertTrue(ok, f"未进入 MOVING，state={self.nav.state}")
        self.assertTrue(self.route.cancel())
        ok = pump(
            self.bridge, self.nav, self.route,
            until=lambda: self.route.state == RouteState.ABORTED,
            timeout=2.0,
        )
        self.assertTrue(ok, f"取消未生效，state={self.route.state}")
        self.assertEqual(self.route.last_reason, "CANCELLED")
        self.assertIn("POSE STOP", self.firmware.received)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(len(pose_sets), 1)  # 第二个航点从未提交

    # ------------------------------------------------------------------
    # 拒绝条件
    # ------------------------------------------------------------------

    def test_start_rejects_empty_and_duplicate_routes(self):
        """空路线拒绝；路线运行期间再次 start 拒绝。"""
        self.ready()
        with self.assertRaises(RouteRejected):
            self.route.start_route([])
        self.firmware.pose_reached_delay_s = 5.0
        self.route.start_route([PoseGoal("a", 100, 100, 0)])
        with self.assertRaises(RouteRejected):
            self.route.start_route([PoseGoal("b", 200, 200, 0)])
        self.assertEqual(self.route.state, RouteState.RUNNING)

    def test_start_rejected_when_navigator_busy(self):
        """导航器已有活动航点时拒绝开新路线。"""
        self.firmware.pose_reached_delay_s = 5.0
        self.ready()
        self.nav.goto_pose(100, 100, 0)
        pump(self.bridge, self.nav, self.route, until=lambda: self.nav.state == NavState.WAIT_POSE_START)
        with self.assertRaises(RouteRejected):
            self.route.start_route([PoseGoal("a", 200, 200, 0)])

    def test_gate_closed_first_submit_aborts_without_retry(self):
        """门禁未开：第一个航点提交即被拒 -> ABORTED 且不重试。"""
        goals = [PoseGoal("a", 100, 100, 0), PoseGoal("b", 200, 200, 0)]
        self.assertEqual(self.route.start_route(goals), 2)  # 未 set_ready
        self.assertEqual(self.route.state, RouteState.ABORTED)
        self.assertIn("rejected", self.route.last_reason)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(pose_sets, [])
        # 持续泵一段时间不得出现自动重试
        pump(self.bridge, self.nav, self.route, until=lambda: False, timeout=0.3)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(pose_sets, [])

    def test_tick_noop_when_idle(self):
        """IDLE/终态下 tick() 是空操作。"""
        self.route.tick()
        self.assertEqual(self.route.state, RouteState.IDLE)


if __name__ == "__main__":
    unittest.main()
