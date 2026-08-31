# -*- coding: utf-8 -*-
"""双速度档单测与集成：先切档后发车 / 缓存去重 / 越界拒绝 / 中止语义。

覆盖验收要点：
  - apply() 发出 PID LIMIT X/Y/YAW 三条命令，且在随后的 POSE SET 之前
    经唯一写者按序写出（同 PRIORITY_MOTION FIFO）
  - 同档重复发车不重发 LIMIT；mark_unknown() 后强制重发
  - 档名未知 KeyError、数值越界 ValueError，且不发送任何命令
  - RouteRunner/Mission 集成：goal.profile 驱动切档；非法档名 -> 路线/
    任务 ABORTED，绝不发车
  - 预设档值在固件 PID LIMIT 硬边界内（0.02~0.30 m/s / 0.02~0.80 rad/s）
"""

import time
import unittest

from app.demo import FakeFirmware
from app.mission import Mission, MissionConfig, MissionState
from app.models import NavState, PoseGoal
from app.navigator import Navigator
from app.route_runner import RouteRunner, RouteState
from app.serial_bridge import SerialBridgeThread
from app.speed_profile import (
    FIRMWARE_LINEAR_MAX_MPS,
    FIRMWARE_LINEAR_MIN_MPS,
    FIRMWARE_YAW_MAX_RADPS,
    FIRMWARE_YAW_MIN_RADPS,
    DEFAULT_PROFILES,
    SpeedProfile,
    SpeedProfileController,
)


def pump(bridge, nav, consumer, *, until, timeout=3.0, step=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        event = bridge.get_event(timeout=step)
        if event is not None:
            nav.handle_event(event)
        nav.tick()
        consumer.tick()
        if until():
            return True
    return until()


def wait_until(predicate, timeout=2.0, step=0.01):
    """等待桥线程把队列命令实际写出（入队与写出异步）。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


class SpeedProfileControllerTests(unittest.TestCase):
    def setUp(self):
        self.firmware = FakeFirmware()
        self.bridge = SerialBridgeThread(
            lambda: self.firmware, ping_interval=0.05, backoff_seconds=0.05
        )
        self.bridge.start()

    def tearDown(self):
        self.bridge.stop()

    def received_limits(self):
        return [c for c in self.firmware.received if c.startswith("PID LIMIT")]

    def test_apply_sends_three_limits(self):
        """apply(cruise) 发出 X/Y/YAW 三条 LIMIT，数值与档位一致。"""
        controller = SpeedProfileController(self.bridge)
        self.assertTrue(controller.apply("cruise"))
        self.assertTrue(wait_until(lambda: len(self.received_limits()) >= 3))
        self.assertEqual(self.received_limits(), [
            "PID LIMIT X 0.250",
            "PID LIMIT Y 0.250",
            "PID LIMIT YAW 0.250",
        ])
        self.assertEqual(controller.current, "cruise")

    def test_same_profile_skips_resend(self):
        """同档重复 apply 不重发。"""
        controller = SpeedProfileController(self.bridge)
        controller.apply("precise")
        self.assertTrue(wait_until(lambda: len(self.received_limits()) >= 3))
        self.assertFalse(controller.apply("precise"))
        time.sleep(0.1)  # 给可能的误发留观察窗
        self.assertEqual(len(self.received_limits()), 3)

    def test_switch_profile_resends(self):
        """切换档位时重发新数值。"""
        controller = SpeedProfileController(self.bridge)
        controller.apply("precise")
        controller.apply("cruise")
        self.assertTrue(wait_until(lambda: len(self.received_limits()) >= 6))
        limits = self.received_limits()
        self.assertEqual(limits[:3], [
            "PID LIMIT X 0.120",
            "PID LIMIT Y 0.120",
            "PID LIMIT YAW 0.120",
        ])
        self.assertEqual(limits[3:], [
            "PID LIMIT X 0.250",
            "PID LIMIT Y 0.250",
            "PID LIMIT YAW 0.250",
        ])

    def test_mark_unknown_forces_resend(self):
        """mark_unknown 后（模拟重连/重新自检）下一次 apply 强制重发。"""
        controller = SpeedProfileController(self.bridge)
        controller.apply("cruise")
        self.assertTrue(wait_until(lambda: len(self.received_limits()) >= 3))
        controller.mark_unknown()
        self.assertIsNone(controller.current)
        self.assertTrue(controller.apply("cruise"))
        self.assertTrue(wait_until(lambda: len(self.received_limits()) >= 6))

    def test_unknown_name_raises_and_sends_nothing(self):
        controller = SpeedProfileController(self.bridge)
        with self.assertRaises(KeyError):
            controller.apply("turbo")
        time.sleep(0.1)
        self.assertEqual(self.received_limits(), [])

    def test_out_of_bounds_rejected_and_sends_nothing(self):
        """越界档（超固件硬上限/低于下限）拒绝且一条命令都不发。"""
        bad = SpeedProfile("bad", FIRMWARE_LINEAR_MAX_MPS + 0.01, 0.25)
        low_yaw = SpeedProfile("low_yaw", 0.20, FIRMWARE_YAW_MIN_RADPS - 0.01)
        controller = SpeedProfileController(
            self.bridge, profiles={"bad": bad, "low_yaw": low_yaw}
        )
        with self.assertRaises(ValueError):
            controller.apply("bad")
        with self.assertRaises(ValueError):
            controller.apply("low_yaw")
        time.sleep(0.1)
        self.assertEqual(self.received_limits(), [])

    def test_defaults_within_firmware_bounds(self):
        """预设两档都在固件 PID LIMIT 硬边界内，且为预期的巡航/定位值。"""
        self.assertEqual(
            DEFAULT_PROFILES["cruise"],
            SpeedProfile("cruise", 0.25, 0.25),
        )
        self.assertEqual(
            DEFAULT_PROFILES["precise"],
            SpeedProfile("precise", 0.12, 0.12),
        )
        for profile in DEFAULT_PROFILES.values():
            self.assertGreaterEqual(profile.linear_mps, FIRMWARE_LINEAR_MIN_MPS)
            self.assertLessEqual(profile.linear_mps, FIRMWARE_LINEAR_MAX_MPS)
            self.assertGreaterEqual(profile.yaw_radps, FIRMWARE_YAW_MIN_RADPS)
            self.assertLessEqual(profile.yaw_radps, FIRMWARE_YAW_MAX_RADPS)

    def test_limits_are_not_motion_commands(self):
        """PID LIMIT 是非运动命令：急停锁存期间仍可发出（安全处置用）。"""
        from app.protocol import is_motion_command

        self.assertFalse(is_motion_command("PID LIMIT X 0.250"))


class RouteProfileIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.firmware = FakeFirmware()
        self.firmware.pose_reached_delay_s = 0.05
        self.bridge = SerialBridgeThread(
            lambda: self.firmware, ping_interval=0.05, backoff_seconds=0.05
        )
        self.nav = Navigator(self.bridge, start_timeout_s=0.5, motion_timeout_s=2.0)
        self.profiles = SpeedProfileController(self.bridge)
        self.route = RouteRunner(self.nav, profiles=self.profiles)
        self.bridge.start()
        self.nav.set_ready(True)

    def tearDown(self):
        self.bridge.stop()

    def test_limit_precedes_each_pose_set_and_switches_once(self):
        """每条腿先 LIMIT 后 POSE SET；同档连续腿不重复切档。"""
        goals = [
            PoseGoal("far", 800, 300, 0, profile="cruise"),
            PoseGoal("near", 950, 300, 0, profile="cruise"),
            PoseGoal("align", 980, 320, 0, profile="precise"),
        ]
        self.route.start_route(goals)
        ok = pump(
            self.bridge, self.nav, self.route,
            until=lambda: self.route.state == RouteState.COMPLETED,
            timeout=5.0,
        )
        self.assertTrue(ok, f"路线未完成，state={self.route.state}")
        received = self.firmware.received
        # 顺序断言：cruise LIMIT -> POSE SET(far) -> POSE SET(near)（同档不重发）
        # -> precise LIMIT -> POSE SET(align)
        i_cruise = received.index("PID LIMIT X 0.250")
        i_far = received.index("POSE SET 800.00 300.00 0.00")
        i_near = received.index("POSE SET 950.00 300.00 0.00")
        i_precise = received.index("PID LIMIT X 0.120")
        i_align = received.index("POSE SET 980.00 320.00 0.00")
        self.assertLess(i_cruise, i_far)
        self.assertLess(i_far, i_near)
        self.assertLess(i_near, i_precise)
        self.assertLess(i_precise, i_align)
        limits = [c for c in received if c.startswith("PID LIMIT")]
        self.assertEqual(len(limits), 6)  # 仅两次切档，各 3 条

    def test_invalid_profile_aborts_without_motion(self):
        """档名非法：路线 ABORTED，串口上没有任何 POSE SET。"""
        goals = [PoseGoal("a", 100, 100, 0, profile="turbo")]
        self.route.start_route(goals)
        self.assertEqual(self.route.state, RouteState.ABORTED)
        self.assertIn("speed profile invalid", self.route.last_reason)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(pose_sets, [])


class MissionProfileIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.firmware = FakeFirmware()
        self.firmware.pose_reached_delay_s = 0.05
        self.bridge = SerialBridgeThread(
            lambda: self.firmware, ping_interval=0.05, backoff_seconds=0.05
        )
        self.nav = Navigator(self.bridge, start_timeout_s=0.5, motion_timeout_s=2.0)
        self.config = MissionConfig(
            pick_approach=PoseGoal("pick_approach", 100, 400, 0, profile="cruise"),
            pick_align=PoseGoal("pick_align", 200, 500, 10),
            drop_approach=PoseGoal("drop_approach", 800, 400, 10, profile="cruise"),
            drop_align=PoseGoal("drop_align", 900, 300, 90),
            return_home=PoseGoal("return_home", 0, 0, 0),
        )
        self.profiles = SpeedProfileController(self.bridge)
        self.mission = Mission(self.nav, self.config, profiles=self.profiles)
        self.bridge.start()
        self.nav.set_ready(True)

    def tearDown(self):
        self.bridge.stop()

    def test_mission_switches_profile_per_leg(self):
        """任务流程中每条腿按 goal.profile 切档：cruise→precise→cruise→…"""
        self.mission.start()
        self.mission.begin_run()
        ok = pump(
            self.bridge, self.nav, self.mission,
            until=lambda: self.mission.state == MissionState.FINISHED,
            timeout=6.0,
        )
        self.assertTrue(ok, f"任务未完成，state={self.mission.state}")
        received = self.firmware.received
        pose_sets = [c for c in received if c.startswith("POSE SET")]
        self.assertEqual(len(pose_sets), 5)
        # 第一腿 cruise：LIMIT 在首个 POSE SET 之前
        self.assertLess(
            received.index("PID LIMIT X 0.250"),
            received.index("POSE SET 100.00 400.00 0.00"),
        )
        # 第二腿默认 precise：在第一、二腿 POSE SET 之间出现
        i_first = received.index("POSE SET 100.00 400.00 0.00")
        i_second = received.index("POSE SET 200.00 500.00 10.00")
        i_precise = received.index("PID LIMIT X 0.120")
        self.assertLess(i_first, i_precise)
        self.assertLess(i_precise, i_second)
        # 第三腿回到 cruise：再次切档
        i_third = received.index("POSE SET 800.00 400.00 10.00")
        i_cruise2 = received.index("PID LIMIT X 0.250", i_precise + 1)
        self.assertLess(i_second, i_cruise2)
        self.assertLess(i_cruise2, i_third)


if __name__ == "__main__":
    unittest.main()
