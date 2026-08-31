# -*- coding: utf-8 -*-
"""比赛任务状态机单测：完整流程 / 机构失败 / 安全急停 / 暂停恢复 / 中止。

覆盖第五阶段验收要点：
  - WAIT_START→…→FINISHED 全流程，POSE SET 序列与预标定航点一致，
    任务自身从不直接发 POSE SET（全部经 Navigator）
  - PICK/DROP 机构失败 -> ABORTED，后续航点不再提交
  - 运动中固件安全故障 -> EMERGENCY_STOP 且急停锁存，绝不自动恢复
  - 运动超时 -> ABORTED
  - pause() -> PAUSED；resume() 经 RECOVERY 重驱被中断航点后完成全场
  - abort() 中止当前运动并进入 ABORTED
  - 非法状态操作抛 MissionRejected；终态后不可再操作
"""

import time
import unittest

from app.demo import FakeFirmware
from app.mission import Mission, MissionConfig, MissionRejected, MissionState, NullMechanism
from app.models import GotoReason, NavState, PoseGoal
from app.navigator import Navigator
from app.serial_bridge import SerialBridgeThread


class FlakyMechanism:
    """可配置成功/失败的机构替身。"""

    def __init__(self, pick_ok: bool = True, drop_ok: bool = True) -> None:
        self.pick_ok = pick_ok
        self.drop_ok = drop_ok
        self.pick_calls = 0
        self.drop_calls = 0

    def pick(self) -> bool:
        self.pick_calls += 1
        return self.pick_ok

    def drop(self) -> bool:
        self.drop_calls += 1
        return self.drop_ok


def make_config() -> MissionConfig:
    return MissionConfig(
        pick_approach=PoseGoal("pick_approach", 100, 400, 0),
        pick_align=PoseGoal("pick_align", 200, 500, 10),
        drop_approach=PoseGoal("drop_approach", 800, 400, 10),
        drop_align=PoseGoal("drop_align", 900, 300, 90),
        return_home=PoseGoal("return_home", 0, 0, 0),
    )


def pose_set_texts(config: MissionConfig) -> list[str]:
    goals = [
        config.pick_approach, config.pick_align,
        config.drop_approach, config.drop_align, config.return_home,
    ]
    return [f"POSE SET {g.x_mm:.2f} {g.y_mm:.2f} {g.yaw_deg:.2f}" for g in goals]


def pump(bridge, nav, mission, *, until, timeout=3.0, step=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        event = bridge.get_event(timeout=step)
        if event is not None:
            nav.handle_event(event)
        nav.tick()
        mission.tick()
        if until():
            return True
    return until()


class MissionTests(unittest.TestCase):
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
        self.config = make_config()
        self.mech = FlakyMechanism()
        self.mission = Mission(self.nav, self.config, self.mech)
        self.bridge.start()

    def tearDown(self):
        self.bridge.stop()

    def ready(self):
        self.nav.set_ready(True)

    def begin(self):
        self.mission.start()
        self.mission.begin_run()

    # ------------------------------------------------------------------
    # 正常全流程
    # ------------------------------------------------------------------

    def test_full_flow_finishes_with_exact_waypoint_sequence(self):
        """全流程 FINISHED；5 个航点严格按配置顺序各提交一次。"""
        self.ready()
        self.begin()
        self.assertEqual(self.mission.state, MissionState.MOVE_TO_PICK)
        ok = pump(
            self.bridge, self.nav, self.mission,
            until=lambda: self.mission.state == MissionState.FINISHED,
            timeout=6.0,
        )
        self.assertTrue(ok, f"未完成，state={self.mission.state}")
        self.assertTrue(self.mission.is_terminal)
        self.assertEqual(self.mech.pick_calls, 1)
        self.assertEqual(self.mech.drop_calls, 1)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(pose_sets, pose_set_texts(self.config))

    def test_operations_before_start_rejected(self):
        """未 start 不能 begin_run；重复 start / begin 也被拒绝。"""
        with self.assertRaises(MissionRejected):
            self.mission.begin_run()
        self.mission.start()
        self.assertEqual(self.mission.state, MissionState.WAIT_START)
        with self.assertRaises(MissionRejected):
            self.mission.start()
        self.mission.begin_run()
        with self.assertRaises(MissionRejected):
            self.mission.begin_run()

    # ------------------------------------------------------------------
    # 机构失败
    # ------------------------------------------------------------------

    def test_pick_failure_aborts_before_move_to_drop(self):
        """PICK 失败 -> ABORTED；MOVE_TO_DROP 及之后航点不提交。"""
        self.mech.pick_ok = False
        self.ready()
        self.begin()
        ok = pump(
            self.bridge, self.nav, self.mission,
            until=lambda: self.mission.is_terminal,
            timeout=4.0,
        )
        self.assertTrue(ok)
        self.assertEqual(self.mission.state, MissionState.ABORTED)
        self.assertIn("PICK FAILED", self.mission.last_reason)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(len(pose_sets), 2)  # approach + align，之后无运动

    def test_drop_failure_aborts_before_return(self):
        """DROP 失败 -> ABORTED；RETURN 不提交。"""
        self.mech.drop_ok = False
        self.ready()
        self.begin()
        ok = pump(
            self.bridge, self.nav, self.mission,
            until=lambda: self.mission.is_terminal,
            timeout=6.0,
        )
        self.assertTrue(ok)
        self.assertEqual(self.mission.state, MissionState.ABORTED)
        self.assertIn("DROP FAILED", self.mission.last_reason)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(len(pose_sets), 4)  # 前 4 个航点完成后失败

    # ------------------------------------------------------------------
    # 安全故障与超时
    # ------------------------------------------------------------------

    def test_safety_fault_enters_emergency_stop_and_latches(self):
        """MOVE_TO_PICK 中 OPS 丢失：EMERGENCY_STOP + 急停锁存 + 不自动恢复。"""
        self.firmware.pose_reached_ok = False
        self.firmware.pose_safety_after_s = 0.15
        self.ready()
        self.begin()
        ok = pump(
            self.bridge, self.nav, self.mission,
            until=lambda: self.mission.state == MissionState.EMERGENCY_STOP,
            timeout=3.0,
        )
        self.assertTrue(ok, f"未进入 EMERGENCY_STOP，state={self.mission.state}")
        self.assertEqual(self.mission.last_reason, GotoReason.SAFETY_FAULT.value)
        self.assertTrue(self.bridge.is_emergency_stopped)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        before = len(pose_sets)
        # 绝不自动恢复：继续泵不得出现新的 POSE SET
        pump(self.bridge, self.nav, self.mission, until=lambda: False, timeout=0.4)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(len(pose_sets), before)
        with self.assertRaises(MissionRejected):
            self.mission.begin_run()  # 终态后操作拒绝

    def test_motion_timeout_aborts(self):
        """航点运动超时 -> ABORTED(MOTION_TIMEOUT)。"""
        self.config = MissionConfig(
            pick_approach=PoseGoal("pick_approach", 100, 400, 0, timeout_s=0.2),
            pick_align=PoseGoal("pick_align", 200, 500, 10),
            drop_approach=PoseGoal("drop_approach", 800, 400, 10),
            drop_align=PoseGoal("drop_align", 900, 300, 90),
            return_home=PoseGoal("return_home", 0, 0, 0),
        )
        self.mission = Mission(self.nav, self.config, self.mech)
        self.firmware.pose_reached_ok = False
        self.ready()
        self.begin()
        ok = pump(
            self.bridge, self.nav, self.mission,
            until=lambda: self.mission.is_terminal,
            timeout=3.0,
        )
        self.assertTrue(ok)
        self.assertEqual(self.mission.state, MissionState.ABORTED)
        self.assertEqual(self.mission.last_reason, GotoReason.MOTION_TIMEOUT.value)

    # ------------------------------------------------------------------
    # 暂停与恢复
    # ------------------------------------------------------------------

    def test_pause_then_resume_completes_mission(self):
        """暂停 -> PAUSED；resume 经 RECOVERY 重驱原航点并完成全场。"""
        self.firmware.pose_reached_delay_s = 5.0  # 第一航点迟迟不到
        self.ready()
        self.begin()
        ok = pump(self.bridge, self.nav, self.mission, until=lambda: self.nav.state == NavState.MOVING)
        self.assertTrue(ok, "未进入 MOVING")
        self.assertTrue(self.mission.pause())
        ok = pump(
            self.bridge, self.nav, self.mission,
            until=lambda: self.mission.state == MissionState.PAUSED,
            timeout=2.0,
        )
        self.assertTrue(ok, f"未暂停，state={self.mission.state}")
        # PAUSED 时机构动作尚未发生
        self.assertEqual(self.mech.pick_calls, 0)
        self.firmware.pose_reached_delay_s = 0.05
        self.assertTrue(self.mission.resume())
        self.assertEqual(self.mission.state, MissionState.RECOVERY)  # tick 前可观察
        ok = pump(
            self.bridge, self.nav, self.mission,
            until=lambda: self.mission.state == MissionState.FINISHED,
            timeout=8.0,
        )
        self.assertTrue(ok, f"恢复后未完成，state={self.mission.state}")
        # 被中断的 pick_approach 提交了两次，其余各一次
        expected = pose_set_texts(self.config)
        expected.insert(1, expected[0])
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(pose_sets, expected)

    def test_pause_in_wait_start_then_abort(self):
        """WAIT_START 可直接暂停；无活动航点时 resume 失败、abort 生效。"""
        self.mission.start()
        self.assertTrue(self.mission.pause())
        self.assertEqual(self.mission.state, MissionState.PAUSED)
        self.assertFalse(self.mission.resume())  # 无被中断航点可恢复
        self.assertTrue(self.mission.abort())
        self.assertEqual(self.mission.state, MissionState.ABORTED)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(pose_sets, [])  # 从未发车

    # ------------------------------------------------------------------
    # 中止
    # ------------------------------------------------------------------

    def test_abort_during_motion(self):
        """运动中中止：POSE STOP 发出，任务 ABORTED 终态。"""
        self.firmware.pose_reached_delay_s = 5.0
        self.ready()
        self.begin()
        ok = pump(self.bridge, self.nav, self.mission, until=lambda: self.nav.state == NavState.MOVING)
        self.assertTrue(ok, "未进入 MOVING")
        self.assertTrue(self.mission.abort())
        ok = pump(
            self.bridge, self.nav, self.mission,
            until=lambda: self.mission.state == MissionState.ABORTED,
            timeout=2.0,
        )
        self.assertTrue(ok, f"未中止，state={self.mission.state}")
        self.assertIn("POSE STOP", self.firmware.received)
        pose_sets = [c for c in self.firmware.received if c.startswith("POSE SET")]
        self.assertEqual(len(pose_sets), 1)  # 后续航点不再执行

    def test_abort_after_terminal_is_noop(self):
        """终态后 abort/pause 返回 False，状态保持。"""
        self.mission.start()
        self.mission.pause()
        self.mission.abort()
        self.assertFalse(self.mission.abort())
        self.assertFalse(self.mission.pause())
        self.assertEqual(self.mission.state, MissionState.ABORTED)


if __name__ == "__main__":
    unittest.main()
