# -*- coding: utf-8 -*-
"""第五阶段：比赛任务状态机（预标定航点序列，无 A*/样条/动态避障）。

状态流：
    IDLE -> WAIT_START -> MOVE_TO_PICK -> ALIGN_PICK -> PICK
         -> MOVE_TO_DROP -> ALIGN_DROP -> DROP -> RETURN -> FINISHED
    附加：PAUSED / RECOVERY / ABORTED / EMERGENCY_STOP

职责边界（必须遵守）：
  - 本模块只调用 navigator.goto_pose()/cancel() 与机构接口
    （MechanismPort.pick/drop），绝不直接发 POSE SET 或触碰串口桥；
    树莓派仍然不直接控制四个底盘电机，运动细节全部由 STM32 闭环负责
  - 航点严格串行：只有上一个航点到达终态后才提交下一个（由 Navigator
    单航点语义保证）；第一版使用预标定航点，路径规划交给人工标定
  - 绝不自动恢复：任何航点失败都进入终态（PAUSED 的显式 resume 除外，
    且 EMERGENCY_STOP/ABORTED/FINISHED 为不可恢复终态）。急停锁存后
    恢复必须重新自检（StartupStateMachine），本模块不会绕过该流程

失败映射（tick() 观察 navigator.last_result）：
    SAFETY_FAULT / CAN_ERROR / DISCONNECTED -> EMERGENCY_STOP
        （固件或桥已停车并锁存；操作员处理后再重新上电自检）
    CANCELLED                               -> PAUSED（pause() 请求）
                                               或 ABORTED（abort() 请求）
    其余（超时/意外停止等）                  -> ABORTED

机构接口约束：pick()/drop() 必须立即返回 bool（是否成功执行），
禁止阻塞实现——阻塞会卡死事件泵与心跳。真实机构应做成非阻塞启动 +
结果查询，或在独立执行器线程中完成。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional, Protocol

from .models import GotoReason, NavState, PoseGoal
from .speed_profile import SpeedProfileController

# 固件/链路级安全故障：电机已停且急停已锁存，任务只能进入 EMERGENCY_STOP
_SAFETY_REASONS = frozenset({
    GotoReason.SAFETY_FAULT,
    GotoReason.CAN_ERROR,
    GotoReason.DISCONNECTED,
})

_TERMINAL_NAV_STATES = (NavState.REACHED, NavState.CANCELLED, NavState.FAILED)


class MissionState(str, Enum):
    IDLE = "IDLE"
    WAIT_START = "WAIT_START"
    MOVE_TO_PICK = "MOVE_TO_PICK"
    ALIGN_PICK = "ALIGN_PICK"
    PICK = "PICK"
    MOVE_TO_DROP = "MOVE_TO_DROP"
    ALIGN_DROP = "ALIGN_DROP"
    DROP = "DROP"
    RETURN = "RETURN"
    FINISHED = "FINISHED"
    PAUSED = "PAUSED"
    RECOVERY = "RECOVERY"  # 显式 resume 后重驱被中断航点的过渡态
    ABORTED = "ABORTED"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class MechanismPort(Protocol):
    """机构接口（夹取/放置）。实现必须非阻塞、立即返回执行结果。"""

    def pick(self) -> bool: ...

    def drop(self) -> bool: ...


class NullMechanism:
    """台架/仿真用空机构：动作立即成功。"""

    def pick(self) -> bool:
        return True

    def drop(self) -> bool:
        return True


@dataclass(frozen=True)
class MissionConfig:
    """预标定任务航点（OPS 原始坐标；timeout_s 从 PoseStarted 起算）。"""

    pick_approach: PoseGoal
    pick_align: PoseGoal
    drop_approach: PoseGoal
    drop_align: PoseGoal
    return_home: PoseGoal


class MissionRejected(RuntimeError):
    """非法状态下的任务操作（未 start、终态后操作等）。"""


class Mission:
    """比赛任务状态机（非阻塞，tick() 驱动；事件由 Navigator 消费）。"""

    _LEG_GOALS = {
        MissionState.MOVE_TO_PICK: "pick_approach",
        MissionState.ALIGN_PICK: "pick_align",
        MissionState.MOVE_TO_DROP: "drop_approach",
        MissionState.ALIGN_DROP: "drop_align",
        MissionState.RETURN: "return_home",
    }
    _LEG_NEXT = {
        MissionState.MOVE_TO_PICK: MissionState.ALIGN_PICK,
        MissionState.ALIGN_PICK: MissionState.PICK,
        MissionState.MOVE_TO_DROP: MissionState.ALIGN_DROP,
        MissionState.ALIGN_DROP: MissionState.DROP,
        MissionState.RETURN: MissionState.FINISHED,
    }
    _ACTION_NEXT = {
        MissionState.PICK: MissionState.MOVE_TO_DROP,
        MissionState.DROP: MissionState.RETURN,
    }

    def __init__(
        self,
        navigator,
        config: MissionConfig,
        mechanism: Optional[MechanismPort] = None,
        *,
        profiles: Optional[SpeedProfileController] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.navigator = navigator
        self.config = config
        self.mechanism: MechanismPort = mechanism if mechanism is not None else NullMechanism()
        self.profiles = profiles  # 可选：每条移动腿发车前按 goal.profile 切换速度档
        self._clock = clock
        self.state = MissionState.IDLE
        self.last_reason: Optional[str] = None
        self._active_gid: Optional[int] = None
        self._active_leg: Optional[MissionState] = None
        self._pause_requested = False
        self._abort_requested = False
        self._paused_leg: Optional[MissionState] = None
        self._pending_resume_leg: Optional[MissionState] = None

    # ------------------------------------------------------------------
    # 对外接口
    # ------------------------------------------------------------------

    @property
    def active_goal_id(self) -> Optional[int]:
        return self._active_gid

    @property
    def is_terminal(self) -> bool:
        return self.state in (
            MissionState.FINISHED,
            MissionState.ABORTED,
            MissionState.EMERGENCY_STOP,
        )

    def start(self) -> None:
        """IDLE -> WAIT_START：等待比赛开始信号。"""
        if self.state != MissionState.IDLE:
            raise MissionRejected(f"start requires IDLE (state={self.state.value})")
        self._enter(MissionState.WAIT_START)

    def begin_run(self) -> None:
        """WAIT_START -> MOVE_TO_PICK：收到开始信号后提交第一个航点。"""
        if self.state != MissionState.WAIT_START:
            raise MissionRejected(
                f"begin_run requires WAIT_START (state={self.state.value})"
            )
        self._enter(MissionState.MOVE_TO_PICK)

    def pause(self) -> bool:
        """请求暂停：取消当前运动，确认后进入 PAUSED。

        返回 False 表示当前状态无法暂停（终态/同步机构动作中）。
        """
        if self.is_terminal:
            return False
        if self.state == MissionState.RECOVERY:
            # 尚未重新发车的恢复请求直接撤回
            self._pending_resume_leg = None
            self._enter(MissionState.PAUSED)
            return True
        if self.state == MissionState.WAIT_START:
            self._paused_leg = None
            self._enter(MissionState.PAUSED)
            return True
        if self.state in self._LEG_GOALS or self.state == MissionState.RECOVERY:
            leg = self._active_leg
            if leg is None or self._active_gid is None:
                return False
            self._pause_requested = True
            return self.navigator.cancel(self._active_gid)
        return False  # PICK/DROP 同步动作期间不支持暂停

    def resume(self) -> bool:
        """PAUSED -> RECOVERY：重驱被中断的航点，随后回到原航点状态。

        仅限显式人工恢复；没有可恢复航点时返回 False。
        """
        if self.state != MissionState.PAUSED or self._paused_leg is None:
            return False
        self._pending_resume_leg = self._paused_leg
        self._paused_leg = None
        self.last_reason = None
        self.state = MissionState.RECOVERY
        return True

    def abort(self) -> bool:
        """请求中止整场任务：取消当前运动并进入 ABORTED（终态）。"""
        if self.is_terminal:
            return False
        if self.state == MissionState.RECOVERY:
            self._pending_resume_leg = None
            self._finish(MissionState.ABORTED, "ABORT REQUESTED")
            return True
        self._abort_requested = True
        if self._active_gid is not None and self.state in self._LEG_GOALS:
            return self.navigator.cancel(self._active_gid)
        self._finish(MissionState.ABORTED, "ABORT REQUESTED")
        return True

    def tick(self) -> None:
        """推进任务：RECOVERY 补提交 + 观察导航器终态。主循环周期调用。"""
        if self.is_terminal:
            return

        if self.state == MissionState.RECOVERY and self._pending_resume_leg is not None:
            leg = self._pending_resume_leg
            self._pending_resume_leg = None
            # 导航器处于 CANCELLED 终态，允许接受新目标；拒绝则中止
            self._enter(leg)
            if self.state == MissionState.RECOVERY:
                self._finish(MissionState.ABORTED, "RESUME SUBMIT FAILED")
            return

        if self.state not in self._LEG_GOALS:
            return
        result = self.navigator.last_result
        if result is None or result.goal_id != self._active_gid:
            return
        if self.navigator.state not in _TERMINAL_NAV_STATES:
            return

        if result.success:
            self._enter(self._LEG_NEXT[self.state])
        elif result.reason == GotoReason.CANCELLED:
            if self._pause_requested:
                self._pause_requested = False
                self._paused_leg = self.state
                self._enter(MissionState.PAUSED)
            else:
                # abort() 或外部 STOP 导致的取消：按中止处理
                self._finish(MissionState.ABORTED, result.reason.value)
        elif result.reason in _SAFETY_REASONS:
            # 固件/桥已停车锁存；任务侧只记录终态，绝不自动恢复
            self._finish(MissionState.EMERGENCY_STOP, result.reason.value)
        else:
            self._finish(MissionState.ABORTED, result.reason.value)

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _enter(self, state: MissionState) -> None:
        """进入新状态：移动腿提交航点，机构状态同步执行动作。"""
        self.state = state
        if state in self._LEG_GOALS:
            goal_name = self._LEG_GOALS[state]
            goal: PoseGoal = getattr(self.config, goal_name)
            # 先选择速度档；二进制桥与目标原子提交，拒绝时绝不发车。
            if self.profiles is not None and goal.profile:
                try:
                    self.profiles.apply(goal.profile)
                except (KeyError, ValueError) as exc:
                    self._finish(MissionState.ABORTED, f"speed profile invalid: {exc}")
                    return
            try:
                self._active_gid = self.navigator.goto_pose(
                    goal.x_mm,
                    goal.y_mm,
                    goal.yaw_deg,
                    motion_timeout_s=goal.timeout_s,
                )
                self._active_leg = state
            except Exception as exc:  # 门禁关闭/急停锁存等：中止且不重试
                self._finish(MissionState.ABORTED, f"goto rejected: {exc}")
        elif state in self._ACTION_NEXT:
            action = self.mechanism.pick if state == MissionState.PICK else self.mechanism.drop
            ok = False
            try:
                ok = bool(action())
            except Exception:
                ok = False
            if ok:
                self._enter(self._ACTION_NEXT[state])
            else:
                self._finish(MissionState.ABORTED, f"{state.value} FAILED")

    def _finish(self, state: MissionState, reason: str) -> None:
        self.state = state
        self.last_reason = reason
        self._active_gid = None
        self._active_leg = None
        self._pause_requested = False
        self._abort_requested = False
        self._paused_leg = None
        self._pending_resume_leg = None
