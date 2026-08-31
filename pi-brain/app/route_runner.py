# -*- coding: utf-8 -*-
"""连续多航点路由调度器：Navigator 之上的严格串行航点序列（第四阶段延伸）。

设计要点：
  - 只通过 navigator.goto_pose()/cancel() 间接驱动底盘，绝不直接发 POSE SET
  - 严格串行：只有观察到上一个航点的终态结果（# POSE TARGET -> REACHED，
    或失败/取消）后，才提交下一个航点。串口上任意时刻最多只有一个
    未完结的 POSE SET，从源头排除旧目标迟到响应的歧义
  - 安全语义完全继承 Navigator/SerialBridge：急停锁存、门禁关闭、
    固件安全停车都会让 goto_pose 抛 NavigationRejected 或返回失败结果；
    本模块遇到任一航点失败即中止整个剩余路线（宁可失败不可信任），
    绝不自动重试、绝不自动恢复
  - 非阻塞事件驱动：主循环周期调用 tick()；本类不自己取事件，
    事件仍由 Navigator 经 EventRouter 消费
"""

from __future__ import annotations

import time
from enum import Enum
from typing import Callable, Optional, Sequence

from .models import GotoReason, NavState, PoseGoal
from .speed_profile import SpeedProfileController

_TERMINAL_NAV_STATES = (NavState.REACHED, NavState.CANCELLED, NavState.FAILED)


class RouteState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    ABORTED = "ABORTED"


class RouteRejected(RuntimeError):
    """start_route() 被拒绝：空路线 / 已在运行 / 导航器忙。"""


class RouteRunner:
    """严格串行的多航点调度器（非阻塞，tick() 驱动）。"""

    def __init__(
        self,
        navigator,
        *,
        profiles: Optional[SpeedProfileController] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.navigator = navigator
        self.profiles = profiles  # 可选：发车前按 goal.profile 切换速度档
        self._clock = clock
        self.state = RouteState.IDLE
        self.last_reason: Optional[str] = None
        self._goals: tuple[PoseGoal, ...] = ()
        self._index = 0
        self._active_gid: Optional[int] = None
        self._cancel_requested = False

    # ------------------------------------------------------------------
    # 对外接口
    # ------------------------------------------------------------------

    @property
    def index(self) -> int:
        """已成功完成的航点数。"""
        return self._index

    @property
    def total(self) -> int:
        return len(self._goals)

    @property
    def current_goal(self) -> Optional[PoseGoal]:
        """当前正在执行的航点；无活动航点时为 None。"""
        if self.state == RouteState.RUNNING and 0 <= self._index < len(self._goals):
            return self._goals[self._index]
        return None

    @property
    def is_finished(self) -> bool:
        return self.state in (RouteState.COMPLETED, RouteState.ABORTED)

    def start_route(self, goals: Sequence[PoseGoal]) -> int:
        """提交一条航点路线，立即串行开始执行；返回航点总数。

        空路线、已有路线在跑或导航器忙（上一目标未到终态）时抛 RouteRejected。
        第一个航点立即经 navigator.goto_pose() 提交；若被门禁/急停拒绝，
        路线进入 ABORTED（原因记录在 last_reason），不会重试。
        """
        if self.state == RouteState.RUNNING:
            raise RouteRejected("route already running")
        if not goals:
            raise RouteRejected("empty route")
        if self.navigator.state in (
            NavState.WAIT_POSE_START,
            NavState.MOVING,
            NavState.CANCELLING,
        ):
            raise RouteRejected(f"navigator busy (state={self.navigator.state.value})")

        self._goals = tuple(goals)
        self._index = 0
        self._active_gid = None
        self._cancel_requested = False
        self.last_reason = None
        self.state = RouteState.RUNNING
        self._submit_current()
        return len(self._goals)

    def cancel(self) -> bool:
        """取消整条路线：对当前航点发起取消；剩余航点不再执行。

        返回 False 表示没有可取消的活动航点（例如尚未提交或已完成）。
        """
        if self.state != RouteState.RUNNING:
            return False
        self._cancel_requested = True
        if self._active_gid is not None:
            return self.navigator.cancel(self._active_gid)
        return True

    def tick(self) -> None:
        """观察导航器终态并推进路线；由主循环周期调用。"""
        if self.state != RouteState.RUNNING:
            return
        result = self.navigator.last_result
        if result is None or result.goal_id != self._active_gid:
            return  # 尚未提交或仍是旧目标的迟到结果
        if self.navigator.state not in _TERMINAL_NAV_STATES:
            return  # 当前航点仍在运动中

        if result.success:
            self._index += 1
            if self._index >= len(self._goals):
                self._finish(RouteState.COMPLETED, GotoReason.REACHED.value)
            else:
                self._submit_current()
        elif self._cancel_requested and result.reason == GotoReason.CANCELLED:
            self._finish(RouteState.ABORTED, GotoReason.CANCELLED.value)
        else:
            self._finish(RouteState.ABORTED, result.reason.value)

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _submit_current(self) -> None:
        """提交当前航点；门禁关闭/急停锁存导致的拒绝会中止整条路线。"""
        goal = self._goals[self._index]
        # 先切速度档再发车：PID LIMIT 与 POSE SET 同优先级按序写出。
        # 档名非法或数值越界直接中止路线，绝不在错误限速下发车。
        if self.profiles is not None and goal.profile:
            try:
                self.profiles.apply(goal.profile)
            except (KeyError, ValueError) as exc:
                self._finish(RouteState.ABORTED, f"speed profile invalid: {exc}")
                return
        try:
            self._active_gid = self.navigator.goto_pose(
                goal.x_mm,
                goal.y_mm,
                goal.yaw_deg,
                motion_timeout_s=goal.timeout_s,
            )
        except Exception as exc:  # NavigationRejected 及任何意外都按失败处理
            self._finish(RouteState.ABORTED, f"goto rejected: {exc}")

    def _finish(self, state: RouteState, reason: str) -> None:
        self.state = state
        self.last_reason = reason
        self._active_gid = None
