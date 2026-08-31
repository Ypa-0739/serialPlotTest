# -*- coding: utf-8 -*-
"""单航点导航控制器：非阻塞 goto_pose + 严格状态机（第四阶段）。

状态机：
    IDLE
    -> WAIT_POSE_START      goto_pose() 被接受，POSE SET 已入队
    -> MOVING               收到 PoseStarted（固件确认闭环开始）
    -> REACHED              收到 PoseReached（到位且电机已停）
    WAIT_POSE_START / MOVING
    -> CANCELLING           cancel() 已发送 POSE STOP
    -> CANCELLED            收到 PoseStopped（或取消升级 STOP 后的确认）
    任意运动状态 -> FAILED  超时 / 安全故障 / 断线 / 协议异常

安全规则（与上位机方案一致）：
  - 门禁双检查：只有 StartupState.READY 后由主循环调用 set_ready(True)
    且 bridge 急停未锁存时，goto_pose() 才被接受；两者是 AND 关系
  - 单航点：非终态（WAIT_POSE_START/MOVING/CANCELLING）期间的新目标
    一律抛 NavigationRejected；终态（REACHED/CANCELLED/FAILED）时
    下一次 goto_pose() 自动清理旧结果并接受新目标
  - 顺序：必须先进 WAIT_POSE_START 再接受 PoseReached；未确认启动就
    收到到位视为协议异常（FAILED(UNEXPECTED_REACHED)），说明 Pi/固件
    状态不同步，宁可失败不可信任
  - 超时：
      WAIT_POSE_START 超时 -> emergency_stop() 并失败；不能假设固件未动，
                              因为可能只是 POSE START 回复丢失
      MOVING 总超时      -> emergency_stop() 并失败（锁存门禁）
      取消确认超时       -> 升级为 STOP（优先级 0）；再超时 ->
                             emergency_stop() 并失败
  - 全局故障（SafetyFault/CanError/UnknownError/SerialDisconnected）：
    立即失败并 emergency_stop()（断线时桥已自行锁存，此处幂等）
  - 绝不自动重发：FAILED 保持终态；重新自检（set_ready(True) 且急停
    解除）后由调用方显式提交新目标
  - 串行化：收到当前目标终态才允许下一个 POSE SET，从源头排除串口上
    旧目标的迟到响应歧义（现阶段无任务序号，靠 Pi 侧串行保证）

运动期间的心跳由 SerialBridgeThread 按 monotonic deadline 自行调度，
本类不干预 PING。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional

from .models import GotoReason, GotoResult, NavState, Pose, PoseGoal
from .protocol import (
    CanError,
    Event,
    PoseReached,
    PoseStarted,
    PoseStopped,
    PRIORITY_MOTION,
    PRIORITY_STOP,
    SafetyFault,
    StopAcknowledged,
    UnknownError,
    encode_pose_set,
)
from .serial_bridge import MotionCommandRejected, SerialDisconnected


class NavigationRejected(RuntimeError):
    """goto_pose() 被拒绝：门禁关闭 / 急停锁存 / 已有活动航点。"""


@dataclass
class _Request:
    goal_id: int
    target: Pose
    started_at: float
    start_deadline: float
    motion_timeout_s: float
    motion_deadline: Optional[float] = None  # 收到 PoseStarted 后设定
    cancel_deadline: Optional[float] = None
    cancel_escalated: bool = False  # 取消确认超时后是否已升级为 STOP


class Navigator:
    """单航点导航控制器（非阻塞，事件驱动）。

    主循环用法：
        gid = navigator.goto_pose(x_mm, y_mm, yaw_deg)
        while True:
            event = bridge.get_event(timeout=0.05)
            navigator.handle_event(event)
            navigator.tick()
            if navigator.last_result:  # 终态结果（REACHED/CANCELLED/FAILED）
                ...
    """

    def __init__(
        self,
        bridge,
        *,
        clock: Callable[[], float] = time.monotonic,
        start_timeout_s: float = 1.0,
        motion_timeout_s: float = 30.0,
        cancel_confirm_timeout_s: float = 1.5,
    ) -> None:
        self.bridge = bridge
        self._clock = clock
        self._start_timeout = start_timeout_s
        self._motion_timeout = motion_timeout_s
        self._cancel_confirm_timeout = cancel_confirm_timeout_s
        self.state = NavState.IDLE
        self.last_result: Optional[GotoResult] = None
        self._gate_open = False
        self._req: Optional[_Request] = None
        self._goal_seq = 0

    # ------------------------------------------------------------------
    # 对外接口
    # ------------------------------------------------------------------

    @property
    def active_goal_id(self) -> Optional[int]:
        return self._req.goal_id if self._req is not None else None

    @property
    def active_target(self) -> Optional[Pose]:
        """当前活动目标的位姿（无活动目标时为 None）。供日志等只读消费者使用。"""
        return self._req.target if self._req is not None else None

    def set_ready(self, ready: bool) -> None:
        """由主循环在 StartupState 进入/离开 READY 时调用（门禁开关）。"""
        self._gate_open = ready

    def goto_pose(
        self,
        x_mm: float,
        y_mm: float,
        yaw_deg: float,
        *,
        start_timeout_s: Optional[float] = None,
        motion_timeout_s: Optional[float] = None,
    ) -> int:
        """提交单航点目标，返回 goal_id（Pi 内部生成，仅本地任务管理）。

        非阻塞：只入队 POSE SET 并进入 WAIT_POSE_START，立即返回。
        被拒绝（未 READY/急停锁存/已有活动航点）时抛 NavigationRejected。
        """
        if not self._gate_open or self.bridge.is_emergency_stopped:
            raise NavigationRejected(
                "navigation gate closed: not READY or emergency stop latched"
            )
        if self.state in (NavState.WAIT_POSE_START, NavState.MOVING, NavState.CANCELLING):
            raise NavigationRejected(
                f"navigation in progress (state={self.state.value})"
            )
        # 终态（REACHED/CANCELLED/FAILED）：当前目标终态已消费，允许接受新目标
        self._goal_seq += 1
        goal_id = self._goal_seq
        target = Pose(x_mm=x_mm, y_mm=y_mm, yaw_deg=yaw_deg)
        now = self._clock()
        effective_start_timeout = (
            self._start_timeout if start_timeout_s is None else start_timeout_s
        )
        effective_motion_timeout = (
            self._motion_timeout if motion_timeout_s is None else motion_timeout_s
        )
        if effective_start_timeout <= 0.0 or effective_motion_timeout <= 0.0:
            raise ValueError("navigation timeouts must be greater than zero")
        self._req = _Request(
            goal_id=goal_id,
            target=target,
            started_at=now,
            start_deadline=now + effective_start_timeout,
            motion_timeout_s=effective_motion_timeout,
        )
        self.state = NavState.WAIT_POSE_START
        try:
            self.bridge.send(
                encode_pose_set(x_mm, y_mm, yaw_deg).decode("ascii").strip(),
                priority=PRIORITY_MOTION,
            )
        except MotionCommandRejected as exc:
            # 竞态：检查后急停被锁存。回退到 IDLE，绝不重试。
            self._req = None
            self.state = NavState.IDLE
            raise NavigationRejected(f"motion rejected by bridge: {exc}") from exc
        return goal_id

    def goto_goal(self, goal: PoseGoal, **kwargs: float) -> int:
        """提交预标定航点；运动超时默认取 PoseGoal.timeout_s。"""
        kwargs.setdefault("motion_timeout_s", goal.timeout_s)
        return self.goto_pose(goal.x_mm, goal.y_mm, goal.yaw_deg, **kwargs)

    def cancel(self, goal_id: int) -> bool:
        """请求取消活动航点：发送 POSE STOP，等待 PoseStopped 确认。

        返回 False 表示没有匹配的活动目标（或已处于取消流程）。
        确认超时由 tick() 升级为 STOP，再超时则 emergency_stop()。
        """
        if self._req is None or self._req.goal_id != goal_id:
            return False
        if self.state not in (NavState.WAIT_POSE_START, NavState.MOVING):
            return False
        self.state = NavState.CANCELLING
        self._req.cancel_deadline = self._clock() + self._cancel_confirm_timeout
        # POSE STOP 不是运动命令，急停锁存期间也可发送
        self.bridge.send("POSE STOP", priority=PRIORITY_STOP)
        return True

    def reset(self) -> None:
        """显式清理终态回 IDLE（last_result 保留，供调用方消费）。"""
        if self.state not in (NavState.WAIT_POSE_START, NavState.MOVING, NavState.CANCELLING):
            self._req = None
            self.state = NavState.IDLE

    # ------------------------------------------------------------------
    # 事件处理（主循环统一取事件后调用，与其它消费者共用同一事件）
    # ------------------------------------------------------------------

    def handle_event(self, event: Event) -> None:
        # 全局故障优先：无论什么状态立即失败并急停
        if isinstance(event, SafetyFault):
            self._abort(GotoReason.SAFETY_FAULT, emergency=True)
            return
        if isinstance(event, CanError):
            self._abort(GotoReason.CAN_ERROR, emergency=True)
            return
        if isinstance(event, UnknownError):
            self._abort(GotoReason.UNKNOWN_ERROR, emergency=True)
            return
        if isinstance(event, SerialDisconnected):
            # 断线时桥线程已自行锁存急停，此处 emergency_stop 幂等
            self._abort(GotoReason.DISCONNECTED, emergency=True)
            return

        if self._req is None:
            return  # 无活动目标：忽略（含终态后的迟到响应）

        if self.state == NavState.WAIT_POSE_START:
            if isinstance(event, PoseStarted):
                # 固件确认闭环开始，计时运动总超时
                self.state = NavState.MOVING
                self._req.motion_deadline = (
                    self._clock() + self._req.motion_timeout_s
                )
            elif isinstance(event, PoseReached):
                # 协议异常：未确认启动就收到到位，Pi/固件状态不同步
                self.bridge.emergency_stop()
                self._finish(
                    GotoReason.UNEXPECTED_REACHED,
                    final=Pose(event.x, event.y, event.yaw),
                )
            elif isinstance(event, (PoseStopped, StopAcknowledged)):
                # 固件侧意外停止（如外部 STOP），未确认启动
                self._finish(GotoReason.UNEXPECTED_STOP)
            return

        if self.state == NavState.MOVING:
            if isinstance(event, PoseReached):
                self._finish(
                    GotoReason.REACHED,
                    final=Pose(event.x, event.y, event.yaw),
                    pos_err=event.error_mm,
                    yaw_err=event.error_yaw,
                )
            elif isinstance(event, (PoseStopped, StopAcknowledged)):
                # 固件侧意外停止（如外部 STOP）
                self._finish(GotoReason.UNEXPECTED_STOP)
            return

        if self.state == NavState.CANCELLING:
            if isinstance(event, (PoseStopped, StopAcknowledged)):
                # 取消确认（POSE STOP 直接确认，或升级 STOP 后 # STOP MODE=...）
                self._finish(GotoReason.CANCELLED)
            elif isinstance(event, PoseReached):
                # 取消竞争中固件先到位并停住：目标已达，视为成功
                self._finish(
                    GotoReason.REACHED,
                    final=Pose(event.x, event.y, event.yaw),
                    pos_err=event.error_mm,
                    yaw_err=event.error_yaw,
                )
            return

    def tick(self) -> None:
        """检查各阶段超时；不阻塞主循环。"""
        if self._req is None:
            return
        now = self._clock()

        if self.state == NavState.WAIT_POSE_START:
            if now >= self._req.start_deadline:
                # POSE START 可能只是回复丢失，STM32实际已开始运动；必须锁存急停。
                self.bridge.emergency_stop()
                self._finish(GotoReason.START_TIMEOUT)
            return

        if self.state == NavState.MOVING:
            if now >= self._req.motion_deadline:
                # 运动总超时：锁存急停并失败（所有运动命令被拒）
                self.bridge.emergency_stop()
                self._finish(GotoReason.MOTION_TIMEOUT)
            return

        if self.state == NavState.CANCELLING:
            if self._req.cancel_deadline is not None and now >= self._req.cancel_deadline:
                if not self._req.cancel_escalated:
                    # 取消确认超时：升级为最高优先级 STOP
                    self._req.cancel_escalated = True
                    self.bridge.send("STOP", priority=PRIORITY_STOP)
                    self._req.cancel_deadline = now + self._cancel_confirm_timeout
                else:
                    # 升级 STOP 后仍无确认：锁存急停并失败
                    self.bridge.emergency_stop()
                    self._finish(GotoReason.CANCEL_UNCONFIRMED)
            return

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _abort(self, reason: GotoReason, *, emergency: bool) -> None:
        """全局故障路径：急停 + 结束活动目标（无活动目标时仅急停）。"""
        if emergency:
            self.bridge.emergency_stop()
        if self._req is not None:
            self._finish(reason)

    def _finish(
        self,
        reason: GotoReason,
        *,
        final: Optional[Pose] = None,
        pos_err: Optional[float] = None,
        yaw_err: Optional[float] = None,
    ) -> None:
        """结算当前目标：写入 last_result 并进入终态。"""
        req = self._req
        elapsed = self._clock() - req.started_at
        self.last_result = GotoResult(
            goal_id=req.goal_id,
            success=reason == GotoReason.REACHED,
            reason=reason,
            target=req.target,
            final_pose=final or req.target,
            position_error_mm=pos_err,
            yaw_error_deg=yaw_err,
            elapsed_s=elapsed,
        )
        self._req = None
        if reason == GotoReason.REACHED:
            self.state = NavState.REACHED
        elif reason == GotoReason.CANCELLED:
            self.state = NavState.CANCELLED
        else:
            self.state = NavState.FAILED
