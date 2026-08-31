# -*- coding: utf-8 -*-
"""导航数据模型：位姿、航点、导航结果（第四阶段）。

与固件协议保持一致：位姿一律使用 OPS 原始坐标（mm / mm / deg），
固件内部自行换算到车体中心（OFFSET_Y=25mm 等）。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


@dataclass(frozen=True)
class Pose:
    """位姿（OPS 原始坐标：mm / mm / deg）。"""

    x_mm: float
    y_mm: float
    yaw_deg: float


@dataclass(frozen=True)
class PoseGoal:
    """预标定航点（第五阶段任务路由用）。

    route = [
        PoseGoal("pick_approach", 800, 300, 0, 10, profile="cruise"),
        PoseGoal("pick_align", 950, 300, 0, 5),
    ]

    profile 指定本航点使用的速度档（app/speed_profile.py 中的档名）：
      - "cruise"：巡航档，外圈宽车道长距离转移
      - "precise"：定位停车档（默认，安全兜底），窄巷/对位/启停区停车
    """

    name: str
    x_mm: float
    y_mm: float
    yaw_deg: float
    timeout_s: float = 30.0  # 初期联调留足余量；从收到 PoseStarted 起算
    profile: str = "precise"  # 缺省低速档：宁可慢不可越线


class NavState(str, Enum):
    """导航状态机。终态（REACHED/CANCELLED/FAILED）可被下一次 goto_pose 清理。"""

    IDLE = "IDLE"
    WAIT_POSE_START = "WAIT_POSE_START"  # POSE SET 已发，等固件确认
    MOVING = "MOVING"  # 收到 PoseStarted，闭环进行中
    CANCELLING = "CANCELLING"  # 已发 POSE STOP，等确认（或升级 STOP）
    REACHED = "REACHED"
    CANCELLED = "CANCELLED"
    FAILED = "FAILED"


class GotoReason(str, Enum):
    """单航点导航的结束原因。success 仅 REACHED 为 True。"""

    REACHED = "REACHED"
    START_TIMEOUT = "START_TIMEOUT"  # 等待PoseStarted超时，已锁存急停
    MOTION_TIMEOUT = "MOTION_TIMEOUT"  # 运动总超时，已 emergency_stop
    CANCELLED = "CANCELLED"
    CANCEL_UNCONFIRMED = "CANCEL_UNCONFIRMED"  # 取消确认超时，升级 STOP 仍无确认
    SAFETY_FAULT = "SAFETY_FAULT"  # 固件安全停车（OPS LOST 等）
    CAN_ERROR = "CAN_ERROR"
    UNKNOWN_ERROR = "UNKNOWN_ERROR"
    DISCONNECTED = "DISCONNECTED"  # 串口断开
    UNEXPECTED_REACHED = "UNEXPECTED_REACHED"  # 未确认启动就收到到位（协议异常）
    UNEXPECTED_STOP = "UNEXPECTED_STOP"  # 固件侧意外停止确认（协议异常）


@dataclass(frozen=True)
class GotoResult:
    """单航点导航结果。

    REACHED 时 position_error_mm / yaw_error_deg 为固件回报的到位误差；
    其余原因这两个字段为 None。final_pose 为固件回报的最终位姿
    （失败时为最后指令目标）。
    """

    goal_id: int
    success: bool
    reason: GotoReason
    target: Pose
    final_pose: Pose
    position_error_mm: Optional[float] = None
    yaw_error_deg: Optional[float] = None
    elapsed_s: float = 0.0
