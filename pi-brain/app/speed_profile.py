# -*- coding: utf-8 -*-
"""工创赛物流搬运双速度档：巡航(cruise) / 定位停车(precise)。

依据（2025 工创赛"智能物流搬运"命题与评分）：
  - 车道约 400mm 宽、车体投影 300×300mm，"投影越过车道边界比赛结束"
    → 窄巷与对位必须低速，单侧余量仅 ±50mm
  - 每轮 3 分钟，两批 6 个物料 + 读二维码 + 回启停区
    → 长距离转移必须巡航速度，否则时间不够
  - 色环放置 1 环 15 分要求毫米级偏差、启停区 300×300 对 300×300 的车
    几乎零余量 → 定位停车档低速收敛

实现方式（不改固件协议）：
  - 复用固件现有 `PID LIMIT X|Y|YAW` 运行时配置命令（硬边界：
    线速度 0.02~0.30 m/s、角速度 0.02~0.80 rad/s，与 main.c 一致）
  - PID LIMIT 是非运动配置命令，但必须与随后的 POSE SET 同用
    PRIORITY_MOTION：唯一写者队列同优先级按序写出，保证
    "先切档、后发车"，绝不允许 POSE SET 先于限速命令到达固件
  - mark_unknown()：断线重连或重新自检后调用；自检流程会重载默认限速，
    此时本地"当前档"缓存作废，下一次发车前必须重发
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .protocol import PRIORITY_MOTION

# 与固件 Core/Src/main.c 的 PID LIMIT 校验边界保持一致
FIRMWARE_LINEAR_MIN_MPS = 0.02
FIRMWARE_LINEAR_MAX_MPS = 0.30
FIRMWARE_YAW_MIN_RADPS = 0.02
FIRMWARE_YAW_MAX_RADPS = 0.80


@dataclass(frozen=True)
class SpeedProfile:
    """一个速度档 = 平移速度上限 + 航向角速度上限（经固件 PID LIMIT 生效）。"""

    name: str
    linear_mps: float
    yaw_radps: float


# 巡航档：外圈 550mm 车道长距离转移；满足 3 分钟时间预算。
# 实车验证稳定后可提升到固件硬上限 0.30 m/s。
SPEED_PROFILE_CRUISE = SpeedProfile("cruise", 0.25, 0.25)
# 定位停车档：400mm 中央窄巷、二维码/色环对位、启停区最终停车。
SPEED_PROFILE_PRECISE = SpeedProfile("precise", 0.12, 0.12)

DEFAULT_PROFILES = {
    SPEED_PROFILE_CRUISE.name: SPEED_PROFILE_CRUISE,
    SPEED_PROFILE_PRECISE.name: SPEED_PROFILE_PRECISE,
}


class SpeedProfileController:
    """速度档控制器：发车前切换 PID LIMIT，缓存当前档避免重复发送。"""

    def __init__(self, bridge, profiles: Optional[dict] = None) -> None:
        self._bridge = bridge
        self._profiles: dict = dict(DEFAULT_PROFILES) if profiles is None else dict(profiles)
        self._current: Optional[str] = None  # None = 未知（必须重发）

    @property
    def current(self) -> Optional[str]:
        """当前缓存档名；None 表示未知（重连/自检后）。"""
        return self._current

    def mark_unknown(self) -> None:
        """作废本地缓存（断线重连、重新自检重载默认限速后必须调用）。"""
        self._current = None

    def profile(self, name: str) -> SpeedProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise KeyError(
                f"unknown speed profile: {name!r}; known: {sorted(self._profiles)}"
            ) from None

    def apply(self, name: str) -> bool:
        """切换到指定速度档；与缓存一致时跳过。返回是否实际发送命令。

        越界或未知档名直接抛异常，绝不发送半套配置。
        """
        profile = self.profile(name)
        self._validate(profile)
        if self._current == name:
            return False
        # 同优先级 FIFO：三条 LIMIT 先于随后的 POSE SET 写出（见模块 docstring）
        self._bridge.send(
            f"PID LIMIT X {profile.linear_mps:.3f}", priority=PRIORITY_MOTION
        )
        self._bridge.send(
            f"PID LIMIT Y {profile.linear_mps:.3f}", priority=PRIORITY_MOTION
        )
        self._bridge.send(
            f"PID LIMIT YAW {profile.yaw_radps:.3f}", priority=PRIORITY_MOTION
        )
        self._current = name
        return True

    @staticmethod
    def _validate(profile: SpeedProfile) -> None:
        if not (
            FIRMWARE_LINEAR_MIN_MPS
            <= profile.linear_mps
            <= FIRMWARE_LINEAR_MAX_MPS
        ):
            raise ValueError(
                f"profile {profile.name}: linear {profile.linear_mps} m/s outside "
                f"firmware bounds [{FIRMWARE_LINEAR_MIN_MPS}, {FIRMWARE_LINEAR_MAX_MPS}]"
            )
        if not (FIRMWARE_YAW_MIN_RADPS <= profile.yaw_radps <= FIRMWARE_YAW_MAX_RADPS):
            raise ValueError(
                f"profile {profile.name}: yaw {profile.yaw_radps} rad/s outside "
                f"firmware bounds [{FIRMWARE_YAW_MIN_RADPS}, {FIRMWARE_YAW_MAX_RADPS}]"
            )
