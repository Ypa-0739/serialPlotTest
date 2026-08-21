# -*- coding: utf-8 -*-
"""响应式固件仿真（演示/测试用，不连接真实 STM32）。

FakeFirmware 模拟 STM32 的串口行为：收到一行命令后立即生成对应的
固件响应行。用于：
  - python -m app.main --demo 在 PC 上演示完整启动自检流程
  - tests/test_integration.py 端到端验证 状态机 + 串口桥 的协作

响应格式与 Core/Src/main.c 的实际 printf 输出一致（含 %.7f/%.8f 精度），
保证状态机的 isclose 校验与真机行为完全一致。
"""

from __future__ import annotations

import time
from typing import Optional


class FakeFirmware:
    """模拟固件：write() 时解析命令并追加响应；readline() 弹出响应。

    POSE SET 仿真（第四阶段导航测试用）：
      - 收到 POSE SET 后按 pose_start_delay_s 延迟回 # POSE START，
        再按 pose_reached_delay_s 延迟回 # POSE TARGET（可注入误差）
      - pose_start_ok / pose_reached_ok 分别关闭启动确认和到位通知，
        用于模拟“未确认启动”和“运动超时（不通知到位）”
      - pose_safety_after_s 在运动开始后模拟 OPS 丢失（# POSE STOP SAFETY）
      - 新 POSE SET 到达时丢弃旧目标未到期的响应（固件侧目标覆盖）
      - POSE STOP 清除未到期响应并回 # POSE STOP（可延迟模拟确认丢失）
    """

    def __init__(
        self,
        *,
        ops_link: str = "OK",
        ops_frames: int = 10,
        can_state: int = 2,
        can_error: int = 0,
        pid_x=(0.0033, 0.0, 0.0),
        pid_y=(0.0033, 0.0, 0.0),
        pid_yaw=(0.02, 0.0, 0.0),
        mode: str = "WORK",
        host_proto: int = 2,
        # ---- POSE 仿真参数 ----
        pose_start_delay_s: float = 0.0,  # POSE SET -> # POSE START 延迟
        pose_reached_delay_s: float = 0.3,  # # POSE START -> # POSE TARGET 延迟
        pose_start_ok: bool = True,
        pose_reached_ok: bool = True,
        pose_error_mm: float = 2.5,  # 注入最终位置误差
        pose_error_yaw: float = 0.4,  # 注入最终航向误差
        pose_safety_after_s: Optional[float] = None,  # 运动开始后 OPS 丢失
        pose_stopped_delay_s: float = 0.0,  # POSE STOP 确认延迟
        ack_stop: bool = True,  # STOP 命令是否回 # STOP MODE=...
    ) -> None:
        self.ops_link = ops_link
        self.ops_frames = ops_frames
        self.can_state = can_state
        self.can_error = can_error
        self.pids = {"X": pid_x, "Y": pid_y, "YAW": pid_yaw}
        self.mode = mode
        self.host_proto = host_proto
        self.pose_start_delay_s = pose_start_delay_s
        self.pose_reached_delay_s = pose_reached_delay_s
        self.pose_start_ok = pose_start_ok
        self.pose_reached_ok = pose_reached_ok
        self.pose_error_mm = pose_error_mm
        self.pose_error_yaw = pose_error_yaw
        self.pose_safety_after_s = pose_safety_after_s
        self.pose_stopped_delay_s = pose_stopped_delay_s
        self.ack_stop = ack_stop
        self._responses: list[str] = []  # 即时响应 FIFO
        self._pose_pending: list[tuple[float, str]] = []  # (due, text) 按到期排序
        self.is_open = True
        self.closed = False
        self.timeout = 0.05
        self.write_timeout = 0.1
        self.received: list[str] = []  # 收到的命令记录（按序）
        self._fail = False

    def fail(self) -> None:
        """模拟物理断线。"""
        self._fail = True

    # ---- 串口接口 ----

    def readline(self) -> bytes:
        if not self.is_open:
            return b""
        if self._fail:
            raise OSError("simulated link failure")
        # 到期响应优先（POSE 延迟仿真）：取最早到期的项
        now = time.monotonic()
        for index, (due, _) in enumerate(self._pose_pending):
            if due <= now:
                return self._pose_pending.pop(index)[1].encode("utf-8")
        if self._responses:
            return self._responses.pop(0).encode("utf-8")
        return b""

    def write(self, data) -> int:
        if not self.is_open:
            raise OSError("write on closed serial")
        if self._fail:
            raise OSError("simulated write failure")
        text = bytes(data).decode("utf-8", errors="replace").strip()
        if text:
            self.received.append(text)
            self._respond(text)
        return len(data)

    def close(self) -> None:
        self.is_open = False
        self.closed = True

    # ---- 固件响应逻辑 ----

    def _respond(self, command: str) -> None:
        if command == "PING":
            self._responses.append("# PONG")
        elif command == "STOP":
            if self.ack_stop:
                self._responses.append(f"# STOP MODE={self.mode}")
        elif command.startswith("POSE SET "):
            self._pose_set(command)
        elif command == "POSE STOP":
            # 固件停止：丢弃未到期的到达/安全响应，确认可延迟（模拟确认丢失）
            self._pose_pending.clear()
            if self.pose_stopped_delay_s > 0:
                self._pose_pending.append(
                    (time.monotonic() + self.pose_stopped_delay_s, "# POSE STOP")
                )
            else:
                self._responses.append("# POSE STOP")
        elif command == "MODE WORK":
            self.mode = "WORK"
            self._responses.append("# MODE WORK PLOT=0 CHANGED=1")
        elif command == "MODE TUNE":
            self.mode = "TUNE"
            self._responses.append("# MODE TUNE PLOT=0 CHANGED=1")
        elif command == "STATUS":
            axis, pid = list(self.pids.items())[0]
            self._responses.append(
                f"# STATUS MODE={self.mode} HOST_PROTO={self.host_proto} "
                f"AXIS={axis} P={pid[0]:.7f} I={pid[1]:.8f} D={pid[2]:.7f} "
                f"MAX_OUT=0.200 STATE=0 PLOT=0 MOTOR_PROTO=EMM "
                f"OPS_FRAMES={self.ops_frames} UART_TX_OK=0 UART_TX_ERR=0"
            )
        elif command == "OPS STATUS":
            self._responses.append(
                f"# OPS LINK={self.ops_link} X=0.00 Y=0.00 YAW=0.00 "
                f"CENTER_X=0.00 CENTER_Y=0.00 OFFSET_X=0.00 OFFSET_Y=25.00 "
                f"BYTES=100 HEADERS=10 FRAMES={self.ops_frames} INVALID=0 "
                f"FORMAT_ERR=0 UART_ERR=0 BYTE_AGE=5 FRAME_AGE=5 LAST=0xAA"
            )
        elif command == "CAN STATUS":
            self._responses.append(
                f"# CAN STATE={self.can_state} ERROR=0x{self.can_error:08X} "
                f"FREE=3 TX_OK=100 TX_ERR=0 RX=50 LAST=0"
            )
        elif command.startswith("PID SET "):
            parts = command.split()  # PID SET <AXIS> <p> <i> <d>
            if len(parts) == 6 and parts[2] in self.pids:
                axis = parts[2]
                pid = tuple(float(v) for v in parts[3:6])
                self.pids[axis] = pid
                self._responses.append(
                    f"# PID LOADED AXIS={axis} P={pid[0]:.7f} "
                    f"I={pid[1]:.8f} D={pid[2]:.7f}"
                )
        elif command.startswith("PID LIMIT "):
            parts = command.split()  # PID LIMIT <AXIS> <output>
            if len(parts) == 4 and parts[2] in self.pids:
                self._responses.append(
                    f"# PID LIMIT AXIS={parts[2]} OUTPUT={parts[3]} UNIT=MPS"
                )
        elif command == "PID STATUS ALL":
            x, y, yaw = self.pids["X"], self.pids["Y"], self.pids["YAW"]
            self._responses.append(
                f"# PID ALL X={x[0]:.7f},{x[1]:.8f},{x[2]:.7f} "
                f"Y={y[0]:.7f},{y[1]:.8f},{y[2]:.7f} "
                f"YAW={yaw[0]:.7f},{yaw[1]:.8f},{yaw[2]:.7f}"
            )
        # 未识别的命令不响应（模拟固件对未知命令的静默/报错）
        elif command.startswith(("POSE", "MOVE", "TURN", "MOTOR", "SET")):
            self._responses.append(f"# ERROR UNKNOWN COMMAND")
        else:
            self._responses.append("# ERROR UNKNOWN COMMAND")

    def _pose_set(self, command: str) -> None:
        """POSE SET：安排延迟的 # POSE START / # POSE TARGET / 安全停车。"""
        parts = command.split()
        if len(parts) != 5:
            self._responses.append("# ERROR POSE VALUE")
            return
        try:
            x, y, yaw = float(parts[2]), float(parts[3]), float(parts[4])
        except ValueError:
            self._responses.append("# ERROR POSE VALUE")
            return
        # 新目标覆盖旧目标：丢弃旧目标未到期的响应（固件侧覆盖语义）
        self._pose_pending.clear()
        start_at = time.monotonic() + self.pose_start_delay_s
        if self.pose_start_ok:
            self._pose_pending.append(
                (
                    start_at,
                    f"# POSE START X={x:.2f} Y={y:.2f} YAW={yaw:.2f} "
                    f"CENTER_X={x:.2f} CENTER_Y={y:.2f} TOL_MM=5.00 TOL_YAW=1.00",
                )
            )
        if self.pose_reached_ok:
            self._pose_pending.append(
                (
                    start_at + self.pose_reached_delay_s,
                    f"# POSE TARGET X={x:.2f} Y={y:.2f} YAW={yaw:.2f} "
                    f"CENTER_X={x:.2f} CENTER_Y={y:.2f} "
                    f"ERROR_MM={self.pose_error_mm:.2f} ERROR_YAW={self.pose_error_yaw:.2f}",
                )
            )
        if self.pose_safety_after_s is not None and self.pose_start_ok:
            self._pose_pending.append(
                (
                    start_at + self.pose_safety_after_s,
                    "# POSE STOP SAFETY",
                )
            )
