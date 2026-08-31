# -*- coding: utf-8 -*-
"""串口桥线程：超时驱动，单线程承担 RX / TX / PING / 重连 四职责。

设计要点（与上位机方案一致）：
  - 唯一写者：只有本线程调用 serial.write()；外部只能通过 send() /
    emergency_stop() 向 TX 优先级队列投递命令，绝不直接碰串口。
  - PING 不进队列：线程按 time.monotonic() deadline 在每轮循环中先于
    TX 队列发送，普通命令再多也不会饿死心跳。
  - 短超时读取：readline() 最多阻塞 read_timeout 秒，循环定期醒来；
    所有周期动作都用单调时钟 deadline 调度，不依赖“50ms=20Hz”的假设。
  - 断线自愈：读/写异常 -> 关闭串口 -> 发布 SerialDisconnected -> 退避 ->
    重新调用 serial_factory() 打开 -> 发布 SerialConnected。
  - 重连只发布连接事件，绝不自动重发断线前的旧命令；恢复运动由 Brain
    流程（STOP -> SELF_CHECK -> PID 重载 -> READY）负责。
"""

from __future__ import annotations

import itertools
import queue
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

from .protocol import (
    FaultReason,
    Command,
    Event,
    RawMessage,
    PoseTelemetry,
    PoseReached,
    PoseStarted,
    PoseStopped,
    PRIORITY_STOP,
    encode_ping,
    is_motion_command,
    parse_line,
    SafetyFault,
    WheelTelemetry,
)
from .binary_protocol import (
    Command as BinaryCommand,
    EventCode as BinaryEventCode,
    FrameDecoder,
    MessageType,
    MotionFault,
    PoseGoal as BinaryPoseGoal,
    Response as BinaryResponse,
    ResponseStatus,
    cancel_goal_data,
    command_frame,
    decode_pose_event,
    speed_limits_data,
)

# 串口对象的最小接口（pyserial.Serial 或测试替身均可）
SerialLike = object


class LinkState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    BACKOFF = "BACKOFF"
    STOPPING = "STOPPING"


@dataclass(frozen=True)
class SerialConnected(Event):
    """重连成功。Brain 收到后进入 SELF_CHECK，不恢复旧运动。"""

    pass


@dataclass(frozen=True)
class SerialDisconnected(Event):
    """链路断开。Brain 应进入安全保持态。"""

    pass


class MotionCommandRejected(RuntimeError):
    """急停锁存期间拒绝了可能启动运动的命令。"""


@dataclass(frozen=True)
class BinaryCommandRejected(Event):
    """STM32 拒绝了一条二进制命令。"""

    command: int = 0
    status: int = 0
    goal_id: int | None = None


@dataclass(frozen=True)
class _BinaryTx:
    command: BinaryCommand
    data: bytes = b""
    goal_id: int | None = None


class SerialBridgeThread(threading.Thread):
    """单线程串口桥：RX 解析 + PING 心跳 + TX 队列 + 断线重连。"""

    def __init__(
        self,
        serial_factory: Callable[[], SerialLike],
        *,
        ping_interval: float = 0.4,
        heartbeat_enabled: bool = True,
        read_timeout: float = 0.05,
        write_timeout: float = 0.1,
        backoff_seconds: float = 1.0,
        binary_recovery_seconds: float = 1.6,
        tx_drain_limit: int = 8,
        telemetry_queue_size: int = 256,
        name: str = "serial-bridge",
    ) -> None:
        super().__init__(daemon=True, name=name)
        self._factory = serial_factory
        self._ping_interval = ping_interval
        self._heartbeat_enabled = heartbeat_enabled
        self._read_timeout = read_timeout
        self._write_timeout = write_timeout
        self._backoff = backoff_seconds
        self._binary_recovery_seconds = binary_recovery_seconds
        self._tx_drain_limit = tx_drain_limit
        self._tx: queue.PriorityQueue = queue.PriorityQueue()
        self._events: queue.Queue = queue.Queue()
        self._telemetry: queue.Queue = queue.Queue(maxsize=max(1, telemetry_queue_size))
        self._serial: Optional[SerialLike] = None
        self._running = False
        self._link_state = LinkState.DISCONNECTED
        self._next_ping = 0.0
        self._seq = itertools.count()
        self._safety_lock = threading.Lock()
        self._emergency_latched = False
        self._binary_mode = False
        self._binary_decoder = FrameDecoder()
        self._binary_sequence = 0
        self._binary_pending: dict[int, _BinaryTx] = {}

    # ------------------------------------------------------------------
    # 对外接口（外部只能通过这些方法，不直接触碰串口）
    # ------------------------------------------------------------------

    @property
    def link_state(self) -> LinkState:
        return self._link_state

    @property
    def is_connected(self) -> bool:
        return self._link_state == LinkState.CONNECTED

    @property
    def is_emergency_stopped(self) -> bool:
        """急停是否已锁存；锁存期间运动命令会被拒绝。"""
        with self._safety_lock:
            return self._emergency_latched

    @property
    def is_binary_mode(self) -> bool:
        with self._safety_lock:
            return self._binary_mode

    def enable_binary_mode(self) -> None:
        """在收到 ``# HOST BINARY READY`` 后原子切换串口编解码模式。"""
        with self._safety_lock:
            self._binary_decoder = FrameDecoder()
            self._binary_pending.clear()
            self._binary_mode = True

    def send_pose_goal(
        self,
        goal_id: int,
        x_mm: float,
        y_mm: float,
        yaw_deg: float,
        timeout_s: float,
        *,
        priority: int,
    ) -> None:
        goal = BinaryPoseGoal(
            goal_id=goal_id,
            x_mm=round(x_mm),
            y_mm=round(y_mm),
            yaw_mrad=round(yaw_deg * 17.45329252),
            timeout_ms=round(timeout_s * 1000.0),
        )
        tx = _BinaryTx(BinaryCommand.SET_POSE_GOAL, goal.command_payload()[1:], goal_id)
        with self._safety_lock:
            if not self._binary_mode:
                raise MotionCommandRejected("binary session is not ready")
            if self._emergency_latched:
                raise MotionCommandRejected("emergency stop is latched")
            self._tx.put((priority, next(self._seq), tx))

    def cancel_pose_goal(self, goal_id: int, *, priority: int) -> None:
        tx = _BinaryTx(BinaryCommand.CANCEL_POSE_GOAL, cancel_goal_data(goal_id), goal_id)
        with self._safety_lock:
            if not self._binary_mode:
                raise MotionCommandRejected("binary session is not ready")
            self._tx.put((priority, next(self._seq), tx))

    def set_speed_limits(
        self, linear_mps: float, yaw_radps: float, *, priority: int
    ) -> None:
        tx = _BinaryTx(
            BinaryCommand.SET_SPEED_LIMITS,
            speed_limits_data(linear_mps, yaw_radps),
        )
        with self._safety_lock:
            if not self._binary_mode:
                raise MotionCommandRejected("binary session is not ready")
            self._tx.put((priority, next(self._seq), tx))

    def send(self, command: Command | str, priority: Optional[int] = None) -> None:
        """投递命令到 TX 队列（唯一入口，绝不直接写串口）。"""
        if isinstance(command, str):
            command = Command(text=command, priority=20 if priority is None else priority)
        with self._safety_lock:
            if self._binary_mode:
                if command.text == "STOP":
                    self._tx.put(
                        (PRIORITY_STOP, next(self._seq), _BinaryTx(BinaryCommand.STOP_ALL))
                    )
                    return
                raise MotionCommandRejected(
                    f"ASCII command is unavailable in binary mode: {command.text}"
                )
            if self._emergency_latched and is_motion_command(command.text):
                raise MotionCommandRejected(
                    f"emergency stop is latched; rejected: {command.text}"
                )
            self._tx.put((command.priority, next(self._seq), command.text))

    def emergency_stop(self) -> None:
        """原子锁存急停、清除旧命令，并只保留一个最高优先级STOP。"""
        with self._safety_lock:
            self._emergency_latched = True
            self._clear_tx_locked()
            outbound = (
                _BinaryTx(BinaryCommand.STOP_ALL) if self._binary_mode else "STOP"
            )
            self._tx.put((PRIORITY_STOP, next(self._seq), outbound))

    def release_emergency_stop(self) -> None:
        """解除软件急停门禁；只应由完成自检后的SafetySupervisor调用。

        本方法不会自动发送任何运动命令，也不会恢复急停前的队列。
        """
        with self._safety_lock:
            self._emergency_latched = False

    def get_event(self, timeout: float = 0.0) -> Optional[Event]:
        """取一个事件；超时返回 None（Brain 用）。"""
        try:
            return self._events.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_telemetry(self, timeout: float = 0.0) -> Optional[Event]:
        """取曲线样本；与控制事件分流，防止高频遥测延迟STOP/POSE确认。"""
        try:
            return self._telemetry.get(timeout=timeout)
        except queue.Empty:
            return None

    def stop(self) -> None:
        """请求停止并等待线程退出。关闭串口以唤醒阻塞的 readline。"""
        self._running = False
        self._link_state = LinkState.STOPPING
        ser = self._serial
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        self.join(timeout=2.0)

    # ------------------------------------------------------------------
    # 线程主循环
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._running = True
        self._set_link(LinkState.CONNECTING)
        while self._running:
            ser = self._open_serial()
            if ser is None:
                self._set_link(LinkState.BACKOFF)
                self._sleep_backoff()
                continue
            self._serial = ser
            self._set_link(LinkState.CONNECTED)
            self._events.put(SerialConnected())
            self._next_ping = time.monotonic() + self._ping_interval
            self._connected_loop(ser)
            # 退出连接循环：断线或停止
            self._serial = None
            with self._safety_lock:
                was_binary = self._binary_mode
                self._binary_mode = False
                self._binary_decoder = FrameDecoder()
                self._binary_pending.clear()
            try:
                ser.close()
            except Exception:
                pass
            if not self._running:
                return
            # 断线：清除旧命令并锁存运动门禁，重连后必须自检并显式解除。
            with self._safety_lock:
                self._emergency_latched = True
                self._clear_tx_locked()
            self._events.put(SerialDisconnected())
            self._set_link(LinkState.BACKOFF)
            self._sleep_backoff(
                minimum_seconds=(self._binary_recovery_seconds if was_binary else 0.0)
            )

    def _connected_loop(self, ser: SerialLike) -> None:
        """连接期间主循环：RX -> PING -> TX -> 连接状态。"""
        while self._running:
            # 1. RX：短超时读一行（外部 stop() 关闭串口后可立即退出）
            try:
                if not getattr(ser, "is_open", True):
                    return
                line = ser.readline()
            except Exception:
                return  # 读失败视为断线
            if line:
                if self.is_binary_mode:
                    for frame in self._binary_decoder.feed(bytes(line)):
                        event = self._decode_binary_frame(frame)
                        if event is not None:
                            self._events.put(event)
                else:
                    text = bytes(line).decode("utf-8", errors="replace").strip()
                    event = parse_line(text)
                    if event is not None:
                        if isinstance(event, (WheelTelemetry, PoseTelemetry)):
                            self._put_latest_telemetry(event)
                        else:
                            self._events.put(event)

            # 2. PING：先于 TX 队列，按单调时钟 deadline 调度
            now = time.monotonic()
            if self._heartbeat_enabled and now >= self._next_ping:
                ping = (
                    self._encode_binary_tx(_BinaryTx(BinaryCommand.PING), track=False)
                    if self.is_binary_mode
                    else encode_ping()
                )
                if not self._write(ser, ping):
                    return
                self._next_ping = now + self._ping_interval

            # 3. TX 队列：限量 drain，保证下一轮仍能回到 RX/PING
            if not self._drain_tx(ser):
                return

    def _drain_tx(self, ser: SerialLike) -> bool:
        """每轮最多发 tx_drain_limit 条命令，防止队列长期独占循环。"""
        sent = 0
        while self._running and sent < self._tx_drain_limit:
            try:
                _, _, outbound = self._tx.get_nowait()
            except queue.Empty:
                break
            # 与 emergency_stop() 共用锁，保证急停返回后不会再写出旧运动。
            with self._safety_lock:
                if (
                    isinstance(outbound, str)
                    and self._emergency_latched
                    and is_motion_command(outbound)
                ):
                    continue
                if isinstance(outbound, _BinaryTx):
                    payload = self._encode_binary_tx(outbound)
                else:
                    payload = outbound.encode("ascii") + b"\n"
                if not self._write(ser, payload):
                    return False
            sent += 1
        return True

    def _encode_binary_tx(self, tx: _BinaryTx, *, track: bool = True) -> bytes:
        sequence = self._binary_sequence
        self._binary_sequence = (sequence + 1) & 0xFF
        if track:
            self._binary_pending[sequence] = tx
        return command_frame(sequence, tx.command, tx.data)

    def _decode_binary_frame(self, frame) -> Optional[Event]:
        if frame.message_type == MessageType.RESPONSE:
            try:
                response = BinaryResponse.decode(frame.payload)
            except (ValueError, IndexError):
                return RawMessage(raw="binary:malformed-response")
            pending = self._binary_pending.pop(response.request_sequence, None)
            if response.status == ResponseStatus.OK:
                return None
            return BinaryCommandRejected(
                raw=(
                    f"binary:response command=0x{response.command:02x} "
                    f"status={response.status.name}"
                ),
                command=response.command,
                status=int(response.status),
                goal_id=pending.goal_id if pending is not None else None,
            )
        if frame.message_type != MessageType.EVENT:
            return RawMessage(raw=f"binary:unexpected-type:{int(frame.message_type)}")
        try:
            event = decode_pose_event(frame.payload)
        except (ValueError, IndexError):
            return RawMessage(raw="binary:malformed-event")
        raw = f"binary:{event.code.name}:goal={event.goal_id}"
        if event.code == BinaryEventCode.POSE_STARTED:
            return PoseStarted(raw=raw, goal_id=event.goal_id)
        if event.code == BinaryEventCode.POSE_CANCELLED:
            return PoseStopped(raw=raw, goal_id=event.goal_id)
        if event.code == BinaryEventCode.POSE_REACHED:
            x_mm, y_mm, yaw_mrad, error_mm, error_yaw_mrad = event.values
            return PoseReached(
                raw=raw,
                x=float(x_mm),
                y=float(y_mm),
                yaw=yaw_mrad / 17.45329252,
                error_mm=float(error_mm),
                error_yaw=error_yaw_mrad / 17.45329252,
                goal_id=event.goal_id,
            )
        try:
            reason_code = (
                MotionFault(event.values[0])
                if event.values
                else MotionFault.UNSPECIFIED
            )
        except ValueError:
            reason_code = MotionFault.UNSPECIFIED
        reason = {
            MotionFault.OPS9_LOST: FaultReason.OPS_LOST,
            MotionFault.HOST_LOST: FaultReason.HOST_LOST,
            MotionFault.CAN_FAULT: FaultReason.CAN_FAULT,
            MotionFault.OUT_OF_BOUNDS: FaultReason.TRANSLATION_LIMIT,
            MotionFault.TIMEOUT: FaultReason.TIMEOUT,
        }.get(reason_code, FaultReason.UNKNOWN)
        return SafetyFault(raw=raw, reason=reason, goal_id=event.goal_id)

    def _clear_tx_locked(self) -> None:
        """清空TX队列。调用者必须持有_safety_lock。"""
        with self._tx.mutex:
            self._tx.queue.clear()

    def _put_latest_telemetry(self, event: Event) -> None:
        """遥测队列满时丢最旧样本；控制事件使用另一无损队列。"""
        try:
            self._telemetry.put_nowait(event)
            return
        except queue.Full:
            pass
        try:
            self._telemetry.get_nowait()
        except queue.Empty:
            pass
        try:
            self._telemetry.put_nowait(event)
        except queue.Full:
            pass

    def _write(self, ser: SerialLike, payload: bytes) -> bool:
        """唯一实际写串口的路径；写失败视为断线。"""
        try:
            ser.write(payload)
            return True
        except Exception:
            return False

    def _open_serial(self) -> Optional[SerialLike]:
        try:
            return self._factory()
        except Exception:
            return None

    def _set_link(self, state: LinkState) -> None:
        self._link_state = state

    def _sleep_backoff(self, *, minimum_seconds: float = 0.0) -> None:
        """退避期间每 0.1s 检查停止请求，避免 stop() 等待过久。"""
        deadline = time.monotonic() + max(self._backoff, minimum_seconds)
        while self._running and time.monotonic() < deadline:
            time.sleep(0.1)
