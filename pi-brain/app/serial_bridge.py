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
    StopAcknowledged,
    PRIORITY_QUERY,
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
    LinkStatsSample,
    PoseSample,
    PoseGoal as BinaryPoseGoal,
    PoseState,
    PoseStatus,
    Response as BinaryResponse,
    ResponseStatus,
    WheelSample,
    cancel_goal_data,
    command_frame,
    decode_telemetry,
    decode_pose_event,
    pose_goal_with_limits_data,
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
class BinaryCommandTimedOut(Event):
    """已发送的二进制命令在截止时间前没有收到匹配响应。"""

    command: int = 0
    goal_id: int | None = None


@dataclass(frozen=True)
class BinaryPoseStatus(Event):
    # QUERY command's Pi-side owner. This is deliberately separate from the
    # firmware-reported goal_id, which may already be IDLE or another goal.
    request_goal_id: int | None = None
    goal_id: int = 0
    state: PoseState = PoseState.IDLE
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_deg: float = 0.0
    fault_reason: int = 0
    robot_mode: int = 0
    host_link: int = 0


@dataclass(frozen=True)
class BinaryLinkStats(Event):
    tick_ms: int = 0
    rx_dropped: int = 0
    tx_dropped: int = 0
    telemetry_replaced: int = 0
    crc_errors: int = 0
    uart_errors: int = 0


@dataclass(frozen=True)
class BinaryBridgeStats:
    pending: int
    command_timeouts: int
    unmatched_responses: int
    decoder_crc_errors: int
    decoder_discarded_bytes: int
    telemetry_dropped: int


@dataclass(frozen=True)
class _BinaryTx:
    command: BinaryCommand
    data: bytes = b""
    goal_id: int | None = None


@dataclass(frozen=True)
class _PendingCommand:
    tx: _BinaryTx
    deadline: float


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
        binary_probe_timeout: float = 0.05,
        binary_command_timeout: float = 0.75,
        max_binary_pending: int = 64,
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
        self._binary_probe_timeout = binary_probe_timeout
        self._binary_command_timeout = binary_command_timeout
        if not 1 <= max_binary_pending <= 255:
            raise ValueError("max_binary_pending must be in 1..255")
        self._max_binary_pending = max_binary_pending
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
        self._binary_pending: dict[int, _PendingCommand] = {}
        self._staged_speed_limits: tuple[float, float] | None = None
        self._preconnected_rx: list[bytes] = []
        self._binary_timeout_count = 0
        self._binary_unmatched_response_count = 0
        self._telemetry_dropped = 0

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

    @property
    def binary_stats(self) -> BinaryBridgeStats:
        with self._safety_lock:
            return BinaryBridgeStats(
                pending=len(self._binary_pending),
                command_timeouts=self._binary_timeout_count,
                unmatched_responses=self._binary_unmatched_response_count,
                decoder_crc_errors=self._binary_decoder.crc_errors,
                decoder_discarded_bytes=self._binary_decoder.discarded_bytes,
                telemetry_dropped=self._telemetry_dropped,
            )

    def enable_binary_mode(
        self, linear_mps: float = 0.20, yaw_radps: float = 0.25
    ) -> None:
        """在收到 ``# HOST BINARY READY`` 后原子切换串口编解码模式。"""
        speed_limits_data(linear_mps, yaw_radps)  # 与固件边界同步校验。
        with self._safety_lock:
            self._binary_decoder = FrameDecoder()
            self._binary_pending.clear()
            self._staged_speed_limits = (linear_mps, yaw_radps)
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
        with self._safety_lock:
            if not self._binary_mode:
                raise MotionCommandRejected("binary session is not ready")
            if self._emergency_latched:
                raise MotionCommandRejected("emergency stop is latched")
            if self._staged_speed_limits is None:
                raise MotionCommandRejected("binary speed limits are unknown")
            linear_mps, yaw_radps = self._staged_speed_limits
            tx = _BinaryTx(
                BinaryCommand.SET_POSE_GOAL_WITH_LIMITS,
                pose_goal_with_limits_data(goal, linear_mps, yaw_radps),
                goal_id,
            )
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
        speed_limits_data(linear_mps, yaw_radps)
        with self._safety_lock:
            if not self._binary_mode:
                raise MotionCommandRejected("binary session is not ready")
            # 二进制运行期不单独写限速；下一条目标把两者原子提交给固件。
            self._staged_speed_limits = (linear_mps, yaw_radps)

    def query_pose_goal(
        self, goal_id: int | None = None, *, priority: int = PRIORITY_QUERY
    ) -> None:
        with self._safety_lock:
            if not self._binary_mode:
                raise MotionCommandRejected("binary session is not ready")
            self._tx.put(
                (
                    priority,
                    next(self._seq),
                    _BinaryTx(BinaryCommand.QUERY_POSE_GOAL, goal_id=goal_id),
                )
            )

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
            self._binary_pending.clear()
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
            self._preconnected_rx.clear()
            if not self._prepare_connection(ser):
                try:
                    ser.close()
                except Exception:
                    pass
                self._serial = None
                self._set_link(LinkState.BACKOFF)
                self._sleep_backoff()
                continue
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
                self._staged_speed_limits = None
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
                line = (
                    self._preconnected_rx.pop(0)
                    if self._preconnected_rx
                    else ser.readline()
                )
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
            self._expire_binary_pending(now)
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
                    try:
                        payload = self._encode_binary_tx(outbound)
                    except MotionCommandRejected as exc:
                        self._events.put(
                            BinaryCommandRejected(
                                raw=f"binary:local-reject:{exc}",
                                command=int(outbound.command),
                                status=int(ResponseStatus.BUSY),
                                goal_id=outbound.goal_id,
                            )
                        )
                        continue
                else:
                    payload = outbound.encode("ascii") + b"\n"
                if not self._write(ser, payload):
                    return False
            sent += 1
        return True

    def _encode_binary_tx(self, tx: _BinaryTx, *, track: bool = True) -> bytes:
        if track and len(self._binary_pending) >= self._max_binary_pending:
            if tx.command == BinaryCommand.STOP_ALL:
                track = False  # 保留停车能力；STOP仍会发出但不占pending。
            else:
                raise MotionCommandRejected("binary pending table is full")
        sequence = self._allocate_binary_sequence()
        if track:
            self._binary_pending[sequence] = _PendingCommand(
                tx=tx,
                deadline=time.monotonic() + self._binary_command_timeout,
            )
        return command_frame(sequence, tx.command, tx.data)

    def _allocate_binary_sequence(self) -> int:
        for _ in range(256):
            sequence = self._binary_sequence
            self._binary_sequence = (sequence + 1) & 0xFF
            if sequence not in self._binary_pending:
                return sequence
        raise MotionCommandRejected("all binary sequence numbers are pending")

    def _expire_binary_pending(self, now: float) -> None:
        with self._safety_lock:
            expired = [
                (sequence, pending)
                for sequence, pending in self._binary_pending.items()
                if now >= pending.deadline
            ]
            for sequence, _ in expired:
                self._binary_pending.pop(sequence, None)
        for sequence, pending in expired:
            self._binary_timeout_count += 1
            self._events.put(
                BinaryCommandTimedOut(
                    raw=(
                        f"binary:timeout sequence={sequence} "
                        f"command=0x{int(pending.tx.command):02x}"
                    ),
                    command=int(pending.tx.command),
                    goal_id=pending.tx.goal_id,
                )
            )

    def _decode_binary_frame(self, frame) -> Optional[Event]:
        if frame.message_type == MessageType.RESPONSE:
            try:
                response = BinaryResponse.decode(frame.payload)
            except (ValueError, IndexError):
                return RawMessage(raw="binary:malformed-response")
            with self._safety_lock:
                pending = self._binary_pending.pop(response.request_sequence, None)
            if pending is None:
                if response.command == BinaryCommand.PING:
                    return None
                self._binary_unmatched_response_count += 1
                return RawMessage(
                    raw=(
                        f"binary:unmatched-response sequence={response.request_sequence} "
                        f"command=0x{response.command:02x}"
                    )
                )
            tx = pending.tx
            if response.command != int(tx.command):
                return RawMessage(
                    raw=(
                        f"binary:mismatched-response sequence={response.request_sequence} "
                        f"expected=0x{int(tx.command):02x} actual=0x{response.command:02x}"
                    )
                )
            if response.status == ResponseStatus.OK:
                if tx.command == BinaryCommand.QUERY_POSE_GOAL:
                    try:
                        status = PoseStatus.decode(response.data)
                    except (ValueError, IndexError):
                        return RawMessage(raw="binary:malformed-query-response")
                    return BinaryPoseStatus(
                        raw=f"binary:pose-status:goal={status.goal_id}",
                        request_goal_id=tx.goal_id,
                        goal_id=status.goal_id,
                        state=status.state,
                        x_mm=float(status.x_mm),
                        y_mm=float(status.y_mm),
                        yaw_deg=status.yaw_mrad / 17.45329252,
                        fault_reason=status.fault_reason,
                        robot_mode=status.robot_mode,
                        host_link=status.host_link,
                    )
                if tx.command == BinaryCommand.STOP_ALL:
                    return StopAcknowledged(raw="binary:stop-ack")
                return None
            return BinaryCommandRejected(
                raw=(
                    f"binary:response command=0x{response.command:02x} "
                    f"status={response.status.name}"
                ),
                command=response.command,
                status=int(response.status),
                goal_id=tx.goal_id,
            )
        if frame.message_type == MessageType.TELEMETRY:
            try:
                sample = decode_telemetry(frame.payload)
            except (ValueError, IndexError):
                return RawMessage(raw="binary:malformed-telemetry")
            if isinstance(sample, WheelSample):
                event = WheelTelemetry(
                    raw=f"binary:wheel:{sample.sequence}",
                    version=2,
                    tick_ms=sample.tick_ms,
                    sequence=sample.sequence,
                    target_rpm=tuple(value / 10.0 for value in sample.target_rpm_tenths),
                    actual_rpm=tuple(value / 10.0 for value in sample.actual_rpm_tenths),
                )
            elif isinstance(sample, PoseSample):
                event = PoseTelemetry(
                    raw=f"binary:pose:{sample.sequence}",
                    version=2,
                    tick_ms=sample.tick_ms,
                    sequence=sample.sequence,
                    ops_x_mm=float(sample.ops_x_mm),
                    ops_y_mm=float(sample.ops_y_mm),
                    ops_yaw_deg=sample.ops_yaw_mrad / 17.45329252,
                    center_x_mm=float(sample.center_x_mm),
                    center_y_mm=float(sample.center_y_mm),
                    plan_vx_mps=sample.plan_vx_um_s / 1_000_000.0,
                    plan_vy_mps=sample.plan_vy_um_s / 1_000_000.0,
                    plan_vz_radps=sample.plan_vz_urad_s / 1_000_000.0,
                )
            else:
                event = BinaryLinkStats(
                    raw="binary:link-stats",
                    tick_ms=sample.tick_ms,
                    rx_dropped=sample.rx_dropped,
                    tx_dropped=sample.tx_dropped,
                    telemetry_replaced=sample.telemetry_replaced,
                    crc_errors=sample.crc_errors,
                    uart_errors=sample.uart_errors,
                )
            self._put_latest_telemetry(event)
            return None
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
        self._telemetry_dropped += 1
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

    def _prepare_connection(self, ser: SerialLike) -> bool:
        """探测遗留二进制会话；若存在，先停车并等待固件退回ASCII。"""
        decoder = FrameDecoder()
        sequence = self._allocate_binary_sequence()
        if not self._write(ser, command_frame(sequence, BinaryCommand.SESSION_PROBE)):
            return False
        deadline = time.monotonic() + self._binary_probe_timeout
        stale_session = False
        while self._running and time.monotonic() < deadline:
            try:
                raw = ser.readline()
            except Exception:
                return False
            if not raw:
                time.sleep(0.005)
                continue
            if not bytes(raw).startswith(b"\xA5\x5A"):
                self._preconnected_rx.append(bytes(raw))
                continue
            for frame in decoder.feed(bytes(raw)):
                if frame.message_type != MessageType.RESPONSE:
                    continue
                try:
                    response = BinaryResponse.decode(frame.payload)
                except (ValueError, IndexError):
                    continue
                if (
                    response.request_sequence == sequence
                    and response.command == BinaryCommand.SESSION_PROBE
                    and response.status == ResponseStatus.OK
                    and len(response.data) == 13
                ):
                    stale_session = bool(response.data[0] or response.data[1])
                    deadline = 0.0
                    break
        if not stale_session:
            return self._running

        stop_sequence = self._allocate_binary_sequence()
        if not self._write(ser, command_frame(stop_sequence, BinaryCommand.STOP_ALL)):
            return False
        # 不发送PING，让旧会话确定性超时；期间丢弃旧STOP响应和取消事件。
        recovery_deadline = time.monotonic() + self._binary_recovery_seconds
        while self._running and time.monotonic() < recovery_deadline:
            try:
                ser.readline()
            except Exception:
                return False
            time.sleep(0.01)
        return self._running

    def _set_link(self, state: LinkState) -> None:
        self._link_state = state

    def _sleep_backoff(self, *, minimum_seconds: float = 0.0) -> None:
        """退避期间每 0.1s 检查停止请求，避免 stop() 等待过久。"""
        deadline = time.monotonic() + max(self._backoff, minimum_seconds)
        while self._running and time.monotonic() < deadline:
            time.sleep(0.1)
