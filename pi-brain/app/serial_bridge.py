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
    Command,
    Event,
    RawMessage,
    PoseTelemetry,
    PRIORITY_STOP,
    encode_ping,
    is_motion_command,
    parse_line,
    WheelTelemetry,
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

    def send(self, command: Command | str, priority: Optional[int] = None) -> None:
        """投递命令到 TX 队列（唯一入口，绝不直接写串口）。"""
        if isinstance(command, str):
            command = Command(text=command, priority=20 if priority is None else priority)
        with self._safety_lock:
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
            self._tx.put((PRIORITY_STOP, next(self._seq), "STOP"))

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
            self._sleep_backoff()

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
                if not self._write(ser, encode_ping()):
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
                _, _, text = self._tx.get_nowait()
            except queue.Empty:
                break
            # 与 emergency_stop() 共用锁，保证急停返回后不会再写出旧运动。
            with self._safety_lock:
                if self._emergency_latched and is_motion_command(text):
                    continue
                if not self._write(ser, text.encode("ascii") + b"\n"):
                    return False
            sent += 1
        return True

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

    def _sleep_backoff(self) -> None:
        """退避期间每 0.1s 检查停止请求，避免 stop() 等待过久。"""
        deadline = time.monotonic() + self._backoff
        while self._running and time.monotonic() < deadline:
            time.sleep(0.1)
