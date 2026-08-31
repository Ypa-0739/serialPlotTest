"""视觉工作线程和有界最新值发布。"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable, Iterable, Optional, Protocol

from .models import VisionHealth, VisionObservation


class VisionProducer(Protocol):
    def start(self) -> None: ...

    def poll(self) -> Iterable[VisionObservation]: ...

    def close(self) -> None: ...


class VisionService:
    """唯一视觉工作线程；从不访问串口或运动控制器。"""

    def __init__(
        self,
        producer: VisionProducer,
        *,
        poll_interval_s: float = 0.02,
        queue_size: int = 32,
        stale_after_s: float = 1.0,
        clock: Callable[[], float] = time.monotonic,
        name: str = "vision-service",
    ) -> None:
        if poll_interval_s < 0.0:
            raise ValueError("poll_interval_s must be non-negative")
        if queue_size < 1:
            raise ValueError("queue_size must be positive")
        if stale_after_s <= 0.0:
            raise ValueError("stale_after_s must be positive")
        self.producer = producer
        self.poll_interval_s = poll_interval_s
        self.stale_after_s = stale_after_s
        self._clock = clock
        self._name = name
        self._queue: deque[VisionObservation] = deque(maxlen=queue_size)
        self._latest: dict[str, VisionObservation] = {}
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._last_poll_at: Optional[float] = None
        self._error = ""
        self._dropped = 0

    @property
    def dropped(self) -> int:
        with self._condition:
            return self._dropped

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        with self._condition:
            # 重新启动绝不暴露上一次运行遗留的目标。
            self._queue.clear()
            self._latest.clear()
            self._last_poll_at = None
            self._error = ""
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, name=self._name, daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> bool:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0.0, timeout))
        stopped = thread is None or not thread.is_alive()
        if stopped:
            self._thread = None
            # stop/close 是视觉生命周期边界。成功停止后不允许消费者继续读取
            # 停止前的目标；与重启、故障和模式切换时的清旧结果语义一致。
            self.clear()
        return stopped

    close = stop

    def latest(self, target_type: Optional[str] = None) -> Optional[VisionObservation]:
        with self._condition:
            if target_type is not None:
                return self._latest.get(target_type)
            if not self._latest:
                return None
            return max(self._latest.values(), key=lambda item: item.timestamp)

    def get(self, timeout: float = 0.0) -> Optional[VisionObservation]:
        with self._condition:
            if not self._queue and timeout > 0.0:
                self._condition.wait(timeout=timeout)
            if not self._queue:
                return None
            return self._queue.popleft()

    def clear(self) -> None:
        """模式切换、故障或重连时清除所有旧观察。"""
        with self._condition:
            self._queue.clear()
            self._latest.clear()
            self._last_poll_at = None

    def health(self) -> VisionHealth:
        now = self._clock()
        with self._condition:
            age = (
                None
                if self._last_poll_at is None
                else max(0.0, now - self._last_poll_at)
            )
            healthy = self._running and not self._error and age is not None and age <= self.stale_after_s
            return VisionHealth(now, self._running, healthy, age, self._error)

    def _offer(self, observation: VisionObservation) -> None:
        with self._condition:
            if len(self._queue) == self._queue.maxlen:
                self._dropped += 1
            self._queue.append(observation)
            self._latest[observation.target_type] = observation
            self._condition.notify_all()

    def _worker(self) -> None:
        try:
            self.producer.start()
            with self._condition:
                self._running = True
            while not self._stop.is_set():
                observations = tuple(self.producer.poll())
                with self._condition:
                    # 成功完成一次采集/检测周期即刷新视觉健康；没有识别到目标
                    # 是正常业务结果，不能误报成摄像头失联。
                    self._last_poll_at = self._clock()
                for observation in observations:
                    if self._stop.is_set():
                        break
                    self._offer(observation)
                if self.poll_interval_s and self._stop.wait(self.poll_interval_s):
                    break
        except Exception as exc:
            with self._condition:
                self._error = f"{type(exc).__name__}: {exc}"
                # 故障后清结果，禁止消费者把异常前观察当成新目标。
                self._queue.clear()
                self._latest.clear()
                self._last_poll_at = None
        finally:
            try:
                self.producer.close()
            except Exception as exc:
                with self._condition:
                    if not self._error:
                        self._error = f"close failed: {type(exc).__name__}: {exc}"
            with self._condition:
                self._running = False
                self._condition.notify_all()
