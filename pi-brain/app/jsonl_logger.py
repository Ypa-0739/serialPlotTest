# -*- coding: utf-8 -*-
"""第六阶段：JSONL 结构化日志（纯旁路订阅 + 独立低优先级写盘线程）。

设计要点（与上位机安全方案一致）：
  - 订阅式消费者：handle_event() 注册到 EventRouter，绝不自己调用
    get_event()；日志是纯旁路，不影响启动状态机/导航器的事件消费顺序
  - 入队永不阻塞：有界双端队列，积压时丢最旧记录并累计 dropped 计数。
    磁盘卡顿只丢日志行，绝不反压事件泵、PING 心跳或 STOP 投递
    （与遥测“保留最新值”队列同一策略）
  - 唯一写盘者：后台守护线程批量写出并定期 flush；主循环只做一次
    加锁 append（微秒级）。写盘异常只累计 failed 计数，不抛出
  - 每行一个 JSON 对象：
      ts(本地ISO时间) / monotonic_s / elapsed_s / task_state / nav_state /
      goal_id / target / event / raw / data
    task/nav 上下文由注入的 context_provider 提供，本模块不反向依赖
    状态机与导航器（避免循环引用）；provider 抛异常时上下文记为 null
  - close() 停止线程并写出一条 LOG_SUMMARY 汇总行（written/dropped/failed）
"""

from __future__ import annotations

import dataclasses
import json
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from .protocol import Event

# context_provider 返回值约定：task_state/nav_state/goal_id/target 四个键均可缺省
ContextProvider = Callable[[], dict]


class JsonlLogger:
    """JSONL 事件日志器。用法：logger.handle_event 注册到 EventRouter。"""

    def __init__(
        self,
        path: str | Path,
        *,
        queue_size: int = 1024,
        context_provider: Optional[ContextProvider] = None,
        clock: Callable[[], float] = time.monotonic,
        wall: Callable[[], float] = time.time,
        autostart: bool = True,
    ) -> None:
        self._path = Path(path)
        self._max = max(1, queue_size)
        self._provider = context_provider
        self._clock = clock
        self._wall = wall
        self._cond = threading.Condition()
        self._queue: deque = deque()
        self._origin = clock()
        self._written = 0
        self._dropped = 0
        self._failed = 0
        self._write_error_streak = 0
        self._stop_requested = False
        self._closed = False
        self._file = None
        self._thread: Optional[threading.Thread] = None
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
        except OSError:
            pass  # 目录创建失败推迟到 start() 时暴露，不影响调用方主循环
        if autostart:
            self.start()

    # ------------------------------------------------------------------
    # 对外接口
    # ------------------------------------------------------------------

    @property
    def dropped(self) -> int:
        return self._dropped

    def start(self) -> None:
        """打开文件并启动写盘线程（幂等）。"""
        if self._thread is not None:
            return
        self._file = open(self._path, "a", encoding="utf-8", newline="\n")
        self._thread = threading.Thread(
            target=self._worker, name="jsonl-logger", daemon=True
        )
        self._thread.start()

    def handle_event(self, event: Event) -> None:
        """EventRouter 消费者入口：格式化并入队，永不抛出、永不阻塞。"""
        try:
            record = self.build_record(event)
        except Exception:
            return  # 单条事件格式化失败不能影响其它消费者
        self.offer(record)

    def build_record(self, event: Event) -> dict:
        """把一个事件转成 JSON 兼容的记录字典。"""
        now_mono = self._clock()
        task_state = nav_state = goal_id = target = None
        if self._provider is not None:
            try:
                ctx = self._provider() or {}
            except Exception:
                ctx = {}
            task_state = ctx.get("task_state")
            nav_state = ctx.get("nav_state")
            goal_id = ctx.get("goal_id")
            target = ctx.get("target")
            # 位姿等 dataclass 上下文转成纯字典，保证 JSON 结构化而非整串 repr
            if dataclasses.is_dataclass(target) and not isinstance(target, type):
                try:
                    target = dataclasses.asdict(target)
                except Exception:
                    pass
        return {
            "ts": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "monotonic_s": now_mono,
            "elapsed_s": now_mono - self._origin,
            "task_state": task_state,
            "nav_state": nav_state,
            "goal_id": goal_id,
            "target": target,
            "event": type(event).__name__,
            "raw": getattr(event, "raw", ""),
            "data": _event_data(event),
        }

    def offer(self, record: dict) -> bool:
        """入队一条记录；队列满时丢最旧。关闭后返回 False。"""
        if self._closed:
            return False
        with self._cond:
            if len(self._queue) >= self._max:
                try:
                    self._queue.popleft()
                except IndexError:
                    pass
                self._dropped += 1
            self._queue.append(record)
            self._cond.notify()
        return True

    def close(self, timeout: float = 2.0) -> None:
        """停止写盘线程、冲刷并汇总（幂等）。"""
        if self._closed:
            return
        with self._cond:
            self._stop_requested = True
            self._cond.notify()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
        self._closed = True

    # ------------------------------------------------------------------
    # 内部：写盘线程
    # ------------------------------------------------------------------

    def _worker(self) -> None:
        while True:
            with self._cond:
                while not self._queue and not self._stop_requested:
                    self._cond.wait(timeout=0.2)
                batch = list(self._queue)
                self._queue.clear()
                stop = self._stop_requested and not self._queue
            if batch:
                self._write_batch(batch)
            if stop:
                break
        self._write_summary()
        try:
            if self._file is not None:
                self._file.close()
        except Exception:
            pass

    def _write_batch(self, batch: list) -> None:
        if self._file is None or self._file.closed:
            self._failed += len(batch)
            return
        try:
            for record in batch:
                self._file.write(json.dumps(record, ensure_ascii=False, default=str))
                self._file.write("\n")
            self._file.flush()
            self._written += len(batch)
            self._write_error_streak = 0
        except Exception:
            self._failed += len(batch)
            self._write_error_streak += 1
            if self._write_error_streak >= 10:
                # 持续写失败（磁盘拔出等）：关闭文件止损，后续批次仅计数
                try:
                    self._file.close()
                except Exception:
                    pass

    def _write_summary(self) -> None:
        if self._file is None or self._file.closed:
            return
        summary = {
            "ts": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "monotonic_s": self._clock(),
            "elapsed_s": self._clock() - self._origin,
            "task_state": None,
            "nav_state": None,
            "goal_id": None,
            "target": None,
            "event": "LOG_SUMMARY",
            "raw": "",
            "data": {
                "written": self._written,
                "dropped": self._dropped,
                "failed": self._failed,
            },
        }
        try:
            self._file.write(json.dumps(summary, ensure_ascii=False, default=str))
            self._file.write("\n")
            self._file.flush()
        except Exception:
            pass


def _event_data(event: Event) -> dict:
    """提取事件的业务字段；raw 单列故从 data 中剔除。非数据类事件返回空表。"""
    if dataclasses.is_dataclass(event) and not isinstance(event, type):
        try:
            data = dataclasses.asdict(event)
        except Exception:
            return {}
        data.pop("raw", None)
        return data
    return {}
