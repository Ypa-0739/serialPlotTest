# -*- coding: utf-8 -*-
"""测试替身：模拟 pyserial 串口对象的最小接口（可注入，不依赖真实端口）。"""

import threading
from typing import Optional


class FakeSerial:
    """可注入的串口替身。

    - readline(): 弹出一行；无数据返回 b''（模拟超时）
    - write(): 记录到 writes；可配置 fail_after 写次数后抛异常模拟断线
    - write_threads: 记录每次写入的线程名，用于验证唯一写者
    - close(): is_open=False；关闭后 readline 返回空、write 抛异常
    """

    def __init__(
        self,
        lines: Optional[list[str]] = None,
        *,
        fail_after: Optional[int] = None,
    ) -> None:
        self.lines: list[str] = list(lines or [])
        self.writes: list[bytes] = []
        self.write_threads: list[str] = []
        self.is_open = True
        self.closed = False
        self._write_count = 0
        self.fail_after = fail_after
        self._fail = False
        # 与 pyserial 对齐的字段（bridge 依赖可读性不强，保留以贴近真实）
        self.timeout = 0.05
        self.write_timeout = 0.1

    def fail(self) -> None:
        """手动注入故障（模拟物理断线）。"""
        self._fail = True

    def readline(self):
        if not self.is_open:
            return b""
        if self._fail:
            raise OSError("simulated serial read failure")
        if self.lines:
            return self.lines.pop(0).encode("utf-8")
        return b""  # 模拟超时

    def write(self, data):
        if not self.is_open:
            raise OSError("write on closed serial")
        self._write_count += 1
        if self._fail or (
            self.fail_after is not None and self._write_count > self.fail_after
        ):
            self._fail = True
            raise OSError("simulated serial write failure")
        self.writes.append(bytes(data))
        self.write_threads.append(threading.current_thread().name)
        return len(data)

    def close(self) -> None:
        self.is_open = False
        self.closed = True
