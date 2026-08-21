# -*- coding: utf-8 -*-
"""同步事件路由器：主循环唯一事件源 -> 多个消费者的顺序广播。

设计约束（与上位机方案一致）：主循环只调用一次 bridge.get_event()，
然后把同一个事件逐个交给所有消费者（启动状态机、导航器、后续日志器）。
各模块绝不自己调用 get_event()，否则事件可能被其中一个模块“抢走”。

第一版刻意保持简单：单一同步 EventRouter，按订阅顺序调用即可。
"""

from __future__ import annotations

import sys
from typing import Callable

from .protocol import Event


class EventRouter:
    """同步广播器。subscribe() 注册消费者，publish() 逐个转发。"""

    def __init__(self) -> None:
        self._handlers: list[Callable[[Event], None]] = []

    def subscribe(self, handler: Callable[[Event], None]) -> None:
        """注册消费者；按订阅顺序调用。"""
        self._handlers.append(handler)

    def publish(self, event: Event) -> None:
        """广播事件；单个消费者异常不影响其它消费者。"""
        for handler in self._handlers:
            try:
                handler(event)
            except Exception as exc:  # noqa: BLE001
                name = getattr(handler, "__name__", repr(handler))
                print(
                    f"[ROUTER] {name} failed on {type(event).__name__}: {exc}",
                    file=sys.stderr,
                )
