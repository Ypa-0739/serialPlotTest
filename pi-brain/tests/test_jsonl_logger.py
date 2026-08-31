# -*- coding: utf-8 -*-
"""JSONL 日志器单测：字段结构 / 上下文注入 / 有界丢弃 / 汇总与关闭安全。

覆盖第六阶段验收要点：
  - 每行一个 JSON：ts/monotonic_s/elapsed_s/task_state/nav_state/goal_id/
    target/event/raw/data 全部存在且类型正确
  - context_provider 注入任务上下文；provider 抛异常不影响记录
  - dataclass 事件字段进入 data（reason 等枚举转为字符串值）
  - 有界队列丢最旧（autostart=False 确定性验证 dropped 计数）
  - close() 写出 LOG_SUMMARY；重复 close 安全；close 后入队被拒绝
"""

import json
import tempfile
import unittest
from pathlib import Path

from app.jsonl_logger import JsonlLogger
from app.protocol import (
    FaultReason,
    Pong,
    RawMessage,
    SafetyFault,
    UnknownError,
    WheelTelemetry,
)


class JsonlLoggerTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)
        self.path = self.dir / "events.jsonl"

    def tearDown(self):
        self._tmp.cleanup()

    def _read_records(self, path: Path) -> list[dict]:
        text = path.read_text(encoding="utf-8")
        return [json.loads(line) for line in text.splitlines() if line.strip()]

    _REQUIRED_KEYS = (
        "ts",
        "monotonic_s",
        "elapsed_s",
        "task_state",
        "nav_state",
        "goal_id",
        "target",
        "event",
        "raw",
        "data",
    )

    # ------------------------------------------------------------------
    # 基本结构与上下文
    # ------------------------------------------------------------------

    def test_basic_fields_and_context(self):
        """事件字段完整；context_provider 的任务上下文逐字段落盘。"""
        logger = JsonlLogger(
            self.path,
            context_provider=lambda: {
                "task_state": "MOVE_TO_PICK",
                "nav_state": "MOVING",
                "goal_id": 7,
                "target": {"x_mm": 1000.0, "y_mm": 500.0},
            },
        )
        logger.handle_event(Pong(raw="# PONG"))
        logger.close()

        records = self._read_records(self.path)
        self.assertEqual(len(records), 2)  # 1 条事件 + LOG_SUMMARY
        record = records[0]
        for key in self._REQUIRED_KEYS:
            self.assertIn(key, record)
        self.assertEqual(record["event"], "Pong")
        self.assertEqual(record["raw"], "# PONG")
        self.assertEqual(record["task_state"], "MOVE_TO_PICK")
        self.assertEqual(record["nav_state"], "MOVING")
        self.assertEqual(record["goal_id"], 7)
        self.assertEqual(record["target"], {"x_mm": 1000.0, "y_mm": 500.0})
        self.assertIsInstance(record["monotonic_s"], float)
        self.assertIsInstance(record["elapsed_s"], float)
        self.assertGreaterEqual(record["elapsed_s"], 0.0)
        self.assertIn("T", record["ts"])  # ISO 时间戳
        self.assertEqual(record["data"], {})

    def test_dataclass_fields_serialized(self):
        """dataclass 事件业务字段进入 data；str 枚举转为其值。"""
        logger = JsonlLogger(self.path)
        event = SafetyFault(reason=FaultReason.OPS_LOST, raw="# POSE STOP SAFETY")
        logger.handle_event(event)
        logger.close()

        record = self._read_records(self.path)[0]
        self.assertEqual(record["event"], "SafetyFault")
        self.assertEqual(record["raw"], "# POSE STOP SAFETY")
        self.assertEqual(record["data"]["reason"], "OPS LOST")
        self.assertNotIn("raw", record["data"])

    def test_wheel_telemetry_tuples_become_arrays(self):
        """W8 遥测的元组字段序列化为 JSON 数组。"""
        logger = JsonlLogger(self.path)
        logger.handle_event(
            WheelTelemetry(
                raw="@W,1,100,5,1,2,3,4,0.1,0.2,0.3,0.4",
                version=1,
                tick_ms=100,
                sequence=5,
                target_rpm=(1.0, 2.0, 3.0, 4.0),
                actual_rpm=(0.1, 0.2, 0.3, 0.4),
            )
        )
        logger.close()

        data = self._read_records(self.path)[0]["data"]
        self.assertEqual(data["target_rpm"], [1.0, 2.0, 3.0, 4.0])
        self.assertEqual(data["actual_rpm"], [0.1, 0.2, 0.3, 0.4])

    def test_context_provider_exception_isolated(self):
        """provider 抛异常：上下文记为 null，日志照常写入。"""
        def broken_provider():
            raise RuntimeError("boom")

        logger = JsonlLogger(self.path, context_provider=broken_provider)
        logger.handle_event(Pong(raw="# PONG"))
        logger.close()

        record = self._read_records(self.path)[0]
        self.assertIsNone(record["task_state"])
        self.assertIsNone(record["nav_state"])
        self.assertIsNone(record["goal_id"])
        self.assertIsNone(record["target"])
        self.assertEqual(record["event"], "Pong")

    def test_unicode_preserved(self):
        """ensure_ascii=False：中文原文保留，不转义。"""
        logger = JsonlLogger(self.path)
        logger.handle_event(UnknownError(text="中文错误信息", raw="# ERROR 中文"))
        logger.close()

        raw_text = self.path.read_text(encoding="utf-8")
        self.assertIn("中文错误信息", raw_text)
        record = json.loads(raw_text.splitlines()[0])
        self.assertEqual(record["data"]["text"], "中文错误信息")

    def test_non_dataclass_event_falls_back_to_empty_data(self):
        """非 dataclass 事件：event/raw 仍记录，data 为空表。"""
        logger = JsonlLogger(self.path)
        logger.handle_event(RawMessage(raw="junk line"))
        logger.close()
        record = self._read_records(self.path)[0]
        self.assertEqual(record["event"], "RawMessage")
        self.assertEqual(record["data"], {})

    # ------------------------------------------------------------------
    # 有界队列与丢弃
    # ------------------------------------------------------------------

    def test_bounded_queue_drops_oldest(self):
        """队列满时丢最旧：dropped 计数正确，最新记录保留。"""
        logger = JsonlLogger(self.path, queue_size=3, autostart=False)
        for index in range(5):
            self.assertTrue(logger.offer({"index": index}))
        self.assertEqual(logger.dropped, 2)

        logger.start()
        logger.close()
        records = self._read_records(self.path)
        events = [r for r in records if r.get("event") != "LOG_SUMMARY"]
        summary = [r for r in records if r.get("event") == "LOG_SUMMARY"]
        self.assertEqual([r["index"] for r in events], [2, 3, 4])
        self.assertEqual(len(summary), 1)
        self.assertEqual(summary[0]["data"]["dropped"], 2)
        self.assertEqual(summary[0]["data"]["written"], 3)

    # ------------------------------------------------------------------
    # 关闭语义
    # ------------------------------------------------------------------

    def test_close_writes_summary_and_is_idempotent(self):
        """close() 写汇总行；重复 close 不抛异常。"""
        logger = JsonlLogger(self.path)
        logger.handle_event(Pong(raw="# PONG"))
        logger.close()
        logger.close()

        records = self._read_records(self.path)
        self.assertEqual(records[-1]["event"], "LOG_SUMMARY")
        self.assertGreaterEqual(records[-1]["data"]["written"], 1)

    def test_offer_after_close_rejected(self):
        """关闭后 offer 返回 False 且 handle_event 安全无异常。"""
        logger = JsonlLogger(self.path)
        logger.handle_event(Pong(raw="# PONG"))
        logger.close()
        self.assertFalse(logger.offer({"late": True}))
        logger.handle_event(Pong(raw="# PONG"))  # 不应抛出

        records = self._read_records(self.path)
        self.assertEqual(len(records), 2)  # 事件 + 汇总，无迟到记录


if __name__ == "__main__":
    unittest.main()
