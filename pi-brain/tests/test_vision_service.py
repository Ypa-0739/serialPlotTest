import threading
import time
import unittest

from app.vision.models import VisionObservation
from app.vision.service import VisionService


class _Producer:
    def __init__(self, fail=False):
        self.started = False
        self.closed = False
        self.index = 0
        self.fail = fail

    def start(self):
        self.started = True

    def poll(self):
        if self.fail:
            raise RuntimeError("camera lost")
        self.index += 1
        return (
            VisionObservation(
                time.monotonic(), self.index, "material:4", 0.9,
                None, None, None, True, "gripper"
            ),
        )

    def close(self):
        self.closed = True


class _EmptyProducer(_Producer):
    def poll(self):
        self.index += 1
        return ()


class VisionServiceTests(unittest.TestCase):
    def test_service_publishes_latest_and_drops_oldest(self):
        producer = _Producer()
        service = VisionService(producer, poll_interval_s=0.001, queue_size=2)
        service.start()
        deadline = time.monotonic() + 1.0
        while (service.latest("material:4") is None or service.dropped == 0) and time.monotonic() < deadline:
            time.sleep(0.005)
        latest = service.latest("material:4")
        self.assertIsNotNone(latest)
        self.assertGreater(service.dropped, 0)
        self.assertTrue(service.stop())
        self.assertTrue(producer.closed)

    def test_failure_clears_old_observation_and_reports_unhealthy(self):
        producer = _Producer(fail=True)
        service = VisionService(producer, poll_interval_s=0.0)
        service.start()
        deadline = time.monotonic() + 1.0
        while not producer.closed and time.monotonic() < deadline:
            time.sleep(0.005)
        health = service.health()
        self.assertFalse(health.healthy)
        self.assertIn("camera lost", health.error)
        self.assertIsNone(service.latest())
        self.assertTrue(service.stop())

    def test_restart_does_not_expose_old_result(self):
        producer = _Producer()
        service = VisionService(producer, poll_interval_s=0.01)
        service.start()
        self.assertIsNotNone(service.get(timeout=1.0))
        # 再产生一个 latest，确认 stop 本身清 queue 和 latest，不依赖调用方补 clear。
        deadline = time.monotonic() + 1.0
        while service.latest() is None and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertIsNotNone(service.latest())
        self.assertTrue(service.stop())
        self.assertIsNone(service.latest())
        self.assertIsNone(service.get())

    def test_no_target_is_still_a_healthy_detection_cycle(self):
        producer = _EmptyProducer()
        service = VisionService(producer, poll_interval_s=0.001)
        service.start()
        deadline = time.monotonic() + 1.0
        while not service.health().healthy and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(service.health().healthy)
        self.assertIsNone(service.latest())
        self.assertTrue(service.stop())


if __name__ == "__main__":
    unittest.main()
