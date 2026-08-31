import unittest

import cv2
import numpy as np

from app.vision.color_detector import ColorDetector
from app.vision.config import load_json_config
from app.vision.material_detector import MaterialDetector
from app.vision.models import CameraFrame
from app.vision.models import ObstacleObservation
from app.vision.obstacle_detector import ObstacleTracker
from app.vision.qr_scanner import QrScanner


class _QrDetector:
    def __init__(self, text):
        self.text = text

    def detectAndDecodeMulti(self, frame):
        return False, (), None, None

    def detectAndDecode(self, frame):
        return self.text, None, None


class VisionDetectorTests(unittest.TestCase):
    def test_rgb_red_is_not_misread_as_blue(self):
        config = load_json_config("color.json")
        config["detection"] = {
            **config["detection"],
            "expected_color_count": 1,
            "min_confirmations": 1,
        }
        detector = ColorDetector(config)
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        image[30:70, 30:70] = (255, 0, 0)  # RGB red；面积低于30%画幅上限
        result = detector.detect(image)
        self.assertIn(1, [item.code for item in result.candidates])
        self.assertNotIn(3, [item.code for item in result.candidates])

    def test_material_pixels_are_not_exported_as_mm(self):
        config = load_json_config("color.json")
        config["detection"] = {
            **config["detection"],
            "expected_color_count": 1,
            "min_confirmations": 1,
        }
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[200:280, 280:360] = (0, 255, 0)
        frame = CameraFrame(1.0, 2, "gripper", image)
        detector = MaterialDetector(
            ColorDetector(config),
            {
                "grip_center": [320, 240],
                "alignment_tolerance_pixels": [18, 18],
                "require_global_ready": False,
            },
        )
        material = detector.detect(frame, 4)
        self.assertIsNotNone(material)
        common = detector.as_common(material)
        self.assertIsNone(common.x_mm)
        self.assertIsNone(common.y_mm)
        self.assertTrue(common.valid)

    def test_unaligned_material_is_visible_but_not_valid_for_task_decision(self):
        config = load_json_config("color.json")
        config["detection"] = {
            **config["detection"],
            "expected_color_count": 1,
            "min_confirmations": 1,
        }
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[200:280, 100:180] = (0, 255, 0)
        detector = MaterialDetector(
            ColorDetector(config),
            {
                "grip_center": [320, 240],
                "alignment_tolerance_pixels": [18, 18],
                "require_global_ready": False,
            },
        )
        material = detector.detect(CameraFrame(1.0, 2, "gripper", image), 4)
        self.assertIsNotNone(material)
        self.assertTrue(material.confirmed)
        self.assertFalse(material.aligned)
        self.assertFalse(detector.as_common(material).valid)

    def test_qr_requires_consecutive_confirmations(self):
        scanner = QrScanner(_QrDetector("452+321+254+312"), confirmations=2)
        frame = CameraFrame(1.0, 1, "front", np.zeros((10, 10, 3), dtype=np.uint8))
        first = scanner.detect(frame)
        second = scanner.detect(CameraFrame(1.1, 2, "front", frame.image))
        self.assertFalse(first.confirmed)
        self.assertTrue(second.confirmed)
        common = scanner.as_common(second)
        self.assertEqual(common.target_id, "452+321+254+312")

    def test_obstacle_memory_keeps_timestamp_and_expires_at_default_gate_age(self):
        tracker = ObstacleTracker(confirmations_required=1)
        observed = ObstacleObservation(1.0, 1, 100.0, 200.0, 60.0, 0.8, False)
        first = tracker.update((observed,), 1.0)
        remembered = tracker.update((), 1.4)
        expired = tracker.update((), 1.6)
        self.assertEqual(first[0].timestamp, 1.0)
        self.assertEqual(remembered[0].timestamp, 1.0)
        self.assertEqual(expired, ())


if __name__ == "__main__":
    unittest.main()
