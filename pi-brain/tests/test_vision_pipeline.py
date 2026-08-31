import unittest

import numpy as np

from app.vision.models import CameraFrame, LineObservation, QrObservation, RoadObservation
from app.vision.pipeline import VisionPipeline


class _Camera:
    def __init__(self, source):
        self.source = source
        self.captures = 0

    def capture(self):
        self.captures += 1
        return CameraFrame(1.0, self.captures, self.source, np.zeros((8, 8, 3), dtype=np.uint8))


class _Cameras:
    def __init__(self):
        self.front = _Camera("front")
        self.gripper = _Camera("gripper")
        self.roles = ()

    def start(self, roles):
        self.roles = tuple(roles)

    def close(self):
        self.roles = ()


class _Qr:
    def __init__(self):
        self.images = []

    def detect(self, frame):
        self.images.append(frame.image)
        return QrObservation(frame.timestamp, frame.frame_id, "task", 1.0, True, frame.source)

    def as_common(self, item):
        from app.vision.qr_scanner import QrScanner
        return QrScanner.as_common(item)


class _Line:
    def __init__(self):
        self.images = []

    def detect(self, frame):
        self.images.append(frame.image)
        return LineObservation(frame.timestamp, frame.frame_id, True, 0.0, (4, 4), 10.0, 1.0, frame.source)

    def as_common(self, item):
        from app.vision.line_detector import LineDetector
        return LineDetector.as_common(item)


class _Road:
    def __init__(self):
        self.images = []

    def detect(self, frame):
        self.images.append(frame.image)
        return RoadObservation(frame.timestamp, frame.frame_id, 1.0, 0.0, 1.0, True, 1.0, frame.source)

    def as_common(self, item):
        from app.vision.road_detector import RoadDetector
        return RoadDetector.as_common(item)


class VisionPipelineTests(unittest.TestCase):
    def test_front_detectors_share_exactly_one_captured_frame(self):
        cameras = _Cameras()
        qr, line, road = _Qr(), _Line(), _Road()
        pipeline = VisionPipeline(cameras, qr=qr, line=line, road=road)
        pipeline.start()
        observations = tuple(pipeline.poll())
        self.assertEqual(cameras.front.captures, 1)
        self.assertEqual(len(observations), 3)
        self.assertIs(qr.images[0], line.images[0])
        self.assertIs(line.images[0], road.images[0])
        pipeline.close()


if __name__ == "__main__":
    unittest.main()
