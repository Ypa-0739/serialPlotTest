import unittest

from app.vision.camera import PiCamera, validate_camera_config


class _Device:
    def __init__(self, number):
        self.number = number
        self.started = False
        self.closed = False

    def create_preview_configuration(self, **kwargs):
        self.kwargs = kwargs
        return kwargs

    def configure(self, config):
        self.config = config

    def start(self):
        self.started = True

    def capture_array(self, stream):
        return [[self.number, stream]]

    def stop(self):
        self.started = False

    def close(self):
        self.closed = True


class VisionCameraTests(unittest.TestCase):
    def setUp(self):
        self.now = 5.0
        self.config = {
            "camera_num": 1,
            "frame_size": [640, 480],
            "format": "RGB888",
            "fps": 15,
            "buffer_count": 3,
        }

    def test_camera_is_not_healthy_until_first_frame(self):
        camera = PiCamera(
            "gripper", self.config, device_factory=_Device, clock=lambda: self.now
        )
        camera.start()
        self.assertFalse(camera.is_healthy())
        frame = camera.capture()
        self.assertEqual(frame.frame_id, 0)
        self.assertEqual(frame.source, "gripper")
        self.assertTrue(camera.is_healthy())
        self.now += 2.0
        self.assertFalse(camera.is_healthy(stale_after_s=1.0))
        camera.close()

    def test_config_rejects_duplicate_devices(self):
        config = {
            "platform": "raspberry_pi_5",
            "front": {**self.config, "camera_num": 0},
            "gripper": {**self.config, "camera_num": 0},
        }
        with self.assertRaises(ValueError):
            validate_camera_config(config)


if __name__ == "__main__":
    unittest.main()
