from pathlib import Path
import unittest


class VisionArchitectureTests(unittest.TestCase):
    def test_vision_package_does_not_import_control_or_serial_modules(self):
        root = Path(__file__).resolve().parents[1] / "app" / "vision"
        forbidden = (
            "serial_bridge",
            "app.protocol",
            "app.navigator",
            "app.route_runner",
            "app.mission",
            "serial.write",
            "import serial",
            "from app",
            "POSE SET",
            "SET_CHASSIS_VELOCITY",
        )
        for path in root.glob("*.py"):
            text = path.read_text(encoding="utf-8")
            for value in forbidden:
                self.assertNotIn(value, text, f"{path.name} contains forbidden {value}")


if __name__ == "__main__":
    unittest.main()
