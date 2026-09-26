"""导航独立安全监控测试。"""

from pathlib import Path
from types import SimpleNamespace
import unittest

from robot_control.navigation_map import NavigationMap, Pose2D
from robot_runtime.models import SafetyReport
from robot_services.navigation_safety import (
    NavigationSafetyConfig,
    NavigationSafetyMonitor,
)


ROOT = Path(__file__).resolve().parents[1]


class _Chassis:
    commanded_motion_active = True
    commanded_speed_mm_s = 300.0


class NavigationSafetyTests(unittest.TestCase):
    def setUp(self):
        self.map = NavigationMap.load(ROOT / "config" / "navigation.json")
        self.config = NavigationSafetyConfig.load(
            ROOT / "config" / "navigation_safety.json"
        )
        self.now = 10.0
        self.pose = Pose2D(1200, 1200, 0.0)
        self.road = SimpleNamespace(observed_at=10.0, boundary_safe=True)
        self.candidates = []
        self.chassis = _Chassis()

    def monitor(self):
        return NavigationSafetyMonitor(
            self.map,
            lambda: self.pose,
            self.chassis,
            link_health=lambda: True,
            camera_health=lambda _age: True,
            road_observation_reader=lambda: self.road,
            obstacle_candidate_reader=lambda: self.candidates,
            config=self.config,
            clock=lambda: self.now,
        )

    def test_dynamic_braking_distance_increases_with_speed(self):
        self.assertGreater(
            self.config.stopping_distance_mm(400),
            self.config.stopping_distance_mm(100),
        )

    def test_candidate_inside_braking_envelope_causes_emergency_stop(self):
        self.candidates = [
            SimpleNamespace(x_mm=1500, y_mm=1200, radius_mm=60)
        ]

        report = self.monitor().check()

        self.assertFalse(report.safe)
        self.assertTrue(report.emergency_stop)
        self.assertIn("制动包络", report.reason)

    def test_visual_forbidden_area_causes_boundary_stop(self):
        self.road = SimpleNamespace(observed_at=10.0, boundary_safe=False)

        report = self.monitor().check()

        self.assertFalse(report.safe)
        self.assertFalse(report.boundary_ok)

    def test_missing_road_is_allowed_only_when_no_motion_is_commanded(self):
        self.road = None
        self.chassis.commanded_motion_active = False
        report = self.monitor().check()
        self.assertEqual(report, SafetyReport())

        self.chassis.commanded_motion_active = True
        self.assertFalse(self.monitor().check().safe)


if __name__ == "__main__":
    unittest.main()
