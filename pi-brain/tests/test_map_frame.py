import unittest

from app.map_frame import MapGoal, StartAnchoredMapFrame
from app.models import Pose
from tools.vision_motion_debug import DEFAULT_CONFIG, load_config


class StartAnchoredMapFrameTests(unittest.TestCase):
    def test_nominal_start_maps_back_to_actual_ops_pose(self):
        frame = StartAnchoredMapFrame(Pose(0, 0, 0), Pose(100, 225, 0))
        goal = frame.to_ops_goal(MapGoal("start", 0, 0, 0))
        self.assertAlmostEqual(goal.x_mm, 100)
        self.assertAlmostEqual(goal.y_mm, 225)
        self.assertAlmostEqual(goal.yaw_deg, 0)

    def test_map_delta_rotates_with_actual_start_yaw(self):
        frame = StartAnchoredMapFrame(Pose(0, 0, 0), Pose(100, 200, 90))
        goal = frame.to_ops_goal(MapGoal("ahead", 1000, 0, 0))
        self.assertAlmostEqual(goal.x_mm, 100)
        self.assertAlmostEqual(goal.y_mm, 1200)
        self.assertAlmostEqual(goal.yaw_deg, 90)

    def test_turning_goal_applies_ops_sensor_offset_at_target_yaw(self):
        frame = StartAnchoredMapFrame(Pose(0, 0, 0), Pose(0, 25, 0))
        goal = frame.to_ops_goal(MapGoal("turn", 0, 0, 90, timeout_s=8))
        self.assertAlmostEqual(goal.x_mm, -25)
        self.assertAlmostEqual(goal.y_mm, 0)
        self.assertAlmostEqual(goal.yaw_deg, 90)
        self.assertEqual(goal.timeout_s, 8)

    def test_default_debug_goal_holds_actual_start_pose(self):
        config = load_config(DEFAULT_CONFIG)
        actual = Pose(123, 456, 7)
        frame = StartAnchoredMapFrame(
            config.nominal_start, actual, *config.ops_offset
        )
        goal = frame.to_ops_goal(config.map_goal)
        self.assertAlmostEqual(goal.x_mm, actual.x_mm)
        self.assertAlmostEqual(goal.y_mm, actual.y_mm)
        self.assertAlmostEqual(goal.yaw_deg, actual.yaw_deg)


if __name__ == "__main__":
    unittest.main()
