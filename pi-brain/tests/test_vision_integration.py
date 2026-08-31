import unittest

from app.models import PoseGoal
from app.vision.models import VisionObservation
from app.vision.observation_adapter import VisionObservationGate, VisionRequirement
from app.protocol import ModeChanged, SafetyFault
from app.serial_bridge import SerialDisconnected
from app.vision_integration import VisionGoalRule, VisionGoalSelector, VisionStateGuard


class _Port:
    def __init__(self, observation):
        self.observation = observation

    def latest(self, target_type=None):
        return self.observation if self.observation.target_type == target_type else None


class _Cache:
    def __init__(self):
        self.clears = 0

    def clear(self):
        self.clears += 1


class VisionIntegrationTests(unittest.TestCase):
    def test_selector_returns_only_precalibrated_local_goal(self):
        observation = VisionObservation(
            5.0, 3, "material:4", 0.9, None, None, None, True, "gripper"
        )
        gate = VisionObservationGate(_Port(observation), clock=lambda: 5.1)
        goal = PoseGoal("green_pick", 800, 300, 0)
        selector = VisionGoalSelector(
            gate,
            {goal.name: goal},
            {
                "pick_green": VisionGoalRule(
                    VisionRequirement("material:4", 0.8, 0.5, "gripper"),
                    goal.name,
                )
            },
        )
        self.assertIs(selector.select("pick_green"), goal)

    def test_selector_rejects_stale_observation_without_fallback(self):
        observation = VisionObservation(
            5.0, 3, "material:4", 0.9, None, None, None, True, "gripper"
        )
        selector = VisionGoalSelector(
            VisionObservationGate(_Port(observation), clock=lambda: 6.0),
            {"green_pick": PoseGoal("green_pick", 800, 300, 0)},
            {
                "pick_green": VisionGoalRule(
                    VisionRequirement("material:4", 0.8, 0.5, "gripper"),
                    "green_pick",
                )
            },
        )
        self.assertIsNone(selector.select("pick_green"))

    def test_state_guard_clears_on_control_boundaries(self):
        cache = _Cache()
        guard = VisionStateGuard(cache)
        guard.handle_event(ModeChanged(mode="WORK"))
        guard.handle_event(SerialDisconnected())
        guard.handle_event(SafetyFault())
        guard.set_control_ready(False)
        self.assertEqual(cache.clears, 4)
        guard.set_control_ready(True)
        self.assertEqual(cache.clears, 4)


if __name__ == "__main__":
    unittest.main()
