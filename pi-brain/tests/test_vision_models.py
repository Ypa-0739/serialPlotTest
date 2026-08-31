import unittest

from app.vision.models import VisionObservation
from app.vision.observation_adapter import VisionObservationGate, VisionRequirement


class _Port:
    def __init__(self, observation):
        self.observation = observation

    def latest(self, target_type=None):
        if target_type is None or self.observation.target_type == target_type:
            return self.observation
        return None


class VisionModelTests(unittest.TestCase):
    def test_observation_validates_confidence_and_coordinates(self):
        with self.assertRaises(ValueError):
            VisionObservation(1.0, 1, "material:4", 1.1, None, None, None, True, "gripper")
        with self.assertRaises(ValueError):
            VisionObservation(1.0, 1, "material:4", 0.5, float("nan"), None, None, True, "gripper")

    def test_gate_rejects_stale_or_low_confidence_observation(self):
        observation = VisionObservation(
            10.0, 7, "material:4", 0.8, None, None, None, True, "gripper"
        )
        gate = VisionObservationGate(_Port(observation), clock=lambda: 10.4)
        requirement = VisionRequirement("material:4", 0.7, 0.5, "gripper")
        self.assertEqual(gate.require(requirement), observation)
        stale = VisionObservationGate(_Port(observation), clock=lambda: 11.0)
        self.assertIsNone(stale.require(requirement))
        strict = VisionRequirement("material:4", 0.9, 0.5, "gripper")
        self.assertIsNone(gate.require(strict))


if __name__ == "__main__":
    unittest.main()
