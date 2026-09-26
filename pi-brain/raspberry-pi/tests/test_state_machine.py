"""状态机无硬件单元测试。"""

import unittest

from robot_runtime.config import RuntimeConfig
from robot_runtime.models import ActionResult, RobotState, SafetyReport
from robot_runtime.state_machine import RobotStateMachine
from robot_simulation.components import HealthyComponent, build_simulated_components


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds

    def advance(self, seconds):
        self.now += seconds


class HangingNavigator(HealthyComponent):
    def navigate_to(self, _target):
        return ActionResult.running("模拟底盘无物理动作", activity=False)

    def cancel(self):
        pass


class RetryOnceNavigator(HealthyComponent):
    def __init__(self):
        self.calls = 0

    def navigate_to(self, _target):
        self.calls += 1
        if self.calls == 1:
            return ActionResult.retryable("模拟丢线")
        return ActionResult.done("恢复后到达")

    def cancel(self):
        pass


class RobotStateMachineTests(unittest.TestCase):
    def make_machine(self, **config_overrides):
        values = {
            "loop_interval_seconds": 0.01,
            "task_code_timeout_seconds": 1.0,
            "action_timeout_seconds": 1.0,
            "inactivity_timeout_seconds": 5.0,
            "max_action_retries": 2,
        }
        values.update(config_overrides)
        config = RuntimeConfig(**values)
        components = build_simulated_components(
            "452+321+254+312",
            auto_start=False,
        )
        clock = FakeClock()
        return RobotStateMachine(components, config, clock), components, clock

    def advance_to_waiting(self, machine, clock):
        for _ in range(50):
            machine.tick()
            clock.advance(0.01)
            if machine.state is RobotState.WAITING_FOR_START:
                return
        self.fail("状态机未进入 WAITING_FOR_START")

    def press_start(self, machine, components, clock):
        machine.tick()  # 先观察到释放状态，完成按钮解锁
        clock.advance(0.01)
        components.start_button.manually_pressed = True
        machine.tick()
        clock.advance(0.01)
        self.assertEqual(machine.state, RobotState.READING_TASK_CODE)

    def test_complete_two_batch_mission(self):
        machine, components, clock = self.make_machine()
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)

        for _ in range(100):
            machine.tick()
            clock.advance(0.01)
            if machine.is_terminal:
                break

        self.assertEqual(machine.state, RobotState.COMPLETED)
        self.assertEqual(components.display.task_code, "452+321+254+312")
        self.assertTrue(components.display.final_statistics["success"])
        self.assertEqual(len(components.statistics.events), 18)
        self.assertEqual(
            [event for event in components.manipulator.events if event[0] == "pickup"],
            [
                ("pickup", 1, 4, 1),
                ("pickup", 1, 5, 2),
                ("pickup", 1, 2, 3),
                ("pickup", 2, 2, 1),
                ("pickup", 2, 5, 2),
                ("pickup", 2, 4, 3),
            ],
        )

    def test_does_not_start_until_button_release_then_press(self):
        machine, components, clock = self.make_machine()
        components.start_button.manually_pressed = True
        self.advance_to_waiting(machine, clock)

        for _ in range(5):
            machine.tick()
            clock.advance(0.01)
        self.assertEqual(machine.state, RobotState.WAITING_FOR_START)

        components.start_button.manually_pressed = False
        machine.tick()
        components.start_button.manually_pressed = True
        machine.tick()
        self.assertEqual(machine.state, RobotState.READING_TASK_CODE)

    def test_invalid_task_code_times_out_to_safe_stop(self):
        machine, components, clock = self.make_machine(
            task_code_timeout_seconds=0.25
        )
        components.task_code_reader.task_code = "452"
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)

        for _ in range(10):
            machine.tick()
            clock.advance(0.05)
            if machine.is_terminal:
                break

        self.assertEqual(machine.state, RobotState.SAFE_STOP)
        self.assertIn("合法任务码", machine.stop_reason)
        self.assertFalse(components.display.final_statistics["success"])

    def test_safety_monitor_forces_stop(self):
        machine, components, clock = self.make_machine()
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)
        components.safety.report = SafetyReport(
            safe=False,
            reason="测试急停",
            emergency_stop=True,
        )

        machine.tick()

        self.assertEqual(machine.state, RobotState.SAFE_STOP)
        self.assertEqual(machine.stop_reason, "测试急停")
        self.assertGreater(components.motion.stop_count, 0)

    def test_inactivity_timeout_forces_stop(self):
        machine, components, clock = self.make_machine(
            action_timeout_seconds=2.0,
            inactivity_timeout_seconds=0.25,
        )
        components.navigator = HangingNavigator()
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)

        for _ in range(20):
            machine.tick()
            clock.advance(0.05)
            if machine.is_terminal:
                break

        self.assertEqual(machine.state, RobotState.SAFE_STOP)
        self.assertIn("未检测到物理动作", machine.stop_reason)

    def test_retryable_action_runs_recovery_then_continues(self):
        machine, components, clock = self.make_machine()
        retrying_navigator = RetryOnceNavigator()
        components.navigator = retrying_navigator
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)

        for _ in range(10):
            machine.tick()
            clock.advance(0.01)
            if retrying_navigator.calls >= 2:
                break

        self.assertGreaterEqual(retrying_navigator.calls, 2)
        self.assertTrue(components.recovery.events)
        self.assertNotEqual(machine.state, RobotState.SAFE_STOP)


if __name__ == "__main__":
    unittest.main()
