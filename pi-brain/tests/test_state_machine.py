# -*- coding: utf-8 -*-

import unittest

from app.models import Pose
from app.protocol import SafetyFault, FaultReason, parse_line
from app.serial_bridge import SerialConnected, SerialDisconnected
from app.state_machine import StartupConfig, StartupState, StartupStateMachine


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeBridge:
    def __init__(self, connected=True):
        self.is_connected = connected
        self.commands = []
        self.is_emergency_stopped = False
        self.release_count = 0
        self.binary_enabled = False

    def send(self, command, priority=None):
        self.commands.append(command)

    def emergency_stop(self):
        self.is_emergency_stopped = True
        self.commands.clear()
        self.commands.append("STOP")

    def release_emergency_stop(self):
        self.is_emergency_stopped = False
        self.release_count += 1

    def enable_binary_mode(self, linear_mps=0.20, yaw_radps=0.25):
        self.binary_enabled = True


GOOD_LINES = (
    "# HOST LINK RPI OK",
    "# STOP MODE=WORK",
    "# MODE WORK PLOT=0 CHANGED=0",
    "# STATUS MODE=WORK HOST_PROTO=4 HOST=RPI AXIS=Y P=0.0010000 I=0.00000000 "
    "D=0.0000000 MAX_OUT=0.150 STATE=0 PLOT=0 MOTOR_PROTO=EMM "
    "OPS_FRAMES=10 UART_TX_OK=5 UART_TX_ERR=0",
    "# OPS LINK=OK X=1.00 Y=2.00 YAW=3.00 CENTER_X=1.00 CENTER_Y=2.00 FRAMES=10",
    "# CAN STATE=2 ERROR=0x00000000 FREE=3 TX_OK=1 TX_ERR=0 RX=1 LAST=0",
    "# PID LOADED AXIS=X P=0.0033000 I=0.00000000 D=0.0000000",
    "# PID LOADED AXIS=Y P=0.0033000 I=0.00000000 D=0.0000000",
    "# PID LOADED AXIS=YAW P=0.0200000 I=0.00000000 D=0.0000000",
    "# PID LIMIT AXIS=X OUTPUT=0.200 UNIT=MPS",
    "# PID LIMIT AXIS=Y OUTPUT=0.200 UNIT=MPS",
    "# PID LIMIT AXIS=YAW OUTPUT=0.250 UNIT=RADPS",
    "# PID ALL X=0.0033000,0.00000000,0.0000000 "
    "Y=0.0033000,0.00000000,0.0000000 "
    "YAW=0.0200000,0.00000000,0.0000000",
    "# HOST BINARY READY VERSION=2 CAPS=0x0000003F",
)


class StartupStateMachineTests(unittest.TestCase):
    def make_machine(self, connected=True, timeout=1.5):
        bridge = FakeBridge(connected=connected)
        clock = FakeClock()
        machine = StartupStateMachine(
            bridge, StartupConfig(step_timeout=timeout), clock=clock
        )
        return machine, bridge, clock

    def drive_happy_path(self, machine):
        for line in GOOD_LINES:
            machine.handle_event(parse_line(line))

    def test_happy_path_command_order_and_release(self):
        machine, bridge, _ = self.make_machine()
        machine.start()
        self.assertTrue(bridge.is_emergency_stopped)
        self.drive_happy_path(machine)
        self.assertEqual(machine.state, StartupState.READY)
        self.assertEqual(machine.ops_pose, Pose(1.0, 2.0, 3.0))
        self.assertFalse(bridge.is_emergency_stopped)
        self.assertEqual(bridge.release_count, 1)
        self.assertTrue(bridge.binary_enabled)
        self.assertEqual(
            bridge.commands,
            [
                "STOP",
                "HOST LINK RPI",
                "STOP",
                "MODE WORK",
                "STATUS",
                "OPS STATUS",
                "CAN STATUS",
                "PID SET X 0.0033 0 0",
                "PID SET Y 0.0033 0 0",
                "PID SET YAW 0.02 0 0",
                "PID LIMIT X 0.20",
                "PID LIMIT Y 0.20",
                "PID LIMIT YAW 0.25",
                "PID STATUS ALL",
                "HOST BINARY START",
            ],
        )

    def test_waits_for_connection_then_starts_with_stop(self):
        machine, bridge, _ = self.make_machine(connected=False)
        machine.start()
        self.assertEqual(machine.state, StartupState.WAIT_CONNECTION)
        bridge.is_connected = True
        machine.handle_event(SerialConnected())
        self.assertEqual(machine.state, StartupState.WAIT_HOST_LINK)
        self.assertEqual(bridge.commands, ["STOP", "HOST LINK RPI"])
        machine.handle_event(parse_line("# HOST LINK RPI OK"))
        self.assertEqual(machine.state, StartupState.WAIT_STOP)
        self.assertEqual(bridge.commands[-1], "STOP")

    def test_tune_round_host_stop_is_valid_stop_ack(self):
        machine, bridge, _ = self.make_machine()
        machine.start()
        machine.handle_event(parse_line("# HOST LINK RPI OK"))
        machine.handle_event(
            parse_line("# ROUND STOP HOST AXIS=X X=0 Y=0 YAW=0 CENTER_X=0 CENTER_Y=0")
        )
        self.assertEqual(machine.state, StartupState.WAIT_MODE_WORK)
        self.assertEqual(bridge.commands[-1], "MODE WORK")

    def test_step_timeout_faults_and_keeps_latch(self):
        machine, bridge, clock = self.make_machine(timeout=0.5)
        machine.start()
        clock.advance(0.51)
        machine.tick()
        self.assertEqual(machine.state, StartupState.FAULT)
        self.assertTrue(bridge.is_emergency_stopped)
        self.assertIn("timeout", machine.fault_reason)

    def test_ops_stale_faults(self):
        machine, bridge, _ = self.make_machine()
        machine.start()
        for line in GOOD_LINES[:4]:
            machine.handle_event(parse_line(line))
        machine.handle_event(parse_line("# OPS LINK=STALE X=0 Y=0 YAW=0 FRAMES=10"))
        self.assertEqual(machine.state, StartupState.FAULT)
        self.assertTrue(bridge.is_emergency_stopped)

    def test_mode_change_after_ready_invalidates_ops_anchor(self):
        machine, bridge, _ = self.make_machine()
        machine.start()
        self.drive_happy_path(machine)
        machine.handle_event(parse_line("# MODE PLOT PLOT=1 CHANGED=1"))
        self.assertEqual(machine.state, StartupState.FAULT)
        self.assertIsNone(machine.ops_pose)
        self.assertTrue(bridge.is_emergency_stopped)

    def test_can_error_faults(self):
        machine, _, _ = self.make_machine()
        machine.start()
        for line in GOOD_LINES[:5]:
            machine.handle_event(parse_line(line))
        machine.handle_event(parse_line("# CAN STATE=3 ERROR=0x00000004 TX_OK=1 TX_ERR=1"))
        self.assertEqual(machine.state, StartupState.FAULT)
        self.assertIn("CAN", machine.fault_reason)

    def test_can_sleep_state_faults_even_without_error_bits(self):
        machine, _, _ = self.make_machine()
        machine.start()
        for line in GOOD_LINES[:5]:
            machine.handle_event(parse_line(line))
        machine.handle_event(parse_line("# CAN STATE=3 ERROR=0x00000000 TX_OK=1 TX_ERR=0"))
        self.assertEqual(machine.state, StartupState.FAULT)
        self.assertIn("not listening", machine.fault_reason)

    def test_pid_ack_mismatch_faults(self):
        machine, bridge, _ = self.make_machine()
        machine.start()
        for line in GOOD_LINES[:6]:
            machine.handle_event(parse_line(line))
        machine.handle_event(
            parse_line("# PID LOADED AXIS=X P=0.0040000 I=0.00000000 D=0.0000000")
        )
        self.assertEqual(machine.state, StartupState.FAULT)
        self.assertTrue(bridge.is_emergency_stopped)

    def test_disconnect_faults_and_reconnect_restarts(self):
        machine, bridge, _ = self.make_machine()
        machine.start()
        bridge.is_connected = False
        machine.handle_event(SerialDisconnected())
        self.assertEqual(machine.state, StartupState.FAULT)
        bridge.is_connected = True
        machine.handle_event(SerialConnected())
        self.assertEqual(machine.state, StartupState.WAIT_HOST_LINK)
        self.assertEqual(bridge.commands, ["STOP", "HOST LINK RPI"])

    def test_safety_fault_from_any_step_latches_stop(self):
        machine, bridge, _ = self.make_machine()
        machine.start()
        machine.handle_event(
            SafetyFault(reason=FaultReason.OPS_LOST, raw="# ROUND STOP OPS LOST")
        )
        self.assertEqual(machine.state, StartupState.FAULT)
        self.assertTrue(bridge.is_emergency_stopped)

    def test_binary_capability_mismatch_stays_faulted(self):
        machine, bridge, _ = self.make_machine()
        machine.start()
        for line in GOOD_LINES[:-1]:
            machine.handle_event(parse_line(line))
        machine.handle_event(
            parse_line("# HOST BINARY READY VERSION=2 CAPS=0x0000001F")
        )
        self.assertEqual(machine.state, StartupState.FAULT)
        self.assertIn("capabilities", machine.fault_reason)
        self.assertTrue(bridge.is_emergency_stopped)

    def test_binary_wire_version_mismatch_stays_faulted(self):
        machine, bridge, _ = self.make_machine()
        machine.start()
        for line in GOOD_LINES[:-1]:
            machine.handle_event(parse_line(line))
        machine.handle_event(
            parse_line("# HOST BINARY READY VERSION=1 CAPS=0x0000003F")
        )
        self.assertEqual(machine.state, StartupState.FAULT)
        self.assertIn("unsupported", machine.fault_reason)
        self.assertTrue(bridge.is_emergency_stopped)


if __name__ == "__main__":
    unittest.main()
