# -*- coding: utf-8 -*-
"""protocol.py 单测：命令编码 + 行解析（事件覆盖见清单）。"""

import unittest

from app.protocol import (
    CanError,
    CanStatus,
    CsvTelemetry,
    Event,
    FaultReason,
    OpsStatus,
    ModeChanged,
    PidLimitSet,
    PidLoaded,
    PidStatusAll,
    Pong,
    PoseReached,
    PoseStarted,
    PoseStopped,
    PoseTelemetry,
    RawMessage,
    RoundStarted,
    RoundStopped,
    SafetyFault,
    Status,
    StopAcknowledged,
    UnknownError,
    WheelTelemetry,
    encode_command,
    encode_ping,
    encode_pose_set,
    encode_status,
    encode_stop,
    is_motion_command,
    parse_line,
)


class EncodeTests(unittest.TestCase):
    def test_encode_ping(self):
        self.assertEqual(encode_ping(), b"PING\n")

    def test_encode_stop(self):
        self.assertEqual(encode_stop(), b"STOP\n")

    def test_encode_status(self):
        self.assertEqual(encode_status(), b"STATUS\n")

    def test_encode_pose_set(self):
        self.assertEqual(encode_pose_set(100, 200, 30), b"POSE SET 100.00 200.00 30.00\n")

    def test_encode_pose_set_negative(self):
        self.assertEqual(encode_pose_set(-5.5, 0.25, -90), b"POSE SET -5.50 0.25 -90.00\n")

    def test_encode_command(self):
        self.assertEqual(encode_command("OPS STATUS"), b"OPS STATUS\n")

    def test_motion_command_classification(self):
        for command in (
            "POSE SET 100 200 0",
            "MOVE FWD 0.1 500",
            "MOVE DIAGONAL 0.1 500",
            "TURN CW 0.1 500",
            "MOTOR RUN 1 100",
            "SET P:0.003 I:0 D:0",
            "PID 0.003 0 0",
            "NAV GOTO 1 100 200 0 0.2",
            "CMD VEL 0.1 0 0",
        ):
            with self.subTest(command=command):
                self.assertTrue(is_motion_command(command))

    def test_safe_command_classification(self):
        for command in (
            "STOP",
            "POSE STOP",
            "MOVE STOP",
            "STATUS",
            "OPS STATUS",
            "CAN STATUS",
            "PID SET X 0.0033 0 0",
            "PID LIMIT X 0.2",
            "RESET",
        ):
            with self.subTest(command=command):
                self.assertFalse(is_motion_command(command))


class ParsePongTests(unittest.TestCase):
    def test_pong(self):
        self.assertIsInstance(parse_line("# PONG"), Pong)

    def test_pong_keeps_raw(self):
        self.assertEqual(parse_line("# PONG").raw, "# PONG")


class ParsePoseTests(unittest.TestCase):
    # POSE SET 确认行（固件 LLM_ProcessCommand POSE SET 分支）
    POSE_START = (
        "# POSE START X=100.00 Y=200.00 YAW=30.00 "
        "CENTER_X=98.75 CENTER_Y=201.25 TOL_MM=5.00 TOL_YAW=1.00"
    )

    def test_pose_start_parsed(self):
        ev = parse_line(self.POSE_START)
        self.assertIsInstance(ev, PoseStarted)
        self.assertAlmostEqual(ev.x, 100.0)
        self.assertAlmostEqual(ev.y, 200.0)
        self.assertAlmostEqual(ev.yaw, 30.0)
        self.assertAlmostEqual(ev.center_x, 98.75)
        self.assertAlmostEqual(ev.center_y, 201.25)
        self.assertAlmostEqual(ev.tol_mm, 5.0)
        self.assertAlmostEqual(ev.tol_yaw, 1.0)

    def test_pose_reached_parsed(self):
        line = (
            "# POSE TARGET X=100.00 Y=200.00 YAW=30.00 "
            "CENTER_X=98.75 CENTER_Y=201.25 ERROR_MM=2.10 ERROR_YAW=0.30"
        )
        ev = parse_line(line)
        self.assertIsInstance(ev, PoseReached)
        self.assertAlmostEqual(ev.error_mm, 2.1)
        self.assertAlmostEqual(ev.error_yaw, 0.3)
        self.assertAlmostEqual(ev.x, 100.0)

    def test_pose_stopped(self):
        self.assertIsInstance(parse_line("# POSE STOP"), PoseStopped)


class ParseSafetyTests(unittest.TestCase):
    """每种安全字符串都必须解析成 SafetyFault 且 reason 正确。"""

    SAFETY_LINES = {
        "# ROUND STOP OPS LOST AXIS=Y X=1.00 Y=2.00 YAW=3.00 CENTER_X=1.00 CENTER_Y=2.00": FaultReason.OPS_LOST,
        "# ROUND STOP HOST LOST AXIS=Y X=1.00 Y=2.00 YAW=3.00 CENTER_X=1.00 CENTER_Y=2.00": FaultReason.HOST_LOST,
        "# ROUND STOP OVERTRAVEL AXIS=X X=1.00 Y=2.00 YAW=3.00 CENTER_X=1.00 CENTER_Y=2.00": FaultReason.OVERTRAVEL,
        "# ROUND STOP CROSS TRACK AXIS=X X=1.00 Y=2.00 YAW=3.00 CENTER_X=1.00 CENTER_Y=2.00": FaultReason.CROSS_TRACK,
        "# ROUND STOP TRANSLATION LIMIT AXIS=YAW X=1.00 Y=2.00 YAW=3.00 CENTER_X=1.00 CENTER_Y=2.00": FaultReason.TRANSLATION_LIMIT,
        "# ROUND STOP WRONG DIR AXIS=Y X=1.00 Y=2.00 YAW=3.00 CENTER_X=1.00 CENTER_Y=2.00": FaultReason.WRONG_DIR,
        "# ROUND STOP YAW LIMIT AXIS=X X=1.00 Y=2.00 YAW=3.00 CENTER_X=1.00 CENTER_Y=2.00": FaultReason.YAW_LIMIT,
        "# ROUND STOP CAN FAULT AXIS=Y X=1.00 Y=2.00 YAW=3.00 CENTER_X=1.00 CENTER_Y=2.00": FaultReason.CAN_FAULT,
        "# POSE STOP SAFETY": FaultReason.POSE_SAFETY_STOP,
        "# MOTION STOP SAFETY TYPE=SINGLE_MOTOR REASON=HOST LOST": FaultReason.HOST_LOST,
        "# MOTION STOP SAFETY TYPE=DEBUG_CHASSIS REASON=CAN FAULT": FaultReason.CAN_FAULT,
    }

    def test_all_safety_lines(self):
        for line, expected_reason in self.SAFETY_LINES.items():
            with self.subTest(reason=expected_reason.value):
                ev = parse_line(line)
                self.assertIsInstance(ev, SafetyFault)
                self.assertEqual(ev.reason, expected_reason)
                self.assertEqual(ev.raw, line)  # 原始行必须保留

    def test_safety_fault_is_event(self):
        self.assertTrue(issubclass(SafetyFault, Event))


class ParseRoundTests(unittest.TestCase):
    def test_round_start(self):
        line = "# ROUND START 3 AXIS=Y DIR -1 X=10.00 Y=20.00 YAW=30.00 CENTER_X=10.00 CENTER_Y=20.00"
        ev = parse_line(line)
        self.assertIsInstance(ev, RoundStarted)
        self.assertEqual(ev.round_no, 3)
        self.assertEqual(ev.axis, "Y")
        self.assertEqual(ev.direction, -1.0)
        self.assertAlmostEqual(ev.x, 10.0)

    def test_round_stop_target_normal(self):
        line = "# ROUND STOP TARGET AXIS=Y X=10.00 Y=20.00 YAW=30.00 CENTER_X=10.00 CENTER_Y=20.00"
        ev = parse_line(line)
        self.assertIsInstance(ev, RoundStopped)
        self.assertEqual(ev.reason, "TARGET")
        self.assertNotIsInstance(ev, SafetyFault)

    def test_round_stop_timeout_normal(self):
        line = "# ROUND STOP TIMEOUT AXIS=Y X=10.00 Y=20.00 YAW=30.00 CENTER_X=10.00 CENTER_Y=20.00"
        ev = parse_line(line)
        self.assertIsInstance(ev, RoundStopped)
        self.assertEqual(ev.reason, "TIMEOUT")

    def test_round_stop_host_user_stop(self):
        line = "# ROUND STOP HOST AXIS=Y X=10.00 Y=20.00 YAW=30.00 CENTER_X=10.00 CENTER_Y=20.00"
        ev = parse_line(line)
        self.assertIsInstance(ev, RoundStopped)
        self.assertEqual(ev.reason, "HOST")

    def test_round_stop_unknown_reason_fallback(self):
        line = "# ROUND STOP WEIRD AXIS=Y X=10.00 Y=20.00 YAW=30.00 CENTER_X=10.00 CENTER_Y=20.00"
        ev = parse_line(line)
        self.assertIsInstance(ev, RoundStopped)
        self.assertEqual(ev.reason, "UNKNOWN")


class ParseStatusTests(unittest.TestCase):
    def test_status_parsed(self):
        line = (
            "# STATUS MODE=WORK HOST_PROTO=2 AXIS=Y P=0.0033000 I=0.00000000 "
            "D=0.0000000 MAX_OUT=0.150 STATE=0 PLOT=0 MOTOR_PROTO=EMM "
            "OPS_FRAMES=1234 UART_TX_OK=5 UART_TX_ERR=0"
        )
        ev = parse_line(line)
        self.assertIsInstance(ev, Status)
        self.assertEqual(ev.mode, "WORK")
        self.assertEqual(ev.host_proto, 2)
        self.assertEqual(ev.axis, "Y")
        self.assertAlmostEqual(ev.kp, 0.0033)
        self.assertEqual(ev.ki, 0.0)
        self.assertAlmostEqual(ev.max_out, 0.15)
        self.assertEqual(ev.ops_frames, 1234)
        self.assertEqual(ev.uart_tx_ok, 5)
        self.assertEqual(ev.uart_tx_err, 0)


class ParseStartupAckTests(unittest.TestCase):
    def test_stop_ack(self):
        ev = parse_line("# STOP MODE=WORK")
        self.assertIsInstance(ev, StopAcknowledged)
        self.assertEqual(ev.mode, "WORK")

    def test_mode_changed(self):
        ev = parse_line("# MODE WORK PLOT=0 CHANGED=1 MOTION=STOPPED")
        self.assertIsInstance(ev, ModeChanged)
        self.assertEqual(ev.mode, "WORK")
        self.assertEqual(ev.plot, 0)
        self.assertEqual(ev.changed, 1)

    def test_pid_loaded(self):
        ev = parse_line("# PID LOADED AXIS=X P=0.0033000 I=0.00000000 D=0.0000000")
        self.assertIsInstance(ev, PidLoaded)
        self.assertEqual(ev.axis, "X")
        self.assertAlmostEqual(ev.kp, 0.0033)

    def test_pid_limit_set(self):
        ev = parse_line("# PID LIMIT AXIS=YAW OUTPUT=0.250 UNIT=RADPS")
        self.assertIsInstance(ev, PidLimitSet)
        self.assertEqual(ev.axis, "YAW")
        self.assertAlmostEqual(ev.output, 0.25)

    def test_pid_status_all(self):
        ev = parse_line(
            "# PID ALL X=0.0033000,0.00000000,0.0000000 "
            "Y=0.0033000,0.00000000,0.0000000 "
            "YAW=0.0200000,0.00000000,0.0000000"
        )
        self.assertIsInstance(ev, PidStatusAll)
        self.assertEqual(ev.x, (0.0033, 0.0, 0.0))
        self.assertEqual(ev.yaw, (0.02, 0.0, 0.0))


class ParseOpsTests(unittest.TestCase):
    def test_ops_status(self):
        line = (
            "# OPS LINK=OK X=100.00 Y=200.00 YAW=30.00 CENTER_X=98.75 CENTER_Y=201.25 "
            "OFFSET_X=0.00 OFFSET_Y=25.00 BYTES=100 HEADERS=10 FRAMES=10 INVALID=0 "
            "FORMAT_ERR=0 UART_ERR=0 BYTE_AGE=5 FRAME_AGE=5 LAST=0xAA"
        )
        ev = parse_line(line)
        self.assertIsInstance(ev, OpsStatus)
        self.assertEqual(ev.link, "OK")
        self.assertAlmostEqual(ev.x, 100.0)
        self.assertAlmostEqual(ev.y, 200.0)
        self.assertEqual(ev.frames, 10)

    def test_ops_status_stale_link(self):
        ev = parse_line("# OPS LINK=STALE X=0.00 Y=0.00 YAW=0.00 CENTER_X=0.00 CENTER_Y=0.00")
        self.assertIsInstance(ev, OpsStatus)
        self.assertEqual(ev.link, "STALE")


class ParseCanTests(unittest.TestCase):
    def test_can_ok(self):
        line = "# CAN STATE=3 ERROR=0x00000000 FREE=3 TX_OK=100 TX_ERR=0 RX=50 LAST=0"
        ev = parse_line(line)
        self.assertIsInstance(ev, CanStatus)
        self.assertEqual(ev.state, 3)
        self.assertEqual(ev.error, 0)
        self.assertEqual(ev.tx_ok, 100)

    def test_can_error(self):
        line = "# CAN STATE=3 ERROR=0x00000004 FREE=3 TX_OK=100 TX_ERR=2 RX=50 LAST=0"
        ev = parse_line(line)
        self.assertIsInstance(ev, CanError)


class ParseCsvTests(unittest.TestCase):
    CSV_LINE = (
        "120,200.00,45.12,0.0430,154.88,0.0033000,0.00000000,0.0000000,"
        "245.10,300.25,30.10,5.20,-0.30,0.0120,0.0030,243.80,298.90"
    )

    def test_csv_parsed(self):
        ev = parse_line(self.CSV_LINE)
        self.assertIsInstance(ev, CsvTelemetry)
        self.assertAlmostEqual(ev.timestamp, 120.0)
        self.assertAlmostEqual(ev.setpoint, 200.0)
        self.assertAlmostEqual(ev.input, 45.12)
        self.assertAlmostEqual(ev.output, 0.043)
        self.assertAlmostEqual(ev.error, 154.88)
        self.assertAlmostEqual(ev.kp, 0.0033)
        self.assertAlmostEqual(ev.yaw, 30.1)
        self.assertAlmostEqual(ev.cross, 5.2)
        self.assertAlmostEqual(ev.center_y, 298.9)

    def test_short_csv_falls_back_to_raw(self):
        ev = parse_line("120,200.00,45.12")
        self.assertIsInstance(ev, RawMessage)


class ParseTaggedTelemetryTests(unittest.TestCase):
    def test_wheel_group_has_four_targets_and_four_feedback_values(self):
        ev = parse_line("@W,1,1234,9,10,20,30,40,11,19,31,39")
        self.assertIsInstance(ev, WheelTelemetry)
        self.assertEqual(ev.tick_ms, 1234)
        self.assertEqual(ev.sequence, 9)
        self.assertEqual(ev.target_rpm, (10.0, 20.0, 30.0, 40.0))
        self.assertEqual(ev.actual_rpm, (11.0, 19.0, 31.0, 39.0))

    def test_pose_group_has_eight_channels(self):
        ev = parse_line("@P,1,1250,4,1,2,3,4,5,0.1,0.2,0.3")
        self.assertIsInstance(ev, PoseTelemetry)
        self.assertEqual(ev.tick_ms, 1250)
        self.assertAlmostEqual(ev.center_y_mm, 5.0)
        self.assertAlmostEqual(ev.plan_vz_radps, 0.3)

    def test_unknown_version_or_non_finite_value_is_raw(self):
        self.assertIsInstance(
            parse_line("@W,2,1,1,1,2,3,4,5,6,7,8"), RawMessage
        )
        self.assertIsInstance(
            parse_line("@P,1,1,1,1,2,nan,4,5,6,7,8"), RawMessage
        )


class ParseMiscTests(unittest.TestCase):
    def test_empty_line_none(self):
        self.assertIsNone(parse_line(""))
        self.assertIsNone(parse_line("  \r\n"))

    def test_unknown_hash_line_raw(self):
        ev = parse_line("# SOME FUTURE FEATURE=1")
        self.assertIsInstance(ev, RawMessage)
        self.assertEqual(ev.raw, "# SOME FUTURE FEATURE=1")

    def test_unknown_error(self):
        ev = parse_line("# ERROR POSE VALUE")
        self.assertIsInstance(ev, UnknownError)
        self.assertEqual(ev.text, "POSE VALUE")

    def test_can_error_message(self):
        ev = parse_line("# ERROR CAN TX")
        self.assertIsInstance(ev, CanError)


if __name__ == "__main__":
    unittest.main()
