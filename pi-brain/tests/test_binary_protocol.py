import struct
import unittest

from app.binary_protocol import (
    Command,
    EventCode,
    Frame,
    FrameDecoder,
    MessageType,
    PoseGoal,
    PoseState,
    PoseStatus,
    Response,
    ResponseStatus,
    TelemetryType,
    command_frame,
    crc16_ccitt,
    decode_frame,
    decode_pose_event,
    decode_telemetry,
    pose_goal_with_limits_data,
    speed_limits_data,
)


class BinaryProtocolTests(unittest.TestCase):
    def test_crc_standard_vector(self):
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_frame_round_trip(self):
        wire = command_frame(7, Command.PING)
        self.assertEqual(
            decode_frame(wire),
            Frame(MessageType.COMMAND, 7, bytes((Command.PING,))),
        )

    def test_stream_recovers_after_noise_and_bad_crc(self):
        bad = bytearray(command_frame(1, Command.PING))
        bad[-1] ^= 0x80
        good = command_frame(2, Command.QUERY_POSE_GOAL)
        decoder = FrameDecoder()
        frames = []
        wire = b"noise" + bytes(bad) + good
        for offset in range(0, len(wire), 3):
            frames.extend(decoder.feed(wire[offset : offset + 3]))
        self.assertEqual(frames, [decode_frame(good)])
        self.assertEqual(decoder.crc_errors, 1)

    def test_pose_goal_and_status_layouts(self):
        goal = PoseGoal(42, 1200, -300, 1571, 35_000)
        payload = goal.command_payload()
        self.assertEqual(payload[0], Command.SET_POSE_GOAL)
        self.assertEqual(struct.unpack("<IiiiI", payload[1:]), (42, 1200, -300, 1571, 35_000))

        raw_status = struct.pack("<IBiiiHBB", 42, PoseState.MOVING, 10, 20, 30, 0, 0, 2)
        self.assertEqual(PoseStatus.decode(raw_status).goal_id, 42)

    def test_response_and_events_keep_goal_identity(self):
        response = Response.decode(bytes((9, Command.SET_POSE_GOAL, ResponseStatus.OK)))
        self.assertEqual(response.request_sequence, 9)

        started = decode_pose_event(bytes((EventCode.POSE_STARTED,)) + struct.pack("<I", 42))
        stale = decode_pose_event(bytes((EventCode.POSE_STARTED,)) + struct.pack("<I", 41))
        self.assertEqual(started.goal_id, 42)
        self.assertNotEqual(stale.goal_id, started.goal_id)

    def test_speed_limits_layout_and_bounds(self):
        self.assertEqual(struct.unpack("<ii", speed_limits_data(0.25, 0.12)), (250000, 120000))
        with self.assertRaises(ValueError):
            speed_limits_data(0.31, 0.12)

    def test_atomic_goal_with_limits_layout(self):
        goal = PoseGoal(7, 100, -200, 300, 4000)
        data = pose_goal_with_limits_data(goal, 0.25, 0.12)
        self.assertEqual(len(data), 28)
        self.assertEqual(struct.unpack_from("<IiiiI", data), (7, 100, -200, 300, 4000))
        self.assertEqual(struct.unpack_from("<ii", data, 20), (250000, 120000))

    def test_binary_telemetry_layouts(self):
        wheel = bytes((TelemetryType.WHEEL,)) + struct.pack(
            "<IH8i", 123, 9, 1, 2, 3, 4, 5, 6, 7, 8
        )
        sample = decode_telemetry(wheel)
        self.assertEqual(sample.sequence, 9)
        self.assertEqual(sample.actual_rpm_tenths, (5, 6, 7, 8))

        stats = bytes((TelemetryType.LINK_STATS,)) + struct.pack(
            "<6I", 500, 1, 2, 3, 4, 5
        )
        self.assertEqual(decode_telemetry(stats).telemetry_replaced, 3)


if __name__ == "__main__":
    unittest.main()
