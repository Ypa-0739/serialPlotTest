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
    command_frame,
    crc16_ccitt,
    decode_frame,
    decode_pose_event,
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

        raw_status = struct.pack("<IBiiiH", 42, PoseState.MOVING, 10, 20, 30, 0)
        self.assertEqual(PoseStatus.decode(raw_status).goal_id, 42)

    def test_response_and_events_keep_goal_identity(self):
        response = Response.decode(bytes((9, Command.SET_POSE_GOAL, ResponseStatus.OK)))
        self.assertEqual(response.request_sequence, 9)

        started = decode_pose_event(bytes((EventCode.POSE_STARTED,)) + struct.pack("<I", 42))
        stale = decode_pose_event(bytes((EventCode.POSE_STARTED,)) + struct.pack("<I", 41))
        self.assertEqual(started.goal_id, 42)
        self.assertNotEqual(stale.goal_id, started.goal_id)


if __name__ == "__main__":
    unittest.main()
