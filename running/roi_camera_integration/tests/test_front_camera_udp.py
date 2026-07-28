import struct
import unittest

from roi_camera_integration.front_camera_udp import (
    FrontCameraUdpReceiver,
    MoraiCameraFrameAssembler,
)


def _packet(sec, nsec, index, payload, tail):
    return (
        b"MOR"
        + struct.pack("<4i", sec, nsec, index, len(payload))
        + payload
        + tail
    )


class MoraiCameraFrameAssemblerTests(unittest.TestCase):
    def test_reassembles_single_packet_jpeg(self):
        assembler = MoraiCameraFrameAssembler()
        frame = assembler.feed(
            _packet(10, 20, 0, b"\xff\xd8jpeg\xff\xd9", b"EI"),
            now_monotonic=1.0,
        )

        self.assertIsNotNone(frame)
        self.assertEqual(frame.jpeg, b"\xff\xd8jpeg\xff\xd9")
        self.assertEqual(frame.fragment_count, 1)
        self.assertEqual(frame.timestamp, 10.00000002)

    def test_reassembles_out_of_order_fragments(self):
        assembler = MoraiCameraFrameAssembler()
        chunks = [b"\xff\xd8abc", b"def", b"ghi\xff\xd9"]

        self.assertIsNone(assembler.feed(_packet(2, 3, 2, chunks[2], b"EI"), 2.0))
        self.assertIsNone(assembler.feed(_packet(2, 3, 0, chunks[0], b"AI"), 2.0))
        frame = assembler.feed(_packet(2, 3, 1, chunks[1], b"AI"), 2.0)

        self.assertIsNotNone(frame)
        self.assertEqual(frame.jpeg, b"\xff\xd8abcdefghi\xff\xd9")
        self.assertEqual(frame.fragment_count, 3)

    def test_missing_fragment_expires(self):
        assembler = MoraiCameraFrameAssembler(frame_timeout_sec=0.25)
        self.assertIsNone(assembler.feed(_packet(3, 0, 0, b"\xff\xd8a", b"AI"), 0.0))
        self.assertIsNone(assembler.feed(_packet(3, 0, 2, b"b\xff\xd9", b"EI"), 0.0))

        # Any later packet triggers expiration of the incomplete frame.
        assembler.feed(b"bad", 0.3)
        self.assertEqual(assembler.dropped_frame_count, 1)
        self.assertEqual(assembler.expired_frame_count, 1)

    def test_invalid_header_is_rejected(self):
        assembler = MoraiCameraFrameAssembler()
        self.assertIsNone(assembler.feed(b"BAD", 0.0))
        self.assertEqual(assembler.invalid_packet_count, 1)
        self.assertEqual(assembler.invalid_reason_counts, {"short_packet": 1})

    def test_non_jpeg_payload_is_dropped(self):
        assembler = MoraiCameraFrameAssembler()
        self.assertIsNone(assembler.feed(_packet(4, 0, 0, b"not-jpeg", b"EI"), 0.0))
        self.assertEqual(assembler.dropped_frame_count, 1)
        self.assertEqual(assembler.non_jpeg_frame_count, 1)

    def test_receiver_counts_datagrams_and_pending_frames(self):
        receiver = FrontCameraUdpReceiver(port=0)
        try:
            receiver.feed_datagram(_packet(5, 0, 0, b"\xff\xd8a", b"AI"), now_monotonic=0.0)
            self.assertEqual(receiver.received_datagram_count, 1)
            self.assertEqual(receiver.assembler.pending_frame_count, 1)
        finally:
            receiver.close()


if __name__ == "__main__":
    unittest.main()
