"""
test_tcp_framed_transport.py - unit tests for tcp_framed_transport.py's
framing protocol: TcpFrameCodec.pack() and TCP_StreamParser.Execute().

Scope: pure in-process framing/parsing logic, no real sockets - consistent
with how test_plc_mock.py avoids live network I/O (mqtt_manager_test.py is
this codebase's manual integration script for that).

Frame format: 4-byte Magic Marker (0x12345678) + 1-byte Node ID + 1-byte
Frame Type + 4-byte Length (payload size only) + payload + a trailing
4-byte copy of the Magic Marker.

- Node ID and Frame Type let a receiver route a frame (which node sent it,
  and whether it's a structured I/O-mapping payload or a data-visualization
  metric) without any ambiguity, instead of guessing from payload size.
- The trailer confirms the length field was read at a genuine frame
  boundary (not a false-positive match of the header magic occurring
  inside unrelated payload data, with a garbage length happening to follow
  it) before the extracted payload is trusted as complete.
"""
import struct
import unittest

from tcp_framed_transport import FrameType, ParsedFrame, TCP_StreamParser, TcpFrameCodec


MAGIC = TcpFrameCodec.MAGIC_MARKER_BYTES


def pack(payload: bytes, frame_type: int = FrameType.IO_MAPPING, node_id: int = 0, **kwargs) -> bytes:
    """Test-local default: most tests don't care about the specific
    frame_type/node_id, just that framing/parsing itself works - only the
    tests that are actually about those fields override them."""
    return TcpFrameCodec.pack(payload, frame_type=frame_type, node_id=node_id, **kwargs)


class TcpFrameCodecPackTest(unittest.TestCase):
    def test_pack_starts_with_magic_marker(self):
        framed = pack(b"hello")
        self.assertEqual(framed[:4], MAGIC)

    def test_pack_ends_with_magic_marker_trailer(self):
        framed = pack(b"hello")
        self.assertEqual(framed[-4:], MAGIC)

    def test_pack_node_id_and_frame_type_occupy_bytes_4_and_5(self):
        framed = pack(b"hello", frame_type=FrameType.METRIC_LAYOUT_1, node_id=7)
        self.assertEqual(framed[4], 7)
        self.assertEqual(framed[5], FrameType.METRIC_LAYOUT_1)

    def test_pack_length_field_covers_payload_only_not_the_trailer(self):
        payload = b"hello world"
        framed = pack(payload, length_endianness="BE")
        length_field = struct.unpack(">I", framed[6:10])[0]
        self.assertEqual(length_field, len(payload))

    def test_pack_total_size_is_header_plus_payload_plus_trailer(self):
        payload = b"x" * 37
        framed = pack(payload)
        self.assertEqual(
            len(framed),
            TcpFrameCodec.HEADER_SIZE + len(payload) + TcpFrameCodec.TRAILER_SIZE,
        )

    def test_pack_respects_little_endian_length_field(self):
        payload = b"abc"
        framed = pack(payload, length_endianness="LE")
        self.assertEqual(struct.unpack("<I", framed[6:10])[0], len(payload))

    def test_pack_empty_payload_still_has_header_and_trailer(self):
        framed = pack(b"")
        self.assertEqual(len(framed), TcpFrameCodec.HEADER_SIZE + TcpFrameCodec.TRAILER_SIZE)
        self.assertEqual(framed[:4], MAGIC)
        self.assertEqual(framed[-4:], MAGIC)

    def test_pack_rejects_a_node_id_that_does_not_fit_in_one_byte(self):
        with self.assertRaises(ValueError):
            TcpFrameCodec.pack(b"x", frame_type=FrameType.IO_MAPPING, node_id=256)

    def test_pack_rejects_a_negative_node_id(self):
        with self.assertRaises(ValueError):
            TcpFrameCodec.pack(b"x", frame_type=FrameType.IO_MAPPING, node_id=-1)

    def test_pack_rejects_a_frame_type_that_does_not_fit_in_one_byte(self):
        with self.assertRaises(ValueError):
            TcpFrameCodec.pack(b"x", frame_type=256, node_id=0)

    def test_pack_accepts_the_boundary_values_zero_and_255(self):
        framed = TcpFrameCodec.pack(b"x", frame_type=255, node_id=255)
        self.assertEqual(framed[4], 255)
        self.assertEqual(framed[5], 255)


def feed(parser: TCP_StreamParser, data: bytes):
    parser.Execute(data, len(data))


class StreamParserSingleFrameTest(unittest.TestCase):
    def test_parses_one_complete_frame_delivered_in_a_single_chunk(self):
        parser = TCP_StreamParser()
        framed = pack(b"payload-data")
        feed(parser, framed)
        self.assertEqual(len(parser.PendingPayloads), 1)
        self.assertEqual(parser.PendingPayloads.popleft().payload, b"payload-data")

    def test_parsed_frame_reports_the_node_id_and_frame_type_it_was_packed_with(self):
        parser = TCP_StreamParser()
        feed(parser, pack(b"data", frame_type=FrameType.METRIC_LAYOUT_1, node_id=3))
        frame = parser.PendingPayloads.popleft()
        self.assertEqual(frame, ParsedFrame(node_id=3, frame_type=FrameType.METRIC_LAYOUT_1, payload=b"data"))

    def test_payload_ready_count_increments_once_per_frame(self):
        parser = TCP_StreamParser()
        feed(parser, pack(b"a"))
        self.assertEqual(parser.PayloadReadyCount, 1)

    def test_extracted_node_id_and_frame_type_fields_mirror_the_parsed_frame(self):
        # The PLC-mirroring single-slot fields (ExtractedNodeId/
        # ExtractedFrameType), not just the Python-only PendingPayloads
        # convenience queue.
        parser = TCP_StreamParser()
        feed(parser, pack(b"data", frame_type=FrameType.METRIC_LAYOUT_1, node_id=5))
        self.assertEqual(parser.ExtractedNodeId, 5)
        self.assertEqual(parser.ExtractedFrameType, FrameType.METRIC_LAYOUT_1)

    def test_empty_payload_frame_is_still_extracted(self):
        parser = TCP_StreamParser()
        feed(parser, pack(b""))
        self.assertEqual(len(parser.PendingPayloads), 1)
        self.assertEqual(parser.PendingPayloads.popleft().payload, b"")

    def test_incomplete_frame_yields_nothing_yet(self):
        parser = TCP_StreamParser()
        framed = pack(b"payload-data")
        feed(parser, framed[:-2])  # missing the last 2 trailer bytes
        self.assertEqual(len(parser.PendingPayloads), 0)

    def test_frame_completes_once_the_rest_arrives_in_a_later_chunk(self):
        parser = TCP_StreamParser()
        framed = pack(b"payload-data")
        feed(parser, framed[:-2])
        self.assertEqual(len(parser.PendingPayloads), 0)
        feed(parser, framed[-2:])
        self.assertEqual(len(parser.PendingPayloads), 1)
        self.assertEqual(parser.PendingPayloads.popleft().payload, b"payload-data")


class StreamParserFragmentationTest(unittest.TestCase):
    def test_byte_by_byte_delivery_still_produces_the_full_payload(self):
        parser = TCP_StreamParser()
        framed = pack(b"slow-trickle")
        for b in framed:
            feed(parser, bytes([b]))
        self.assertEqual(len(parser.PendingPayloads), 1)
        self.assertEqual(parser.PendingPayloads.popleft().payload, b"slow-trickle")

    def test_split_exactly_at_header_boundary(self):
        parser = TCP_StreamParser()
        framed = pack(b"boundary-test")
        feed(parser, framed[:TcpFrameCodec.HEADER_SIZE])
        self.assertEqual(len(parser.PendingPayloads), 0)
        feed(parser, framed[TcpFrameCodec.HEADER_SIZE:])
        self.assertEqual(parser.PendingPayloads.popleft().payload, b"boundary-test")

    def test_split_exactly_at_trailer_boundary(self):
        parser = TCP_StreamParser()
        framed = pack(b"boundary-test-2")
        split_at = len(framed) - TcpFrameCodec.TRAILER_SIZE
        feed(parser, framed[:split_at])
        self.assertEqual(len(parser.PendingPayloads), 0)
        feed(parser, framed[split_at:])
        self.assertEqual(parser.PendingPayloads.popleft().payload, b"boundary-test-2")

    def test_multiple_frames_back_to_back_in_one_chunk_all_parse(self):
        parser = TCP_StreamParser()
        combined = pack(b"first") + pack(b"second") + pack(b"third")
        feed(parser, combined)
        self.assertEqual(len(parser.PendingPayloads), 3)
        self.assertEqual([f.payload for f in parser.PendingPayloads], [b"first", b"second", b"third"])

    def test_multiple_frames_with_different_node_ids_and_types_all_parse_correctly(self):
        parser = TCP_StreamParser()
        combined = (
            pack(b"io", frame_type=FrameType.IO_MAPPING, node_id=1)
            + pack(b"metric", frame_type=FrameType.METRIC_LAYOUT_1, node_id=2)
        )
        feed(parser, combined)
        frames = list(parser.PendingPayloads)
        self.assertEqual(frames[0], ParsedFrame(1, FrameType.IO_MAPPING, b"io"))
        self.assertEqual(frames[1], ParsedFrame(2, FrameType.METRIC_LAYOUT_1, b"metric"))


class StreamParserResyncTest(unittest.TestCase):
    def test_garbage_bytes_before_a_valid_frame_are_skipped(self):
        parser = TCP_StreamParser()
        garbage = b"\x00\xff\xaa\xbb\xcc"
        feed(parser, garbage + pack(b"real-payload"))
        self.assertEqual(len(parser.PendingPayloads), 1)
        self.assertEqual(parser.PendingPayloads.popleft().payload, b"real-payload")

    def test_trailer_mismatch_rejects_the_false_frame_and_finds_the_next_real_one(self):
        # A header-magic false positive: MAGIC bytes followed by a
        # node_id/frame_type/length that does NOT correspond to a real
        # frame boundary (the bytes at the position a real trailer would
        # occupy are not MAGIC). Without the trailer check this would be
        # misread as a 4-byte payload of b"\xde\xad\xbe\xef".
        bogus_header = MAGIC + bytes((0, FrameType.IO_MAPPING)) + struct.pack(">I", 4)
        bogus_frame = bogus_header + b"\xde\xad\xbe\xef" + b"\x00\x00\x00\x00"  # trailer mismatch
        real_frame = pack(b"genuine")

        parser = TCP_StreamParser()
        feed(parser, bogus_frame + real_frame)

        payloads = [f.payload for f in parser.PendingPayloads]
        self.assertNotIn(b"\xde\xad\xbe\xef", payloads, "the trailer-mismatched frame must not be accepted")
        self.assertIn(b"genuine", payloads)

    def test_magic_bytes_embedded_inside_a_real_payload_do_not_cause_a_false_split(self):
        # The payload itself contains a byte sequence identical to the
        # magic marker. A naive re-scan-from-marker approach could
        # misinterpret this as a second frame start; the length-based
        # extraction (not a second magic search) must keep it intact as
        # part of the ONE real payload it belongs to.
        payload = b"before" + MAGIC + b"after"
        parser = TCP_StreamParser()
        feed(parser, pack(payload))
        self.assertEqual(len(parser.PendingPayloads), 1)
        self.assertEqual(parser.PendingPayloads.popleft().payload, payload)

    def test_resync_after_trailer_mismatch_still_works_when_delivered_byte_by_byte(self):
        bogus_header = MAGIC + bytes((0, FrameType.IO_MAPPING)) + struct.pack(">I", 4)
        bogus_frame = bogus_header + b"\xde\xad\xbe\xef" + b"\x00\x00\x00\x00"
        real_frame = pack(b"trickled")
        combined = bogus_frame + real_frame

        parser = TCP_StreamParser()
        for b in combined:
            feed(parser, bytes([b]))

        self.assertIn(b"trickled", [f.payload for f in parser.PendingPayloads])


class StreamParserSafetyBoundsTest(unittest.TestCase):
    def test_a_too_long_declared_length_is_skipped_without_losing_the_next_frame(self):
        parser = TCP_StreamParser()
        huge_length = struct.pack(">I", parser.PAYLOAD_LENGTH + 100)
        malformed = MAGIC + bytes((0, FrameType.IO_MAPPING)) + huge_length + b"short"
        feed(parser, malformed + pack(b"right-behind-it"))          # one read: both
        self.assertEqual([f.payload for f in parser.PendingPayloads], [b"right-behind-it"])
        self.assertGreater(parser.SkippedBytes, 0)
        self.assertLessEqual(parser.BufferLength, 3)

    def test_zero_length_frames_do_not_eat_what_follows(self):
        parser = TCP_StreamParser()
        feed(parser, pack(b"") + pack(b"after-empty") + pack(b""))
        self.assertEqual([f.payload for f in parser.PendingPayloads], [b"", b"after-empty", b""])

    def test_large_reads_are_parsed_whole(self):
        parser = TCP_StreamParser()
        stream = b"".join(pack(bytes([i]) * 200) for i in range(30))          # ~6.7 kB in one read
        feed(parser, stream)
        self.assertEqual(len(parser.PendingPayloads), 30)


class ServerRepliesTest(unittest.TestCase):
    """The server answers one client (send_to) and reports per client."""

    def setUp(self):
        import socket
        import threading
        from tcp_framed_transport import TcpFramedClient, TcpFramedServer
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        self.got = {"a": [], "b": []}
        self.server_frames = []
        self.server = TcpFramedServer("127.0.0.1", port, self.server_frames.append, max_payload=64)
        self.server.start()
        self.addCleanup(self.server.stop)
        self.clients = {}
        for name in ("a", "b"):
            c = TcpFramedClient("127.0.0.1", port, self.got[name].append)
            c.start()
            self.addCleanup(c.stop)
            self.clients[name] = c
        self.wait(lambda: self.server.client_count == 2 and all(c.connected for c in self.clients.values()))
        self.threading = threading

    def wait(self, predicate, timeout=5.0):
        import time
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.01)
        self.fail("timed out")

    def test_send_to_reaches_only_the_asking_client(self):
        self.clients["a"].send(b"question", frame_type=FrameType.LAYOUT_CHECK, node_id=7)
        self.wait(lambda: self.server_frames)
        frame = self.server_frames[0]
        self.assertIsNotNone(frame.conn)
        self.assertTrue(self.server.send_to(frame.conn, b"answer", frame_type=FrameType.LAYOUT_CHECK))
        self.wait(lambda: self.got["a"])
        import time
        time.sleep(0.2)
        self.assertEqual([f.payload for f in self.got["a"]], [b"answer"])
        self.assertEqual(self.got["b"], [])                             # the other client never sees it
        info = {i["conn"]: i for i in self.server.client_info()}
        self.assertEqual(info[frame.conn]["node_id"], 7)
        self.assertEqual(info[frame.conn]["received"], 1)

    def test_payloads_over_the_limit_are_not_sent(self):
        self.assertFalse(self.server.send(b"x" * 65, frame_type=FrameType.IO_MAPPING))
        self.assertEqual(self.server.stats()["oversize"], 1)
        self.assertTrue(self.server.send(b"x" * 64, frame_type=FrameType.IO_MAPPING))


if __name__ == "__main__":
    unittest.main()
