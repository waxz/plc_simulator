"""
test_tcp_exchange.py - end-to-end PLC communication tests over real localhost
TCP sockets: the simulator side (MQTTSyncEngine + TcpFramedServer, driven by
SimulationRunner exactly as main.py --sim-thread does) against PLC-side
clients (TcpFramedClient, or a raw socket where a test needs byte-level
control).

Covers: closed-loop belt speed control (PLC setpoint in -> drive ramp ->
BELT_FEEDBACK out), publish rate with the runner thread, a stalled client
not blocking the simulation or other clients, latest-frame-wins delivery,
fragmented inbound frames, a bad inbound frame not killing the connection,
and the belt feedback channel staying opt-in (IO_MAPPING layout unchanged).
"""
import socket
import threading
import time
import unittest

from config_parser import DriveConfig, load_config
from mqtt_data_exchange import PUBLISH_TOPICS, SUBSCRIBE_TOPICS, MQTTSyncEngine
from mqtt_message_binding import ByteType, ComponentMessage, belt_feedback_mapping, custom_component_mapping
from physics import PhysicsEngine
from sim_runner import SimulationRunner
from tcp_framed_transport import (
    FrameType,
    TcpFrameCodec,
    TcpFramedClient,
    _ClientWriter,
)

BELT = "belt_1_1_1"


def make_config(port: int, belt_feedback: bool = False):
    """test_config.json has every component PUBLISH_TOPICS/SUBSCRIBE_TOPICS
    reference. Sources are cleared so no parcels spawn - these tests are
    about the wire, not the flow."""
    config = load_config("test_config.json")
    config.sources = []
    settings = config.mqttSettings
    settings.enabled = True
    settings.transport = "tcp"
    settings.tcpBindHost = "127.0.0.1"
    settings.tcpServerHost = "127.0.0.1"
    settings.tcpPort = port
    settings.beltFeedback = belt_feedback
    return config


def endian(config) -> str:
    return "<" if config.mqttSettings.endianness.upper() == "LE" else ">"


class FrameRecorder:
    """on_frame callback that keeps (receive time, ParsedFrame) per frame type."""

    def __init__(self):
        self.lock = threading.Lock()
        self.frames = {}

    def __call__(self, frame):
        with self.lock:
            self.frames.setdefault(frame.frame_type, []).append((time.perf_counter(), frame))

    def of(self, frame_type):
        with self.lock:
            return list(self.frames.get(int(frame_type), []))


def connect(port: int, config, on_frame) -> TcpFramedClient:
    client = TcpFramedClient("127.0.0.1", port, on_frame, length_endianness=config.mqttSettings.endianness)
    client.start()
    deadline = time.time() + 5.0
    while not client.connected and time.time() < deadline:
        time.sleep(0.02)
    return client


def wait_for(predicate, timeout=5.0, poll=0.01):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(poll)
    return predicate()


class ExchangeFixture(unittest.TestCase):
    """Simulator side: physics + exchange on a SimulationRunner thread."""

    port = None
    belt_feedback = False

    def setUp(self):
        self.config = make_config(self.port, belt_feedback=self.belt_feedback)
        self.lock = threading.Lock()
        self.physics = PhysicsEngine(self.config, lock=self.lock)
        self.engine = MQTTSyncEngine(self.config, lock=self.lock)
        self.runner = SimulationRunner(self.physics, self.engine, tick_s=0.002)
        self.clients = []

    def tearDown(self):
        self.runner.stop()
        for client in self.clients:
            client.stop()
        self.engine.stop()

    def plc_client(self, on_frame):
        client = connect(self.port, self.config, on_frame)
        self.clients.append(client)
        self.assertTrue(client.connected, "PLC client could not connect")
        self.assertTrue(wait_for(lambda: self.engine.stats().get("clients") == 1))
        return client

    def output_message(self) -> ComponentMessage:
        mapping = custom_component_mapping(self.config, SUBSCRIBE_TOPICS["output"]["custom_ids"], direction="in")
        return ComponentMessage(mapping, endianness=endian(self.config))

    def send_belt_setpoint(self, client, speed: float, plc_config=None):
        """Build the PLC's output frame the way a PLC would: its own copy of
        the I/O image, with one belt's speed command set."""
        plc_config = plc_config or load_config("test_config.json")
        next(b for b in plc_config.belts if b.id == BELT).speed = speed
        payload = self.output_message().serialize(plc_config)
        self.assertTrue(client.send(payload, frame_type=FrameType.IO_MAPPING, node_id=7))


class ClosedLoopBeltSpeedTest(ExchangeFixture):
    port = 19401
    belt_feedback = True

    def test_plc_setpoint_ramps_through_the_drive_and_comes_back_as_feedback(self):
        belt = next(b for b in self.config.belts if b.id == BELT)
        belt.drive = DriveConfig(accel=2.0, decel=2.0, tau=0.0, countsPerM=10000.0)
        recorder = FrameRecorder()
        client = self.plc_client(recorder)
        self.runner.start()

        t_cmd = time.perf_counter()
        self.send_belt_setpoint(client, 2.0)
        self.assertTrue(wait_for(lambda: belt.speed == 2.0), "setpoint never applied")

        decoder_config = load_config("test_config.json")
        decoder = ComponentMessage(belt_feedback_mapping(decoder_config), endianness=endian(self.config))
        self.assertTrue(wait_for(lambda: len(recorder.of(FrameType.BELT_FEEDBACK)) > 0))

        samples = []

        def decode_all():
            samples.clear()
            fb_belt = next(b for b in decoder_config.belts if b.id == BELT)
            for t, frame in recorder.of(FrameType.BELT_FEEDBACK):
                self.assertEqual(len(frame.payload), decoder.payload_size)
                decoder.deserialize(frame.payload, decoder_config)
                samples.append((t - t_cmd, fb_belt.actualSpeed, fb_belt.encoderCount))
            return samples

        self.assertTrue(wait_for(lambda: decode_all() and samples[-1][1] >= 2.0 - 1e-6, timeout=5.0),
                        "actual speed never reached the setpoint")
        time.sleep(0.5)  # half a second at constant speed for the encoder check
        decode_all()

        speeds = [s for _, s, _ in samples]
        self.assertEqual(speeds, sorted(speeds), "feedback speed must rise monotonically during the ramp")
        # 0 -> 2 m/s at 2 m/s^2 takes 1.0 s; allow for wall-clock scheduling.
        t_reached = next(t for t, s, _ in samples if s >= 2.0 - 1e-6)
        self.assertGreater(t_reached, 0.8)
        self.assertLess(t_reached, 1.5)
        # Mid-ramp samples lie on the 2 m/s^2 line (within exchange latency).
        for t, s, _ in samples:
            if 0.2 < t < 0.8:
                self.assertAlmostEqual(s, 2.0 * t, delta=0.15)
        # Steady state: encoder advances at 2 m/s * 10000 counts/m.
        steady = [(t, c) for t, s, c in samples if s >= 2.0 - 1e-6]
        (t0, c0), (t1, c1) = steady[0], steady[-1]
        self.assertGreater(t1 - t0, 0.3)
        self.assertAlmostEqual((c1 - c0) / (t1 - t0), 20000.0, delta=0.15 * 20000.0)


class PublishRateTest(ExchangeFixture):
    port = 19402

    def test_runner_thread_publishes_at_the_configured_interval(self):
        recorder = FrameRecorder()
        self.plc_client(recorder)
        self.runner.start()
        time.sleep(0.3)  # settle
        start = len(recorder.of(FrameType.IO_MAPPING))
        time.sleep(1.0)
        rate = len(recorder.of(FrameType.IO_MAPPING)) - start
        interval_ms = PUBLISH_TOPICS["input"]["interval"]
        expected = 1000.0 / interval_ms
        self.assertGreater(rate, 0.75 * expected, f"only {rate} frames/s, configured {expected:.0f}/s")
        self.assertLess(rate, 1.15 * expected)

    def test_sim_time_advances_with_wall_time(self):
        self.runner.start()
        time.sleep(1.0)
        self.runner.stop()
        self.assertAlmostEqual(self.physics.sim_time, 1.0, delta=0.15)
        self.assertEqual(self.physics.dropped_time, 0.0)


class StalledClientTest(ExchangeFixture):
    port = 19403
    belt_feedback = True

    def test_a_client_that_stops_reading_blocks_neither_sync_nor_other_clients(self):
        # Raw socket that connects and never reads, with a tiny receive window
        # so its kernel buffers fill within a second or two.
        stalled = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        stalled.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
        stalled.connect(("127.0.0.1", self.port))
        self.addCleanup(stalled.close)
        self.assertTrue(wait_for(lambda: self.engine.stats().get("clients") == 1))

        recorder = FrameRecorder()
        healthy = connect(self.port, self.config, recorder)
        self.clients.append(healthy)
        self.assertTrue(wait_for(lambda: self.engine.stats().get("clients") == 2))

        # Drive sync() directly so each call can be timed.
        worst = 0.0
        deadline = time.perf_counter() + 4.0
        while time.perf_counter() < deadline:
            t0 = time.perf_counter()
            self.engine.sync()
            worst = max(worst, time.perf_counter() - t0)
            time.sleep(0.002)
            if self.engine.stats()["dropped"] > 0 and time.perf_counter() > deadline - 2.5:
                break

        self.assertLess(worst, 0.02, f"sync() blocked for {worst * 1000:.1f} ms")
        self.assertGreater(self.engine.stats()["dropped"], 0, "stalled client never fell behind")
        # The healthy client kept getting current frames the whole time.
        frames = recorder.of(FrameType.IO_MAPPING)
        self.assertGreater(len(frames), 100)
        self.assertLess(time.perf_counter() - frames[-1][0], 0.1)

    def test_stop_returns_promptly_with_a_stalled_client(self):
        stalled = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        stalled.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
        stalled.connect(("127.0.0.1", self.port))
        self.addCleanup(stalled.close)
        self.assertTrue(wait_for(lambda: self.engine.stats().get("clients") == 1))
        for _ in range(500):
            self.engine.sync()
            time.sleep(0.002)
        t0 = time.perf_counter()
        self.engine.stop()
        self.assertLess(time.perf_counter() - t0, 3.5)


class InboundRobustnessTest(ExchangeFixture):
    port = 19404

    def test_frame_split_into_single_bytes_is_reassembled_and_applied(self):
        belt = next(b for b in self.config.belts if b.id == BELT)
        plc_config = load_config("test_config.json")
        next(b for b in plc_config.belts if b.id == BELT).speed = 1.75
        payload = self.output_message().serialize(plc_config)
        packet = TcpFrameCodec.pack(payload, frame_type=FrameType.IO_MAPPING,
                                    length_endianness=self.config.mqttSettings.endianness)

        raw = socket.create_connection(("127.0.0.1", self.port))
        self.addCleanup(raw.close)
        raw.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        raw.sendall(b"\x00\xff garbage")  # leading noise must be skipped
        for byte in packet:
            raw.sendall(bytes([byte]))
            time.sleep(0.0005)
        self.assertTrue(wait_for(lambda: belt.speed == 1.75), "fragmented frame was not applied")

    def test_a_frame_that_fails_to_apply_does_not_drop_the_connection(self):
        recorder = FrameRecorder()
        client = self.plc_client(recorder)
        # The PLC's frame layout is fixed - build it while the belt exists.
        output_message = self.output_message()
        plc_config = load_config("test_config.json")
        # A component the output mapping references disappears at runtime
        # (e.g. deleted in the editor) -> deserialize() raises.
        with self.lock:
            removed = next(b for b in self.config.belts if b.id == BELT)
            self.config.belts.remove(removed)
        next(b for b in plc_config.belts if b.id == BELT).speed = 1.0
        self.assertTrue(client.send(output_message.serialize(plc_config), frame_type=FrameType.IO_MAPPING))
        self.assertTrue(wait_for(lambda: self.engine.rx_errors == 1))
        self.assertTrue(client.connected)
        self.assertEqual(self.engine.stats()["clients"], 1)

        with self.lock:
            self.config.belts.append(removed)
        self.send_belt_setpoint(client, 1.25)
        self.assertTrue(wait_for(lambda: removed.speed == 1.25), "connection unusable after a bad frame")

    def test_unknown_frame_type_is_ignored(self):
        recorder = FrameRecorder()
        client = self.plc_client(recorder)
        self.assertTrue(client.send(b"\x01\x02\x03", frame_type=250))
        self.send_belt_setpoint(client, 0.5)
        belt = next(b for b in self.config.belts if b.id == BELT)
        self.assertTrue(wait_for(lambda: belt.speed == 0.5))
        self.assertEqual(self.engine.rx_errors, 0)


class BeltFeedbackOptInTest(unittest.TestCase):
    def test_disabled_by_default_and_io_mapping_layout_is_unchanged(self):
        sizes = {}
        for port, enabled in ((19405, False), (19406, True)):
            config = make_config(port, belt_feedback=enabled)
            engine = MQTTSyncEngine(config)
            try:
                sizes[enabled] = [(ch.frame_type, ch.message.payload_size) for ch in engine._pub_channels]
            finally:
                engine.stop()
        self.assertEqual(make_config(19407).mqttSettings.beltFeedback, False)
        self.assertNotIn(int(FrameType.BELT_FEEDBACK), [ft for ft, _ in sizes[False]])
        io_only = [entry for entry in sizes[True] if entry[0] == FrameType.IO_MAPPING]
        self.assertEqual(io_only, sizes[False])
        feedback = [size for ft, size in sizes[True] if ft == FrameType.BELT_FEEDBACK]
        config = make_config(19408)
        self.assertEqual(feedback, [len(config.belts) * (4 + 4)])

    def test_feedback_mapping_types(self):
        config = load_config("test_config.json")
        mapping = belt_feedback_mapping(config, [BELT])
        self.assertEqual(mapping, [(ByteType.FLOAT, f"{BELT}/actualSpeed"),
                                   (ByteType.UINT32, f"{BELT}/encoderCount")])

    def test_sim_clock_is_serialized_from_simulated_time(self):
        config = load_config("test_config.json")
        config.simTimeNs = 1_234_567_890
        msg = ComponentMessage([(ByteType.UINT64, "clock/simNanoseconds")], endianness="<")
        self.assertEqual(int.from_bytes(msg.serialize(config), "little"), 1_234_567_890)


class _BlockingSocket:
    """Fake socket whose sendall() blocks until released - a client whose
    TCP window is full."""

    def __init__(self):
        self.release = threading.Event()
        self.sent = []

    def sendall(self, data):
        self.release.wait(5)
        self.sent.append(data)


class ClientWriterTest(unittest.TestCase):
    def test_latest_frame_of_each_type_wins_while_the_client_is_blocked(self):
        sock = _BlockingSocket()
        writer = _ClientWriter(sock, on_error=lambda: None)
        self.addCleanup(writer.stop)
        writer.enqueue((1, 0), b"io-0")  # taken by the writer, now blocked in sendall
        time.sleep(0.05)
        for i in range(1, 6):
            writer.enqueue((1, 0), f"io-{i}".encode())
        writer.enqueue((6, 0), b"fb-1")
        sock.release.set()
        self.assertTrue(wait_for(lambda: len(sock.sent) == 3))
        self.assertEqual(sock.sent, [b"io-0", b"io-5", b"fb-1"])
        self.assertEqual(writer.dropped, 4)
        self.assertEqual(writer.sent, 3)


if __name__ == "__main__":
    unittest.main()
