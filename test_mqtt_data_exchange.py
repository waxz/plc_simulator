"""
test_mqtt_data_exchange.py - integration tests for MQTTSyncEngine's
metrics channels: pre-declared binary METRICS_MAPPING* payloads route
into a MetricsStore over the SAME TcpFramedServer/port the rest of the
exchange already uses - no separate server - disambiguated from the
"output" channel's payload (and from EACH OTHER, since different nodes
can report differently-shaped metric data) by the frame's explicit
FrameType header byte (see FrameType/ParsedFrame in
tcp_framed_transport.py and METRICS_CHANNELS in mqtt_data_exchange.py),
not by payload size, so channels whose payload sizes happen to collide
still route correctly. The header's Node ID byte is threaded through into
MetricsStore's group naming (`"perf"` -> `"node0/perf"`), so multiple
physical nodes reporting metrics don't collide with each other either.
"""
import time
import unittest

from config_parser import load_config
from metric_layout import layout_mapping
from metrics_store import MetricsStore
from mqtt_data_exchange import METRICS_CHANNELS, MetricLayout1, MetricLayout2, MQTTSyncEngine
from mqtt_message_binding import MetricsMessage

# The wire (ByteType, "group/name") lists, derived from the layout
# dataclasses - MetricsMessage still accepts a plain list directly, which
# is what these tests exercise.
METRICS_MAPPING = layout_mapping(MetricLayout1)
METRICS_MAPPING_2 = layout_mapping(MetricLayout2)
from tcp_framed_transport import FrameType, TcpFramedClient


def make_tcp_config(port: int):
    """test_config.json already has every component PUBLISH_TOPICS/
    SUBSCRIBE_TOPICS reference (MQTTSyncEngine.__init__ raises via
    verify_component_list() otherwise) - reused here rather than hand-
    building a matching config from scratch."""
    config = load_config("test_config.json")
    config.mqttSettings.enabled = True
    config.mqttSettings.transport = "tcp"
    config.mqttSettings.tcpBindHost = "127.0.0.1"
    config.mqttSettings.tcpServerHost = "127.0.0.1"
    config.mqttSettings.tcpPort = port
    return config


def connect_client(port: int, endianness: str) -> TcpFramedClient:
    client = TcpFramedClient("127.0.0.1", port, lambda frame: None, length_endianness=endianness)
    client.start()
    deadline = time.time() + 5.0
    while not client.connected and time.time() < deadline:
        time.sleep(0.05)
    return client


class MetricsChannelOverTcpTest(unittest.TestCase):
    def test_metrics_frame_lands_in_the_metrics_store(self):
        store = MetricsStore()
        config = make_tcp_config(19301)
        engine = MQTTSyncEngine(config, metrics_store=store)
        try:
            client = connect_client(19301, config.mqttSettings.endianness)
            self.assertTrue(client.connected, "mock client failed to connect")

            struct_endianness = "<" if config.mqttSettings.endianness == "LE" else ">"
            message = MetricsMessage(METRICS_MAPPING, endianness=struct_endianness)
            payload = message.serialize({("perf", "scan_time_ms"): 8.5, ("perf", "sample_count"): 3})
            client.send(payload, frame_type=FrameType.METRIC_LAYOUT_1)

            deadline = time.time() + 3.0
            while store.latest("node0/perf", "scan_time_ms") is None and time.time() < deadline:
                time.sleep(0.05)

            self.assertAlmostEqual(store.latest("node0/perf", "scan_time_ms").value, 8.5, places=3)
            self.assertEqual(store.latest("node0/perf", "sample_count").value, 3)
            client.stop()
        finally:
            engine.stop()

    def test_vector_metric_arrives_intact_over_the_real_socket(self):
        store = MetricsStore()
        config = make_tcp_config(19302)
        engine = MQTTSyncEngine(config, metrics_store=store)
        try:
            client = connect_client(19302, config.mqttSettings.endianness)
            self.assertTrue(client.connected)

            struct_endianness = "<" if config.mqttSettings.endianness == "LE" else ">"
            message = MetricsMessage(METRICS_MAPPING, endianness=struct_endianness)
            payload = message.serialize({("results", "belt_1_velocity"): [1.0, 0.5]})
            client.send(payload, frame_type=FrameType.METRIC_LAYOUT_1)

            deadline = time.time() + 3.0
            while store.latest("node0/results", "belt_1_velocity") is None and time.time() < deadline:
                time.sleep(0.05)

            self.assertEqual(store.latest("node0/results", "belt_1_velocity").value, [1.0, 0.5])
            self.assertEqual(store.kind_of("node0/results", "belt_1_velocity"), "vector")
            client.stop()
        finally:
            engine.stop()

    def test_node_id_is_threaded_through_into_distinct_metrics_groups(self):
        store = MetricsStore()
        config = make_tcp_config(19305)
        engine = MQTTSyncEngine(config, metrics_store=store)
        try:
            client = connect_client(19305, config.mqttSettings.endianness)
            self.assertTrue(client.connected)

            struct_endianness = "<" if config.mqttSettings.endianness == "LE" else ">"
            message = MetricsMessage(METRICS_MAPPING, endianness=struct_endianness)
            client.send(
                message.serialize({("perf", "sample_count"): 1}), frame_type=FrameType.METRIC_LAYOUT_1, node_id=0
            )
            client.send(
                message.serialize({("perf", "sample_count"): 2}), frame_type=FrameType.METRIC_LAYOUT_1, node_id=1
            )

            deadline = time.time() + 3.0
            while store.latest("node1/perf", "sample_count") is None and time.time() < deadline:
                time.sleep(0.05)

            self.assertEqual(store.latest("node0/perf", "sample_count").value, 1)
            self.assertEqual(store.latest("node1/perf", "sample_count").value, 2)
            client.stop()
        finally:
            engine.stop()

    def test_output_channel_frames_still_route_to_config_not_metrics(self):
        """Regression: adding metrics-frame routing must not break the
        pre-existing output-channel (LED/belt) frame path, which shares
        the same TCP connection."""
        store = MetricsStore()
        config = make_tcp_config(19303)
        engine = MQTTSyncEngine(config, metrics_store=store)
        try:
            client = connect_client(19303, config.mqttSettings.endianness)
            self.assertTrue(client.connected)

            output_channel = engine._sub_channels[0]
            payload = output_channel.message.serialize(config)  # a real, correctly-sized "output" payload
            client.send(payload, frame_type=FrameType.IO_MAPPING)
            time.sleep(0.5)  # give the recv thread a chance to process it

            self.assertEqual(store.groups(), [], "an IO_MAPPING frame must not be misrouted into the metrics store")
            client.stop()
        finally:
            engine.stop()

    def test_frame_type_not_payload_size_determines_routing(self):
        """The whole point of the explicit FrameType header byte: even a
        metrics-shaped, metrics-sized payload sent with frame_type
        IO_MAPPING must NOT be routed into the metrics store - routing no
        longer depends on guessing from payload size."""
        store = MetricsStore()
        config = make_tcp_config(19306)
        engine = MQTTSyncEngine(config, metrics_store=store)
        try:
            client = connect_client(19306, config.mqttSettings.endianness)
            self.assertTrue(client.connected)

            struct_endianness = "<" if config.mqttSettings.endianness == "LE" else ">"
            message = MetricsMessage(METRICS_MAPPING, endianness=struct_endianness)
            metrics_shaped_payload = message.serialize({("perf", "sample_count"): 99})
            client.send(metrics_shaped_payload, frame_type=FrameType.IO_MAPPING)
            time.sleep(0.5)

            self.assertEqual(
                store.groups(), [],
                "a metrics-shaped payload mislabeled IO_MAPPING must not land in the metrics store",
            )
            client.stop()
        finally:
            engine.stop()

    def test_metrics_channels_are_empty_when_no_store_is_given(self):
        config = make_tcp_config(19304)
        engine = MQTTSyncEngine(config)  # no metrics_store passed
        try:
            self.assertEqual(engine._metrics_channels, {})
            self.assertIsNone(engine.metrics_store)
        finally:
            engine.stop()


class MultiLayoutMetricsTest(unittest.TestCase):
    """Different nodes can report different metric data: each declared
    layout in METRICS_CHANNELS gets its own frame_type and its own
    MetricsMessage/mapping, so a node picks which layout it's sending by
    which frame_type it tags its frames with."""

    def test_a_channel_exists_for_every_declared_layout(self):
        config = make_tcp_config(19307)
        store = MetricsStore()
        engine = MQTTSyncEngine(config, metrics_store=store)
        try:
            self.assertEqual(set(engine._metrics_channels.keys()), set(METRICS_CHANNELS.keys()))
            self.assertIn(FrameType.METRIC_LAYOUT_1, engine._metrics_channels)
            self.assertIn(FrameType.METRIC_LAYOUT_2, engine._metrics_channels)
        finally:
            engine.stop()

    def test_a_layout_2_frame_lands_under_its_own_mapping_and_group(self):
        store = MetricsStore()
        config = make_tcp_config(19308)
        engine = MQTTSyncEngine(config, metrics_store=store)
        try:
            client = connect_client(19308, config.mqttSettings.endianness)
            self.assertTrue(client.connected)

            struct_endianness = "<" if config.mqttSettings.endianness == "LE" else ">"
            message = MetricsMessage(METRICS_MAPPING_2, endianness=struct_endianness)
            payload = message.serialize({("env", "temperature_c"): 21.5, ("env", "humidity_pct"): 40.0})
            client.send(payload, frame_type=FrameType.METRIC_LAYOUT_2)

            deadline = time.time() + 3.0
            while store.latest("node0/env", "temperature_c") is None and time.time() < deadline:
                time.sleep(0.05)

            self.assertAlmostEqual(store.latest("node0/env", "temperature_c").value, 21.5, places=3)
            self.assertAlmostEqual(store.latest("node0/env", "humidity_pct").value, 40.0, places=3)
            client.stop()
        finally:
            engine.stop()

    def test_layout_1_and_layout_2_frames_route_correctly_over_the_same_connection(self):
        """Two different nodes' differently-shaped metric data, sent back
        to back on the SAME socket, must both land correctly - proving
        frame_type (not payload_size, not which client connection) is
        what actually selects the mapping."""
        store = MetricsStore()
        config = make_tcp_config(19309)
        engine = MQTTSyncEngine(config, metrics_store=store)
        try:
            client = connect_client(19309, config.mqttSettings.endianness)
            self.assertTrue(client.connected)

            struct_endianness = "<" if config.mqttSettings.endianness == "LE" else ">"
            layout1 = MetricsMessage(METRICS_MAPPING, endianness=struct_endianness)
            layout2 = MetricsMessage(METRICS_MAPPING_2, endianness=struct_endianness)

            client.send(
                layout1.serialize({("perf", "sample_count"): 7}),
                frame_type=FrameType.METRIC_LAYOUT_1, node_id=1,
            )
            client.send(
                layout2.serialize({("env", "vibration_alarm_code"): 3}),
                frame_type=FrameType.METRIC_LAYOUT_2, node_id=2,
            )

            deadline = time.time() + 3.0
            while store.latest("node2/env", "vibration_alarm_code") is None and time.time() < deadline:
                time.sleep(0.05)

            self.assertEqual(store.latest("node1/perf", "sample_count").value, 7)
            self.assertEqual(store.latest("node2/env", "vibration_alarm_code").value, 3)
            # Neither frame's data leaked into the other's node/group.
            self.assertIsNone(store.latest("node2/perf", "sample_count"))
            self.assertIsNone(store.latest("node1/env", "vibration_alarm_code"))
            client.stop()
        finally:
            engine.stop()


if __name__ == "__main__":
    unittest.main()
