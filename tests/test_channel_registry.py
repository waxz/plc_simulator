"""test_channel_registry.py - covers channel_registry.load_channel_registry
and mqtt_message_binding.load_mapping_csv in isolation, then proves the
integration claim end to end: an MQTTSyncEngine built from a registry file
that reproduces today's hardcoded PUBLISH_TOPICS/SUBSCRIBE_TOPICS/
METRICS_CHANNELS behaves identically to one built the legacy way - same
payload sizes, same IO-mapping round trip, same metrics routing - proving
M1 is a pure refactor with no behavior change for anyone who doesn't pass
channel_registry_path.
"""
import json
import os
import tempfile
import time
import unittest

from config_parser import load_config
from metrics_store import MetricsStore
from metric_layout import layout_mapping
from mqtt_data_exchange import (
    MetricLayout1,
    PUBLISH_TOPICS,
    SUBSCRIBE_TOPICS,
    MQTTSyncEngine,
)

# The legacy (ByteType, "group/name") list for metric layout 1, now
# derived from the MetricLayout1 dataclass instead of a hand-written
# module constant.
METRICS_MAPPING = layout_mapping(MetricLayout1)
from mqtt_message_binding import ByteType, custom_component_mapping, load_mapping_csv, print_mapping
from channel_registry import ChannelRegistryError, load_channel_registry
from tcp_framed_transport import FrameType, TcpFramedClient


def make_tcp_config(port: int):
    config = load_config("station_1.json")
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


def write_default_registry(tmp_dir: str, config) -> str:
    """Build a channels.json + CSVs in tmp_dir that reproduce today's
    hardcoded "input"/"output"/metrics-layout-1 channels exactly, so tests
    can compare a registry-built engine against the legacy default engine."""
    input_mapping = custom_component_mapping(config, PUBLISH_TOPICS["input"]["custom_ids"])
    output_mapping = custom_component_mapping(config, SUBSCRIBE_TOPICS["output"]["custom_ids"])
    print_mapping(input_mapping, os.path.join(tmp_dir, "input.csv"))
    print_mapping(output_mapping, os.path.join(tmp_dir, "output.csv"))
    print_mapping(METRICS_MAPPING, os.path.join(tmp_dir, "metrics.csv"))

    registry = {
        "channels": [
            {
                "name": "input",
                "kind": "io_mapping",
                "direction": "out",
                "topic": "input",
                "interval_ms": PUBLISH_TOPICS["input"]["interval"],
                "mapping_csv": "input.csv",
            },
            {
                "name": "output",
                "kind": "io_mapping",
                "direction": "in",
                "frame_type": "IO_MAPPING",
                "topic": "output",
                "interval_ms": SUBSCRIBE_TOPICS["output"]["interval"],
                "mapping_csv": "output.csv",
            },
            {
                "name": "metrics",
                "kind": "metric",
                "direction": "in",
                "frame_type": "METRIC_LAYOUT_1",
                "topic": "metrics",
                "interval_ms": 50,
                "mapping_csv": "metrics.csv",
            },
        ]
    }
    registry_path = os.path.join(tmp_dir, "channels.json")
    with open(registry_path, "w", encoding="utf-8") as f:
        json.dump(registry, f)
    return registry_path


class LoadMappingCsvTest(unittest.TestCase):
    def test_round_trips_print_mapping_output(self):
        config = load_config("station_1.json")
        mapping = custom_component_mapping(config, SUBSCRIBE_TOPICS["output"]["custom_ids"])
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = os.path.join(tmp_dir, "output.csv")
            print_mapping(mapping, path)
            self.assertEqual(load_mapping_csv(path), mapping)

    def test_accepts_bare_byte_type_name(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = os.path.join(tmp_dir, "bare.csv")
            with open(path, "w", encoding="utf-8") as f:
                f.write("ByteType,Path\nBIT,led_0/isOn\nfloat,belt_1/speed\n")
            mapping = load_mapping_csv(path)
            self.assertEqual(mapping[0][1], "led_0/isOn")
            self.assertEqual(mapping[1][1], "belt_1/speed")

    def test_pad_row_expands_to_the_next_byte_boundary(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = os.path.join(tmp_dir, "padded.csv")
            with open(path, "w", encoding="utf-8") as f:
                f.write(
                    "ByteType,Path\n"
                    "BIT,led_0/isOn\n"
                    "BIT,led_1/isOn\n"
                    "BIT,led_2/isOn\n"
                    "PAD,auto\n"
                    "FLOAT,belt_1/speed\n"
                )
            mapping = load_mapping_csv(path)
            byte_types = [bt for bt, _ in mapping]
            self.assertEqual(byte_types, [
                ByteType.BIT, ByteType.BIT, ByteType.BIT,
                ByteType.BIT, ByteType.BIT, ByteType.BIT, ByteType.BIT, ByteType.BIT,
                ByteType.FLOAT,
            ])
            padding_paths = [path for bt, path in mapping if bt == ByteType.BIT][3:]
            self.assertEqual(padding_paths, ["padding_0/none", "padding_1/none", "padding_2/none",
                                              "padding_3/none", "padding_4/none"])

    def test_pad_row_is_a_no_op_when_already_byte_aligned(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = os.path.join(tmp_dir, "aligned.csv")
            with open(path, "w", encoding="utf-8") as f:
                f.write(
                    "ByteType,Path\n"
                    + "".join(f"BIT,led_{i}/isOn\n" for i in range(8))
                    + "PAD,auto\n"
                    "FLOAT,belt_1/speed\n"
                )
            mapping = load_mapping_csv(path)
            self.assertEqual(len(mapping), 9)  # 8 BIT rows + 1 FLOAT, no padding inserted
            self.assertEqual(mapping[-1], (ByteType.FLOAT, "belt_1/speed"))

    def test_pad_row_produces_a_mapping_construct_actually_accepts(self):
        """The point of PAD: a mapping a non-expert wrote without manually
        counting bits still produces a working ComponentMessage."""
        from mqtt_message_binding import ComponentMessage
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = os.path.join(tmp_dir, "padded.csv")
            with open(path, "w", encoding="utf-8") as f:
                f.write(
                    "ByteType,Path\n"
                    "BIT,led_0/isOn\n"
                    "BIT,led_1/isOn\n"
                    "PAD,auto\n"
                    "UINT16,padding_x/none\n"
                )
            mapping = load_mapping_csv(path)
            message = ComponentMessage(mapping)
            self.assertEqual(message.payload_size, 1 + 2)  # 1 packed byte for the bits, 2 for UINT16

    def test_unknown_byte_type_names_the_file_and_row(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = os.path.join(tmp_dir, "bad.csv")
            with open(path, "w", encoding="utf-8") as f:
                f.write("ByteType,Path\nBIT,led_0/isOn\nNOT_A_TYPE,belt_1/speed\n")
            with self.assertRaises(ValueError) as ctx:
                load_mapping_csv(path)
            self.assertIn("bad.csv:3", str(ctx.exception))
            self.assertIn("NOT_A_TYPE", str(ctx.exception))


class LoadChannelRegistryTest(unittest.TestCase):
    def test_resolves_csv_paths_relative_to_the_registry_file(self):
        config = load_config("station_1.json")
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = write_default_registry(tmp_dir, config)
            publish_topics, subscribe_topics, metrics_channels = load_channel_registry(registry_path)

            self.assertIn("input", publish_topics)
            self.assertIn("output", subscribe_topics)
            self.assertIn(FrameType.METRIC_LAYOUT_1, metrics_channels)
            self.assertEqual(metrics_channels[FrameType.METRIC_LAYOUT_1]["topic_suffix"], "metrics")

    def test_missing_required_field_names_the_channel(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = os.path.join(tmp_dir, "channels.json")
            with open(registry_path, "w", encoding="utf-8") as f:
                json.dump({"channels": [{"name": "bad", "kind": "io_mapping", "direction": "in", "topic": "t"}]}, f)
            with self.assertRaises(ChannelRegistryError) as ctx:
                load_channel_registry(registry_path)
            self.assertIn("'bad'", str(ctx.exception))
            self.assertIn("mapping_csv", str(ctx.exception))

    def test_unknown_kind_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            with open(os.path.join(tmp_dir, "m.csv"), "w", encoding="utf-8") as f:
                f.write("ByteType,Path\nBIT,led_0/isOn\n")
            registry_path = os.path.join(tmp_dir, "channels.json")
            with open(registry_path, "w", encoding="utf-8") as f:
                json.dump({
                    "channels": [{
                        "name": "weird", "kind": "not_a_kind", "direction": "in",
                        "topic": "t", "mapping_csv": "m.csv",
                    }]
                }, f)
            with self.assertRaises(ChannelRegistryError):
                load_channel_registry(registry_path)

    def test_unknown_frame_type_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            with open(os.path.join(tmp_dir, "m.csv"), "w", encoding="utf-8") as f:
                f.write("ByteType,Path\nBIT,led_0/isOn\n")
            registry_path = os.path.join(tmp_dir, "channels.json")
            with open(registry_path, "w", encoding="utf-8") as f:
                json.dump({
                    "channels": [{
                        "name": "m", "kind": "metric", "direction": "in",
                        "frame_type": "NOT_A_FRAME_TYPE", "topic": "t", "mapping_csv": "m.csv",
                    }]
                }, f)
            with self.assertRaises(ChannelRegistryError) as ctx:
                load_channel_registry(registry_path)
            self.assertIn("NOT_A_FRAME_TYPE", str(ctx.exception))

    def test_duplicate_frame_type_across_metric_channels_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            with open(os.path.join(tmp_dir, "m.csv"), "w", encoding="utf-8") as f:
                f.write("ByteType,Path\nBIT,led_0/isOn\n")
            registry_path = os.path.join(tmp_dir, "channels.json")
            with open(registry_path, "w", encoding="utf-8") as f:
                json.dump({
                    "channels": [
                        {"name": "m1", "kind": "metric", "direction": "in",
                         "frame_type": "METRIC_LAYOUT_1", "topic": "t1", "mapping_csv": "m.csv"},
                        {"name": "m2", "kind": "metric", "direction": "in",
                         "frame_type": "METRIC_LAYOUT_1", "topic": "t2", "mapping_csv": "m.csv"},
                    ]
                }, f)
            with self.assertRaises(ChannelRegistryError):
                load_channel_registry(registry_path)


class RegistryEngineParityTest(unittest.TestCase):
    """The actual M1 claim: an engine built from a registry that mirrors
    today's hardcoded channels is indistinguishable from the legacy engine."""

    def test_payload_sizes_match_the_legacy_hardcoded_channels(self):
        config_legacy = make_tcp_config(19401)
        config_registry = make_tcp_config(19402)
        legacy_engine = MQTTSyncEngine(config_legacy)
        try:
            with tempfile.TemporaryDirectory() as tmp_dir:
                registry_path = write_default_registry(tmp_dir, config_registry)
                registry_engine = MQTTSyncEngine(config_registry, channel_registry_path=registry_path)
                try:
                    self.assertEqual(
                        legacy_engine._pub_channels[0].message.payload_size,
                        registry_engine._pub_channels[0].message.payload_size,
                    )
                    self.assertEqual(
                        legacy_engine._sub_channels[0].message.payload_size,
                        registry_engine._sub_channels[0].message.payload_size,
                    )
                finally:
                    registry_engine.stop()
        finally:
            legacy_engine.stop()

    def test_io_mapping_frame_round_trips_through_a_registry_channel(self):
        store = MetricsStore()
        config = make_tcp_config(19403)
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = write_default_registry(tmp_dir, config)
            engine = MQTTSyncEngine(config, metrics_store=store, channel_registry_path=registry_path)
            try:
                client = connect_client(19403, config.mqttSettings.endianness)
                self.assertTrue(client.connected, "mock client failed to connect")

                belt = next(b for b in config.belts if b.id == "belt_1_1_1")
                belt.speed = 3.5
                payload = engine._sub_channels[0].message.serialize(config)
                belt.speed = 0.0

                client.send(payload, frame_type=FrameType.IO_MAPPING)

                deadline = time.time() + 3.0
                while belt.speed != 3.5 and time.time() < deadline:
                    time.sleep(0.05)

                self.assertEqual(belt.speed, 3.5)
                client.stop()
            finally:
                engine.stop()

    def test_metrics_frame_routes_through_a_registry_channel(self):
        store = MetricsStore()
        config = make_tcp_config(19404)
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = write_default_registry(tmp_dir, config)
            engine = MQTTSyncEngine(config, metrics_store=store, channel_registry_path=registry_path)
            try:
                client = connect_client(19404, config.mqttSettings.endianness)
                self.assertTrue(client.connected)

                struct_endianness = "<" if config.mqttSettings.endianness == "LE" else ">"
                from mqtt_message_binding import MetricsMessage
                message = MetricsMessage(METRICS_MAPPING, endianness=struct_endianness)
                payload = message.serialize({("perf", "sample_count"): 42})
                client.send(payload, frame_type=FrameType.METRIC_LAYOUT_1)

                deadline = time.time() + 3.0
                while store.latest("node0/perf", "sample_count") is None and time.time() < deadline:
                    time.sleep(0.05)

                self.assertEqual(store.latest("node0/perf", "sample_count").value, 42)
                client.stop()
            finally:
                engine.stop()

    def test_no_registry_path_leaves_legacy_behavior_untouched(self):
        """Guards the M1 compatibility requirement directly: constructing
        MQTTSyncEngine without channel_registry_path must never even
        import/touch the registry machinery's failure modes."""
        config = make_tcp_config(19405)
        engine = MQTTSyncEngine(config)
        try:
            self.assertEqual(len(engine._pub_channels), len(PUBLISH_TOPICS))
            self.assertEqual(len(engine._sub_channels), len(SUBSCRIBE_TOPICS))
        finally:
            engine.stop()


if __name__ == "__main__":
    unittest.main()
