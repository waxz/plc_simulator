"""test_main.py - tests for main.py's wiring between the exchange engine
(MQTTSyncEngine), the channel registry, and the web UI (web_app.py):

  build_argparser()  - the new --channel-registry/--web-ui/--web-host/--web-port flags
  build_engine()     - forwards channel_registry_path to MQTTSyncEngine
  build_web_app()    - builds a Flask app sharing the SAME MetricsStore the engine writes into

The last test in this file is the actual end-to-end proof the evaluated
workflow now closes: a metric channel declared in a registry, a real TCP
frame tagged with that channel's frame_type, and the same sample showing
up through the web app's /api/metrics/latest - registry-driven parsing and
the Live Data panel sharing one live store, exactly as main.py wires it.
"""
import threading
import time
import unittest

from metric_layout import layout_mapping
from metrics_store import MetricsStore
from mqtt_data_exchange import MetricLayout1
from mqtt_message_binding import MetricsMessage

METRICS_MAPPING = layout_mapping(MetricLayout1)
from tcp_framed_transport import FrameType
from test_channel_registry import connect_client, make_tcp_config, write_default_registry

import main


class BuildArgparserTest(unittest.TestCase):
    def test_defaults(self):
        args = main.build_argparser().parse_args(["sim_config.json"])
        self.assertIsNone(args.channel_registry)
        self.assertFalse(args.web_ui)
        self.assertEqual(args.web_host, "127.0.0.1")
        self.assertEqual(args.web_port, 5050)

    def test_channel_registry_and_web_ui_flags_parse(self):
        args = main.build_argparser().parse_args([
            "sim_config.json", "--channel-registry", "channels.json",
            "--web-ui", "--web-host", "0.0.0.0", "--web-port", "6000",
        ])
        self.assertEqual(args.channel_registry, "channels.json")
        self.assertTrue(args.web_ui)
        self.assertEqual(args.web_host, "0.0.0.0")
        self.assertEqual(args.web_port, 6000)


class BuildEngineTest(unittest.TestCase):
    def test_without_a_registry_uses_the_legacy_hardcoded_channels(self):
        config = make_tcp_config(19501)
        store = MetricsStore()
        engine = main.build_engine(config, threading.Lock(), store, channel_registry_path=None)
        try:
            self.assertIn(FrameType.METRIC_LAYOUT_1, engine._metrics_channels)
        finally:
            engine.stop()

    def test_with_a_registry_uses_registry_derived_channels(self):
        import tempfile
        config = make_tcp_config(19502)
        store = MetricsStore()
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = write_default_registry(tmp_dir, config)
            engine = main.build_engine(config, threading.Lock(), store, channel_registry_path=registry_path)
            try:
                # write_default_registry's "metrics" channel also uses
                # METRIC_LAYOUT_1 - the real signal that it came from the
                # registry, not the hardcoded dict, is that the sub/pub
                # channels match the registry's "input"/"output" entries.
                self.assertEqual(len(engine._pub_channels), 1)
                self.assertEqual(len(engine._sub_channels), 1)
                self.assertIn(FrameType.METRIC_LAYOUT_1, engine._metrics_channels)
            finally:
                engine.stop()


class BuildWebAppTest(unittest.TestCase):
    def test_shares_the_given_metrics_store(self):
        store = MetricsStore()
        store.add_sample(name="x", value=1.0, group="g")
        app = main.build_web_app("station_1.json", None, store)
        client = app.test_client()
        tree = client.get("/api/metrics/tree").get_json()
        self.assertEqual(tree, {"g": {"x": "scalar"}})

    def test_reads_the_given_registry_path(self):
        import tempfile
        config = make_tcp_config(19503)
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = write_default_registry(tmp_dir, config)
            app = main.build_web_app("station_1.json", registry_path, MetricsStore())
            client = app.test_client()
            body = client.get("/api/registry").get_json()
            self.assertEqual({c["name"] for c in body["channels"]}, {"input", "output", "metrics"})


class EndToEndWiringTest(unittest.TestCase):
    """The actual workflow from the evaluation: web UI declares a channel
    (here, a saved registry standing in for that), the TCP server parses a
    frame using it, and the SAME sample is visible through the web app's
    Live Data endpoints - main.py's build_engine()/build_web_app() sharing
    one MetricsStore is the entire fix."""

    def test_a_registry_driven_metric_frame_appears_through_the_shared_web_app(self):
        import tempfile
        config = make_tcp_config(19504)
        store = MetricsStore()
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = write_default_registry(tmp_dir, config)
            engine = main.build_engine(config, threading.Lock(), store, channel_registry_path=registry_path)
            try:
                client = connect_client(19504, config.mqttSettings.endianness)
                self.assertTrue(client.connected, "mock client failed to connect")

                struct_endianness = "<" if config.mqttSettings.endianness == "LE" else ">"
                message = MetricsMessage(METRICS_MAPPING, endianness=struct_endianness)
                client.send(message.serialize({("perf", "sample_count"): 42}), frame_type=FrameType.METRIC_LAYOUT_1)

                deadline = time.time() + 3.0
                while store.latest("node0/perf", "sample_count") is None and time.time() < deadline:
                    time.sleep(0.05)
                self.assertIsNotNone(store.latest("node0/perf", "sample_count"))

                web_app = main.build_web_app("station_1.json", registry_path, store)
                web_client = web_app.test_client()
                latest = web_client.get("/api/metrics/latest").get_json()
                row = next(r for r in latest if r["name"] == "sample_count")
                self.assertEqual(row["value"], 42)
                self.assertEqual(row["group"], "node0/perf")

                client.stop()
            finally:
                engine.stop()


if __name__ == "__main__":
    unittest.main()
