"""
test_plc_mock.py - unit tests for plc_mock.py's testable units.

Scope: `_struct_endianness`, `PerformanceMetrics`, `_extract_latency_ms`, and
`_build_layout1_sample` are pure-ish helpers with no prior coverage. `main()`
itself is CLI/network orchestration (argparse, live MQTT/TCP connections, an
infinite send loop) - not unit-tested here, consistent with how
main.py/visualizer.py's own entry points aren't either;
test_mqtt_data_exchange.py covers the metrics channel's real-socket wiring
end to end, and the TCP link's tests (test_tcp_exchange.py) the live path.
"""
import io
import time
import unittest
from contextlib import redirect_stdout

from config_parser import MqttSettings, SimulatorConfig
from plc_mock import PerformanceMetrics, _build_layout1_sample, _extract_latency_ms, _struct_endianness


def make_config(endianness="LE"):
    return SimulatorConfig(mqttSettings=MqttSettings(endianness=endianness))


class StructEndiannessTest(unittest.TestCase):
    def test_topic_def_endianness_overrides_config(self):
        config = make_config("BE")
        self.assertEqual(_struct_endianness({"endianness": "LE"}, config), "<")

    def test_falls_back_to_config_endianness_when_topic_def_omits_it(self):
        config = make_config("BE")
        self.assertEqual(_struct_endianness({}, config), ">")

    def test_defaults_to_little_endian_with_no_config_and_no_topic_def_value(self):
        self.assertEqual(_struct_endianness({}, None), "<")

    def test_is_case_insensitive(self):
        self.assertEqual(_struct_endianness({"endianness": "le"}, None), "<")
        self.assertEqual(_struct_endianness({"endianness": "be"}, None), ">")

    def test_unrecognized_value_falls_back_to_big_endian_marker(self):
        # Only an exact "LE" match selects little-endian; anything else
        # (including a typo/garbage value) is silently treated as
        # big-endian rather than raising - documenting this permissive
        # fallback rather than letting it go unnoticed.
        self.assertEqual(_struct_endianness({"endianness": "XYZ"}, None), ">")


class PerformanceMetricsTest(unittest.TestCase):
    def test_record_rx_accumulates_count_bytes_and_latency_stats(self):
        metrics = PerformanceMetrics(report_interval=60.0)
        metrics.record_rx(100, latency_ms=5.0)
        metrics.record_rx(50, latency_ms=15.0)

        self.assertEqual(metrics.rx_count, 2)
        self.assertEqual(metrics.rx_bytes, 150)
        self.assertEqual(metrics.latency_count, 2)
        self.assertAlmostEqual(metrics.latency_total_ms, 20.0)
        self.assertEqual(metrics.latency_min_ms, 5.0)
        self.assertEqual(metrics.latency_max_ms, 15.0)

    def test_record_rx_ignores_negative_latency(self):
        metrics = PerformanceMetrics(report_interval=60.0)
        metrics.record_rx(100, latency_ms=-1.0)

        self.assertEqual(metrics.rx_count, 1)
        self.assertEqual(metrics.latency_count, 0)
        self.assertIsNone(metrics.latency_min_ms)

    def test_record_rx_without_latency_does_not_affect_latency_stats(self):
        metrics = PerformanceMetrics(report_interval=60.0)
        metrics.record_rx(100)

        self.assertEqual(metrics.rx_count, 1)
        self.assertEqual(metrics.latency_count, 0)

    def test_record_tx_sent_increments_tx_counters(self):
        metrics = PerformanceMetrics(report_interval=60.0)
        metrics.record_tx(200, sent=True)

        self.assertEqual(metrics.tx_count, 1)
        self.assertEqual(metrics.tx_bytes, 200)
        self.assertEqual(metrics.tx_drop_count, 0)

    def test_record_tx_dropped_increments_drop_counter_not_tx_count(self):
        metrics = PerformanceMetrics(report_interval=60.0)
        metrics.record_tx(200, sent=False)

        self.assertEqual(metrics.tx_count, 0)
        self.assertEqual(metrics.tx_bytes, 0)
        self.assertEqual(metrics.tx_drop_count, 1)

    def test_record_malformed_increments_counter(self):
        metrics = PerformanceMetrics(report_interval=60.0)
        metrics.record_malformed()
        metrics.record_malformed()

        self.assertEqual(metrics.malformed_count, 2)

    def test_maybe_report_does_nothing_before_interval_elapses(self):
        metrics = PerformanceMetrics(report_interval=60.0)
        metrics.record_rx(100)
        # _last_report was just set in __post_init__, so far less than 60s
        # has "elapsed" - maybe_report should be a no-op.
        buf = io.StringIO()
        with redirect_stdout(buf):
            metrics.maybe_report()
        self.assertEqual(buf.getvalue(), "")

    def test_maybe_report_prints_and_resets_deltas_after_interval_elapses(self):
        metrics = PerformanceMetrics(report_interval=1.0)
        metrics.record_rx(100, latency_ms=10.0)
        metrics.record_tx(50, sent=True)
        metrics.record_malformed()
        metrics._last_report = time.monotonic() - 2.0  # force "enough time elapsed"

        buf = io.StringIO()
        with redirect_stdout(buf):
            metrics.maybe_report()
        output = buf.getvalue()

        self.assertIn("[METRICS]", output)
        self.assertIn("rx 1", output)
        self.assertIn("tx 1", output)
        self.assertIn("malformed 1", output)
        self.assertIn("avg 10.00 ms", output)

        # Deltas reset on a successful report: an immediate second call (no
        # time elapsed since the report above) should be a no-op again.
        buf2 = io.StringIO()
        with redirect_stdout(buf2):
            metrics.maybe_report()
        self.assertEqual(buf2.getvalue(), "")

    def test_maybe_report_shows_na_latency_when_nothing_recorded(self):
        metrics = PerformanceMetrics(report_interval=1.0)
        metrics.record_rx(100)  # no latency_ms
        metrics._last_report = time.monotonic() - 2.0

        buf = io.StringIO()
        with redirect_stdout(buf):
            metrics.maybe_report()

        self.assertIn("latency n/a", buf.getvalue())


class ExtractLatencyMsTest(unittest.TestCase):
    def test_returns_none_when_topic_has_no_custom_ids(self):
        self.assertIsNone(_extract_latency_ms(b"\x00" * 8, {"custom_ids": []}))

    def test_returns_none_when_last_custom_id_is_not_clock(self):
        topic_def = {"custom_ids": ["belt_1", "sensor_1"]}
        self.assertIsNone(_extract_latency_ms(b"\x00" * 8, topic_def))

    def test_returns_none_when_payload_too_short(self):
        topic_def = {"custom_ids": ["belt_1", "clock"]}
        self.assertIsNone(_extract_latency_ms(b"\x00" * 4, topic_def))

    def test_returns_none_for_zero_timestamp(self):
        # sent_ns is decoded unsigned, so 0 (not "negative") is the only
        # way `<= 0` triggers - a payload of all zero bytes.
        topic_def = {"custom_ids": ["clock"], "endianness": "LE"}
        payload = (0).to_bytes(8, "little")
        self.assertIsNone(_extract_latency_ms(payload, topic_def))

    def test_computes_positive_latency_for_a_past_timestamp_little_endian(self):
        sent_ns = time.time_ns() - 5_000_000  # 5ms ago
        payload = sent_ns.to_bytes(8, "little")
        topic_def = {"custom_ids": ["clock"], "endianness": "LE"}

        latency = _extract_latency_ms(payload, topic_def)

        self.assertIsNotNone(latency)
        self.assertGreaterEqual(latency, 5.0)
        self.assertLess(latency, 1000.0)  # generous bound, not flaky-tight

    def test_respects_big_endian_from_topic_def(self):
        sent_ns = time.time_ns() - 2_000_000
        payload = sent_ns.to_bytes(8, "big")
        topic_def = {"custom_ids": ["clock"], "endianness": "BE"}

        latency = _extract_latency_ms(payload, topic_def)

        self.assertIsNotNone(latency)
        self.assertGreaterEqual(latency, 2.0)

    def test_respects_endianness_from_config_when_topic_def_omits_it(self):
        sent_ns = time.time_ns() - 3_000_000
        payload = sent_ns.to_bytes(8, "big")
        topic_def = {"custom_ids": ["clock"]}
        config = SimulatorConfig(mqttSettings=MqttSettings(endianness="BE"))

        latency = _extract_latency_ms(payload, topic_def, config)

        self.assertIsNotNone(latency)
        self.assertGreaterEqual(latency, 3.0)

    def test_wrong_endianness_does_not_crash_and_differs_from_correct_decode(self):
        sent_ns = time.time_ns() - 1_000_000
        payload = sent_ns.to_bytes(8, "little")
        correct = _extract_latency_ms(payload, {"custom_ids": ["clock"], "endianness": "LE"})
        mismatched = _extract_latency_ms(payload, {"custom_ids": ["clock"], "endianness": "BE"})

        self.assertIsNotNone(correct)
        self.assertLess(correct, 1000.0)
        # Decoding with the wrong byte order scrambles the value - it must
        # not silently produce the same correct-looking small latency.
        self.assertNotAlmostEqual(mismatched if mismatched is not None else -1.0, correct, delta=1.0)


class BuildLayout1SampleTest(unittest.TestCase):
    """_build_layout1_sample() packages plc_mock's own measured values as a
    MetricLayout1 instance (metric_layout.py / mqtt_data_exchange.py),
    which MetricsMessage.serialize() takes directly - no hand-keyed
    (group, name) dict, no drift from the layout declaration."""

    def test_measured_values_land_in_the_right_typed_fields(self):
        sample = _build_layout1_sample(
            loop_period_ms=12.5,
            rtt_latency_ms=3.2,
            send_elapsed_us=456.0,
            tx_count=7,
            sent_ok=True,
            belt_speed=2.0,
        )
        self.assertEqual(sample.perf.scan_time_ms, 12.5)
        self.assertEqual(sample.perf.cycle_time_ms, 3.2)
        self.assertEqual(sample.perf.fn_elapsed_us, 456.0)
        self.assertEqual(sample.perf.sample_count, 7)
        self.assertEqual(sample.results.belt_1_velocity, (2.0, 0.0))

    def test_sent_ok_true_maps_to_result_code_zero(self):
        sample = _build_layout1_sample(
            loop_period_ms=0.0, rtt_latency_ms=0.0, send_elapsed_us=0.0,
            tx_count=0, sent_ok=True, belt_speed=0.0,
        )
        self.assertEqual(sample.results.last_result_code, 0)

    def test_sent_ok_false_maps_to_result_code_one(self):
        sample = _build_layout1_sample(
            loop_period_ms=0.0, rtt_latency_ms=0.0, send_elapsed_us=0.0,
            tx_count=0, sent_ok=False, belt_speed=0.0,
        )
        self.assertEqual(sample.results.last_result_code, 1)

    def test_flattens_to_every_slot_the_layout_declares(self):
        from metric_layout import layout_mapping, layout_values
        from mqtt_data_exchange import MetricLayout1
        from mqtt_message_binding import parse_path

        declared_keys = {
            parse_path(path)[:2] for _bt, path in layout_mapping(MetricLayout1)
        }
        sample = _build_layout1_sample(
            loop_period_ms=1.0, rtt_latency_ms=1.0, send_elapsed_us=1.0,
            tx_count=1, sent_ok=True, belt_speed=1.0,
        )
        self.assertEqual(set(layout_values(sample).keys()), declared_keys)

    def test_serializes_through_a_real_MetricsMessage(self):
        """End-to-end sanity check without a socket: the instance this
        function builds must pack through the real
        MetricsMessage.from_layout(MetricLayout1) and come back out of a
        MetricsStore with the expected (group, name) keys."""
        from mqtt_data_exchange import MetricLayout1
        from mqtt_message_binding import MetricsMessage
        from metrics_store import MetricsStore

        message = MetricsMessage.from_layout(MetricLayout1, endianness="<")
        sample = _build_layout1_sample(
            loop_period_ms=16.7, rtt_latency_ms=4.1, send_elapsed_us=210.0,
            tx_count=42, sent_ok=False, belt_speed=1.5,
        )
        payload = message.serialize(sample)

        store = MetricsStore()
        # node_id=5 - as if this client were run with --node-id 5 (see
        # tcp_framed_transport.py's FrameType/ParsedFrame and
        # mqtt_data_exchange.py's node-tagged routing) - deserialize()
        # prefixes every group it writes with it.
        message.deserialize(payload, store, node_id=5)
        self.assertAlmostEqual(store.latest("node5/perf", "scan_time_ms").value, 16.7, places=3)
        self.assertEqual(store.latest("node5/perf", "sample_count").value, 42)
        self.assertEqual(store.latest("node5/results", "last_result_code").value, 1)
        self.assertAlmostEqual(store.latest("node5/results", "belt_1_velocity").value[0], 1.5, places=3)


class BuildLayout3SampleTest(unittest.TestCase):
    """_build_layout3_sample() - the second layout this client reports
    (METRIC_LAYOUT_3). Must be built as a MetricLayout3 (the top-level
    layout), not a bare MetricLayout3Test123 group."""

    def test_serializes_through_a_real_MetricsMessage(self):
        from mqtt_data_exchange import MetricLayout3
        from mqtt_message_binding import MetricsMessage
        from metrics_store import MetricsStore
        from plc_mock import _build_layout3_sample

        message = MetricsMessage.from_layout(MetricLayout3, endianness="<")
        sample = _build_layout3_sample(led_on=True, led2_on=False, value=2.5)

        store = MetricsStore()
        message.deserialize(message.serialize(sample), store, node_id=2)
        self.assertEqual(store.latest("node2/test123", "led").value, True)
        self.assertEqual(store.latest("node2/test123", "led2").value, False)
        self.assertAlmostEqual(store.latest("node2/test123", "v").value, 2.5, places=3)


if __name__ == "__main__":
    unittest.main()
