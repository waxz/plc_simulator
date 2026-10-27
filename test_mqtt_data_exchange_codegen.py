"""test_mqtt_data_exchange_codegen.py - tests for the higher-stakes half
of the "load existing config, edit it, save it back into
mqtt_data_exchange.py" feature. Every write test here operates on a
throwaway COPY of the real file (never REAL_TARGET itself, exactly like
test_generate_frame_type.py's own pattern) - and several tests exist
specifically to prove the safety gates (compile-check, re-validation,
backup) actually stop a bad write before it touches anything.
"""
import ast
import os
import shutil
import tempfile
import unittest

from config_parser import load_config
from metric_layout import layout_mapping
import mqtt_data_exchange_codegen as codegen
from mqtt_message_binding import ByteType, MetricsMessage
from tcp_framed_transport import FrameType

REAL_TARGET = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mqtt_data_exchange.py")


def _exec_and_read(source: str):
    """Exec a generated source string in isolation (never sys.modules) and
    return (PUBLISH_TOPICS, SUBSCRIBE_TOPICS, METRICS_CHANNELS) - the same
    check write_hardcoded_channels() itself runs internally, exposed here
    so tests can inspect the *result* of a regeneration directly."""
    namespace = {}
    exec(compile(source, "<test>", "exec"), namespace)
    return namespace["PUBLISH_TOPICS"], namespace["SUBSCRIBE_TOPICS"], namespace["METRICS_CHANNELS"]


def _mapping_of(metrics_channels, frame_type):
    """The wire (ByteType, path) list for one entry of an exec'd
    METRICS_CHANNELS, whichever key shape it uses ("layout" class, or a
    pre-migration "mapping" list)."""
    channel_def = metrics_channels[frame_type]
    if "layout" in channel_def:
        return layout_mapping(channel_def["layout"])
    return channel_def["mapping"]


class ReadHardcodedChannelsTest(unittest.TestCase):
    def test_matches_the_real_hardcoded_definitions(self):
        entries = codegen.read_hardcoded_channels()
        names = {e["name"] for e in entries}
        # A subset check, not assertEqual: REAL_TARGET is the actual project
        # file, which someone using the web editor is free to add channels
        # to at any time (that's the whole point) - pinning an exact,
        # closed set here would make this test fail on every legitimate new
        # channel instead of only on a real regression in the four this
        # test actually cares about.
        self.assertTrue({"input", "output", "metrics", "metrics2"}.issubset(names))

        output = next(e for e in entries if e["name"] == "output")
        self.assertEqual(output["kind"], "io_mapping")
        self.assertEqual(output["direction"], "in")
        self.assertIn("led_1_1_1", output["component_ids"])

        metrics = next(e for e in entries if e["name"] == "metrics")
        self.assertEqual(metrics["kind"], "metric")
        self.assertEqual(metrics["frame_type"], "METRIC_LAYOUT_1")
        self.assertIn({"byte_type": "FLOAT", "path": "perf/scan_time_ms"}, metrics["rows"])


class GenerateSourceTest(unittest.TestCase):
    def setUp(self):
        with open(REAL_TARGET, "r", encoding="utf-8") as f:
            self.source = f.read()
        self.channels = codegen.read_hardcoded_channels()

    def test_regenerating_with_unchanged_data_is_semantically_idempotent(self):
        new_source = codegen.generate_source(self.source, self.channels)
        publish, subscribe, metrics = _exec_and_read(new_source)
        self.assertEqual(set(publish.keys()), {"input"})
        self.assertEqual(set(subscribe.keys()), {"output"})
        # Derived from self.channels (whatever REAL_TARGET actually
        # declares right now), not a hardcoded literal - regenerating from
        # UNCHANGED data must reproduce exactly the channels that went in,
        # whether that's 2 metric channels or 5.
        expected_frame_types = {FrameType[c["frame_type"]] for c in self.channels if c["kind"] == "metric"}
        self.assertEqual(set(metrics.keys()), expected_frame_types)

        input_entry = next(c for c in self.channels if c["name"] == "input")
        self.assertEqual(publish["input"]["custom_ids"], input_entry["component_ids"])
        self.assertEqual(
            _mapping_of(metrics, FrameType.METRIC_LAYOUT_1),
            [(ByteType[r["byte_type"]], r["path"])
             for r in next(c for c in self.channels if c["name"] == "metrics")["rows"]],
        )

    def test_editing_an_existing_metric_channel_regenerates_its_layout_class(self):
        channels = [dict(c) for c in self.channels]
        metrics = next(c for c in channels if c["name"] == "metrics")
        metrics["rows"] = [{"byte_type": "FLOAT", "path": "perf/new_field"}]

        new_source = codegen.generate_source(self.source, channels)
        self.assertIn("@dataclass\nclass MetricLayout1Perf:", new_source)
        self.assertIn("    new_field: f32", new_source)
        # Layout 2's own classes are untouched - only the edited channel's changed.
        self.assertIn("temperature_c: f32", new_source)

        _, _, metrics_channels = _exec_and_read(new_source)
        self.assertEqual(
            _mapping_of(metrics_channels, FrameType.METRIC_LAYOUT_1),
            [(ByteType.FLOAT, "perf/new_field")],
        )

    def test_adding_a_new_metric_channel_mints_a_new_layout_without_touching_existing_ones(self):
        minimal_source = (
            "from typing import Dict, List, Any, Tuple\n"
            "from dataclasses import dataclass\n"
            "from metric_layout import f32, u8, u16, u32, u64, bit, Vec2, Vec3\n"
            "from tcp_framed_transport import FrameType\n\n"
            "@dataclass\nclass MetricLayout1Perf:\n    x: f32\n\n\n"
            "@dataclass\nclass MetricLayout1:\n    perf: MetricLayout1Perf\n\n\n"
            'PUBLISH_TOPICS: Dict[str, Dict[str, Any]] = {"input": {"interval": 10, "custom_ids": ["clock"]}}\n'
            'SUBSCRIBE_TOPICS: Dict[str, Dict[str, Any]] = {"output": {"interval": 10, "custom_ids": ["led_0"]}}\n'
            "METRICS_CHANNELS: Dict[int, Dict[str, Any]] = {\n"
            '    FrameType.METRIC_LAYOUT_1: {"topic_suffix": "metrics", "layout": MetricLayout1},\n'
            "}\n"
        )
        channels = [
            {"name": "input", "kind": "io_mapping", "direction": "out", "topic": "input",
             "interval_ms": 10, "component_ids": ["clock"]},
            {"name": "output", "kind": "io_mapping", "direction": "in", "topic": "output",
             "interval_ms": 10, "component_ids": ["led_0"]},
            {"name": "metrics", "kind": "metric", "direction": "in", "topic": "metrics",
             "interval_ms": 50, "frame_type": "METRIC_LAYOUT_1", "rows": [{"byte_type": "FLOAT", "path": "perf/x"}]},
            {"name": "metrics2", "kind": "metric", "direction": "in", "topic": "metrics2",
             "interval_ms": 50, "frame_type": "METRIC_LAYOUT_2", "rows": [{"byte_type": "UINT16", "path": "extra/counter"}]},
        ]

        new_source = codegen.generate_source(minimal_source, channels)
        self.assertIn("class MetricLayout1Perf:\n    x: f32", new_source)  # untouched
        self.assertIn("class MetricLayout2Extra:\n    counter: u16", new_source)  # newly minted

        _, _, metrics_channels = _exec_and_read(new_source)
        self.assertEqual(_mapping_of(metrics_channels, FrameType.METRIC_LAYOUT_1), [(ByteType.FLOAT, "perf/x")])
        self.assertEqual(_mapping_of(metrics_channels, FrameType.METRIC_LAYOUT_2), [(ByteType.UINT16, "extra/counter")])

    def test_removing_a_metric_channel_leaves_its_layout_class_declared_but_unused(self):
        channels = [c for c in self.channels if c["name"] != "metrics2"]
        new_source = codegen.generate_source(self.source, channels)

        _, _, metrics_channels = _exec_and_read(new_source)
        self.assertNotIn(FrameType.METRIC_LAYOUT_2, metrics_channels)
        # MetricLayout2 itself is still declared (never deleted) even though
        # nothing references it anymore.
        self.assertIn("class MetricLayout2:", new_source)

    def test_editing_io_mapping_component_ids_regenerates_custom_ids(self):
        channels = [dict(c) for c in self.channels]
        output = next(c for c in channels if c["name"] == "output")
        output["component_ids"] = ["led_0", "belt_2"]

        new_source = codegen.generate_source(self.source, channels)
        _, subscribe, _ = _exec_and_read(new_source)
        self.assertEqual(subscribe["output"]["custom_ids"], ["led_0", "belt_2"])

    def test_unknown_byte_type_raises_before_touching_anything(self):
        channels = [dict(c) for c in self.channels]
        metrics = next(c for c in channels if c["name"] == "metrics")
        metrics["rows"] = [{"byte_type": "NOT_A_TYPE", "path": "x/y"}]
        with self.assertRaises(codegen.CodegenError):
            codegen.generate_source(self.source, channels)

    def test_a_blank_metric_field_name_raises_a_clear_error_not_a_syntax_error(self):
        # The editor lets a half-filled metric row be saved (the wire
        # layout still validates); an empty Name would previously reach the
        # generated file as `    : f32` and fail the compile gate with an
        # opaque "invalid syntax" - now it's caught with the row named.
        channels = [dict(c) for c in self.channels]
        metrics = next(c for c in channels if c["name"] == "metric3")
        metrics["rows"] = [
            {"byte_type": "BIT", "path": "n/led"},
            {"byte_type": "FLOAT", "path": "n/"},  # Name still blank
        ]
        with self.assertRaises(codegen.CodegenError) as ctx:
            codegen.generate_source(self.source, channels)
        self.assertIn("Name", str(ctx.exception))
        self.assertIn("'n/'", str(ctx.exception))

    def test_a_blank_metric_group_raises_a_clear_error(self):
        channels = [dict(c) for c in self.channels]
        metrics = next(c for c in channels if c["name"] == "metric3")
        metrics["rows"] = [{"byte_type": "FLOAT", "path": "/x"}]
        with self.assertRaises(codegen.CodegenError) as ctx:
            codegen.generate_source(self.source, channels)
        self.assertIn("Group", str(ctx.exception))

    def test_a_pad_row_after_a_bit_run_is_accepted_not_rejected(self):
        # Regression test: the web editor's auto-pad logic (app.js) inserts
        # a raw {"byte_type": "PAD", "path": "auto"} row to close out an
        # unaligned run of BIT rows. parse_mapping_rows() (which
        # _format_layout_classes runs first) resolves that shorthand; the
        # expanded padding entries are then dropped from the layout class
        # (a run of `bit` fields already rounds to a whole byte in the wire
        # format). An earlier bug re-implemented the ByteType lookup
        # without the PAD step and crashed with "unknown ByteType 'PAD'".
        channels = [dict(c) for c in self.channels]
        metrics = next(c for c in channels if c["name"] == "metrics")
        metrics["rows"] = [
            {"byte_type": "BIT", "path": "io/bit_0"},
            {"byte_type": "BIT", "path": "io/bit_1"},
            {"byte_type": "PAD", "path": "auto"},
            {"byte_type": "FLOAT", "path": "io/after_pad"},
        ]

        new_source = codegen.generate_source(self.source, channels)  # must not raise
        self.assertIn("@dataclass\nclass MetricLayout1Io:", new_source)

        _, _, metrics_channels = _exec_and_read(new_source)
        resolved = _mapping_of(metrics_channels, FrameType.METRIC_LAYOUT_1)
        # 2 bits + 1 float - padding is implicit in the bit-run byte rounding.
        self.assertEqual(resolved, [
            (ByteType.BIT, "io/bit_0"),
            (ByteType.BIT, "io/bit_1"),
            (ByteType.FLOAT, "io/after_pad"),
        ])
        # 2 bits round to 1 byte, then a float -> 5 bytes, same as if the
        # padding had been packed explicitly.
        message = MetricsMessage.from_layout(metrics_channels[FrameType.METRIC_LAYOUT_1]["layout"], endianness="<")
        self.assertEqual(message.payload_size, 5)

    def test_generated_source_always_compiles(self):
        new_source = codegen.generate_source(self.source, self.channels)
        compile(new_source, "<test>", "exec")  # raises SyntaxError if this is ever untrue


class RevalidationGateTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config("test_fixture_config.json")

    def test_rejects_source_with_a_syntax_error(self):
        with self.assertRaises(codegen.CodegenError):
            codegen._revalidate_generated_source("PUBLISH_TOPICS = {", "<test>", self.config)

    def test_rejects_a_document_with_an_unknown_component_id(self):
        with open(REAL_TARGET, "r", encoding="utf-8") as f:
            source = f.read()
        channels = codegen.read_hardcoded_channels()
        output = next(c for c in channels if c["name"] == "output")
        output["component_ids"] = ["not_a_real_component"]
        new_source = codegen.generate_source(source, channels)
        with self.assertRaises(codegen.CodegenError) as ctx:
            codegen._revalidate_generated_source(new_source, REAL_TARGET, self.config)
        self.assertIn("not_a_real_component", str(ctx.exception))

    def test_accepts_the_unmodified_real_definitions(self):
        with open(REAL_TARGET, "r", encoding="utf-8") as f:
            source = f.read()
        channels = codegen.read_hardcoded_channels()
        new_source = codegen.generate_source(source, channels)
        codegen._revalidate_generated_source(new_source, REAL_TARGET, self.config)  # must not raise


class WriteHardcodedChannelsTest(unittest.TestCase):
    """Every test in this class writes to a throwaway COPY - never
    REAL_TARGET - and several assert the real file is untouched to guard
    the test suite itself against a bug that would otherwise corrupt it."""

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp_dir.cleanup)
        self.target = os.path.join(self.tmp_dir.name, "mqtt_data_exchange.py")
        shutil.copyfile(REAL_TARGET, self.target)
        self.config = load_config("test_fixture_config.json")

    def test_writes_a_working_backup_and_a_valid_new_file(self):
        with open(REAL_TARGET, "r", encoding="utf-8") as f:
            original = f.read()
        channels = codegen.read_hardcoded_channels()
        metrics = next(c for c in channels if c["name"] == "metrics")
        metrics["rows"] = [{"byte_type": "FLOAT", "path": "perf/new_field"}]

        codegen.write_hardcoded_channels(channels, self.config, target_file=self.target)

        with open(self.target + ".bak", "r", encoding="utf-8") as f:
            self.assertEqual(f.read(), original)
        with open(self.target, "r", encoding="utf-8") as f:
            new_source = f.read()
        compile(new_source, self.target, "exec")
        self.assertIn("new_field: f32", new_source)

    def test_an_invalid_edit_is_rejected_and_nothing_is_written(self):
        with open(self.target, "r", encoding="utf-8") as f:
            before = f.read()
        channels = codegen.read_hardcoded_channels()
        output = next(c for c in channels if c["name"] == "output")
        output["component_ids"] = ["not_a_real_component"]

        with self.assertRaises(codegen.CodegenError):
            codegen.write_hardcoded_channels(channels, self.config, target_file=self.target)

        with open(self.target, "r", encoding="utf-8") as f:
            self.assertEqual(f.read(), before)  # untouched
        self.assertFalse(os.path.exists(self.target + ".bak"))  # never even backed up

    def test_the_real_project_file_is_never_touched_by_this_test_suite(self):
        with open(REAL_TARGET, "r", encoding="utf-8") as f:
            before = f.read()
        # A REAL_TARGET.bak can legitimately already exist here - someone
        # using the actual web editor against the real file leaves one
        # behind on every save (see write_hardcoded_channels()'s docstring).
        # What this test guards against is THIS SUITE creating or changing
        # one, so it snapshots before/after instead of asserting outright
        # non-existence, which would fail on that unrelated, legitimate
        # state instead of on an actual leak.
        bak_path = REAL_TARGET + ".bak"
        bak_before = open(bak_path, "r", encoding="utf-8").read() if os.path.exists(bak_path) else None

        channels = codegen.read_hardcoded_channels()
        codegen.write_hardcoded_channels(channels, self.config, target_file=self.target)

        with open(REAL_TARGET, "r", encoding="utf-8") as f:
            self.assertEqual(f.read(), before)
        bak_after = open(bak_path, "r", encoding="utf-8").read() if os.path.exists(bak_path) else None
        self.assertEqual(bak_after, bak_before)


if __name__ == "__main__":
    unittest.main()
