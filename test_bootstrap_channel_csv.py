"""test_bootstrap_channel_csv.py - unit tests for bootstrap_channel_csv.py's
run(), plus one real subprocess invocation proving the CLI entry point
itself (argv parsing, __main__) actually works end to end."""
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

import bootstrap_channel_csv
from config_parser import load_config
from mqtt_message_binding import custom_component_mapping, load_mapping_csv
from channel_registry import load_channel_registry
from tcp_framed_transport import FrameType


class BootstrapChannelCsvTest(unittest.TestCase):
    def test_writes_a_csv_for_known_component_ids(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            out_path = os.path.join(tmp_dir, "out.csv")
            stdout = io.StringIO()
            with redirect_stdout(stdout):
                code = bootstrap_channel_csv.run([
                    "--config", "test_config.json",
                    "--ids", "led_0,belt_2",
                    "--out", out_path,
                ])
            self.assertEqual(code, 0)
            self.assertTrue(os.path.exists(out_path))
            mapping = load_mapping_csv(out_path)
            paths = [p for _, p in mapping]
            self.assertIn("led_0/isOn", paths)
            self.assertIn("belt_2/speed", paths)
            self.assertIn("Wrote", stdout.getvalue())

    def test_ids_file_skips_blank_lines_and_comments(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ids_path = os.path.join(tmp_dir, "ids.txt")
            out_path = os.path.join(tmp_dir, "out.csv")
            with open(ids_path, "w", encoding="utf-8") as f:
                f.write("# leds for node 2\nled_0\n\nbelt_2\n")
            code = bootstrap_channel_csv.run([
                "--config", "test_config.json",
                "--ids-file", ids_path,
                "--out", out_path,
            ])
            self.assertEqual(code, 0)
            mapping = load_mapping_csv(out_path)
            expected = custom_component_mapping(load_config("test_config.json"), ["led_0", "belt_2"])
            self.assertEqual(mapping, expected)

    def test_unknown_component_id_fails_loudly(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            out_path = os.path.join(tmp_dir, "out.csv")
            stderr = io.StringIO()
            with redirect_stderr(stderr):
                code = bootstrap_channel_csv.run([
                    "--config", "test_config.json",
                    "--ids", "led_0,not_a_real_component",
                    "--out", out_path,
                ])
            self.assertEqual(code, 1)
            self.assertFalse(os.path.exists(out_path))
            self.assertIn("not_a_real_component", stderr.getvalue())

    def test_cli_entry_point_runs_as_a_real_subprocess(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            out_path = os.path.join(tmp_dir, "out.csv")
            result = subprocess.run(
                [sys.executable, "bootstrap_channel_csv.py",
                 "--config", "test_config.json", "--ids", "led_0", "--out", out_path],
                capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(os.path.exists(out_path))


class BootstrapChannelRegistryTest(unittest.TestCase):
    """--registry: bootstrap_channel_csv.py also creating/updating channels.json."""

    def test_registry_flag_creates_a_new_channels_json(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            out_path = os.path.join(tmp_dir, "output_node2.csv")
            registry_path = os.path.join(tmp_dir, "channels.json")
            code = bootstrap_channel_csv.run([
                "--config", "test_config.json", "--ids", "led_0", "--out", out_path,
                "--registry", registry_path, "--name", "output_node2",
                "--direction", "in", "--topic", "output2", "--frame-type", "IO_MAPPING",
            ])
            self.assertEqual(code, 0)
            self.assertTrue(os.path.exists(registry_path))

            _, subscribe_topics, _ = load_channel_registry(registry_path)
            self.assertIn("output2", subscribe_topics)
            self.assertEqual(subscribe_topics["output2"]["mapping"][0][1], "led_0/isOn")

    def test_registry_entry_round_trips_through_load_channel_registry(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            out_path = os.path.join(tmp_dir, "metrics_env.csv")
            registry_path = os.path.join(tmp_dir, "channels.json")
            code = bootstrap_channel_csv.run([
                "--config", "test_config.json", "--ids", "led_0", "--out", out_path,
                "--registry", registry_path, "--name", "env", "--kind", "metric",
                "--direction", "in", "--topic", "env", "--frame-type", "METRIC_LAYOUT_2",
                "--interval-ms", "25",
            ])
            self.assertEqual(code, 0)
            _, _, metrics_channels = load_channel_registry(registry_path)
            self.assertIn(FrameType.METRIC_LAYOUT_2, metrics_channels)
            self.assertEqual(metrics_channels[FrameType.METRIC_LAYOUT_2]["topic_suffix"], "env")
            self.assertEqual(metrics_channels[FrameType.METRIC_LAYOUT_2]["interval"], 25)

    def test_registry_flag_appends_a_second_channel(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = os.path.join(tmp_dir, "channels.json")
            for name, topic in (("a", "topic_a"), ("b", "topic_b")):
                out_path = os.path.join(tmp_dir, f"{name}.csv")
                code = bootstrap_channel_csv.run([
                    "--config", "test_config.json", "--ids", "led_0", "--out", out_path,
                    "--registry", registry_path, "--name", name,
                    "--direction", "in", "--topic", topic, "--frame-type", "IO_MAPPING",
                ])
                self.assertEqual(code, 0)

            with open(registry_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.assertEqual([c["name"] for c in data["channels"]], ["a", "b"])

    def test_registry_flag_replaces_an_entry_with_the_same_name_instead_of_duplicating(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = os.path.join(tmp_dir, "channels.json")
            out_path = os.path.join(tmp_dir, "output.csv")
            for ids in ("led_0", "led_0,belt_2"):
                code = bootstrap_channel_csv.run([
                    "--config", "test_config.json", "--ids", ids, "--out", out_path,
                    "--registry", registry_path, "--name", "output",
                    "--direction", "in", "--topic", "output", "--frame-type", "IO_MAPPING",
                ])
                self.assertEqual(code, 0)

            with open(registry_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.assertEqual(len(data["channels"]), 1)

    def test_registry_args_are_required_together(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            out_path = os.path.join(tmp_dir, "out.csv")
            registry_path = os.path.join(tmp_dir, "channels.json")
            stderr = io.StringIO()
            with redirect_stderr(stderr):
                code = bootstrap_channel_csv.run([
                    "--config", "test_config.json", "--ids", "led_0", "--out", out_path,
                    "--registry", registry_path,
                ])
            self.assertEqual(code, 1)
            self.assertFalse(os.path.exists(registry_path))
            self.assertIn("--name", stderr.getvalue())

    def test_mapping_csv_path_is_relative_to_the_registry_directory(self):
        """The registry and the CSV can live in different directories -
        channel_registry.load_channel_registry resolves mapping_csv
        relative to the registry file, so the written path must be too."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            csv_dir = os.path.join(tmp_dir, "channels")
            os.makedirs(csv_dir)
            out_path = os.path.join(csv_dir, "output.csv")
            registry_path = os.path.join(tmp_dir, "channels.json")
            code = bootstrap_channel_csv.run([
                "--config", "test_config.json", "--ids", "led_0", "--out", out_path,
                "--registry", registry_path, "--name", "output",
                "--direction", "in", "--topic", "output", "--frame-type", "IO_MAPPING",
            ])
            self.assertEqual(code, 0)
            with open(registry_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.assertEqual(data["channels"][0]["mapping_csv"], "channels/output.csv")
            # And it actually resolves, not just looks right:
            _, subscribe_topics, _ = load_channel_registry(registry_path)
            self.assertIn("output", subscribe_topics)


if __name__ == "__main__":
    unittest.main()
