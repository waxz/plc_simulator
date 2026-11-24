"""test_validate_channel_registry.py - unit tests for
validate_channel_registry.py's run(), plus one real subprocess invocation
proving the CLI entry point itself works end to end."""
import io
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

import validate_channel_registry
from config_parser import load_config
from test_channel_registry import write_default_registry


class ValidateChannelRegistryTest(unittest.TestCase):
    def test_reports_ok_for_a_valid_registry(self):
        config = load_config("station_1.json")
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = write_default_registry(tmp_dir, config)
            stdout = io.StringIO()
            with redirect_stdout(stdout):
                code = validate_channel_registry.run(["--registry", registry_path, "--config", "station_1.json"])
            self.assertEqual(code, 0)
            output = stdout.getvalue()
            self.assertIn("io_mapping:input", output)
            self.assertIn("io_mapping:output", output)
            self.assertIn("metric:metrics", output)
            self.assertIn("Registry is valid.", output)

    def test_reports_the_bad_row_for_an_unknown_component(self):
        config = load_config("station_1.json")
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = write_default_registry(tmp_dir, config)
            # Corrupt the already-written output.csv with a component that
            # doesn't exist in station_1.json.
            with open(os.path.join(tmp_dir, "output.csv"), "a", encoding="utf-8") as f:
                f.write("ByteType.BIT,not_a_real_component/isOn\n")

            stderr = io.StringIO()
            with redirect_stderr(stderr):
                code = validate_channel_registry.run(["--registry", registry_path, "--config", "station_1.json"])
            self.assertEqual(code, 1)
            self.assertIn("not_a_real_component", stderr.getvalue())

    def test_malformed_registry_file_is_reported_not_raised(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = os.path.join(tmp_dir, "channels.json")
            with open(registry_path, "w", encoding="utf-8") as f:
                f.write("{not valid json")
            stderr = io.StringIO()
            with redirect_stderr(stderr):
                code = validate_channel_registry.run(["--registry", registry_path, "--config", "station_1.json"])
            self.assertEqual(code, 1)
            self.assertIn(registry_path, stderr.getvalue())

    def test_cli_entry_point_runs_as_a_real_subprocess(self):
        config = load_config("station_1.json")
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = write_default_registry(tmp_dir, config)
            result = subprocess.run(
                [sys.executable, "validate_channel_registry.py",
                 "--registry", registry_path, "--config", "station_1.json"],
                capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("Registry is valid.", result.stdout)


if __name__ == "__main__":
    unittest.main()
