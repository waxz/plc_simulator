"""test_generate_frame_type.py - unit tests for generate_frame_type.py's
parsing/insertion helpers, plus end-to-end runs against a temporary copy of
the real tcp_framed_transport.py (never the checked-in file itself) via
--target, including one real subprocess invocation of the CLI entry point.
"""
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

import generate_frame_type as gft
from tcp_framed_transport import FrameType

REAL_TARGET = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tcp_framed_transport.py")


def write_registry(path: str, metric_frame_types, io_mapping_frame_type=None):
    channels = []
    for i, frame_type in enumerate(metric_frame_types):
        channels.append({
            "name": f"m{i}", "kind": "metric", "direction": "in",
            "topic": f"t{i}", "frame_type": frame_type, "mapping_csv": f"m{i}.csv",
        })
    if io_mapping_frame_type:
        channels.append({
            "name": "io", "kind": "io_mapping", "direction": "in",
            "topic": "io", "frame_type": io_mapping_frame_type, "mapping_csv": "io.csv",
        })
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"channels": channels}, f)


class RegistryMetricFrameTypesTest(unittest.TestCase):
    def test_collects_only_metric_channels_deduplicated(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            registry_path = os.path.join(tmp_dir, "channels.json")
            write_registry(registry_path, ["METRIC_A", "METRIC_B", "METRIC_A"], io_mapping_frame_type="IO_MAPPING")
            names = gft.registry_metric_frame_types(registry_path)
            self.assertEqual(names, ["METRIC_A", "METRIC_B"])


class MembersDefinedInSourceTest(unittest.TestCase):
    def test_parses_name_value_pairs_from_the_class_body(self):
        source = (
            "class FrameType(IntEnum):\n"
            '    """doc"""\n'
            "    IO_MAPPING = 1      # comment\n"
            "    METRIC_LAYOUT_1 = 2  # comment\n"
            "\n\nclass Other:\n"
            "    pass\n"
        )
        self.assertEqual(gft.members_defined_in_source(source), {"IO_MAPPING": 1, "METRIC_LAYOUT_1": 2})


class InsertNewMembersTest(unittest.TestCase):
    def test_appends_after_the_last_member_preserving_everything_else(self):
        source = (
            "class FrameType(IntEnum):\n"
            '    """doc"""\n'
            "    IO_MAPPING = 1      # comment\n"
            "    METRIC_LAYOUT_1 = 2  # comment\n"
            "\n\nclass Other:\n"
            "    pass\n"
        )
        updated = gft._insert_new_members(source, {"METRIC_LAYOUT_3": 3})
        self.assertIn("    METRIC_LAYOUT_3 = 3  # added by generate_frame_type.py\n", updated)
        # Order: new member lands right after the last existing one, before the blank line / next class.
        lines = updated.splitlines()
        self.assertEqual(lines[3], "    METRIC_LAYOUT_1 = 2  # comment")
        self.assertEqual(lines[4], "    METRIC_LAYOUT_3 = 3  # added by generate_frame_type.py")
        self.assertEqual(lines[7], "class Other:")

    def test_raises_if_the_class_is_not_found(self):
        with self.assertRaises(RuntimeError):
            gft._insert_new_members("class SomethingElse:\n    pass\n", {"X": 1})


class RunEndToEndTest(unittest.TestCase):
    """Every run here targets a throwaway COPY of the real file - never
    REAL_TARGET itself."""

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp_dir.cleanup)
        self.target = os.path.join(self.tmp_dir.name, "tcp_framed_transport.py")
        shutil.copyfile(REAL_TARGET, self.target)
        self.registry_path = os.path.join(self.tmp_dir.name, "channels.json")

    def test_adds_a_new_member_with_the_next_free_value(self):
        write_registry(self.registry_path, ["METRIC_LAYOUT_TEST"])
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = gft.run(["--registry", self.registry_path, "--target", self.target])
        self.assertEqual(code, 0)

        with open(self.target, "r", encoding="utf-8") as f:
            updated = f.read()
        expected_value = max(m.value for m in FrameType) + 1
        self.assertIn(f"METRIC_LAYOUT_TEST = {expected_value}", updated)
        compile(updated, self.target, "exec")  # still valid Python

    def test_leaves_existing_members_values_untouched(self):
        write_registry(self.registry_path, ["METRIC_LAYOUT_1", "METRIC_LAYOUT_TEST"])
        gft.run(["--registry", self.registry_path, "--target", self.target])
        with open(self.target, "r", encoding="utf-8") as f:
            members = gft.members_defined_in_source(f.read())
        self.assertEqual(members["METRIC_LAYOUT_1"], FrameType.METRIC_LAYOUT_1.value)
        self.assertEqual(members["IO_MAPPING"], FrameType.IO_MAPPING.value)

    def test_ignores_io_mapping_channels(self):
        write_registry(self.registry_path, [], io_mapping_frame_type="SOME_IO_NAME")
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = gft.run(["--registry", self.registry_path, "--target", self.target])
        self.assertEqual(code, 0)
        self.assertIn("nothing to do", stdout.getvalue())
        with open(self.target, "r", encoding="utf-8") as f:
            self.assertNotIn("SOME_IO_NAME", f.read())

    def test_already_defined_frame_types_report_nothing_to_do(self):
        write_registry(self.registry_path, ["METRIC_LAYOUT_1", "METRIC_LAYOUT_2"])
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = gft.run(["--registry", self.registry_path, "--target", self.target])
        self.assertEqual(code, 0)
        self.assertIn("nothing to do", stdout.getvalue())
        with open(self.target, "r", encoding="utf-8") as f:
            original = f.read()
        with open(REAL_TARGET, "r", encoding="utf-8") as f:
            self.assertEqual(original, f.read())  # untouched, byte for byte

    def test_check_mode_reports_without_writing(self):
        write_registry(self.registry_path, ["METRIC_LAYOUT_TEST"])
        with open(self.target, "r", encoding="utf-8") as f:
            before = f.read()
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = gft.run(["--registry", self.registry_path, "--target", self.target, "--check"])
        self.assertEqual(code, 0)
        self.assertIn("METRIC_LAYOUT_TEST", stdout.getvalue())
        with open(self.target, "r", encoding="utf-8") as f:
            self.assertEqual(before, f.read())

    def test_a_second_run_in_the_same_process_is_idempotent(self):
        """The real-world case this guards: run it twice without restarting
        Python in between (so the live FrameType import still doesn't know
        about the first run's addition) - the second run must still see it,
        by reading the target file's own text, and add nothing twice."""
        write_registry(self.registry_path, ["METRIC_LAYOUT_TEST"])
        gft.run(["--registry", self.registry_path, "--target", self.target])
        with open(self.target, "r", encoding="utf-8") as f:
            after_first = f.read()

        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = gft.run(["--registry", self.registry_path, "--target", self.target])
        self.assertEqual(code, 0)
        self.assertIn("nothing to do", stdout.getvalue())
        with open(self.target, "r", encoding="utf-8") as f:
            after_second = f.read()
        self.assertEqual(after_first, after_second)  # not duplicated

    def test_a_lowercase_or_mixed_case_name_is_still_idempotent(self):
        """Regression: FrameType names aren't required to be uppercase (a
        person can type anything into the web editor's Frame type field),
        and members_defined_in_source() used to only recognize ALL-CAPS
        member lines - a lowercase name like "fh" was invisible to its own
        re-run, so a second click of "Generate FrameType" duplicated it."""
        write_registry(self.registry_path, ["fh"])
        gft.run(["--registry", self.registry_path, "--target", self.target])
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = gft.run(["--registry", self.registry_path, "--target", self.target])
        self.assertEqual(code, 0)
        self.assertIn("nothing to do", stdout.getvalue())
        with open(self.target, "r", encoding="utf-8") as f:
            content = f.read()
        self.assertEqual(content.count("fh = "), 1)

    def test_missing_registry_file_fails_cleanly(self):
        stderr = io.StringIO()
        with redirect_stderr(stderr):
            code = gft.run(["--registry", os.path.join(self.tmp_dir.name, "nope.json"), "--target", self.target])
        self.assertEqual(code, 1)
        self.assertIn("nope.json", stderr.getvalue())

    def test_cli_entry_point_runs_as_a_real_subprocess(self):
        write_registry(self.registry_path, ["METRIC_LAYOUT_TEST"])
        result = subprocess.run(
            [sys.executable, "generate_frame_type.py",
             "--registry", self.registry_path, "--target", self.target],
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        with open(self.target, "r", encoding="utf-8") as f:
            self.assertIn("METRIC_LAYOUT_TEST", f.read())


if __name__ == "__main__":
    unittest.main()
