"""test_web_app.py - tests for web_app.py's Flask routes: the registry
editor API (config catalog, load/validate/save) and the live metrics API,
using Flask's test client so nothing here needs a real running server."""
import json
import os
import shutil
import tempfile
import unittest
from unittest.mock import patch

from config_parser import load_config
from metrics_store import MetricsStore
from test_channel_registry import write_default_registry
from tcp_framed_transport import FrameType
import generate_frame_type
import mqtt_data_exchange_codegen
from web_app import create_app

REAL_FRAME_TYPE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tcp_framed_transport.py")
REAL_MQTT_DATA_EXCHANGE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mqtt_data_exchange.py")


def valid_channel_body():
    return {
        "name": "output_test",
        "kind": "io_mapping",
        "direction": "in",
        "topic": "output_test",
        "interval_ms": 10,
        "frame_type": "IO_MAPPING",
        "mapping_csv": "output_test.csv",
        "rows": [
            {"byte_type": "BIT", "path": "led_0/isOn"},
            {"byte_type": "PAD", "path": "auto"},
            {"byte_type": "FLOAT", "path": "belt_2/speed"},
        ],
    }


class RegistryApiTest(unittest.TestCase):
    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp_dir.cleanup)
        self.registry_path = os.path.join(self.tmp_dir.name, "channels.json")
        app = create_app(config_path="test_fixture_config.json", registry_path=self.registry_path)
        app.testing = True
        self.client = app.test_client()

    def test_config_endpoint_lists_known_components(self):
        res = self.client.get("/api/config")
        body = res.get_json()
        self.assertEqual(res.status_code, 200)
        ids = {c["id"] for c in body["components"]}
        self.assertIn("led_0", ids)
        self.assertIn("belt_2", ids)
        self.assertIn("clock", ids)

    def test_registry_endpoint_returns_empty_channels_for_a_missing_file(self):
        res = self.client.get("/api/registry")
        body = res.get_json()
        self.assertEqual(res.status_code, 200)
        self.assertEqual(body["channels"], [])
        self.assertEqual(body["path"], self.registry_path)

    def test_registry_endpoint_inlines_csv_rows_for_an_existing_registry(self):
        config = load_config("test_fixture_config.json")
        write_default_registry(self.tmp_dir.name, config)
        # write_default_registry names its file channels.json in the same dir already
        app = create_app(config_path="test_fixture_config.json",
                          registry_path=os.path.join(self.tmp_dir.name, "channels.json"))
        client = app.test_client()
        res = client.get("/api/registry")
        body = res.get_json()
        names = {c["name"] for c in body["channels"]}
        self.assertEqual(names, {"input", "output", "metrics"})
        output_channel = next(c for c in body["channels"] if c["name"] == "output")
        self.assertTrue(len(output_channel["rows"]) > 0)
        self.assertIn("byte_type", output_channel["rows"][0])

    def test_validate_reports_ok_for_a_valid_channel(self):
        res = self.client.post("/api/validate", json={"channels": [valid_channel_body()]})
        body = res.get_json()
        self.assertEqual(res.status_code, 200)
        self.assertTrue(body["results"][0]["ok"])
        self.assertIsInstance(body["results"][0]["payload_size"], int)

    def test_validate_reports_the_row_for_an_unknown_component(self):
        bad = valid_channel_body()
        bad["rows"].append({"byte_type": "BIT", "path": "not_a_real_component/isOn"})
        res = self.client.post("/api/validate", json={"channels": [bad]})
        body = res.get_json()
        result = body["results"][0]
        self.assertFalse(result["ok"])
        self.assertTrue(any("not_a_real_component" in e for e in result["errors"]))

    def test_a_saved_metric_channel_round_trips_through_get_registry(self):
        """Regression check for "the web UI should load existing config":
        save a channel with real topic/frame_type/group/name values (what
        the form actually submits), then confirm GET /api/registry hands
        back exactly what a fresh page load would need to repopulate the
        form - if this passes, a channel appearing empty after a reload
        means it was never actually saved, not a load bug."""
        channel = {
            "name": "perf_metrics", "kind": "metric", "direction": "in", "topic": "metrics",
            "frame_type": "METRIC_LAYOUT_1", "mapping_csv": "perf_metrics.csv",
            "rows": [
                {"byte_type": "BIT", "path": "flags/armed"},
                {"byte_type": "BIT", "path": "flags/faulted"},
                {"byte_type": "FLOAT", "path": "perf/scan_time_ms"},
            ],
        }
        save_res = self.client.post("/api/registry", json={"channels": [channel]})
        self.assertTrue(save_res.get_json()["saved"], save_res.get_json())

        get_res = self.client.get("/api/registry")
        loaded = get_res.get_json()["channels"][0]
        self.assertEqual(loaded["topic"], "metrics")
        self.assertEqual(loaded["frame_type"], "METRIC_LAYOUT_1")
        self.assertEqual(loaded["rows"], [
            {"byte_type": "BIT", "path": "flags/armed"},
            {"byte_type": "BIT", "path": "flags/faulted"},
            {"byte_type": "FLOAT", "path": "perf/scan_time_ms"},
        ])

    def test_validate_reports_ok_for_a_metric_channel_with_free_form_field_names(self):
        """Regression: verify_component_list() (real config components -
        sensors/belts/buttons/leds) must never run against a metric
        channel's rows - "perf/scan_time_ms" isn't a component id and was
        never meant to be checked as one."""
        channel = {
            "name": "perf_metrics", "kind": "metric", "direction": "in", "topic": "perf",
            "frame_type": "METRIC_LAYOUT_1",
            "rows": [
                {"byte_type": "FLOAT", "path": "perf/scan_time_ms"},
                {"byte_type": "UINT32", "path": "perf/sample_count"},
            ],
        }
        res = self.client.post("/api/validate", json={"channels": [channel]})
        result = res.get_json()["results"][0]
        self.assertTrue(result["ok"], result["errors"])
        self.assertEqual(result["payload_size"], 8)

    def test_validate_flags_a_metric_row_with_a_blank_name(self):
        """A half-filled metric row (Group set, Name still blank) must fail
        validation with the row number, so the editor highlights it as you
        type instead of it only blowing up at save."""
        channel = {
            "name": "perf_metrics", "kind": "metric", "direction": "in", "topic": "perf",
            "frame_type": "METRIC_LAYOUT_1",
            "rows": [
                {"byte_type": "BIT", "path": "n/led"},
                {"byte_type": "FLOAT", "path": "n/"},  # Name blank
            ],
        }
        res = self.client.post("/api/validate", json={"channels": [channel]})
        result = res.get_json()["results"][0]
        self.assertFalse(result["ok"])
        self.assertTrue(any(":2:" in e and "Name" in e for e in result["errors"]), result["errors"])

    def test_validate_reports_duplicate_channel_names(self):
        channels = [valid_channel_body(), valid_channel_body()]
        res = self.client.post("/api/validate", json={"channels": channels})
        body = res.get_json()
        self.assertTrue(all(not r["ok"] for r in body["results"]))
        self.assertTrue(any("more than one channel" in e for r in body["results"] for e in r["errors"]))

    def test_validate_reports_duplicate_frame_types_across_metric_channels(self):
        m1 = {"name": "m1", "kind": "metric", "direction": "in", "topic": "t1",
              "frame_type": "METRIC_LAYOUT_1", "rows": [{"byte_type": "FLOAT", "path": "perf/x"}]}
        m2 = {"name": "m2", "kind": "metric", "direction": "in", "topic": "t2",
              "frame_type": "METRIC_LAYOUT_1", "rows": [{"byte_type": "FLOAT", "path": "perf/y"}]}
        res = self.client.post("/api/validate", json={"channels": [m1, m2]})
        body = res.get_json()
        self.assertTrue(any("also used by channel" in e for r in body["results"] for e in r["errors"]))

    def test_save_writes_the_registry_and_csv_files(self):
        res = self.client.post("/api/registry", json={"channels": [valid_channel_body()]})
        body = res.get_json()
        self.assertEqual(res.status_code, 200)
        self.assertTrue(body["saved"])
        self.assertTrue(os.path.exists(self.registry_path))

        with open(self.registry_path, "r", encoding="utf-8") as f:
            saved = json.load(f)
        self.assertEqual(saved["channels"][0]["name"], "output_test")
        self.assertNotIn("rows", saved["channels"][0])  # on-disk shape has no inline rows

        csv_path = os.path.join(self.tmp_dir.name, "output_test.csv")
        self.assertTrue(os.path.exists(csv_path))
        with open(csv_path, "r", encoding="utf-8") as f:
            content = f.read()
        self.assertIn("PAD", content)  # saved as authored, not expanded

    def test_save_rejects_an_invalid_registry_without_writing_files(self):
        bad = valid_channel_body()
        bad["rows"] = [{"byte_type": "NOT_A_TYPE", "path": "x/y"}]
        res = self.client.post("/api/registry", json={"channels": [bad]})
        body = res.get_json()
        self.assertEqual(res.status_code, 400)
        self.assertFalse(body["saved"])
        self.assertFalse(os.path.exists(self.registry_path))

    def test_config_endpoint_reports_each_sensors_own_pack_format(self):
        res = self.client.get("/api/config")
        body = res.get_json()
        sensor_1 = next(c for c in body["components"] if c["id"] == "sensor_1")
        self.assertEqual(sensor_1["pack_format"], "uint32")
        self.assertEqual(sensor_1["bits"], 32)
        other_sensor = next(c for c in body["components"] if c["type"] == "sensor" and c["id"] != "sensor_1")
        self.assertEqual(other_sensor["pack_format"], "bit")

    def test_validate_with_component_ids_regenerates_rows_from_config(self):
        """The core of the fix: no ByteType/padding is authored by the
        caller - component_ids alone produces sensor_1's compact UINT32
        field, exactly as custom_component_mapping() would build it."""
        channel = {
            "name": "output_node2", "kind": "io_mapping", "direction": "in",
            "topic": "output2", "frame_type": "IO_MAPPING", "mapping_csv": "output_node2.csv",
            "component_ids": ["sensor_1"],
        }
        res = self.client.post("/api/validate", json={"channels": [channel]})
        result = res.get_json()["results"][0]
        self.assertTrue(result["ok"], result["errors"])
        self.assertEqual(result["rows"], [{"byte_type": "UINT32", "path": "sensor_1/sensor_states"}])
        self.assertEqual(result["payload_size"], 4)

    def test_validate_with_component_ids_reports_an_unknown_id(self):
        channel = {
            "name": "output_node2", "kind": "io_mapping", "direction": "in",
            "topic": "output2", "component_ids": ["not_a_real_component"],
        }
        res = self.client.post("/api/validate", json={"channels": [channel]})
        result = res.get_json()["results"][0]
        self.assertFalse(result["ok"])
        self.assertTrue(any("not_a_real_component" in e for e in result["errors"]))

    def test_validate_with_component_ids_auto_pads_bit_items(self):
        """Two single-bit buttons plus sensor_1 (compact UINT32) - the 6
        padding bits between them must appear with no PAD row authored."""
        channel = {
            "name": "mixed", "kind": "io_mapping", "direction": "in", "topic": "mixed",
            "component_ids": ["button_0", "sensor_1"],
        }
        res = self.client.post("/api/validate", json={"channels": [channel]})
        result = res.get_json()["results"][0]
        self.assertTrue(result["ok"], result["errors"])
        byte_types = [r["byte_type"] for r in result["rows"]]
        self.assertEqual(byte_types[0], "BIT")
        self.assertEqual(byte_types.count("BIT"), 8)  # 1 real bit + 7 auto padding
        self.assertEqual(result["rows"][-1], {"byte_type": "UINT32", "path": "sensor_1/sensor_states"})

    def test_save_with_component_ids_persists_ids_and_writes_a_resolved_csv(self):
        channel = {
            "name": "output_node2", "kind": "io_mapping", "direction": "in",
            "topic": "output2", "frame_type": "IO_MAPPING", "mapping_csv": "output_node2.csv",
            "component_ids": ["sensor_1"],
        }
        res = self.client.post("/api/registry", json={"channels": [channel]})
        body = res.get_json()
        self.assertTrue(body["saved"], body.get("results"))

        with open(self.registry_path, "r", encoding="utf-8") as f:
            saved = json.load(f)
        self.assertEqual(saved["channels"][0]["component_ids"], ["sensor_1"])
        self.assertNotIn("rows", saved["channels"][0])

        csv_path = os.path.join(self.tmp_dir.name, "output_node2.csv")
        with open(csv_path, "r", encoding="utf-8") as f:
            content = f.read()
        self.assertIn("UINT32", content)
        self.assertIn("sensor_1/sensor_states", content)

    def test_registry_get_recomputes_rows_for_a_component_ids_channel(self):
        channel = {
            "name": "output_node2", "kind": "io_mapping", "direction": "in",
            "topic": "output2", "frame_type": "IO_MAPPING", "mapping_csv": "output_node2.csv",
            "component_ids": ["sensor_1"],
        }
        self.client.post("/api/registry", json={"channels": [channel]})
        res = self.client.get("/api/registry")
        loaded = res.get_json()["channels"][0]
        self.assertEqual(loaded["component_ids"], ["sensor_1"])
        self.assertEqual(loaded["rows"], [{"byte_type": "UINT32", "path": "sensor_1/sensor_states"}])

    def test_config_endpoint_errors_cleanly_without_config_configured(self):
        app = create_app()
        client = app.test_client()
        self.assertEqual(client.get("/api/config").status_code, 400)

    def test_registry_endpoint_falls_back_to_hardcoded_definitions_without_a_registry_path(self):
        """No --channel-registry given: the editor shows what's actually
        running (PUBLISH_TOPICS/SUBSCRIBE_TOPICS/METRICS_CHANNELS) instead
        of a blank page or an error."""
        app = create_app(config_path="test_fixture_config.json")
        client = app.test_client()
        res = client.get("/api/registry")
        body = res.get_json()
        self.assertEqual(res.status_code, 200)
        self.assertEqual(body["source"], "hardcoded")
        names = {c["name"] for c in body["channels"]}
        # A subset check, not assertEqual: this reads the actual project
        # file, which the web editor itself is free to add channels to at
        # any time - pinning an exact, closed set would fail on every
        # legitimate new channel instead of only on a real regression.
        self.assertTrue({"input", "output", "metrics", "metrics2"}.issubset(names))
        output = next(c for c in body["channels"] if c["name"] == "output")
        self.assertTrue(len(output["rows"]) > 0)  # computed from config, since one was given

    def test_registry_endpoint_falls_back_to_hardcoded_definitions_even_without_config(self):
        app = create_app()  # no config either - read_hardcoded_channels() itself needs none
        client = app.test_client()
        res = client.get("/api/registry")
        self.assertEqual(res.status_code, 200)
        self.assertEqual(res.get_json()["source"], "hardcoded")

    def test_index_serves_the_editor_page(self):
        res = self.client.get("/")
        self.assertEqual(res.status_code, 200)
        self.assertIn(b"Channel Registry", res.data)


class GenerateFrameTypesApiTest(unittest.TestCase):
    """The web counterpart to generate_frame_type.py's CLI - every test
    here patches default_target_file() to a throwaway copy so the real
    tcp_framed_transport.py in this repo is never touched."""

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp_dir.cleanup)
        self.target = os.path.join(self.tmp_dir.name, "tcp_framed_transport.py")
        shutil.copyfile(REAL_FRAME_TYPE_FILE, self.target)

        patcher = patch.object(generate_frame_type, "default_target_file", return_value=self.target)
        patcher.start()
        self.addCleanup(patcher.stop)

        app = create_app()
        self.client = app.test_client()

    def test_adds_a_new_member_and_says_a_restart_is_needed(self):
        res = self.client.post("/api/frame-types/generate", json={"names": ["METRIC_LAYOUT_TEST"]})
        body = res.get_json()
        self.assertEqual(res.status_code, 200)
        expected_value = max(m.value for m in FrameType) + 1
        self.assertEqual(body["added"], {"METRIC_LAYOUT_TEST": expected_value})
        self.assertIn("restart", body["message"].lower())

        with open(self.target, "r", encoding="utf-8") as f:
            self.assertIn(f"METRIC_LAYOUT_TEST = {expected_value}", f.read())

    def test_already_defined_name_adds_nothing(self):
        res = self.client.post("/api/frame-types/generate", json={"names": ["METRIC_LAYOUT_1"]})
        body = res.get_json()
        self.assertEqual(body["added"], {})
        self.assertIn("nothing to add", body["message"].lower())
        with open(self.target, "r", encoding="utf-8") as f:
            target_content = f.read()
        with open(REAL_FRAME_TYPE_FILE, "r", encoding="utf-8") as f:
            self.assertEqual(target_content, f.read())

    def test_requires_at_least_one_name(self):
        res = self.client.post("/api/frame-types/generate", json={"names": []})
        self.assertEqual(res.status_code, 400)

    def test_does_not_touch_the_real_project_file(self):
        """Guards the test suite itself: confirms the patch actually took
        effect, so a bug here can't silently start editing the real file."""
        self.client.post("/api/frame-types/generate", json={"names": ["METRIC_LAYOUT_TEST_2"]})
        with open(REAL_FRAME_TYPE_FILE, "r", encoding="utf-8") as f:
            self.assertNotIn("METRIC_LAYOUT_TEST_2", f.read())


class HardcodedRegistrySaveApiTest(unittest.TestCase):
    """POST /api/registry when the server has no --channel-registry: it
    should rewrite mqtt_data_exchange.py in place (see
    mqtt_data_exchange_codegen.py). Every test here patches
    default_target_file() to a throwaway copy so the real file is never
    touched."""

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp_dir.cleanup)
        self.target = os.path.join(self.tmp_dir.name, "mqtt_data_exchange.py")
        shutil.copyfile(REAL_MQTT_DATA_EXCHANGE_FILE, self.target)

        patcher = patch.object(mqtt_data_exchange_codegen, "default_target_file", return_value=self.target)
        patcher.start()
        self.addCleanup(patcher.stop)

        app = create_app(config_path="test_fixture_config.json")  # no registry_path -> hardcoded mode
        self.client = app.test_client()

    def test_saving_an_edit_rewrites_the_target_file_and_backs_up_the_original(self):
        channels = mqtt_data_exchange_codegen.read_hardcoded_channels()
        metrics = next(c for c in channels if c["name"] == "metrics")
        metrics["rows"] = [{"byte_type": "FLOAT", "path": "perf/edited_field"}]

        res = self.client.post("/api/registry", json={"channels": channels})
        body = res.get_json()
        self.assertEqual(res.status_code, 200)
        self.assertTrue(body["saved"], body)
        self.assertIn("restart", body["message"].lower())

        with open(self.target, "r", encoding="utf-8") as f:
            # The edit lands as a typed field on the regenerated layout
            # dataclass, not a "group/name" string.
            self.assertIn("edited_field: f32", f.read())
        self.assertTrue(os.path.exists(self.target + ".bak"))

    def test_an_invalid_edit_is_rejected_and_the_file_is_untouched(self):
        with open(self.target, "r", encoding="utf-8") as f:
            before = f.read()
        channels = mqtt_data_exchange_codegen.read_hardcoded_channels()
        output = next(c for c in channels if c["name"] == "output")
        output["component_ids"] = ["not_a_real_component"]

        res = self.client.post("/api/registry", json={"channels": channels})
        body = res.get_json()
        self.assertEqual(res.status_code, 400)
        self.assertFalse(body["saved"])

        with open(self.target, "r", encoding="utf-8") as f:
            self.assertEqual(f.read(), before)

    def test_does_not_touch_the_real_project_file(self):
        # A REAL_MQTT_DATA_EXCHANGE_FILE.bak can legitimately already exist
        # here - someone using the actual web editor against the real file
        # leaves one behind on every save. What this test guards against is
        # THIS test creating or changing one, so it snapshots before/after
        # instead of asserting outright non-existence, which would fail on
        # that unrelated, legitimate state instead of on an actual leak.
        bak_path = REAL_MQTT_DATA_EXCHANGE_FILE + ".bak"
        bak_before = open(bak_path, "r", encoding="utf-8").read() if os.path.exists(bak_path) else None

        channels = mqtt_data_exchange_codegen.read_hardcoded_channels()
        self.client.post("/api/registry", json={"channels": channels})

        with open(REAL_MQTT_DATA_EXCHANGE_FILE, "r", encoding="utf-8") as f:
            self.assertNotIn(".bak", f.read())  # sanity: nothing about this leaked into the real file
        bak_after = open(bak_path, "r", encoding="utf-8").read() if os.path.exists(bak_path) else None
        self.assertEqual(bak_after, bak_before)


class MetricsApiTest(unittest.TestCase):
    def test_endpoints_return_503_with_no_live_store(self):
        app = create_app()
        client = app.test_client()
        for path in ("/api/metrics/tree", "/api/metrics/latest", "/api/metrics/series?group=g&name=n"):
            res = client.get(path)
            self.assertEqual(res.status_code, 503)

    def test_tree_and_latest_reflect_a_live_store(self):
        store = MetricsStore()
        store.add_sample(name="scan_time_ms", value=8.5, group="node0/perf")
        store.add_sample(name="sample_count", value=3, group="node0/perf")
        app = create_app(metrics_store=store)
        client = app.test_client()

        tree = client.get("/api/metrics/tree").get_json()
        self.assertEqual(set(tree["node0/perf"].keys()), {"scan_time_ms", "sample_count"})

        latest = client.get("/api/metrics/latest").get_json()
        values = {row["name"]: row["value"] for row in latest}
        self.assertEqual(values["scan_time_ms"], 8.5)
        self.assertEqual(values["sample_count"], 3)

    def test_series_only_returns_points_after_since(self):
        store = MetricsStore()
        store.add_sample(name="x", value=1.0, group="g", ts=100.0)
        store.add_sample(name="x", value=2.0, group="g", ts=200.0)
        app = create_app(metrics_store=store)
        client = app.test_client()

        all_points = client.get("/api/metrics/series?group=g&name=x").get_json()
        self.assertEqual(len(all_points), 2)

        newer_only = client.get("/api/metrics/series?group=g&name=x&since=150").get_json()
        self.assertEqual(len(newer_only), 1)
        self.assertEqual(newer_only[0]["value"], 2.0)

    def test_vector_metrics_round_trip_as_lists(self):
        store = MetricsStore()
        store.add_sample(name="v", value=[1.0, 2.0], group="g", kind="vector")
        app = create_app(metrics_store=store)
        client = app.test_client()
        points = client.get("/api/metrics/series?group=g&name=v").get_json()
        self.assertEqual(points[0]["value"], [1.0, 2.0])


if __name__ == "__main__":
    unittest.main()
