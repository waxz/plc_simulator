"""
test_foxglove_bridge.py - the PLC -> Foxglove middleware, end to end over
real sockets: a PLC (TcpFramedClient, the same encoders plc_mock uses) sends
raw binary frames to plc_foxglove_bridge.py, and a Foxglove WebSocket client
(foxglove_ws_probe.FoxgloveProbe) subscribes and checks the JSON it gets:
outputs, inputs, line states, metrics, events, stats, the layout check
reply - and plc_mock.py itself reporting to the bridge with no simulator.
"""
import copy
import json
import os
import socket
import subprocess
import sys
import time
import unittest

import devices
import io_layout
import plc_records
from config_parser import load_config
from foxglove_ws_probe import FoxgloveProbe
from mqtt_data_exchange import MetricLayout1, MetricLayout1Perf, MetricLayout1Results, apply_exchange_config
from mqtt_message_binding import ComponentMessage, MetricsMessage
from plc_foxglove_bridge import PlcFoxgloveBridge, decode_fields
from plc_logic import LineController
from tcp_framed_transport import FrameType, TcpFramedClient

HERE = os.path.dirname(os.path.abspath(__file__))


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_until(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config(os.path.join(HERE, "station_1.json"))
        apply_exchange_config(self.config, os.path.join(HERE, "tcp_exchange_config.json"))
        self.tcp_port, self.ws_port = free_port(), free_port()
        self.bridge = PlcFoxgloveBridge(copy.deepcopy(self.config), listen=("127.0.0.1", self.tcp_port),
                                        foxglove_at=("127.0.0.1", self.ws_port))
        self.bridge.start()
        self.addCleanup(self.bridge.stop)
        self.replies = []
        self.plc = TcpFramedClient("127.0.0.1", self.tcp_port, self.replies.append,
                                   length_endianness=self.config.mqttSettings.endianness)
        self.plc.start()
        self.addCleanup(self.plc.stop)
        self.assertTrue(wait_until(lambda: self.plc.connected))
        self.probe = FoxgloveProbe(f"ws://127.0.0.1:{self.ws_port}")
        self.addCleanup(self.probe.close)
        topics = ["/plc/outputs", "/plc/inputs", "/plc/lines", "/plc/events", "/bridge/stats", "/plc/metrics/metrics"]
        advertised = self.probe.wait_for_topics(topics, timeout=5.0)
        self.assertTrue(set(topics) <= set(advertised), sorted(advertised))
        self.probe.subscribe(topics)
        # The PLC side: this layout's program and I/O images, as plc_mock builds them.
        self.ctl = LineController(self.config)
        self.out_msg, self.in_msg = self.bridge.output_msg, self.bridge.input_msg
        time.sleep(1.6)   # the WebSocket server starts delivering a new subscription after ~1.5 s

    def send_images(self):
        self.ctl.step()
        self.plc.send(self.out_msg.serialize(self.config), frame_type=FrameType.IO_MAPPING)
        self.plc.send(self.in_msg.serialize(self.config), frame_type=FrameType.INPUT_IMAGE)

    def collect(self, seconds=1.0):
        got = {}
        for topic, _t, msg in self.probe.messages(seconds):
            got.setdefault(topic, []).append(msg)
        return got

    def test_io_images_become_json_with_line_states(self):
        buttons = {b.label: b for b in self.config.buttons}
        for label in ("L1 START", "L2 START"):
            devices.press(buttons[label])
            self.ctl.step()
            devices.release(buttons[label])
        banner = next(s for s in self.config.sensors if s.label == "L2 BANNER")
        banner.sensor_states = "0" * 8 + "1" * 6 + "0" * 18          # 6 beams blocked: L2 holds?
        ir = next(s for s in self.config.sensors if s.label == "L1 IR")
        ir.sensor_states = "1"                                       # parcel at the merge: yes, gives way
        for _ in range(3):
            self.send_images()
            time.sleep(0.05)
        got = self.collect(1.5)

        out = got["/plc/outputs"][-1]
        self.assertEqual(out["belts"]["belt_1_1_1"], {"speed": 1.0, "run": True, "moving": True})
        self.assertEqual(out["belts"]["belt_1_2_1"]["moving"], False)              # L2 gives way
        self.assertEqual(out["belts"]["belt_1_3_1"]["moving"], False)              # L3 not started
        self.assertEqual(out["leds"]["led_1_1_1"], {"on": True, "label": "L1 AUTO"})
        self.assertEqual(out["leds"]["led_1_3_2"], {"on": True, "label": "L3 MANUAL"})
        self.assertIn("timestamp", out)

        inp = got["/plc/inputs"][-1]
        self.assertEqual(inp["sensors"]["sensor_1_2_1"]["beams"], banner.sensor_states)
        self.assertEqual(inp["sensors"]["sensor_1_2_1"]["beams_blocked"], 6)
        self.assertTrue(inp["sensors"]["sensor_1_1_1"]["blocked"])
        self.assertFalse(inp["sensors"]["sensor_1_3_1"]["blocked"])
        self.assertEqual(inp["buttons"]["button_1_1_2"], {"pressed": False, "label": "L1 STOP"})   # NC, released
        self.assertGreaterEqual(inp["image_age_ms"], 0.0)

        lines = got["/plc/lines"][-1]["lines"]
        self.assertEqual({n: v["state"] for n, v in lines.items()}, {"L1": "RUN", "L2": "HOLD", "L3": "STOP"})
        self.assertEqual(lines["L2"]["sensors_blocked"], ["sensor_1_2_1"])
        self.assertEqual(lines["L1"]["belts_moving"], 5)

    def test_events_metrics_stats_and_a_bad_frame(self):
        self.send_images()
        metrics = MetricsMessage.from_layout(MetricLayout1, endianness="<")
        sample = MetricLayout1(perf=MetricLayout1Perf(scan_time_ms=50.0, cycle_time_ms=3.5, fn_elapsed_us=120.0,
                                                      sample_count=7),
                               results=MetricLayout1Results(last_result_code=0, belt_1_velocity=(1.25, 0.0)))
        self.plc.send(metrics.serialize(sample), frame_type=FrameType.METRIC_LAYOUT_1, node_id=3)
        self.plc.send(b"\x01\x02", frame_type=FrameType.IO_MAPPING)                   # wrong size
        got = self.collect(2.2)
        m = got["/plc/metrics/metrics"][-1]
        self.assertEqual(m["node"], 3)
        self.assertEqual(m["perf"]["sample_count"], 7)
        self.assertAlmostEqual(m["perf"]["cycle_time_ms"], 3.5)
        self.assertEqual(m["results"]["belt_1_velocity"], [1.25, 0.0])
        stats = got["/bridge/stats"][-1]
        self.assertEqual(stats["malformed"], 1)
        self.assertEqual(stats["nodes"]["0"]["frames"]["IO_MAPPING"], 2)
        self.assertEqual(stats["nodes"]["3"]["frames"]["METRIC_LAYOUT_1"], 1)
        self.assertTrue(got["/plc/events"])                                          # foxglove.Log (protobuf)
        self.assertTrue(any(b"PLC node 0 is sending" in e for e in got["/plc/events"]))

    def test_json_frames_are_published_as_is(self):
        status = {"topic": "stations", "link_ok": True, "stations": [{"running": True, "error": False}],
                  "speed": 0.5}
        self.plc.send(json.dumps(status).encode("utf-8"), frame_type=FrameType.JSON, node_id=4)
        self.assertTrue(wait_until(lambda: "stations" in self.bridge.ch_json))
        self.assertIn("/plc/json/stations", self.probe.wait_for_topics(["/plc/json/stations"], timeout=5.0))
        self.probe.subscribe(["/plc/json/stations", "/bridge/stats"])
        time.sleep(1.6)
        self.plc.send(json.dumps(status).encode("utf-8"), frame_type=FrameType.JSON, node_id=4)
        self.plc.send(b"{not json", frame_type=FrameType.JSON, node_id=4)
        self.plc.send(b"[1, 2]", frame_type=FrameType.JSON, node_id=4)                 # not an object
        got = self.collect(2.2)
        msg = got["/plc/json/stations"][-1]
        self.assertEqual(msg["node"], 4)
        self.assertEqual(msg["stations"], [{"running": True, "error": False}])
        self.assertEqual(msg["speed"], 0.5)
        self.assertNotIn("topic", msg)
        self.assertIn("timestamp", msg)
        self.assertEqual(got["/bridge/stats"][-1]["malformed"], 2)
        self.assertEqual(got["/bridge/stats"][-1]["nodes"]["4"]["frames"]["JSON"], 4)

    def test_records_field_list_once_then_values(self):
        fields = [("link_ok", "BOOL"), ("belt_speed", "REAL"), ("stations.0.name", "STRING"),
                  ("stations.0.running", "BOOL"), ("stations.1.name", "STRING"), ("stations.1.running", "BOOL")]
        values = [True, 0.5, "L1", True, "L2", False]
        self.plc.send(plc_records.pack_values(0, fields, values), frame_type=FrameType.RECORD_VALUES, node_id=4)
        self.plc.send(plc_records.pack_schema(0, "stations", fields), frame_type=FrameType.RECORD_SCHEMA, node_id=4)
        self.plc.send(plc_records.pack_values(0, fields, values), frame_type=FrameType.RECORD_VALUES, node_id=4)
        self.assertTrue(wait_until(lambda: "stations" in self.bridge.ch_json))
        self.assertIn("/plc/json/stations", self.probe.wait_for_topics(["/plc/json/stations"], timeout=5.0))
        self.probe.subscribe(["/plc/json/stations", "/bridge/stats"])
        time.sleep(1.6)
        values[1] = 0.25
        self.plc.send(plc_records.pack_values(0, fields, values), frame_type=FrameType.RECORD_VALUES, node_id=4)
        self.plc.send(plc_records.pack_values(0, fields[:-1], values[:-1]), frame_type=FrameType.RECORD_VALUES,
                      node_id=4)                                                  # a field short: malformed
        got = self.collect(2.2)
        msg = got["/plc/json/stations"][-1]
        self.assertEqual({k: v for k, v in msg.items() if k != "timestamp"},
                         {"node": 4, "link_ok": True, "belt_speed": 0.25,
                          "stations": [{"name": "L1", "running": True}, {"name": "L2", "running": False}]})
        stats = got["/bridge/stats"][-1]
        self.assertEqual(stats["values_without_schema"], 1)                     # the one before the schema
        self.assertEqual(stats["malformed"], 1)

    def test_layout_check_is_answered(self):
        out_map, in_map = self.bridge.output_mapping, self.bridge.input_mapping
        self.plc.send(io_layout.pack_layout_check(in_map, out_map, endianness="<"), frame_type=FrameType.LAYOUT_CHECK)
        self.assertTrue(wait_until(lambda: any(f.frame_type == FrameType.LAYOUT_CHECK for f in self.replies)))
        reply = io_layout.unpack_layout_check(self.replies[-1].payload, "<")
        self.assertEqual(reply[4], io_layout.CHECK_MATCH)
        self.assertTrue(self.bridge.layout_ok[0])
        # A PLC built for another layout (one output less) is told so.
        self.plc.send(io_layout.pack_layout_check(in_map, out_map[:-8], endianness="<"),
                      frame_type=FrameType.LAYOUT_CHECK, node_id=2)
        self.assertTrue(wait_until(lambda: self.bridge.layout_ok.get(2) is False))
        self.assertTrue(wait_until(lambda: len(self.replies) >= 2))
        self.assertEqual(io_layout.unpack_layout_check(self.replies[-1].payload, "<")[4], io_layout.CHECK_MISMATCH)

    def test_decode_round_trips_the_encoder(self):
        msg = ComponentMessage(self.bridge.output_mapping, endianness="<")
        self.config.belts[0].speed = 0.75
        fields = dict(decode_fields(msg, msg.serialize(self.config)))
        self.assertAlmostEqual(fields["belt_1_1_1/speed"], 0.75)
        self.assertFalse(any(path.startswith("padding") for path in fields))


class PlcMockToBridgeTest(unittest.TestCase):
    """Step 3: plc_mock.py reporting to the bridge, no simulator running."""

    def test_plc_mock_reports_without_a_simulator(self):
        config = load_config(os.path.join(HERE, "station_1.json"))
        apply_exchange_config(config, os.path.join(HERE, "tcp_exchange_config.json"))
        tcp_port, ws_port, sim_port = free_port(), free_port(), free_port()
        bridge = PlcFoxgloveBridge(config, listen=("127.0.0.1", tcp_port), foxglove_at=("127.0.0.1", ws_port))
        bridge.start()
        self.addCleanup(bridge.stop)
        exchange = os.path.join(HERE, f"_tmp_exchange_{sim_port}.json")
        with open(exchange, "w", encoding="utf-8") as f:
            f.write(f'{{"transport": "tcp", "enabled": true, "tcpServerHost": "127.0.0.1", "tcpPort": {sim_port}, '
                    f'"endianness": "LE"}}')
        self.addCleanup(os.remove, exchange)
        proc = subprocess.Popen([sys.executable, os.path.join(HERE, "plc_mock.py"), os.path.join(HERE, "station_1.json"),
                                 "--exchange-config", exchange, "--bridge", f"127.0.0.1:{tcp_port}", "--autostart",
                                 "--send-interval", "0.5"],
                                cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                env={**os.environ, "PYTHONUNBUFFERED": "1"})   # its log survives kill()
        self.addCleanup(proc.kill)
        try:
            self.assertTrue(wait_until(lambda: bridge.layout_ok.get(0) is True, timeout=15.0), "no layout check")
            self.assertTrue(wait_until(lambda: bridge.last_outputs is not None and bridge.last_inputs is not None, 10))
            self.assertTrue(wait_until(lambda: any(k[1] == "METRIC_LAYOUT_1" for k in bridge.counts), 10))
            states = bridge.composer.line_states(bridge.last_outputs, bridge.last_inputs)["lines"]
            self.assertEqual({n: v["state"] for n, v in states.items()}, {"L1": "RUN", "L2": "RUN", "L3": "RUN"})
        finally:
            proc.kill()
            output = proc.communicate(timeout=10)[0]
        self.assertIn("Program: LineController", output)
        self.assertIn("Bridge layout check: OK", output)
        self.assertIn("No simulator yet", output)


if __name__ == "__main__":
    unittest.main()
