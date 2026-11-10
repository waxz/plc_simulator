"""
test_view3d.py - the 3D view: the layout and the moving parts as 3D data
(scene3d.py: heights of belts, pallet loads, lift riders, gripped parts),
the server (view3d.py: page, scene, live event stream, button presses, the
local three.js swap) and the simulator window's 3D View button.
"""
import json
import os
import tempfile
import threading
import time
import unittest
import urllib.request
from unittest import mock

import scene3d
import view3d
from config_parser import load_config
from physics import ParcelView, PhysicsEngine

HERE = os.path.dirname(os.path.abspath(__file__))
DEMO = os.path.join(HERE, "machines_demo.json")


def body_rows(state):
    return {row[0]: row for row in state["bodies"]}


class SceneTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config(DEMO)
        self.engine = PhysicsEngine(self.config)
        self.lh = scene3d.level_height(self.config)

    def test_static_scene_has_every_part_of_the_layout(self):
        scene = scene3d.static_scene(self.config, DEMO)
        self.assertEqual(len(scene["belts"]), len(self.config.belts))
        self.assertEqual(scene["levelHeight"], self.config.elevators[0].levelHeight)
        for belt in scene["belts"]:
            self.assertGreaterEqual(len(belt["outline"]), 4)
        lift_belt = next(b for b in scene["belts"] if b["id"] == self.config.elevators[0].beltId)
        self.assertEqual(lift_belt["elevator"], self.config.elevators[0].id)
        robot = scene["robots"][0]
        self.assertAlmostEqual(robot["pick"][2], scene3d.surface_z(self.config, self.config.robots[0].pickLevel))
        for key in ("sensors", "stoppers", "elevators", "sources", "sinks"):
            self.assertIn(key, scene)
        json.dumps(scene)                                      # plain data

    def test_heights_follow_levels_pallets_lifts_and_grippers(self):
        self.engine.run_for(1.0)
        pallet = next(v for v in self.engine.parcel_views() if v.kind == "pallet")
        base = scene3d.surface_z(self.config, pallet.level)
        views = [pallet,
                 ParcelView("load_1", pallet.points, pallet.x, pallet.y, "#ffaa00", 0.0, None, "parcel",
                            pallet.level, "load"),
                 ParcelView("up_1", pallet.points, 9.0, 9.0, "#ffaa00", 0.0, None, "parcel", 2, ""),
                 ParcelView("ride_1", pallet.points, 9.0, 9.0, "#ffaa00", 0.0, None, "parcel",
                            scene3d.SHAFT_LEVEL, "")]
        lift = self.config.elevators[0]
        lift.position = 0.5                                    # halfway between floors 0 and 1
        rows = body_rows(scene3d.dynamic_state(self.config, views, 1.0))
        self.assertAlmostEqual(rows[pallet.id][4], base, places=3)
        self.assertAlmostEqual(rows["load_1"][4], base + scene3d.PALLET_H, places=3)
        self.assertAlmostEqual(rows["up_1"][4], 2 * self.lh + scene3d.BELT_TOP, places=3)
        self.assertAlmostEqual(rows["ride_1"][4], 0.5 * self.lh + scene3d.BELT_TOP, places=3)
        self.assertAlmostEqual(rows[pallet.id][8], scene3d.PALLET_H)
        lo, hi = scene3d.PARCEL_H
        self.assertTrue(lo <= rows["load_1"][8] <= hi)
        self.assertAlmostEqual(rows["load_1"][8], scene3d.parcel_height("load_1"), places=3)   # stable per id

    def test_a_gripped_part_hangs_from_the_tool_and_the_arm_sends_its_joints(self):
        robot = self.config.robots[0]
        robot.holding = True
        robot.tcpX, robot.tcpY, robot.tcpZ = 3.0, 1.0, 1.6
        robot.j1, robot.j2, robot.j5 = 30.0, 20.0, 70.0
        some = self.engine.parcel_views()[0]
        held = ParcelView("held_1", some.points, 3.0, 1.0, "#ffaa00", 0.0, None, "parcel", 0, "held")
        state = scene3d.dynamic_state(self.config, [held], 1.0)
        row = body_rows(state)["held_1"]
        self.assertAlmostEqual(row[4], 1.6 - row[8], places=3)          # its top at the TCP
        joints = state["robots"][robot.id]
        self.assertEqual(joints[:6], [30.0, 20.0, robot.j3, robot.j4, 70.0, robot.j6])
        self.assertTrue(joints[6])
        scene = scene3d.static_scene(self.config, DEMO)
        geometry = scene["robots"][0]["geometry"]
        self.assertAlmostEqual(geometry["reach"], robot.reach)
        self.assertAlmostEqual(geometry["mount"], robot.mountHeight)

    def test_buttons_and_lamps_become_control_panels(self):
        config = load_config(os.path.join(HERE, "station_1.json"))
        config.background = None
        panels = scene3d.static_scene(config)["panels"]
        self.assertEqual(sorted(p["title"] for p in panels), ["L1", "L2", "L3"])     # one console per line
        for panel in panels:
            rows = {}
            for d in panel["devices"]:
                rows.setdefault(d["row"], set()).add(d["style"])
            self.assertEqual(rows, {0: {"lamp"}, 1: {"button"}})            # pilot lamps above pushbuttons
            labels = [d["label"].split()[1] for d in panel["devices"]]
            self.assertEqual(labels, ["AUTO", "MANUAL", "ERROR", "START", "STOP", "RESET"])
        from config_parser import ControlButton
        config.buttons.append(ControlButton(id="es", x=18.9, y=-19.4, label="L1 E-STOP", color="#ff0000",
                                            buttonType="latching", normallyClosed=True))
        l1 = next(p for p in scene3d.control_panels(config) if p["title"] == "L1")
        self.assertEqual(next(d for d in l1["devices"] if d["id"] == "es")["style"], "estop")

    def test_a_running_simulation_streams_live_states(self):
        self.engine.run_for(6.0)
        state = scene3d.dynamic_state(self.config, self.engine.parcel_views(), self.engine.sim_time)
        self.assertTrue(any(row[1] == "pallet" for row in state["bodies"]))
        lift = self.config.elevators[0]
        self.assertEqual(state["belts"][lift.beltId][1], round(lift.position * self.lh + scene3d.BELT_TOP, 4))
        self.assertEqual(set(state["stoppers"]), {s.id for s in self.config.stoppers})
        json.dumps(state)


class LogisticsSceneTest(unittest.TestCase):
    def test_equipment_and_live_states_reach_the_view(self):
        import random
        random.seed(3)
        config = load_config(os.path.join(HERE, "logistics_demo.json"))
        scene = scene3d.static_scene(config)
        self.assertEqual([s["type"] for s in scene["scanners"]], ["dws"])
        self.assertEqual({d["type"] for d in scene["diverters"]}, {"pusher", "wheels"})
        self.assertEqual(len(scene["eyes"]), 3)                          # one photo-eye per zone
        self.assertEqual({c["id"] for c in scene["chutes"]}, {"chute_1", "chute_2", "chute_3"})
        table_belt = next(b for b in scene["belts"] if b["id"] == "tt_belt")
        self.assertEqual(table_belt["table"]["x"], config.turntables[0].x)
        engine = PhysicsEngine(config)
        engine.run_for(30.0)
        state = scene3d.dynamic_state(config, engine.parcel_views(), engine.sim_time)
        for key in ("scanners", "diverters", "turntables", "accumulators", "chutes", "doors", "safety"):
            self.assertIn(key, state)
        self.assertEqual(state["doors"]["door_1"], [True, False])
        self.assertTrue(any(row[10] > 1000000 for row in state["bodies"]))   # barcodes on the parcels
        self.assertEqual(len(state["belts"]["tt_belt"]), 3)              # speed, z, table angle
        # A diverter: active, the angle it is told to steer at (divertAngle +
        # commandAngle) and where its wheels point now.
        wheels = next(d for d in config.diverters if d.diverterType == "wheels")
        self.assertEqual(state["diverters"][wheels.id][1], wheels.divertAngle)
        wheels.control, wheels.commandAngle = "manual", -30.0
        wheels.wheelAngle = 12.5
        state = scene3d.dynamic_state(config, engine.parcel_views(), engine.sim_time)
        self.assertEqual(state["diverters"][wheels.id][1:3], [wheels.divertAngle - 30.0, 12.5])


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config(DEMO)
        self.server = view3d.View3DServer(port=0)
        self.server.set_layout(self.config, DEMO)
        self.server.start()
        self.addCleanup(self.server.stop)
        self.base = self.server.url

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=10) as r:
            return r.status, r.headers.get("Content-Type"), r.read()

    def test_page_scene_and_files(self):
        status, kind, body = self.get("")
        self.assertEqual(status, 200)
        self.assertIn("text/html", kind)
        self.assertIn(b"viewer.js", body)
        self.assertIn(view3d.THREE_CDN.encode(), body)
        status, kind, body = self.get("viewer.js")
        self.assertIn("javascript", kind)
        scene = json.loads(self.get("scene")[2])
        self.assertEqual(scene["title"], "machines_demo.json")
        self.assertEqual(scene["version"], self.server.version)
        with self.assertRaises(urllib.error.HTTPError):
            self.get("../config_parser.py")

    def test_a_local_three_js_replaces_the_cdn(self):
        tmp = tempfile.mkdtemp()
        os.makedirs(os.path.join(tmp, "build"))
        open(os.path.join(tmp, "build", "three.module.js"), "w").close()
        with mock.patch.object(view3d, "THREE_LOCAL", tmp):
            body = self.get("")[2]
        self.assertIn(b'"three": "/vendor/three/build/three.module.js"', body)

    def test_live_events_and_button_presses(self):
        frames = []

        def listen():
            with urllib.request.urlopen(self.base + "events", timeout=10) as r:
                for line in r:
                    if line.startswith(b"data: "):
                        frames.append(json.loads(line[6:]))
                        return

        reader = threading.Thread(target=listen, daemon=True)
        reader.start()
        end = time.monotonic() + 5
        while self.server.clients == 0 and time.monotonic() < end:
            time.sleep(0.02)
        self.assertEqual(self.server.clients, 1)
        self.assertTrue(self.server.publish(lambda: {"t": 1.5, "bodies": []}, force=True))
        reader.join(5)
        self.assertEqual(frames[0]["t"], 1.5)
        self.assertEqual(frames[0]["layout"], self.server.version)

        req = urllib.request.Request(self.base + "button", data=json.dumps({"id": "b1", "pressed": True}).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=10) as r:
            self.assertEqual(json.loads(r.read()), {"ok": True})
        self.assertEqual(self.server.take_commands(), [("button", "b1", True)])
        req = urllib.request.Request(self.base + "door", data=json.dumps({"id": "door_1"}).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=10) as r:
            self.assertEqual(json.loads(r.read()), {"ok": True})
        self.assertEqual(self.server.take_commands(), [("door", "door_1", True)])

    def test_a_new_simulator_is_a_new_layout_to_an_open_page(self):
        # A page left open from an earlier simulator compares layout versions:
        # a new server's first layout must not reuse the old one's number.
        time.sleep(0.002)
        later = view3d.View3DServer(port=0)
        later.set_layout(self.config, DEMO)
        self.assertNotEqual(later.version, self.server.version)

    def test_a_taken_port_is_not_shared(self):
        second = view3d.View3DServer(port=self.server.port)
        second.set_layout(self.config, DEMO)
        url = second.start()
        self.addCleanup(second.stop)
        self.assertNotEqual(second.port, self.server.port)              # moved to a free one
        self.assertEqual(url, second.url)
        status, _, _ = self.get("scene")                                # the first still answers its own
        self.assertEqual(status, 200)

    def test_no_frames_are_built_without_a_viewer(self):
        built = []
        self.assertFalse(self.server.publish(lambda: built.append(1) or {}, force=True))
        self.assertEqual(built, [])


class SimulatorButtonTest(unittest.TestCase):
    def test_the_window_button_opens_the_view_and_presses_reach_the_simulation(self):
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        from visualizer import Renderer
        config = load_config(os.path.join(HERE, "station_1.json"))
        r = Renderer(config, headless=False)
        self.addCleanup(r.quit)
        r.config_path = "station_1.json"
        old = view3d.SERVER
        view3d.SERVER = view3d.View3DServer(port=0)
        self.addCleanup(lambda: (view3d.SERVER.stop(), setattr(view3d, "SERVER", old)))
        with mock.patch("view3d.webbrowser.open") as browser:
            r.open_view3d()
        browser.assert_called()
        self.assertTrue(view3d.SERVER.running)
        self.assertIs(view3d.SERVER.config, config)
        button = config.buttons[0]
        view3d.SERVER.commands.put(("button", button.id, True))
        engine = PhysicsEngine(config)
        view3d.pump(r, engine)
        self.assertTrue(button.pressed)
        self.assertIn("view3d_btn", dir(r.editor_ui))


if __name__ == "__main__":
    unittest.main()
