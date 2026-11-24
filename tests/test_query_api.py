"""The simulator's query API (view3d.py /api) and its client sim_query.py:
state, PLC tags, their history, components and parcels, from a running
engine, over HTTP on 127.0.0.1."""
import io
import threading
import time
import types
import unittest
from contextlib import redirect_stdout

import sim_query
import view3d
from config_parser import Belt, LinkWatchdog, SimulatorConfig, Sink
from physics import PhysicsEngine


def layout():
    belt = Belt(id="feed", type="linear", x=0.0, y=0.0, length=4.0, beltWidth=0.6, rotation=0.0, speed=1.0,
                directionAngle=0.0, color="#64748b", commandFormat="vfd", run=True)
    chute = Sink(id="chute", x=2.4, y=0.0, width=0.8, height=0.8, label="", capacity=3)
    return SimulatorConfig(belts=[belt], sinks=[chute], watchdogs=[LinkWatchdog(id="link", x=0.0, y=-2.0)])


class QueryApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = layout()
        cls.engine = PhysicsEngine(cls.config)
        cls.engine.add_parcel(-1.0, 0.0, 0.3, 0.3)
        cls.server = view3d.View3DServer(port=0)
        cls.server.set_layout(cls.config, "test_layout.json")
        cls.server.start()
        lock = threading.Lock()
        # what view3d.pump needs of the window
        cls.renderer = types.SimpleNamespace(
            config=cls.config, sim_lock=lock, playing=True, speed=1.0, exchange=None,
            _sim_health=lambda pe: {"ratio": 1.0}, health_status=lambda pe: ("OK", None),
            _toggle_door=lambda d: None, _set_button=lambda b, p: None)
        cls.stop = threading.Event()

        def loop():                              # the window's loop: physics, then pump
            while not cls.stop.is_set():
                with lock:
                    cls.engine.run_for(0.02)
                view3d.SERVER, saved = cls.server, view3d.SERVER
                try:
                    view3d.pump(cls.renderer, cls.engine)
                finally:
                    view3d.SERVER = saved
                time.sleep(0.01)
        cls.thread = threading.Thread(target=loop, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.stop.set()
        cls.thread.join(timeout=2)
        cls.server.stop()

    def run_cli(self, *argv) -> str:
        out = io.StringIO()
        with redirect_stdout(out):
            self.assertEqual(sim_query.main([*argv, "--port", str(self.server.port)]), 0)
        return out.getvalue()

    def test_the_index(self):
        data = sim_query.query("/api", self.server.port)
        self.assertTrue(data["simulator"])
        self.assertIn("/api/state", data["endpoints"])

    def test_state(self):
        state = sim_query.query("/api/state", self.server.port)
        self.assertEqual(state["layout"], "test_layout.json")
        self.assertGreater(state["simTime"], 0.0)
        self.assertEqual([c["id"] for c in state["chutes"]], ["chute"])
        self.assertEqual(state["watchdogs"][0]["id"], "link")
        self.assertIn("simTime", self.run_cli("state", "--json"))
        self.assertIn("chute      chute: ", self.run_cli("state"))

    def test_tags_now_and_their_history(self):
        tags = sim_query.query("/api/tags", self.server.port, match="chute")
        names = {t["name"]: t for t in tags["tags"]}
        self.assertIn("chute_full", names)
        self.assertTrue(all("chute" in n for n in names))
        outputs = sim_query.query("/api/tags", self.server.port, area="Q")
        self.assertTrue(all(t["address"].startswith("%Q") for t in outputs["tags"]))
        time.sleep(0.4)
        history = sim_query.query("/api/history", self.server.port, match="link_simHeartbeat", seconds=5)
        self.assertEqual(history["names"][0], "link_simHeartbeat")   # (and its echo, which the substring matches)
        beats = [row[1] for row in history["samples"]]
        self.assertGreater(len(beats), 2)
        self.assertGreater(beats[-1], beats[0], "the simulator's heartbeat counts")
        self.assertIn("chute_count", self.run_cli("tags", "chute"))

    def test_components_and_parcels(self):
        comps = sim_query.query("/api/components", self.server.port, match="feed,chute")["components"]
        self.assertEqual(set(comps), {"feed", "chute"})
        self.assertIn("run", comps["feed"]["values"])
        time.sleep(0.3)
        parcels = sim_query.query("/api/parcels", self.server.port)["parcels"]
        self.assertLessEqual(len(parcels), 1)
        self.assertIn("parcels", self.run_cli("parcels"))


if __name__ == "__main__":
    unittest.main()
