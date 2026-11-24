"""
test_chute_stats.py - each chute's statistics (chute_stats.py): the total since
the last reset, per minute and per hour in simulated time; Reset Stats in the
simulator window, and the same numbers and Reset in the 3D view.
"""
import json
import os
import unittest
import urllib.request
from unittest import mock

import chute_stats
import view3d
from chute_stats import ChuteStats
from config_parser import load_config
from physics import PhysicsEngine

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class ChuteStatsTest(unittest.TestCase):
    def test_total_and_rates_over_the_time_counted(self):
        s = ChuteStats()
        s.update(0.0, {})
        for k in range(1, 31):                       # one load every 2 s into chute_1 for 60 s
            s.update(2.0 * k, {"chute_1": k})
        row = s.row("chute_1")
        self.assertEqual(row["total"], 30)
        self.assertAlmostEqual(row["perMin"], 30.0)
        self.assertAlmostEqual(row["perHour"], 1800.0)  # projected over the 60 s counted
        self.assertEqual(s.row("chute_2"), {"id": "chute_2", "total": 0, "perMin": 0.0, "perHour": 0.0,
                                            "since": 60.0})

    def test_per_minute_is_the_last_minute(self):
        s = ChuteStats()
        s.update(0.0, {"c": 0})
        s.update(10.0, {"c": 50})                    # a burst, then nothing for two minutes
        s.update(130.0, {"c": 50})
        row = s.row("c")
        self.assertEqual(row["total"], 50)
        self.assertEqual(row["perMin"], 0.0)
        self.assertAlmostEqual(row["perHour"], 50 * 3600 / 130, delta=0.5)

    def test_reset_counts_again_and_keeps_the_chutes(self):
        s = ChuteStats()
        s.update(0.0, {"c": 0})
        s.update(30.0, {"c": 10})
        s.reset(s.now)
        self.assertEqual(s.row("c")["total"], 0)
        s.update(60.0, {"c": 13})                    # only what came after the reset
        self.assertEqual(s.row("c")["total"], 3)
        self.assertAlmostEqual(s.row("c")["perMin"], 6.0)

    def test_a_simulation_reset_starts_it_again(self):
        s = ChuteStats()
        s.update(0.0, {"c": 0})
        s.update(50.0, {"c": 20})
        s.update(0.5, {})                            # time and counts back to zero
        self.assertEqual(s.row("c")["total"], 0)
        s.update(5.0, {"c": 2})
        self.assertEqual(s.row("c")["total"], 2)

    def test_text(self):
        self.assertEqual(chute_stats.text({"total": 12, "perMin": 6.04, "perHour": 362.4}), "12 | 6.0/min | 362/h")


def renderer_for(name):
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    from visualizer import Renderer
    config = load_config(os.path.join(HERE, name))
    return Renderer(config, headless=False), config


class WindowTest(unittest.TestCase):
    def test_counted_every_frame_and_reset_from_the_toolbar(self):
        r, config = renderer_for("merge_demo.json")
        self.addCleanup(r.quit)
        engine = PhysicsEngine(config)
        sink = config.sinks[0].id
        engine.sim_time, engine.delivered = 1.0, {}
        r.update_chute_stats(engine)
        engine.sim_time, engine.delivered = 31.0, {sink: 4}
        r.update_chute_stats(engine)
        self.assertEqual(r.chute_stats.row(sink)["total"], 4)
        self.assertAlmostEqual(r.chute_stats.row(sink)["perMin"], 8.0)
        r.draw_sink(config.sinks[0])                 # its label, above the chute
        self.assertTrue(any("4 | 8.0/min" in str(item) for item in r._labels))
        self.assertIn("sim_stats_btn", dir(r.editor_ui))
        r.run_command("RESET_STATS")
        self.assertEqual(r.chute_stats.row(sink)["total"], 0)


class View3DTest(unittest.TestCase):
    def test_the_view_shows_them_and_its_reset_reaches_the_simulation(self):
        r, config = renderer_for("merge_demo.json")
        self.addCleanup(r.quit)
        engine = PhysicsEngine(config)
        sink = config.sinks[0].id
        r.chute_stats.update(0.0, {})
        r.chute_stats.update(60.0, {sink: 6})
        rows = view3d.chute_rows(r)
        self.assertEqual([row["id"] for row in rows], [s.id for s in config.sinks])
        self.assertEqual(rows[0]["total"], 6)

        old = view3d.SERVER
        view3d.SERVER = view3d.View3DServer(port=0)
        self.addCleanup(lambda: (view3d.SERVER.stop(), setattr(view3d, "SERVER", old)))
        with mock.patch("view3d.webbrowser.open"):
            r.open_view3d()
        req = urllib.request.Request(view3d.SERVER.url + "reset_stats", data=b"{}",
                                     headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=10) as resp:
            self.assertEqual(json.loads(resp.read()), {"ok": True})
        engine.sim_time, engine.delivered = 60.0, {sink: 6}
        view3d.pump(r, engine)
        self.assertEqual(view3d.chute_rows(r)[0]["total"], 0)
        self.assertTrue(view3d.SERVER.build_api_state(r, engine, force=True))
        self.assertEqual(view3d.SERVER._api_state["chuteStats"][0]["id"], sink)

    def test_the_page_draws_them(self):
        with open(os.path.join(HERE, "view3d_web", "viewer.js"), encoding="utf-8") as f:
            js = f.read()
        with open(os.path.join(HERE, "view3d_web", "index.html"), encoding="utf-8") as f:
            html = f.read()
        self.assertIn("updateChutes(state.chuteStats)", js)
        self.assertIn('fetch("reset_stats"', js)
        self.assertIn('id="chutes"', html)


if __name__ == "__main__":
    unittest.main()
