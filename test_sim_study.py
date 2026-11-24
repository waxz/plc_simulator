"""
test_sim_study.py - a study (sim_study.py): parameters by path (an id glob,
nested fields, the layout itself), every combination of the sweeps, a run's
generic measures on any layout, the program's own counters (the merge
train's), seeds aggregated, runs in parallel, and the JSON / CSV it writes.
"""
import csv
import json
import os
import shutil
import tempfile
import unittest

import sim_study
from config_parser import load_config

HERE = os.path.dirname(os.path.abspath(__file__))
MERGE = os.path.join(HERE, "merge_demo.json")
STATION = os.path.join(HERE, "station_1.json")


class ParameterTest(unittest.TestCase):
    def test_paths_set_what_they_name(self):
        c = load_config(MERGE)
        self.assertEqual(sim_study.apply_setting(c, "belts.belt_2_*.drive.accel", 3.5), 5)
        self.assertEqual({b.drive.accel for b in c.belts if b.id.startswith("belt_2_")}, {3.5})
        self.assertGreater(sim_study.apply_setting(c, "sources.*.interval", 1.5), 1)
        sim_study.apply_setting(c, "diverters.div_1.rowControl", False)
        self.assertFalse(c.diverters[0].rowControl)
        with self.assertRaises(ValueError):
            sim_study.apply_setting(c, "belts.nothing_*.speed", 1.0)
        with self.assertRaises(ValueError):
            sim_study.apply_setting(c, "belts.speed", 1.0)

    def test_values_and_combinations(self):
        self.assertEqual(sim_study.parse_setting("a.b.c=2"), ("a.b.c", 2))
        self.assertEqual(sim_study.parse_setting("a.b.c=true"), ("a.b.c", True))
        self.assertEqual(sim_study.parse_setting("a.b.c=head"), ("a.b.c", "head"))
        v = sim_study.variants(["x.y.z=1"], ["a.b.c=1,2", "d.e.f=x,y,z"])
        self.assertEqual(len(v), 6)
        self.assertTrue(all(s["x.y.z"] == 1 for s in v))
        self.assertEqual(sim_study.variants([], []), [{}])


class RunTest(unittest.TestCase):
    def test_a_layout_with_the_line_program_is_measured(self):
        r = sim_study.run(STATION, {}, seed=1, seconds=40.0, warmup=10.0)
        for key in ("flow", "wip", "lead_s", "routes", "sources", "belts", "events", "program", "speed"):
            self.assertIn(key, r)
        self.assertGreater(r["speed"], 0.5)
        self.assertGreaterEqual(r["wip"]["max"], r["wip"]["mean"])
        self.assertTrue(r["sources"])

    def test_the_merge_train_counts_and_seeds_aggregate(self):
        results = sim_study.study(MERGE, [{}], [1, 2], jobs=2, seconds=60.0, warmup=20.0)
        self.assertEqual(len(results[0]["runs"]), 2)
        run = results[0]["runs"][0]
        self.assertGreater(run["flow"]["per_min"], 60.0)
        self.assertIn("train.empty_cars", run["program"])
        self.assertGreater(run["program"]["train.landings"], 30)
        self.assertEqual(run["headline"][0], "program.train.landings")
        agg = results[0]["aggregate"]["flow.per_min"]
        self.assertLessEqual(agg["min"], agg["mean"])
        self.assertLessEqual(agg["mean"], agg["max"])
        dest = {k: v for k, v in run["routes"].items() if k != "-"}
        self.assertTrue(dest)                                       # where each destination went

    def test_the_command_line_writes_json_and_csv(self):
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, True)
        out_json, out_csv = os.path.join(tmp, "s.json"), os.path.join(tmp, "s.csv")
        self.assertEqual(sim_study.main([STATION, "--seconds", "20", "--warmup", "5", "--sweep",
                                         "sources.*.interval=2,3", "--json", out_json, "--csv", out_csv]), 0)
        with open(out_json, encoding="utf-8") as f:
            self.assertEqual(len(json.load(f)), 2)
        with open(out_csv, encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 2)
        self.assertIn("flow.per_min", rows[0])


if __name__ == "__main__":
    unittest.main()
