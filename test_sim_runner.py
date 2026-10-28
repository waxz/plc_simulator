"""
test_sim_runner.py - the simulation thread decoupled from rendering
(sim_runner.SimulationRunner): real time, k times real time, as fast as
possible (with the physics lock released between slices so the window and
the PLC exchange still get in), and pause.
"""
import threading
import time
import unittest

from config_parser import SimulatorConfig, load_config
from physics import PhysicsEngine
from sim_runner import SimulationRunner


def run(engine, speed, seconds=1.2, playing=True):
    runner = SimulationRunner(engine, None, tick_s=0.002, speed=speed)
    runner.playing = playing
    s0, t0 = engine.sim_time, time.perf_counter()
    runner.start()
    time.sleep(seconds)
    runner.stop()
    return (engine.sim_time - s0) / (time.perf_counter() - t0)


class RunnerSpeedTest(unittest.TestCase):
    def test_real_time_k_times_and_max(self):
        engine = PhysicsEngine(SimulatorConfig())        # empty plant: cheap steps
        self.assertAlmostEqual(run(engine, 1.0), 1.0, delta=0.1)
        self.assertAlmostEqual(run(engine, 4.0), 4.0, delta=0.4)
        self.assertGreater(run(engine, None), 6.0)        # as fast as possible
        self.assertEqual(run(engine, 1.0, playing=False), 0.0)

    def test_max_speed_lets_other_threads_take_the_lock(self):
        lock = threading.Lock()
        engine = PhysicsEngine(load_config("test_config.json"), lock=lock)
        runner = SimulationRunner(engine, None, speed=None)
        runner.start()
        self.addCleanup(runner.stop)
        waits = []
        for _ in range(20):                               # like the window drawing a frame
            t = time.perf_counter()
            with lock:
                waits.append(time.perf_counter() - t)
            time.sleep(0.01)
        self.assertLess(max(waits), 0.1)                  # a slice is ~8 ms, never a long hold
        self.assertGreater(engine.sim_time, 0.0)

    def test_step_for_runs_whole_steps(self):
        engine = PhysicsEngine(SimulatorConfig())
        steps = engine.step_for(0.0)
        self.assertEqual(steps, 1)
        self.assertAlmostEqual(engine.sim_time, engine.config.physicsDt)


if __name__ == "__main__":
    unittest.main()
