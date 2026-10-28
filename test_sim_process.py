"""
test_sim_process.py - the simulation in its own process (sim_process.py):
the PhysicsProxy driving a real child process (play, parcels, operator
buttons, clear, reset, speed, stop), the window drawing from its snapshots,
main.py starting that way, and the child ending when the window process is
killed.
"""
import os
import re
import socket
import subprocess
import sys
import tempfile
import time
import unittest

from config_parser import load_config
from sim_process import PhysicsProxy, wait_for

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUT = os.path.join(HERE, "test_config.json")


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def exchange_file(tmp: str) -> str:
    path = os.path.join(tmp, "exchange.json")
    with open(path, "w", encoding="utf-8") as f:
        f.write(f'{{"transport": "tcp", "enabled": true, "tcpBindHost": "127.0.0.1", '
                f'"tcpServerHost": "127.0.0.1", "tcpPort": {free_port()}, "endianness": "LE"}}')
    return path


class ProxyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        cls.config = load_config(LAYOUT)
        cls.proxy = PhysicsProxy(cls.config, {"config": LAYOUT, "exchange_config": exchange_file(cls.tmp),
                                              "speed": 1.0})
        cls.proxy.start()

    @classmethod
    def tearDownClass(cls):
        cls.proxy.stop()

    def test_1_runs_and_mirrors_the_plant(self):
        p = self.proxy
        self.assertTrue(p.alive)
        p.set_playing(True)
        self.assertTrue(wait_for(p, lambda: len(p.parcel_views()) >= 3 and p.sim_time > 1.0))
        view = p.parcel_views()[0]
        self.assertEqual(len(view.points), 4)
        self.assertIsInstance(view.color, str)
        self.assertTrue(p.exchange._enabled)                      # the PLC link lives in the child
        self.assertIn("clients", p.exchange.stats())
        belt = p.config.belts[0]
        self.assertTrue(wait_for(p, lambda: belt.actualSpeed is not None))

    def test_2_buttons_reach_the_child(self):
        p = self.proxy
        button = next(b for b in self.config.buttons if b.label == "L2 START")
        p.set_button(button.id, True)
        self.assertTrue(wait_for(p, lambda: button.sensor_states in (True, 1, "1")))
        p.set_button(button.id, False)
        self.assertTrue(wait_for(p, lambda: button.sensor_states in (False, 0, "0")))

    def test_3_pause_speed_clear_reset(self):
        p = self.proxy
        p.set_playing(False)
        p.poll(wait=1.0)
        p.poll(wait=1.0)
        t = p.sim_time
        time.sleep(0.5)
        p.poll(wait=1.0)
        p.poll(wait=1.0)
        self.assertAlmostEqual(p.sim_time, t, places=6)            # paused
        p.set_speed(4.0)
        p.set_playing(True)
        t0, w0 = p.sim_time, time.monotonic()
        wait_for(p, lambda: p.sim_time > t0 + 4.0, timeout=10)
        self.assertGreater((p.sim_time - t0) / (time.monotonic() - w0), 2.0)   # well above real time
        p.set_speed(1.0)
        p.clear_parcels()
        p.set_playing(False)
        self.assertTrue(wait_for(p, lambda: len(p.parcel_views()) == 0))
        p.reset()
        self.assertTrue(wait_for(p, lambda: p.sim_time == 0.0 and sum(p.delivered.values()) == 0))

    def test_4_the_window_draws_from_snapshots(self):
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        from visualizer import Renderer
        p = self.proxy
        p.set_playing(True)
        self.assertTrue(wait_for(p, lambda: len(p.parcel_views()) >= 2))
        r = Renderer(self.config, headless=False)
        self.addCleanup(r.quit)
        r.physics = p
        r.exchange = p.exchange
        r.render(p)
        self.assertEqual(r._parcel_count, len(p.parcel_views()))
        button = next(b for b in self.config.buttons if b.label == "L3 START")
        r._set_button(button, True)                               # the window presses: the child sees it
        self.assertTrue(wait_for(p, lambda: button.sensor_states in (True, 1, "1")))
        r._set_button(button, False)


class MainProcessModeTest(unittest.TestCase):
    def test_main_starts_the_simulation_process_and_it_ends_with_the_window(self):
        tmp = tempfile.mkdtemp()
        env = {**os.environ, "SDL_VIDEODRIVER": "dummy", "PYTHONUNBUFFERED": "1",
               "CONVEYOR_SIM_SETTINGS": os.path.join(tmp, "s.json")}
        proc = subprocess.Popen([sys.executable, os.path.join(HERE, "main.py"), LAYOUT,
                                 "--exchange-config", exchange_file(tmp)],
                                cwd=HERE, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.addCleanup(proc.stdout.close)
        self.addCleanup(proc.kill)
        lines, pid, end = [], None, time.monotonic() + 60
        while time.monotonic() < end and pid is None:
            line = proc.stdout.readline()
            if not line:
                break
            lines.append(line)
            m = re.search(r"own process \(pid (\d+)", line)
            if m:
                pid = int(m.group(1))
        self.assertIsNotNone(pid, "".join(lines[-20:]))
        time.sleep(2.0)
        self.assertIsNone(proc.poll(), "main.py stopped: " + "".join(lines[-20:]))
        self.assertTrue(_pid_alive(pid))
        proc.kill()                                               # the window process dies hard
        proc.wait(timeout=10)
        end = time.monotonic() + 15
        while time.monotonic() < end and _pid_alive(pid):
            time.sleep(0.2)
        self.assertFalse(_pid_alive(pid), "the simulation process outlived the window")


class SingleSimulatorTest(unittest.TestCase):
    """Run in Simulator twice: the running simulator reloads the layout in
    the same process instead of a second window opening (sim_link.py)."""

    def test_the_running_simulator_reloads_on_request(self):
        import json
        import shutil
        tmp = tempfile.mkdtemp()
        settings = os.path.join(tmp, "s.json")
        other = os.path.join(tmp, "other.json")
        shutil.copy(LAYOUT, other)
        env = {**os.environ, "SDL_VIDEODRIVER": "dummy", "PYTHONUNBUFFERED": "1", "CONVEYOR_SIM_SETTINGS": settings}
        proc = subprocess.Popen([sys.executable, os.path.join(HERE, "main.py"), LAYOUT, "--no-sim-process",
                                 "--exchange-config", exchange_file(tmp)],
                                cwd=HERE, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.addCleanup(proc.stdout.close)
        self.addCleanup(proc.kill)
        lines = []

        def read_until(pattern, timeout=60):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                line = proc.stdout.readline()
                if not line:
                    return False
                lines.append(line)
                if pattern in line:
                    return True
            return False

        self.assertTrue(read_until("Starting simulation loop"), "".join(lines[-20:]))
        registry = os.path.join(tmp, "sim_instance.json")
        with open(registry, encoding="utf-8") as f:
            entry = json.load(f)
        self.assertIn("port", entry)        # (its pid can differ from proc.pid: a venv launcher on Windows)
        old_env = os.environ.get("CONVEYOR_SIM_SETTINGS")
        os.environ["CONVEYOR_SIM_SETTINGS"] = settings
        try:
            import sim_link
            self.assertEqual(sim_link.request_reload(other), entry["pid"])
        finally:
            if old_env is None:
                del os.environ["CONVEYOR_SIM_SETTINGS"]
            else:
                os.environ["CONVEYOR_SIM_SETTINGS"] = old_env
        self.assertTrue(read_until("Reloading"), "".join(lines[-20:]))
        self.assertTrue(read_until("Starting simulation loop"), "".join(lines[-20:]))
        self.assertIsNone(proc.poll(), "the same simulator keeps running")
        with open(registry, encoding="utf-8") as f:
            self.assertEqual(os.path.normcase(json.load(f)["config"]), os.path.normcase(os.path.abspath(other)))
        proc.terminate()
        proc.wait(timeout=10)


def _pid_alive(pid: int) -> bool:
    if sys.platform == "win32":
        out = subprocess.run(["tasklist", "/FI", f"PID eq {pid}", "/NH"], capture_output=True, text=True).stdout
        return str(pid) in out
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


if __name__ == "__main__":
    unittest.main()
