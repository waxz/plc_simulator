"""device_check.py as a test: every belt, roller, curve, pusher, wheel
diverter and turntable does what its PLC command says, the commands and
feedback going through plc_mock.PlcImage's frames; and the window of a
simulation in its own process draws the wheels where they point."""
import os
import tempfile
import unittest

import device_check
from sim_process import PhysicsProxy, wait_for


class DeviceCheckTest(unittest.TestCase):
    def test_every_device_follows_its_command(self):
        res = device_check.run_all()
        self.assertGreaterEqual(len(res.rows), 39)
        self.assertEqual(res.failed, [], "\n".join(f"{d}: {w}: {x}" for d, w, _ok, x in res.failed))


class WheelsMirroredTest(unittest.TestCase):
    """The window mirrors the wheels' angle (it drew them straight while they steered)."""

    def test_the_window_sees_the_wheels_turn(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = os.path.join(tmp.name, "devices.json")
        with open(path, "w", encoding="utf-8") as f:
            f.write(device_check.layout().model_dump_json())
        config = device_check.layout()
        proxy = PhysicsProxy(config, {"config": path, "speed": 1.0})
        proxy.start()
        self.addCleanup(proxy.stop)
        proxy.set_playing(True)
        proxy.set_io("wheels", "control", "manual")
        proxy.set_io("wheels", "enable", True)
        proxy.set_io("wheels", "commandAngle", -30.0)                     # the swivel motor: to the branch
        wheels = next(d for d in config.diverters if d.id == "wheels")
        self.assertTrue(wait_for(proxy, lambda: abs(wheels.wheelAngle + 30.0) < 0.5, timeout=10.0),
                        f"the window's wheels at {wheels.wheelAngle}")


if __name__ == "__main__":
    unittest.main()
