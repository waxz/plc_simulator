"""
test_track_boxes.py - the Exchange Table's tracked rows drawn as boxes, end to
end: a PLC (plc_mock.py, the merge train) sends its rows over TCP to the
simulation in its own process; the window mirrors them and draws every
active row by the type it was given in the Exchange Table - Tracked load
(role "load": an outlined box), Track slot ("slot": a band, filled faintly),
Sync load ("sync": an outlined box) - and the 3D view gets the same rows,
with their roles (scene3d "tracks", view3d_web/viewer.js).
"""
import os
import subprocess
import sys
import tempfile
import unittest

import scene3d
from config_parser import load_config
from sim_process import PhysicsProxy, wait_for
from test_sim_process import exchange_file

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUT = os.path.join(HERE, "merge_demo.json")
ROLES = {"load", "slot", "sync"}


class TrackBoxesEndToEndTest(unittest.TestCase):
    def test_every_type_of_row_from_the_plc_is_drawn_in_the_window_and_the_3d_view(self):
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        from layout_editor import TRACK_ROLES
        from visualizer import Renderer
        config = load_config(LAYOUT)
        self.assertEqual({t.role for t in config.trackedLoads}, set(TRACK_ROLES.values()))   # every type in it
        tmp = tempfile.mkdtemp()
        exchange = exchange_file(tmp)
        proxy = PhysicsProxy(config, {"config": LAYOUT, "exchange_config": exchange, "speed": 1.0})
        proxy.start()
        self.addCleanup(proxy.stop)
        proxy.set_playing(True)
        plc = subprocess.Popen([sys.executable, os.path.join(HERE, "plc_mock.py"), LAYOUT, "--exchange-config",
                                exchange, "--autostart", "--program", "merge-train"],
                               cwd=HERE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.addCleanup(plc.kill)

        def shown_roles():
            return {t.role for t in config.trackedLoads if t.active and t.loadLength > 0 and t.loadWidth > 0}

        # the window process's copy of the rows, from the child's snapshots
        self.assertTrue(wait_for(proxy, lambda: shown_roles() == ROLES, timeout=60.0),
                        f"rows shown: {shown_roles()}")
        # the 3D view: every active row, with its role
        state = scene3d.dynamic_state(config, proxy.parcel_views(), proxy.sim_time)
        self.assertEqual({row[1] for row in state["tracks"]}, ROLES)
        # the window: each role's colour on the screen where its row is
        r = Renderer(config, headless=False)
        self.addCleanup(r.quit)
        r.physics = proxy
        r.render(proxy)
        drawn = set()
        for t in config.trackedLoads:
            if not (t.active and t.loadLength > 0):
                continue
            x, y = r._world_to_screen(t.loadX, t.loadY)
            color = r.TRACK_COLORS[t.role]
            reach = int(max(t.loadLength, t.loadWidth) * r.scale * r.zoom) + 3
            for dx in range(-reach, reach + 1):
                for dy in range(-reach, reach + 1, 2):
                    px, py = x + dx, y + dy
                    if 0 <= px < r.width and 0 <= py < r.height and tuple(r.screen.get_at((px, py)))[:3] == color:
                        drawn.add(t.role)
                        break
                if t.role in drawn:
                    break
        self.assertEqual(drawn, ROLES)


if __name__ == "__main__":
    unittest.main()
