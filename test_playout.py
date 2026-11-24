"""
test_playout.py - the 3D page's playout buffer (view3d_web/playout.js): frames
are kept and played back evenly a little behind, so a page over a tunnel does
not jump. The logic runs in node (view3d_web/playout_test.mjs); the server
stamps every frame with its clock (`w`), which the buffer plays back on.
"""
import json
import os
import shutil
import subprocess
import unittest

import view3d

HERE = os.path.dirname(os.path.abspath(__file__))


class PlayoutTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("node"), "node is not installed")
    def test_bursty_frames_play_back_evenly(self):
        out = subprocess.run(["node", os.path.join(HERE, "view3d_web", "playout_test.mjs")],
                             capture_output=True, text=True, timeout=60)
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertIn("playout ok", out.stdout)

    def test_every_frame_carries_the_servers_clock(self):
        server = view3d.View3DServer(port=0)
        server._clients = 1                       # as if a page were open
        self.assertTrue(server.publish(lambda: {"t": 1.0}, force=True))
        first = json.loads(server._frame)
        self.assertTrue(server.publish(lambda: {"t": 1.1}, force=True))
        second = json.loads(server._frame)
        self.assertGreaterEqual(second["w"], first["w"])

    def test_the_page_uses_it(self):
        with open(os.path.join(HERE, "view3d_web", "viewer.js"), encoding="utf-8") as f:
            js = f.read()
        with open(os.path.join(HERE, "view3d_web", "index.html"), encoding="utf-8") as f:
            html = f.read()
        self.assertIn('from "./playout.js"', js)
        self.assertIn("playout.push(state", js)
        self.assertIn('id="smooth"', html)
        self.assertIn('id="buffer"', html)


if __name__ == "__main__":
    unittest.main()
