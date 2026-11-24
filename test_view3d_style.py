"""view3d.py: a layout's 3D style (view3d_web/style.js) read and saved next to it."""
import json
import os
import shutil
import tempfile
import unittest
import urllib.request

import view3d
from config_parser import load_config

HERE = os.path.dirname(os.path.abspath(__file__))


class StyleTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.layout = os.path.join(self.tmp, "plant.json")
        shutil.copy(os.path.join(HERE, "station_1.json"), self.layout)
        self.server = view3d.View3DServer(port=0)
        self.server.set_layout(load_config(self.layout), self.layout)
        self.server.start()
        self.addCleanup(self.server.stop)

    def call(self, path, body=None):
        req = urllib.request.Request(self.server.url + path, data=None if body is None else json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"}, method="GET" if body is None else "POST")
        with urllib.request.urlopen(req, timeout=5) as res:
            return json.loads(res.read())

    def test_none_yet_then_saved_next_to_the_layout(self):
        first = self.call("style")
        self.assertEqual(first["style"], None)
        self.assertEqual(first["path"], os.path.join(self.tmp, "plant.view3d.json"))
        style = {"belt": {"thickness": 0.3, "surface": "#1d4ed8"}, "items": {"belt_1_1_1": {"hidden": True}}}
        saved = self.call("style", style)
        self.assertTrue(saved["ok"])
        self.assertTrue(os.path.isfile(os.path.join(self.tmp, "plant.view3d.json")))
        self.assertEqual(self.call("style")["style"], style)

    def test_a_run_copy_keeps_its_style_next_to_the_designer_layout(self):
        copy = os.path.join(self.tmp, "run_copy.json")
        shutil.copy(self.layout, copy)
        self.server.set_layout(load_config(copy), copy, source_path=self.layout)
        self.assertEqual(self.call("style")["path"], os.path.join(self.tmp, "plant.view3d.json"))

    def test_not_an_object_is_refused(self):
        with self.assertRaises(urllib.error.HTTPError) as err:
            self.call("style", [1, 2, 3])
        self.assertEqual(err.exception.code, 400)
        self.assertFalse(os.path.exists(os.path.join(self.tmp, "plant.view3d.json")))

    def test_the_page_has_the_style_panel(self):
        with urllib.request.urlopen(self.server.url, timeout=5) as res:
            page = res.read().decode()
        self.assertIn('id="styleBtn"', page)
        self.assertIn('id="style"', page)
        with urllib.request.urlopen(self.server.url + "style.js", timeout=5) as res:
            self.assertIn("export const DEFAULTS", res.read().decode())


if __name__ == "__main__":
    unittest.main()
