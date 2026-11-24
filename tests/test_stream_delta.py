"""
test_stream_delta.py - the live stream sent as what changed (stream_delta.py,
view3d_web/delta.js, view3d.py /events?delta=1): any state of dicts, rows
and values rebuilt exactly, in Python and in the page's JavaScript; the 3D
server's stream with deltas and gzip; far smaller than whole frames.
"""
import gzip
import json
import os
import random
import shutil
import subprocess
import tempfile
import threading
import time
import unittest
import urllib.request
import zlib

import stream_delta
import view3d

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def states(n=200, seed=3):
    """A generic stream: rows that move, come and go; dicts that change; values; keys that appear."""
    rnd = random.Random(seed)
    rows = {f"b{i}": [f"b{i}", "kind", 0.0, 0.0, 1.0, "#fff", i] for i in range(8)}
    sensors = {f"s{i}": False for i in range(6)}
    out = []
    for k in range(n):
        for key in list(rows):
            if rnd.random() < 0.03:
                del rows[key]
        if rnd.random() < 0.2:
            rows[f"b{100 + k}"] = [f"b{100 + k}", "other", 0.0, 0.0, 2.0, "#000", k]
        for r in rows.values():
            r[2] = round(r[2] + rnd.random() * 0.02, 4)
            if rnd.random() < 0.3:
                r[3] = round(rnd.random(), 4)
        if rnd.random() < 0.2:
            sensors[f"s{rnd.randrange(6)}"] = rnd.random() < 0.5
        state = {"t": k * 0.03, "w": 100 + k * 0.03, "layout": 1, "bodies": [list(r) for r in rows.values()],
                 "sensors": dict(sensors), "mode": "auto" if k < 100 else "manual", "empty": []}
        if k % 50 == 10:
            state["extra"] = {"x": k}                     # a key that comes and goes
        if k % 7 == 0 and rows:
            state["bodies"] = list(reversed(state["bodies"]))   # the rows' order changes
        out.append(state)
    return out


class DeltaTest(unittest.TestCase):
    def test_every_state_rebuilt_exactly(self):
        enc, got = stream_delta.DeltaEncoder(), None
        for s in states():
            msg = json.loads(json.dumps(enc.encode(s)))
            got = stream_delta.apply(got, msg)
            self.assertEqual(got, s)

    def test_the_first_frame_and_a_new_layout_are_whole(self):
        enc = stream_delta.DeltaEncoder()
        a, b = states(3)[:2]
        self.assertNotIn("Δ", enc.encode(a))
        self.assertIn("Δ", enc.encode(b))
        b2 = dict(b, layout=2)
        self.assertNotIn("Δ", enc.encode(b2))

    def test_unchanged_is_almost_nothing(self):
        enc = stream_delta.DeltaEncoder()
        s = states(1)[0]
        enc.encode(s)
        msg = enc.encode(dict(s, t=s["t"] + 0.03, w=s["w"] + 0.03))
        self.assertEqual(msg["Δ"], {})

    @unittest.skipUnless(shutil.which("node"), "node is not installed")
    def test_the_page_rebuilds_the_same_states(self):
        stream = states()
        enc = stream_delta.DeltaEncoder()
        msgs = [enc.encode(s) for s in stream]
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, True)
        with open(os.path.join(tmp, "msgs.json"), "w", encoding="utf-8") as f:
            json.dump({"msgs": msgs, "states": stream}, f, ensure_ascii=False)
        script = os.path.join(tmp, "check.mjs")
        delta_js = "file:///" + os.path.join(HERE, "view3d_web", "delta.js").replace("\\", "/")
        with open(script, "w", encoding="utf-8") as f:
            f.write(f'import {{ applyDelta }} from "{delta_js}";\n'
                    'import fs from "node:fs";\n'
                    'const {msgs, states} = JSON.parse(fs.readFileSync(process.argv[2], "utf8"));\n'
                    'let s = null, bad = 0;\n'
                    'msgs.forEach((m, i) => { s = applyDelta(s, m); '
                    'if (JSON.stringify(s) !== JSON.stringify(states[i])) bad++; });\n'
                    'console.log(bad === 0 ? "same" : `${bad} differ`);\n')
        out = subprocess.run(["node", script, os.path.join(tmp, "msgs.json")], capture_output=True, text=True,
                             timeout=60)
        self.assertEqual(out.stdout.strip(), "same", out.stderr)


class StreamTest(unittest.TestCase):
    def setUp(self):
        self.server = view3d.View3DServer(port=0)
        self.server.start()
        self.addCleanup(self.server.stop)

    def read_frames(self, path, headers, count):
        frames, raw = [], []

        def listen():
            req = urllib.request.Request(self.server.url + path, headers=headers)
            with urllib.request.urlopen(req, timeout=10) as r:
                self.encoding = r.headers.get("Content-Encoding")
                unpack = zlib.decompressobj(31) if self.encoding == "gzip" else None
                text = ""
                while len(frames) < count:
                    chunk = r.read1(65536) if hasattr(r, "read1") else r.read(1)
                    if not chunk:
                        break
                    raw.append(len(chunk))
                    text += (unpack.decompress(chunk) if unpack else chunk).decode("utf-8")
                    while "\n\n" in text:
                        event, text = text.split("\n\n", 1)
                        if event.startswith("data: "):
                            frames.append(json.loads(event[6:]))

        reader = threading.Thread(target=listen)
        reader.start()
        end = time.monotonic() + 5
        while self.server.clients == 0 and time.monotonic() < end:
            time.sleep(0.01)
        for s in states(count):
            self.server.publish(lambda s=s: dict(s), force=True)
            time.sleep(0.005)
        reader.join(10)
        return frames, sum(raw)

    def test_deltas_and_gzip_rebuild_the_frames(self):
        frames, size = self.read_frames("events?delta=1", {"Accept-Encoding": "gzip"}, 60)
        self.assertEqual(self.encoding, "gzip")
        got = None
        rebuilt = []
        for f in frames:
            got = stream_delta.apply(got, f)
            rebuilt.append(got)
        sent = states(60)
        self.assertGreaterEqual(len(rebuilt), 5)        # a busy machine: the server skips to the newest frame
        last = rebuilt[-1]
        expected = next(s for s in sent if abs(s["t"] - last["t"]) < 1e-9)
        self.assertEqual({k: v for k, v in last.items() if k not in ("layout", "w")},
                         {k: v for k, v in expected.items() if k not in ("layout", "w")})
        whole = sum(len(json.dumps(s, separators=(",", ":"))) for s in sent[:len(frames)])
        self.assertLess(size, whole / 3)        # far less on the wire (random test data; a real plant: ~16 x)

    def test_a_page_that_does_not_ask_gets_whole_frames(self):
        frames, _size = self.read_frames("events", {}, 5)
        self.assertIsNone(self.encoding)
        self.assertTrue(all("Δ" not in f and "bodies" in f for f in frames))


if __name__ == "__main__":
    unittest.main()
