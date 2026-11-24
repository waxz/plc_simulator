"""
view3d.py - the 3D view of the running simulation, in a browser.

The simulator window (main.py) serves a small page on 127.0.0.1 (port 8765
by default): view3d_web/index.html + viewer.js draw the layout with three.js
and follow the simulation live. Open it with the window's 3D View button
(key 3), `python main.py layout.json --view3d`, or any browser at the URL.

  GET  /              the viewer page (view3d_web/index.html, viewer.js)
  GET  /scene         the layout in 3D (scene3d.static_scene), once per layout
  GET  /events        the live state (scene3d.dynamic_state) as server-sent
                      events, ~30 a second, while the page is open
  POST /button        {"id": ..., "pressed": true|false} - press an operator
                      button in the 3D view (applied by the window's loop)
  POST /door          {"id": ...} - open / close a guard door
  GET  /style         the layout's 3D style (view3d_web/style.js): {"path", "style"}
  POST /style         save it, next to the layout: <layout>.view3d.json
  GET  /tags/meta     the layout's PLC tags (plc_tags.tags_of): name, address, type, comment
  GET  /tags/data?since=N   their values sampled since sample N: the page's tag monitor and charts

The query API (read-only JSON, for debugging - sim_query.py is its client):
  GET  /api                     what it offers
  GET  /api/state               time, play / speed, health, the PLC link, faults, chutes,
                                watchdogs, lamps, parcels and deliveries
  GET  /api/tags?match=a,b&area=I|Q   PLC tags now: name, address, type, value (match: substrings
                                or names; none: every tag)
  GET  /api/history?match=...&seconds=N   their values over the last N s (sim time, number)
  GET  /api/components?match=...      components: their PLC signals now and their control
  GET  /api/parcels             every parcel: id, x, y, speed, barcode, destination
  GET  /api/tracking            the PLC's tracked loads checked against the parcels (tracking.py):
                                errors along / across the travel, length, width; the last loads

Standard library only (http.server + server-sent events). The server lives
for the whole simulator process, across layout reloads (set_layout).
"""
import json
import os
import queue
import socket
import sys
import threading
import time
import urllib.parse
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional

import scene3d

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(HERE, "view3d_web")
# three.js comes from the CDN unless a copy of the npm package three@0.160.0
# is unpacked here (view3d_web/vendor/three/build/three.module.js ...) - for
# a PC without internet access.
THREE_CDN = "https://cdn.jsdelivr.net/npm/three@0.160.0/"
THREE_LOCAL = os.path.join(STATIC_DIR, "vendor", "three")
# Chart.js (the tag monitor's charts) likewise: view3d_web/vendor/chart.js/chart.umd.min.js when there.
CHART_CDN = "https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/"
CHART_LOCAL = os.path.join(STATIC_DIR, "vendor", "chart.js")
DEFAULT_PORT = 8765
STYLE_SUFFIX = ".view3d.json"   # the 3D style of layout.json: layout.view3d.json, next to it
MAX_STYLE_BYTES = 1 << 20
TAG_SAMPLE_S = 0.05             # the tag monitor's samples: 20 a second, while a page asks for them
TAG_KEEP = 2400                 # ... the last 2 minutes
API_STATE_S = 0.2               # /api/state rebuilt at most this often, while it is asked for
API_IDLE_S = 30.0
TAG_IDLE_S = 600.0              # no page asked this long: stop sampling (a hidden tab polls rarely:
                                # keep the record whole meanwhile; it costs one read of the tags 20 times a second)
FRAME_INTERVAL_S = 1.0 / 30.0
KEEPALIVE_S = 10.0
CONTENT_TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8"}


class _ExclusiveServer(ThreadingHTTPServer):
    """One server per port. http.server binds with SO_REUSEADDR, which on
    Windows lets a second simulator bind a port the first still serves -
    and the browser then talks to the old one (the old layout). Bind it
    exclusively instead (Windows: SO_EXCLUSIVEADDRUSE)."""
    allow_reuse_address = sys.platform != "win32"
    daemon_threads = True

    def server_bind(self):
        if sys.platform == "win32":
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


class View3DServer:
    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_PORT):
        self.host, self.port = host, port
        self.config = None
        self.config_path: Optional[str] = None
        self.source_path: Optional[str] = None     # the layout a run copy (config_path) was made from
        # Layout versions go out with every frame; the page reloads the scene
        # when the number changes. Start from the clock, not 0: a page still
        # open from an earlier simulator reconnects to this one and must see a
        # new number, not the "1" its old layout also had.
        self.version = int(time.time() * 1000)
        self._scene_json: Optional[bytes] = None
        self._frame: Optional[bytes] = None
        self._frame_id = 0
        self._state: Optional[dict] = None      # the last frame as a dict: each client's delta is made from it
        self._cond = threading.Condition()
        self._clients = 0
        self._last_publish = 0.0
        self.commands: "queue.Queue[tuple]" = queue.Queue()
        # The PLC tag monitor: the layout's tags, and their values sampled (pump)
        # while a page asks: (seq, sim time, [numbers]); the newest texts.
        self._tags = None
        self._tag_meta: Optional[bytes] = None
        self._tag_samples: "list" = []
        self._tag_seq = 0
        self._tag_texts: list = []
        self._tag_last_sample = 0.0
        self._tag_wanted_at = -1e9
        # The query API's state (built in pump, under the simulation's lock, while asked for).
        self._api_state: Optional[dict] = None
        self._api_built = 0.0
        self._api_wanted_at = -1e9
        self._httpd: Optional[ThreadingHTTPServer] = None

    # -- lifecycle -------------------------------------------------------------------------

    @property
    def url(self) -> str:
        return f"http://{self.host}:{self.port}/"

    @property
    def running(self) -> bool:
        return self._httpd is not None

    def start(self) -> str:
        if self._httpd is None:
            server = self

            class Handler(_Handler):
                view = server

            try:
                self._httpd = _ExclusiveServer((self.host, self.port), Handler)
            except OSError:
                # Another simulator (or program) has the port: take a free one.
                print(f"[3D] Port {self.port} is in use (another simulator?) - using a free port.")
                self._httpd = _ExclusiveServer((self.host, 0), Handler)
            self.port = self._httpd.server_address[1]          # port 0: the one the OS picked
            threading.Thread(target=self._httpd.serve_forever, name="view3d", daemon=True).start()
            print(f"[3D] 3D view at {self.url}")
        return self.url

    def stop(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None

    def set_layout(self, config, config_path: Optional[str], source_path: Optional[str] = None) -> None:
        """A (new) layout: the page fetches /scene again."""
        with self._cond:
            self.config, self.config_path, self.source_path = config, config_path, source_path
            self._tags, self._tag_meta, self._tag_samples, self._tag_texts = None, None, [], []
            self.version += 1
            self._scene_json = None

    # -- data --------------------------------------------------------------------------------

    def scene_json(self) -> bytes:
        with self._cond:
            if self._scene_json is None and self.config is not None:
                scene = scene3d.static_scene(self.config, self.config_path, self.version)
                scene["title"] = os.path.basename(self.config_path) if self.config_path else "layout"
                self._scene_json = json.dumps(scene, separators=(",", ":")).encode("utf-8")
            return self._scene_json or b"{}"

    @property
    def style_path(self) -> Optional[str]:
        """Where this layout's 3D style is kept: next to the layout file (the
        designer's, when this is its run copy); None without a file."""
        base = self.source_path or self.config_path
        return os.path.splitext(os.path.abspath(base))[0] + STYLE_SUFFIX if base else None

    def read_style(self) -> dict:
        path = self.style_path
        style = None
        if path and os.path.isfile(path):
            try:
                with open(path, encoding="utf-8") as f:
                    style = json.load(f)
            except (OSError, ValueError) as exc:
                print(f"[3D] Ignoring {path}: {exc}")
        return {"path": path or "", "style": style if isinstance(style, dict) else None}

    def write_style(self, style: dict) -> str:
        path = self.style_path
        if not path:
            raise ValueError("no layout file to keep the style next to")
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(style, f, indent=2)
        os.replace(tmp, path)
        return path

    # -- the PLC tag monitor -------------------------------------------------------------------

    def _tag_list(self):
        if self._tags is None and self.config is not None:
            import plc_tags
            try:
                self._tags, summary = plc_tags.tags_of(self.config)
            except Exception as exc:  # noqa: BLE001 - a layout the exchange can't describe
                self._tags, summary = [], f"no tags: {exc}"
            self._tag_meta = json.dumps({"version": self.version, "summary": summary, "tags": [
                {"name": t.name, "address": t.address, "type": t.data_type, "area": t.area, "comment": t.comment,
                 "component": t.component, "signal": t.signal,
                 "shownOnly": t.component in {v.id for v in getattr(self.config, "plcValues", [])},
                 "tracked": t.component in {v.id for v in getattr(self.config, "trackedLoads", [])}}
                for t in self._tags]}, separators=(",", ":")).encode("utf-8")
        return self._tags or []

    def tag_meta(self) -> bytes:
        with self._cond:
            self._tag_list()
            return self._tag_meta or b'{"tags":[]}'

    @property
    def tags_wanted(self) -> bool:
        return time.monotonic() - self._tag_wanted_at < TAG_IDLE_S

    def sample_tags(self, sim_time: float, force: bool = False) -> bool:
        """Read every tag's value (call under the simulation's lock): a sample
        at most every TAG_SAMPLE_S, and only while a page asks for them."""
        if not force and (not self.tags_wanted or time.perf_counter() - self._tag_last_sample < TAG_SAMPLE_S):
            return False
        self._tag_last_sample = time.perf_counter()
        import plc_tags
        with self._cond:
            tags = self._tag_list()
        numbers, texts = plc_tags.read_values(self.config, tags)
        with self._cond:
            self._tag_seq += 1
            self._tag_samples.append((self._tag_seq, round(float(sim_time), 4), [round(n, 5) for n in numbers]))
            del self._tag_samples[:-TAG_KEEP]
            self._tag_texts = texts
        return True

    # -- the query API ------------------------------------------------------------------------

    @property
    def api_wanted(self) -> bool:
        return time.monotonic() - self._api_wanted_at < API_IDLE_S

    def build_api_state(self, renderer, physics_engine, force: bool = False) -> bool:
        """The simulation's state for /api/state (call under its lock)."""
        if not force and (not self.api_wanted or time.perf_counter() - self._api_built < API_STATE_S):
            return False
        self._api_built = time.perf_counter()
        c = renderer.config
        exchange = getattr(renderer, "exchange", None)
        stats = {}
        if exchange is not None and getattr(exchange, "_enabled", False):
            try:
                stats = dict(exchange.stats())
            except Exception:  # noqa: BLE001 - debug data only
                stats = {}
        health = renderer._sim_health(physics_engine) if hasattr(renderer, "_sim_health") else {}
        status = renderer.health_status(physics_engine) if hasattr(renderer, "health_status") else None
        views = physics_engine.parcel_views() if physics_engine is not None else []
        state = {
            "layout": self.source_path or self.config_path, "layoutVersion": self.version,
            "simTime": round(float(getattr(physics_engine, "sim_time", 0.0)), 4),
            "playing": bool(getattr(renderer, "playing", False)), "speed": getattr(renderer, "speed", None),
            "health": status[0] if status else None, "timing": health,
            "plcLink": stats,
            "parcels": len(views), "delivered": dict(getattr(physics_engine, "delivered", {})),
            "droppedTime": round(float(getattr(physics_engine, "dropped_time", 0.0)), 3),
            "beltFaults": [b.id for b in c.belts if b.fault],
            "beltsRunning": [b.id for b in c.belts if b.running],
            "chutes": [{"id": s.id, "count": s.count, "capacity": s.capacity, "full": bool(s.full),
                        "fillMode": getattr(s, "fillMode", "count")} for s in c.sinks if s.capacity],
            "chuteStats": chute_rows(renderer),
            "watchdogs": [{"id": w.id, "plcAlive": bool(w.plcAlive), "tripped": bool(w.tripped), "trips": w.trips,
                           "plcHeartbeat": w.plcHeartbeat, "simHeartbeat": w.simHeartbeat,
                           "latencyMs": w.latencyMs, "latencyMaxMs": w.latencyMaxMs}
                          for w in getattr(c, "watchdogs", [])],
            "lamps": {led.id: (int(led.mode) if getattr(led, "mode", 0) else bool(led.isOn)) for led in c.leds},
            "manual": [comp.id for kind in ("belts", "diverters", "sinks", "stoppers", "turntables", "transfers")
                       for comp in getattr(c, kind, []) if getattr(comp, "control", None) == "manual"],
        }
        import tracking
        report = tracking.report_of(physics_engine) if physics_engine is not None else None
        if report is not None:
            state["tracking"] = {k: v for k, v in report.items() if k not in ("recentLoads", "units")}
            state["_tracking"] = report
        with self._cond:
            self._api_state = state
        return True

    def api(self, path: str, params: dict) -> dict:
        """One query (the HTTP thread)."""
        self._api_wanted_at = time.monotonic()
        match = [m.strip() for m in params.get("match", "").split(",") if m.strip()]

        def matches(*names) -> bool:
            if not match:
                return True
            return any(m.lower() == n.lower() or m.lower() in n.lower() for m in match for n in names if n)

        if path in ("/api", "/api/"):
            return {"simulator": True, "endpoints": {
                "/api/state": "time, play / speed, health, the PLC link, faults, chutes, watchdogs, lamps, parcels",
                "/api/tags?match=a,b&area=I|Q": "PLC tags now (match: names or substrings; area: I or Q)",
                "/api/history?match=a,b&seconds=30": "the tags' values over the last seconds (sim time, number)",
                "/api/components?match=a,b": "components: their PLC signals now and their control",
                "/api/parcels": "every parcel: id, x, y, speed, barcode, destination",
                "/api/tracking": "the PLC's tracked loads checked against the parcels: errors (mm), the last loads"}}
        if path == "/api/state":
            for _ in range(20):                         # the first query: wait for a state (up to 1 s)
                with self._cond:
                    state = self._api_state
                if state is not None:
                    return {k: v for k, v in state.items() if not k.startswith("_")}
                time.sleep(0.05)
            return {"error": "no state yet: is the simulator window running?"}
        if path in ("/api/tags", "/api/history"):
            self._tag_wanted_at = time.monotonic()
            for _ in range(20):
                with self._cond:
                    tags, texts, samples = self._tags, list(self._tag_texts), list(self._tag_samples)
                if tags is not None and texts:
                    break
                time.sleep(0.05)
            if not tags or not texts:
                return {"error": "no tag values yet (sampling starts with this query; ask again)"}
            area = params.get("area", "").upper()
            picked = [(i, t) for i, t in enumerate(tags) if matches(t.name, t.component) and
                      (not area or t.area == area)]
            if path == "/api/tags":
                return {"simTime": samples[-1][1] if samples else None, "tags": [
                    {"name": t.name, "address": t.address, "type": t.data_type, "value": texts[i],
                     "comment": t.comment} for i, t in picked]}
            try:
                seconds = float(params.get("seconds", "30") or 30)
            except ValueError:
                seconds = 30.0
            end = samples[-1][1] if samples else 0.0
            rows = [s for s in samples if s[1] >= end - seconds]
            return {"names": [t.name for _i, t in picked], "samples": [[s[1]] + [s[2][i] for i, _t in picked]
                                                                       for s in rows]}
        if path == "/api/components":
            import io_signals
            from device_kinds import DEVICE_KINDS
            c = self.config
            lists = ["belts", "sensors", "buttons", "leds", "sinks", *DEVICE_KINDS]
            ids = [comp.id for name in dict.fromkeys(lists) for comp in getattr(c, name, []) if matches(comp.id)]
            return {"components": io_signals.read(c, ids) if ids else {}}
        if path == "/api/parcels":
            with self._cond:
                parcels = list((self._api_state or {}).get("_parcel_rows", []))
            return {"parcels": parcels}
        if path == "/api/tracking":
            with self._cond:
                report = (self._api_state or {}).get("_tracking")
            return report if report is not None else {"error": "no tracked loads: the layout has no trackedLoads "
                                                              "(or no watchdog), or no state yet"}
        return {"error": f"no {path}: see /api"}

    def tag_data(self, since: int) -> bytes:
        self._tag_wanted_at = time.monotonic()
        with self._cond:
            samples = [s for s in self._tag_samples if s[0] > since]
            return json.dumps({"version": self.version, "seq": self._tag_seq, "samples": samples,
                               "texts": self._tag_texts}, separators=(",", ":")).encode("utf-8")

    @property
    def clients(self) -> int:
        return self._clients

    def publish(self, make_state: Callable[[], dict], force: bool = False) -> bool:
        """Build and send a frame if a page is open and one is due (~30/s).
        `make_state` runs on the caller's thread (under its lock)."""
        if self._clients <= 0:
            return False
        now = time.perf_counter()
        if not force and now - self._last_publish < FRAME_INTERVAL_S:
            return False
        self._last_publish = now
        state = make_state()
        state["layout"] = self.version
        state["w"] = round(now, 4)          # the server's clock when sent: the page plays frames back on it
        data = json.dumps(state, separators=(",", ":")).encode("utf-8")
        with self._cond:
            self._frame, self._state, self._frame_id = data, state, self._frame_id + 1
            self._cond.notify_all()
        return True

    def _wait_frame(self, last_id: int, timeout: float):
        with self._cond:
            self._cond.wait_for(lambda: self._frame_id != last_id or self._httpd is None, timeout)
            return self._frame_id, self._frame

    def _wait_state(self, last_id: int, timeout: float):
        with self._cond:
            self._cond.wait_for(lambda: self._frame_id != last_id or self._httpd is None, timeout)
            return self._frame_id, self._state

    def _client(self, delta: int) -> None:
        with self._cond:
            self._clients += delta

    def take_commands(self):
        out = []
        while True:
            try:
                out.append(self.commands.get_nowait())
            except queue.Empty:
                return out


class _Handler(BaseHTTPRequestHandler):
    view: View3DServer = None
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):          # quiet: no line per request
        pass

    def _send(self, code: int, body: bytes, content_type: str = "application/json"):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/scene":
            self._send(200, self.view.scene_json())
        elif path == "/style":
            self._send(200, json.dumps(self.view.read_style()).encode("utf-8"))
        elif path == "/tags/meta":
            self._send(200, self.view.tag_meta())
        elif path == "/api" or path.startswith("/api/"):
            query = self.path.split("?", 1)[1] if "?" in self.path else ""
            from urllib.parse import parse_qs
            params = {k: v[-1] for k, v in parse_qs(query).items()}
            self._send(200, json.dumps(self.view.api(path, params), default=str).encode("utf-8"))
        elif path == "/tags/data":
            query = self.path.split("?", 1)[1] if "?" in self.path else ""
            params = dict(p.partition("=")[::2] for p in query.split("&") if p)
            try:
                since = int(params.get("since", "0") or 0)
            except ValueError:
                since = 0
            self._send(200, self.view.tag_data(since))
        elif path == "/events":
            self._events()
        else:
            name = "index.html" if path in ("/", "") else path.lstrip("/")
            full = os.path.normpath(os.path.join(STATIC_DIR, name))
            if not full.startswith(STATIC_DIR) or not os.path.isfile(full):
                self._send(404, b"not found", "text/plain")
                return
            with open(full, "rb") as f:
                body = f.read()
            if name == "index.html" and os.path.isfile(os.path.join(THREE_LOCAL, "build", "three.module.js")):
                body = body.replace(THREE_CDN.encode(), b"/vendor/three/")
            if name == "index.html" and os.path.isfile(os.path.join(CHART_LOCAL, "chart.umd.min.js")):
                body = body.replace(CHART_CDN.encode(), b"/vendor/chart.js/")
            self._send(200, body, CONTENT_TYPES.get(os.path.splitext(full)[1], "application/octet-stream"))

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/style":
            self._post_style()
            return
        if path == "/reset_stats":                  # the page's Reset of the chute statistics
            self.rfile.read(int(self.headers.get("Content-Length", 0) or 0))
            self.view.commands.put(("reset_stats", "", True))
            self._send(200, b'{"ok":true}')
            return
        if path not in ("/button", "/door"):
            self._send(404, b"not found", "text/plain")
            return
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0) or 0)) or b"{}")
            if path == "/door":
                self.view.commands.put(("door", str(body["id"]), True))
            else:
                self.view.commands.put(("button", str(body["id"]), bool(body["pressed"])))
        except (ValueError, KeyError, TypeError):
            self._send(400, b'{"ok":false}')
            return
        self._send(200, b'{"ok":true}')

    def _post_style(self):
        length = int(self.headers.get("Content-Length", 0) or 0)
        if length > MAX_STYLE_BYTES:
            self._send(413, b'{"ok":false,"error":"too big"}')
            return
        try:
            style = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(style, dict):
                raise ValueError("a style is a JSON object")
            path = self.view.write_style(style)
        except (ValueError, OSError) as exc:
            self._send(400, json.dumps({"ok": False, "error": str(exc)}).encode("utf-8"))
            return
        self._send(200, json.dumps({"ok": True, "path": path}).encode("utf-8"))

    def _events(self):
        """The live stream. A page that asks for deltas (events?delta=1, stream_delta.py) gets its
        first frame whole and then only what changed; one that accepts gzip gets the stream
        compressed as one (each frame flushed: consecutive frames compress against each other).
        Over a slow link (a tunnel) that is about 16 x less than whole frames."""
        import zlib
        import stream_delta
        query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
        deltas = query.get("delta", ["0"])[0] == "1"
        gzip = "gzip" in (self.headers.get("Accept-Encoding") or "") and query.get("raw", ["0"])[0] != "1"
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store, no-transform")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")          # a proxy in between: do not hold frames back
        if gzip:
            self.send_header("Content-Encoding", "gzip")
        self.end_headers()
        packer = zlib.compressobj(6, zlib.DEFLATED, 31) if gzip else None

        def send(chunk: bytes) -> None:
            if packer is not None:
                chunk = packer.compress(chunk) + packer.flush(zlib.Z_SYNC_FLUSH)
            self.wfile.write(chunk)
            self.wfile.flush()

        view = self.view
        view._client(+1)
        encoder = stream_delta.DeltaEncoder()
        last = -1
        try:
            while view.running:
                if deltas:
                    frame_id, state = view._wait_state(last, KEEPALIVE_S)
                    frame = None if state is None or frame_id == last else json.dumps(
                        encoder.encode(state), separators=(",", ":"), ensure_ascii=False).encode("utf-8")
                else:
                    frame_id, frame = view._wait_frame(last, KEEPALIVE_S)
                    if frame_id == last:
                        frame = None
                if frame is None:
                    send(b": keepalive\n\n")
                else:
                    last = frame_id
                    send(b"data: " + frame + b"\n\n")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            view._client(-1)


# ---------------------------------------------------------------------------
# One server per simulator process
# ---------------------------------------------------------------------------

SERVER: Optional[View3DServer] = None


def server(port: int = DEFAULT_PORT) -> View3DServer:
    global SERVER
    if SERVER is None:
        SERVER = View3DServer(port=port)
    return SERVER


def open_view(config, config_path: Optional[str], browser: bool = True, port: int = DEFAULT_PORT) -> str:
    """Start serving (if not yet) the 3D view of this layout; open it in the
    default browser."""
    s = server(port)
    if s.config is not config:
        s.set_layout(config, config_path)
    url = s.start()
    if browser:
        webbrowser.open(url)
    return url


def chute_rows(renderer) -> list:
    """Each sink's statistics (chute_stats.py, the window's): total, per minute, per hour."""
    stats = getattr(renderer, "chute_stats", None)
    return stats.rows([s.id for s in renderer.config.sinks]) if stats is not None else []


def pump(renderer, physics_engine) -> None:
    """Once per window frame (main.run_pygame): the newest state to open 3D
    views, and their button presses to the simulation."""
    s = SERVER
    if s is None or not s.running:
        return
    if s.config is not renderer.config:
        s.set_layout(renderer.config, getattr(renderer, "config_path", None), getattr(renderer, "source_path", None))
    for kind, item_id, pressed in s.take_commands():
        if kind == "reset_stats" and hasattr(renderer, "chute_stats"):
            renderer.chute_stats.reset(renderer.chute_stats.now)
            continue
        if kind == "door":
            door = next((d for d in renderer.config.doors if d.id == item_id), None)
            if door is not None:
                renderer._toggle_door(door)
            continue
        button = next((b for b in renderer.config.buttons if b.id == item_id), None)
        if kind == "button" and button is not None:
            renderer._set_button(button, pressed)
    if s.tags_wanted and physics_engine is not None:
        with renderer.sim_lock:
            s.sample_tags(physics_engine.sim_time)
    if s.api_wanted and physics_engine is not None:
        with renderer.sim_lock:
            if s.build_api_state(renderer, physics_engine):
                s._api_state["_parcel_rows"] = [
                    {"id": v.id, "kind": v.kind, "x": round(v.x, 3), "y": round(v.y, 3),
                     "speed": round(float(v.actual_speed or 0.0), 3), "barcode": v.barcode,
                     "destination": v.destination, "level": v.level, "carrier": v.carrier}
                    for v in physics_engine.parcel_views()]
    if s.clients and physics_engine is not None:
        with renderer.sim_lock:
            s.publish(lambda: dict(scene3d.dynamic_state(renderer.config, physics_engine.parcel_views(),
                                                         physics_engine.sim_time),
                                   chuteStats=chute_rows(renderer)))
