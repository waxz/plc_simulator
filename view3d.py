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

Standard library only (http.server + server-sent events). The server lives
for the whole simulator process, across layout reloads (set_layout).
"""
import json
import os
import queue
import threading
import time
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
DEFAULT_PORT = 8765
FRAME_INTERVAL_S = 1.0 / 30.0
KEEPALIVE_S = 10.0
CONTENT_TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8"}


class View3DServer:
    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_PORT):
        self.host, self.port = host, port
        self.config = None
        self.config_path: Optional[str] = None
        self.version = 0
        self._scene_json: Optional[bytes] = None
        self._frame: Optional[bytes] = None
        self._frame_id = 0
        self._cond = threading.Condition()
        self._clients = 0
        self._last_publish = 0.0
        self.commands: "queue.Queue[tuple]" = queue.Queue()
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

            self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
            self._httpd.daemon_threads = True
            self.port = self._httpd.server_address[1]          # port 0: the one the OS picked
            threading.Thread(target=self._httpd.serve_forever, name="view3d", daemon=True).start()
            print(f"[3D] 3D view at {self.url}")
        return self.url

    def stop(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None

    def set_layout(self, config, config_path: Optional[str]) -> None:
        """A (new) layout: the page fetches /scene again."""
        with self._cond:
            self.config, self.config_path = config, config_path
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
        data = json.dumps(state, separators=(",", ":")).encode("utf-8")
        with self._cond:
            self._frame, self._frame_id = data, self._frame_id + 1
            self._cond.notify_all()
        return True

    def _wait_frame(self, last_id: int, timeout: float):
        with self._cond:
            self._cond.wait_for(lambda: self._frame_id != last_id or self._httpd is None, timeout)
            return self._frame_id, self._frame

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
            self._send(200, body, CONTENT_TYPES.get(os.path.splitext(full)[1], "application/octet-stream"))

    def do_POST(self):
        path = self.path.split("?", 1)[0]
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

    def _events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        view = self.view
        view._client(+1)
        last = -1
        try:
            while view.running:
                frame_id, frame = view._wait_frame(last, KEEPALIVE_S)
                if frame is None or frame_id == last:
                    self.wfile.write(b": keepalive\n\n")
                else:
                    last = frame_id
                    self.wfile.write(b"data: " + frame + b"\n\n")
                self.wfile.flush()
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


def pump(renderer, physics_engine) -> None:
    """Once per window frame (main.run_pygame): the newest state to open 3D
    views, and their button presses to the simulation."""
    s = SERVER
    if s is None or not s.running:
        return
    if s.config is not renderer.config:
        s.set_layout(renderer.config, getattr(renderer, "config_path", None))
    for kind, item_id, pressed in s.take_commands():
        if kind == "door":
            door = next((d for d in renderer.config.doors if d.id == item_id), None)
            if door is not None:
                renderer._toggle_door(door)
            continue
        button = next((b for b in renderer.config.buttons if b.id == item_id), None)
        if kind == "button" and button is not None:
            renderer._set_button(button, pressed)
    if s.clients and physics_engine is not None:
        with renderer.sim_lock:
            s.publish(lambda: scene3d.dynamic_state(renderer.config, physics_engine.parcel_views(),
                                                    physics_engine.sim_time))
