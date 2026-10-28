import os
import subprocess
import sys
from collections import deque
from contextlib import nullcontext
import pygame
import pygame_gui
import math
import time
from typing import Tuple, Optional, List
from config_parser import SimulatorConfig, Belt, Sensor, ControlButton, LedIndicator
from math_utils import get_rect_vertices, point_in_polygon, point_in_arc, get_belt_polygon, get_default_quad_vertices
from surface import belt_contains
import devices
import hmi_style as hmi
from cad.belts import flow_arrows
from editor_ui import BOTTOM_BAR, CANVAS_TOP, TOP_BAR
from sim_drawing import DrawingLayer
from cad.checks import belt_footprint
from cad.path import belt_world_path, path_length, segments as path_segments
from cad.ports import centerline_points
from editor_ui import EditorUI


SPEEDS = (1.0, 2.0, 4.0, 8.0, None)   # real-time factors the Speed button cycles; None = max
FRAME_LOCKED_MAX_SPEED = 8.0          # "max" when physics runs once per frame (--no-sim-thread)


def speed_label(speed: Optional[float]) -> str:
    return "MAX" if speed is None else f"{speed:g}x"


class Renderer:
    def __init__(self, config: SimulatorConfig, headless: bool = False, scale: float = 100.0):
        self.config = config
        self.headless = headless
        self.scale = scale  # pixels per meter
        self.width = 1200
        self.height = 800
        self.screen = None
        self.clock = pygame.time.Clock()
        self.fps = 60
        
        if not self.headless:
            pygame.init()
            self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
            pygame.display.set_caption("Conveyor Flow Simulator")
        if not self.headless or pygame.font.get_init():
            self._init_fonts()

        # Cached rotated/filled Surfaces (belt/sensor/source/sink) and
        # rendered text (button/LED labels, the static Controls line) -
        # these used to be rebuilt/re-rasterized from scratch every single
        # frame for every component even when nothing about them had
        # changed since the last frame. Keyed by the exact parameters that
        # determine their pixels, so a real change (size, color, rotation,
        # label text) naturally produces a fresh cache entry instead of
        # needing manual invalidation.
        self._surface_cache = {}
        self._text_cache = {}
        self._surface_cache_zoom = None

        # Labels queued this frame (see _queue_label) and the [V] toggle.
        self._labels = []
        self.show_values = False
        self.exchange = None  # set by main.run_pygame: the status bar shows the PLC link
        self.physics = None   # set by main.run_pygame: Clear Parcels / Reset act on it
        self.config_path = None  # set by main.run_pygame: "Edit in Designer" opens it
        # The layout's DXF background (config.background), read on the first
        # frame - config_path, which its path is relative to, is set by then.
        self.drawing = DrawingLayer()
        # Simulation speed (sim_runner.py): a real-time factor, None = as fast
        # as possible. The window renders at its own rate; it holds the
        # physics lock (set by main.py) only while drawing the scene.
        self.speed: Optional[float] = 1.0
        self.sim_lock = nullcontext()
        self._ratio_samples = deque()   # (wall, sim_time) over the last couple of seconds
        self.show_drawing = True
        self._drawing_loaded = False

        # Camera - initialize to center the origin
        self.camera_x = self.width / 2.0
        self.camera_y = self.height / 2.0
        self.zoom = 1.0
        self.is_dragging = False
        self.drag_start = (0, 0)
        self.camera_start = (0.0, 0.0)
        
        # Editor State
        self.playing = False
        self.selected_item = None
        self.pressed_button = None

        # For the "Sim/Real" timing indicator - time.perf_counter() rather
        # than pygame.time.get_ticks(), since the latter needs pygame.init()
        # which is skipped entirely in headless mode. Only wall time spent
        # PLAYING counts: the app starts paused, and dividing sim time by
        # time-since-launch reported e.g. 0.48x after 18 s spent paused.
        self.sim_start_perf = time.perf_counter()
        self.play_real_time = 0.0
        self._play_clock_last = self.sim_start_perf
        
        # GUI
        if not self.headless:
            hmi.apply_saved_theme()
            self.manager = pygame_gui.UIManager((self.width, self.height), hmi.pygame_gui_theme())
            self.editor_ui = EditorUI(self.manager, self.width, self.height, self.config)
            
    def set_theme(self, name: str):
        """Switch the whole window between the dark and light palette (and
        remember it for both windows)."""
        hmi.set_theme(name)
        hmi.save_settings(theme=name)
        self._surface_cache.clear()
        self._text_cache.clear()
        if not self.headless:
            self.manager.get_theme().load_theme(hmi.pygame_gui_theme())
            self.manager.rebuild_all_from_changed_theme_data()

    def open_designer(self):
        """Open this layout in the layout editor (its own process): layouts
        are designed there, then run here. A designer already running shows
        it instead of a second one starting (sim_link.py)."""
        import sim_link
        if self.config_path and sim_link.request_open_designer(self.config_path) is not None:
            print(f"[Sim] The designer is showing {self.config_path}.")
            return
        starting = getattr(self, "_designer_process", None)
        if starting is not None and starting.poll() is None:
            print("[Sim] The designer is still starting.")
            return
        editor = os.path.join(os.path.dirname(os.path.abspath(__file__)), "layout_editor.py")
        args = [sys.executable, editor] + ([self.config_path] if self.config_path else [])
        try:
            self._designer_process = subprocess.Popen(args, cwd=os.path.dirname(editor))
            print(f"[Sim] Opened the designer on {self.config_path or 'a new layout'} - save there, "
                  f"then Run in Simulator to see the changes here.")
        except OSError as exc:
            print(f"[Sim] Could not start the designer: {exc}")

    def run_command(self, cmd):
        """The simulation buttons / keys: returns the command for main.py's
        log, after doing what the window itself can."""
        if cmd == "TOGGLE_PLAY":
            self.playing = not self.playing
        elif cmd == "TOGGLE_THEME":
            self.set_theme(hmi.other_theme())
        elif cmd == "TOGGLE_DRAWING":
            self.show_drawing = not self.show_drawing
        elif cmd in ("CYCLE_SPEED", "SPEED_UP", "SPEED_DOWN"):
            i = SPEEDS.index(self.speed) if self.speed in SPEEDS else 0
            step = -1 if cmd == "SPEED_DOWN" else 1
            if cmd == "CYCLE_SPEED":
                i = (i + 1) % len(SPEEDS)
            else:
                i = max(0, min(len(SPEEDS) - 1, i + step))
            self.speed = SPEEDS[i]
            print(f"[Sim] Speed {speed_label(self.speed)}")
        elif cmd == "CLEAR" and self.physics is not None:
            self.physics.clear_parcels()
        elif cmd == "RESET" and self.physics is not None:
            self.physics.reset()
            self.play_real_time = 0.0
            self._play_clock_last = time.perf_counter()
        return cmd

    def _init_fonts(self):
        pygame.font.init()
        regular, bold = hmi.bundled_font_path(), hmi.bundled_font_path(bold=True)
        self._font_paths = (regular, bold)
        self._sized_fonts = {}
        self.font = pygame.font.Font(regular, hmi.HUD_PX)          # kept for older callers
        self.small_font = pygame.font.Font(regular, hmi.LABEL_PX)
        self.label_font = pygame.font.Font(regular, hmi.LABEL_PX)
        self.hud_font = pygame.font.Font(regular, hmi.HUD_PX)
        self.hud_bold = pygame.font.Font(bold, 12)
        self.hud_small = pygame.font.Font(regular, 11)

    def _label_font_px(self, size: int):
        font = self._sized_fonts.get(size)
        if font is None:
            font = self._sized_fonts[size] = pygame.font.Font(self._font_paths[1], size)
        return font

    def _world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        sx = int(x * self.scale * self.zoom) + self.camera_x
        # Standard CAD: Y is positive UP. Screen Y is positive DOWN.
        sy = int(-y * self.scale * self.zoom) + self.camera_y
        return (int(sx), int(sy))

    def _screen_to_world(self, sx: int, sy: int) -> Tuple[float, float]:
        x = (sx - self.camera_x) / (self.scale * self.zoom)
        y = -(sy - self.camera_y) / (self.scale * self.zoom)
        return (x, y)

    def _scale_val(self, val: float) -> int:
        return int(val * self.scale * self.zoom)

    def _hex_to_rgb(self, hex_color: str) -> Tuple[int, int, int]:
        hex_color = hex_color.lstrip('#')
        if len(hex_color) == 6:
            return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        return (200, 200, 200)

    def _cached_rotated_fill(self, w: int, h: int, rgba: tuple, rotation: float):
        """A flat-color SRCALPHA surface, rotated, cached by its exact
        (size, color, rotation) - covers the belt/sensor/source/sink shapes
        that are otherwise identical frame to frame while static."""
        key = (w, h, rgba, rotation)
        cached = self._surface_cache.get(key)
        if cached is not None:
            return cached
        surf = pygame.Surface((w, h), pygame.SRCALPHA)
        surf.fill(rgba)
        rotated = pygame.transform.rotate(surf, rotation)
        self._surface_cache[key] = rotated
        return rotated

    def _cached_text(self, font, text: str, color: tuple):
        """Rendered text, cached by (font, text, color) - covers labels
        that repeat identically frame after frame (button/LED labels, the
        static Controls help line) instead of re-rasterizing them every
        frame regardless of whether the text actually changed."""
        key = (font, text, color)
        cached = self._text_cache.get(key)
        if cached is not None:
            return cached
        surf = font.render(text, True, color)
        self._text_cache[key] = surf
        return surf

    def _zoom_at(self, factor: float, mouse_pos: Tuple[int, int]):
        old_zoom = self.zoom
        self.zoom *= factor
        # Adjust camera to zoom into mouse pos
        mx, my = mouse_pos
        self.camera_x = mx - (mx - self.camera_x) * (self.zoom / old_zoom)
        self.camera_y = my - (my - self.camera_y) * (self.zoom / old_zoom)

    def process_events(self) -> str:
        """Returns string commands like 'QUIT', 'TOGGLE_PLAY', 'RESET', 'CLEAR' or None."""
        if self.headless:
            return None
        
        cmd = None
        time_delta = self.clock.get_time() / 1000.0
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return 'QUIT'
            elif event.type == pygame.VIDEORESIZE:
                self.width, self.height = event.size
                self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
                self.manager.set_window_resolution((self.width, self.height))
                self.editor_ui.handle_resize(self.width, self.height)
                
            self.manager.process_events(event)
            ui_cmd = self.editor_ui.handle_event(event)
            if ui_cmd in ("TOGGLE_PLAY", "CLEAR", "RESET", "TOGGLE_THEME", "TOGGLE_DRAWING", "CYCLE_SPEED"):
                cmd = self.run_command(ui_cmd)
                continue
            if ui_cmd:
                cmd = ui_cmd
                if isinstance(ui_cmd, tuple) and ui_cmd[0] == "SELECT_ITEM":
                    self.selected_item = ui_cmd[1]
                    continue
                
            # A button's keyboard key (ControlButton.key) presses it while held.
            if event.type in (pygame.KEYDOWN, pygame.KEYUP) and self._button_for_key(event) is not None:
                if self.playing:
                    button = self._button_for_key(event)
                    self._set_button(button, event.type == pygame.KEYDOWN)
                    if self.selected_item == button:
                        self.editor_ui.refresh_button_state()
                continue

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    cmd = self.run_command('TOGGLE_PLAY')
                elif event.key == pygame.K_r:
                    cmd = self.run_command('RESET')
                elif event.key == pygame.K_v:
                    self.show_values = not self.show_values
                elif event.key == pygame.K_t:
                    cmd = self.run_command('TOGGLE_THEME')
                elif event.key == pygame.K_d:
                    cmd = self.run_command('TOGGLE_DRAWING')
                elif event.key == pygame.K_RIGHTBRACKET:
                    cmd = self.run_command('SPEED_UP')
                elif event.key == pygame.K_LEFTBRACKET:
                    cmd = self.run_command('SPEED_DOWN')
                elif event.key == pygame.K_c:
                    cmd = self.run_command('CLEAR')
                    
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: # Left click
                    # Check if clicked on UI panels
                    if self.editor_ui.is_ui_blocking_point(event.pos):
                        continue
                        
                    # Hit testing logic for selection
                    wx, wy = self._screen_to_world(event.pos[0], event.pos[1])
                    clicked_item = self._hit_test(wx, wy)
                    self.selected_item = clicked_item
                    self.editor_ui.show_props_for(clicked_item)

                    if isinstance(clicked_item, ControlButton):
                        if  self.playing:
                            self._set_button(clicked_item, True)  # momentary/latching, NO/NC contact
                            self.pressed_button = clicked_item
                            self.editor_ui.show_props_for(clicked_item)
                            self.editor_ui.refresh_button_state()
                            continue
                        else:
                            print(f"[Sim] '{clicked_item.label or clicked_item.id}' is selected, not pressed - "
                                  f"press Start (or Space) to run the simulation so button presses register.")

                    self.is_dragging = True
                    self.drag_start = event.pos
                    self.camera_start = (self.camera_x, self.camera_y)
                elif event.button == 4: # Scroll up
                    self._zoom_at(1.1, event.pos)
                elif event.button == 5: # Scroll down
                    self._zoom_at(1 / 1.1, event.pos)
                    
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    if self.pressed_button:
                        self._set_button(self.pressed_button, False)
                        if self.selected_item == self.pressed_button:
                            self.editor_ui.show_props_for(self.pressed_button)
                            self.editor_ui.refresh_button_state()
                        self.pressed_button = None
                    self.is_dragging = False
                    
            elif event.type == pygame.MOUSEMOTION:
                if self.is_dragging:
                    # Drag pans the view; components are moved in the designer.
                    dx = event.pos[0] - self.drag_start[0]
                    dy = event.pos[1] - self.drag_start[1]
                    self.camera_x = self.camera_start[0] + dx
                    self.camera_y = self.camera_start[1] + dy

        self.manager.update(time_delta)
        
        if cmd == "OPEN_DESIGNER":
            self.open_designer()
        elif cmd == "OPEN_MANAGER":
            self.editor_ui.open_manager_panel()
        return cmd

    def _set_button(self, button, pressed: bool):
        """Press / release an operator button: here (the lamp-and-contact
        state shown at once) and, when the simulation runs in its own
        process, there too (sim_process.PhysicsProxy.set_button)."""
        (devices.press if pressed else devices.release)(button)
        remote = getattr(self.physics, "set_button", None)
        if remote is not None:
            remote(button.id, pressed)

    def _button_for_key(self, event):
        """The button whose `key` is this key event's character, if any."""
        name = (getattr(event, "unicode", "") or pygame.key.name(event.key) or "").lower()
        if not name:
            return None
        for button in self.config.buttons:
            if button.key and button.key.lower() == name:
                return button
        return None

    def _hit_test(self, wx: float, wy: float) -> Optional[object]:
        """Check if a world coordinate intersects a component."""
        # Check sensors
        for sensor in self.config.sensors:
            verts = get_rect_vertices(sensor.x, sensor.y, sensor.width, sensor.height, sensor.rotation)
            if point_in_polygon(wx, wy, verts):
                return sensor

        for button in self.config.buttons:
            verts = get_rect_vertices(button.x, button.y, button.width, button.height, button.rotation)
            if point_in_polygon(wx, wy, verts):
                return button

        for led in self.config.leds:
            if math.hypot(wx - led.x, wy - led.y) <= led.radius:
                return led
                
        # Check sources
        for source in self.config.sources:
            # Sources are upright
            w = source.maxWidth if hasattr(source, 'maxWidth') else 0.5
            h = source.maxHeight if hasattr(source, 'maxHeight') else 0.5
            verts = get_rect_vertices(source.x, source.y, w, h, 0)
            if point_in_polygon(wx, wy, verts):
                return source

        # Check sinks
        for sink in self.config.sinks:
            verts = get_rect_vertices(sink.x, sink.y, sink.width, sink.height, 0)
            if point_in_polygon(wx, wy, verts):
                return sink
                
        # Check belts
        for belt in self.config.belts:
            if belt.type == "linear":
                if point_in_polygon(wx, wy, get_belt_polygon(belt)):
                    return belt
            elif belt.type == "curved":
                if point_in_arc(wx, wy, belt.x, belt.y, belt.radius, belt.beltWidth, belt.startAngle, belt.endAngle):
                    return belt
            elif belt.type == "path":
                if belt_contains(belt, wx, wy):
                    return belt
        return None

    # -- labels -----------------------------------------------------------------
    #
    # Labels are queued while the geometry is drawn and placed afterwards,
    # highest priority first; a label that would overlap one already placed
    # is skipped (the selected item's always shows). So zoomed out, the
    # canvas shows only what fits, and labels never pile up on each other.

    PRIORITY_SELECTED, PRIORITY_ALARM, PRIORITY_DEVICE, PRIORITY_BELT = 0, 1, 2, 3

    def _queue_label(self, text: str, pos, priority: int, color=None, align: str = "center", force: bool = False):
        if text:
            self._labels.append((priority, len(self._labels), text, pos, color or hmi.TEXT, align, force))

    def _flush_labels(self):
        """Draw the queued labels; returns the texts that were placed."""
        placed, texts = [], []
        for _prio, _i, text, (x, y), color, align, force in sorted(self._labels):
            surf = self._cached_text(self.label_font, text, color)
            w, h = surf.get_width() + 8, surf.get_height() + 2
            if align == "left":
                rect = pygame.Rect(int(x), int(y - h / 2), w, h)
            elif align == "below":
                rect = pygame.Rect(int(x - w / 2), int(y), w, h)
            else:
                rect = pygame.Rect(int(x - w / 2), int(y - h / 2), w, h)
            # Touching is fine (stacked buttons / lamps), overlapping is not.
            if not force and (rect.inflate(-2, -4).collidelist(placed) != -1
                              or not self._canvas_rect().contains(rect)):
                continue
            placed.append(rect)
            backdrop = self._surface_cache.get(("tag", w, h))
            if backdrop is None:
                backdrop = pygame.Surface((w, h), pygame.SRCALPHA)
                pygame.draw.rect(backdrop, hmi.TAG_BG, backdrop.get_rect(), border_radius=3)
                self._surface_cache[("tag", w, h)] = backdrop
            self.screen.blit(backdrop, rect)
            self.screen.blit(surf, (rect.x + 4, rect.y + 1))
            texts.append(text)
        self._labels = []
        return texts

    def _canvas_rect(self) -> pygame.Rect:
        return pygame.Rect(0, CANVAS_TOP, self.width, self.height - CANVAS_TOP - BOTTOM_BAR)

    def _polygon_outline(self, points, color, width=1):
        if len(points) >= 3:
            pygame.draw.polygon(self.screen, color, points, width)

    def _belt_outline_points(self, belt: Belt):
        try:
            shape = belt_footprint(belt)
        except Exception:  # noqa: BLE001 - drawing only
            return []
        if shape.is_empty or shape.geom_type != "Polygon":
            return []
        return [self._world_to_screen(x, y) for x, y in shape.exterior.coords]

    # -- equipment ----------------------------------------------------------------

    def _draw_fault_outline(self, belt: Belt):
        """A tripped drive (drive.py: overload or injected fault): red outline + tag."""
        points = self._belt_outline_points(belt)
        if points:
            self._polygon_outline(points, hmi.ALARM, 3)
            self._queue_label(f"{belt.id} FAULT", self._world_to_screen(belt.x, belt.y), self.PRIORITY_ALARM,
                              hmi.ALARM, force=True)

    def draw_belt(self, belt: Belt):
        color = self._hex_to_rgb(belt.color)
        if belt.type == "path":
            # Drawn belt: the offset footprint (cad/path.py), a loop's inner
            # hole cut back out.
            shape = belt_footprint(belt)
            if shape.is_empty:
                return
            pygame.draw.polygon(self.screen, color, [self._world_to_screen(x, y) for x, y in shape.exterior.coords])
            for hole in shape.interiors:
                pygame.draw.polygon(self.screen, hmi.BG, [self._world_to_screen(x, y) for x, y in hole.coords])
        elif belt.type == "linear" and getattr(belt, 'shape', 'rectangle') == 'quadrilateral':
            pygame.draw.polygon(self.screen, color, [self._world_to_screen(wx, wy) for wx, wy in get_belt_polygon(belt)])
        elif belt.type == "linear":
            w = self._scale_val(belt.length)
            h = self._scale_val(belt.beltWidth)
            if w <= 0 or h <= 0:
                return
            # Rotate surface (pygame rotation is counter-clockwise, matching World CCW)
            rotated_surf = self._cached_rotated_fill(w, h, (*color, 255), belt.rotation)
            self.screen.blit(rotated_surf, rotated_surf.get_rect(center=self._world_to_screen(belt.x, belt.y)))
        elif belt.type == "curved":
            if belt.radius <= 0 or belt.beltWidth <= 0:
                return
            angle_diff = belt.endAngle - belt.startAngle
            if angle_diff <= 0:
                angle_diff += 360
            segments = max(10, int(belt.radius * math.radians(angle_diff) / 5))
            inner_r, outer_r = belt.radius - belt.beltWidth / 2.0, belt.radius + belt.beltWidth / 2.0
            s_rad = math.radians(belt.startAngle)
            e_rad = math.radians(belt.startAngle + angle_diff)
            points = []
            for r, order in ((outer_r, range(segments + 1)), (inner_r, range(segments, -1, -1))):
                for i in order:
                    a = s_rad + (e_rad - s_rad) * (i / segments)
                    points.append(self._world_to_screen(belt.x + r * math.cos(a), belt.y + r * math.sin(a)))
            if len(points) >= 3:
                pygame.draw.polygon(self.screen, color, points)
        outline = self._belt_outline_points(belt)
        if outline and self.zoom * self.scale >= 12:
            self._polygon_outline(outline, hmi.shade(color, 0.55), 1)
        self._draw_flow(belt)
        # Its id, only where the belt is long enough on screen to carry it.
        if self._scale_val(self._belt_run_length(belt)) >= self.label_font.size(belt.id)[0] + 40:
            self._queue_label(belt.id, self._world_to_screen(belt.x, belt.y), self.PRIORITY_BELT, hmi.TEXT_DIM)

    @staticmethod
    def _belt_run_length(belt: Belt) -> float:
        """Length along the flow (m)."""
        if belt.type == "curved":
            sweep = (belt.endAngle - belt.startAngle) % 360.0 or 360.0
            return belt.radius * math.radians(sweep)
        if belt.type == "path" and belt.path:
            pts, bulges = belt_world_path(belt)
            return path_length(path_segments(pts, bulges, belt.closed))
        if belt.vertexs:
            xs = [v["x"] for v in belt.vertexs]
            ys = [v["y"] for v in belt.vertexs]
            return max(max(xs) - min(xs), max(ys) - min(ys))
        return belt.length

    def _draw_flow(self, belt: Belt):
        """Chevrons pointing the way parcels go - only while the belt moves,
        so a glance tells running from stopped."""
        speed = belt.actualSpeed if belt.drive is not None else devices.commanded_speed(belt)
        if abs(speed) < 0.01 or self._scale_val(belt.beltWidth) < 8:
            return
        size = max(3, min(9, self._scale_val(belt.beltWidth) // 4))
        for x, y, heading in flow_arrows(belt):
            if speed < 0:
                heading += 180.0  # flow_arrows shows the forward direction; the motor runs backwards
            cx, cy = self._world_to_screen(x, y)
            rad = math.radians(heading)
            ux, uy = math.cos(rad), -math.sin(rad)  # screen y points down
            for k in (-1, 0, 1):
                bx, by = cx + ux * k * size * 1.6, cy + uy * k * size * 1.6
                tip = (bx + ux * size, by + uy * size)
                left = (bx - uy * size, by + ux * size)
                right = (bx + uy * size, by - ux * size)
                pygame.draw.lines(self.screen, hmi.FLOW, False, [left, tip, right], 2)

    def draw_sensor(self, sensor: Sensor, physics_engine=None):
        rad = math.radians(sensor.rotation)
        # The sensor's long axis (its local y): where the beam(s) span.
        ax, ay = -math.sin(rad), math.cos(rad)
        state = sensor.sensor_states if isinstance(sensor.sensor_states, str) else ""
        blocked_any = sensor.blockedBeams > 0 if hasattr(sensor, "blockedBeams") else "1" in state
        cx, cy = sensor.x, sensor.y
        half = sensor.height / 2.0
        p0 = self._world_to_screen(cx - ax * half, cy - ay * half)
        p1 = self._world_to_screen(cx + ax * half, cy + ay * half)
        if sensor.sensorType == "laser_banner":
            count = int(sensor.laserCount or 16)
            dist = float(sensor.beamDistance or 0.05)
            span = dist * (count - 1)
            # emitter and receiver columns
            bx, by = math.cos(rad) * sensor.width / 2.0, math.sin(rad) * sensor.width / 2.0
            for side in (-1, 1):
                a = self._world_to_screen(cx + side * bx - ax * span / 2, cy + side * by - ay * span / 2)
                b = self._world_to_screen(cx + side * bx + ax * span / 2, cy + side * by + ay * span / 2)
                pygame.draw.line(self.screen, hmi.TEXT_FAINT, a, b, 3)
            r = 1 if self.zoom * self.scale * dist < 6 else 2
            for i in range(count):
                t = -span / 2 + i * dist
                on = i < len(state) and state[i] == ("0" if sensor.invert else "1")
                pygame.draw.circle(self.screen, hmi.ALARM if on else hmi.RUNNING,
                                   self._world_to_screen(cx + t * ax, cy + t * ay), r)
        else:
            blocked = (state == ("0" if sensor.invert else "1")) if state else False
            pygame.draw.line(self.screen, hmi.ALARM if blocked else hmi.RUNNING, p0, p1, 2)
            for end in (p0, p1):
                pygame.draw.rect(self.screen, hmi.TEXT_FAINT, pygame.Rect(end[0] - 3, end[1] - 3, 6, 6))
            blocked_any = blocked
        label = sensor.label if sensor.label and sensor.label.upper() != "SENSOR" else sensor.id
        lx, ly = max(p0, p1, key=lambda p: p[1])
        self._queue_label(label, (lx, ly + 6), self.PRIORITY_DEVICE,
                          hmi.ALARM if blocked_any else hmi.TEXT_DIM, align="below")

    def _fit_font(self, text: str, w: int, h: int):
        for size in (13, 12, 11):
            font = self._label_font_px(size)
            if font.size(text)[0] <= w - 6 and font.get_height() <= h:
                return font
        return None

    def draw_button(self, button: ControlButton):
        color = self._hex_to_rgb(button.color)
        pressed = bool(getattr(button, "pressed", False)) or (
            not getattr(button, "normallyClosed", False) and bool(button.sensor_states))
        w = self._scale_val(button.width)
        h = self._scale_val(button.height)
        if w <= 0 or h <= 0:
            return
        face = hmi.shade(color, 0.55) if pressed else hmi.shade(color, 0.8)
        text = button.label or button.id
        font = self._fit_font(text, w, h)
        key = ("button", w, h, face, pressed, text, button.rotation, font)
        rotated_surf = self._surface_cache.get(key)
        if rotated_surf is None:
            surf = pygame.Surface((max(w, 1), max(h, 1)), pygame.SRCALPHA)
            radius = max(1, min(w, h) // 5)
            pygame.draw.rect(surf, (*face, 255), surf.get_rect(), border_radius=radius)
            pygame.draw.rect(surf, (*(hmi.TEXT if pressed else hmi.shade(color, 0.35)), 255), surf.get_rect(),
                             2 if pressed else 1, border_radius=radius)
            if font is not None:
                label = font.render(text, True, (245, 247, 250))
                surf.blit(label, label.get_rect(center=(w // 2, h // 2)))
            rotated_surf = pygame.transform.rotate(surf, button.rotation)
            self._surface_cache[key] = rotated_surf
        center = self._world_to_screen(button.x, button.y)
        self.screen.blit(rotated_surf, rotated_surf.get_rect(center=center))
        if font is None:   # too small for its text: a tag beside it, if there is room
            self._queue_label(text, (center[0] + w // 2 + 4, center[1]), self.PRIORITY_DEVICE, hmi.TEXT_DIM,
                              align="left")

    def draw_led(self, led: LedIndicator):
        cx, cy = self._world_to_screen(led.x, led.y)
        radius = max(3, self._scale_val(led.radius))
        base = self._hex_to_rgb(led.color)
        if led.isOn:
            glow = self._surface_cache.get(("glow", radius, base))
            if glow is None:
                glow = pygame.Surface((radius * 4, radius * 4), pygame.SRCALPHA)
                pygame.draw.circle(glow, (*base, 60), (radius * 2, radius * 2), radius * 2)
                self._surface_cache[("glow", radius, base)] = glow
            self.screen.blit(glow, (cx - radius * 2, cy - radius * 2))
        pygame.draw.circle(self.screen, (52, 57, 64), (cx, cy), radius + max(1, radius // 4))   # bezel
        pygame.draw.circle(self.screen, base if led.isOn else hmi.shade(base, 0.22), (cx, cy), radius)
        if led.isOn and radius >= 5:
            pygame.draw.circle(self.screen, hmi.shade(base, 1.5), (cx - radius // 3, cy - radius // 3),
                               max(1, radius // 4))
        if led.label:
            self._queue_label(led.label, (cx + radius + 6, cy), self.PRIORITY_DEVICE,
                              hmi.TEXT if led.isOn else hmi.TEXT_DIM, align="left")

    def draw_source(self, source):
        w = self._scale_val(source.maxWidth if hasattr(source, 'maxWidth') else 0.5)
        h = self._scale_val(source.maxHeight if hasattr(source, 'maxHeight') else 0.5)
        if w <= 0 or h <= 0:
            return
        rotated_surf = self._cached_rotated_fill(w, h, (*hmi.SOURCE, 70), source.rotation)
        rect = rotated_surf.get_rect(center=self._world_to_screen(source.x, source.y))
        self.screen.blit(rotated_surf, rect)
        corners = get_rect_vertices(source.x, source.y, getattr(source, "maxWidth", 0.5),
                                    getattr(source, "maxHeight", 0.5), source.rotation)
        self._polygon_outline([self._world_to_screen(x, y) for x, y in corners], hmi.SOURCE, 1)
        self._queue_label(getattr(source, "label", "") or source.id, (rect.centerx, rect.bottom + 2),
                          self.PRIORITY_DEVICE, hmi.TEXT_DIM, align="below")

    def draw_sink(self, sink):
        w = self._scale_val(sink.width)
        h = self._scale_val(sink.height)
        if w <= 0 or h <= 0:
            return
        surf = self._cached_rotated_fill(w, h, (*hmi.SINK, 70), 0.0)
        rect = surf.get_rect(center=self._world_to_screen(sink.x, sink.y))
        self.screen.blit(surf, rect)
        pygame.draw.rect(self.screen, hmi.SINK, rect, 1)
        self._queue_label(getattr(sink, "label", "") or sink.id, (rect.centerx, rect.bottom + 2),
                          self.PRIORITY_DEVICE, hmi.TEXT_DIM, align="below")

    def draw_grid(self):
        grid_spacing = getattr(self.config, 'gridSize', 0.5)
        if grid_spacing <= 0:
            grid_spacing = 0.5
        # Lines at least 16 px apart; every 5th line a little brighter.
        pixel_spacing = grid_spacing * self.scale * self.zoom
        if pixel_spacing < 16.0:
            grid_spacing *= math.ceil(16.0 / pixel_spacing)

        w_min_x, w_max_y = self._screen_to_world(0, 0)
        w_max_x, w_min_y = self._screen_to_world(self.width, self.height)
        i0 = math.floor(w_min_x / grid_spacing)
        i1 = math.ceil(w_max_x / grid_spacing)
        for i in range(i0, i1 + 1):
            x = i * grid_spacing
            sx, _ = self._world_to_screen(x, 0)
            if abs(x) < 1e-9:
                pygame.draw.line(self.screen, hmi.AXIS_Y, (sx, 0), (sx, self.height), 1)
            else:
                pygame.draw.line(self.screen, hmi.GRID_MAJOR if i % 5 == 0 else hmi.GRID, (sx, 0), (sx, self.height))
        j0 = math.floor(w_min_y / grid_spacing)
        j1 = math.ceil(w_max_y / grid_spacing)
        for j in range(j0, j1 + 1):
            y = j * grid_spacing
            _, sy = self._world_to_screen(0, y)
            if abs(y) < 1e-9:
                pygame.draw.line(self.screen, hmi.AXIS_X, (0, sy), (self.width, sy), 1)
            else:
                pygame.draw.line(self.screen, hmi.GRID_MAJOR if j % 5 == 0 else hmi.GRID, (0, sy), (self.width, sy))

    def draw_origin_and_direction(self, item):
        if self.headless:
            return
        x = getattr(item, 'x', None)
        y = getattr(item, 'y', None)
        if x is None or y is None:
            return
        
        sx, sy = self._world_to_screen(x, y)
        
        # Calculate angle
        angle = 0.0
        if hasattr(item, 'rotation'):
            angle = item.rotation
        
        # Draw origin crosshair and dot
        color_origin = (0, 180, 255) # Cyanish blue
        color_dir = (255, 69, 0) # Red-orange
        
        pygame.draw.circle(self.screen, color_origin, (sx, sy), 3)
        pygame.draw.circle(self.screen, color_origin, (sx, sy), 6, 1)
        
        # Draw direction line + arrow if it has rotation
        if hasattr(item, 'rotation'):
            rad = math.radians(angle)
            arrow_len = 25
            sdx = arrow_len * math.cos(rad)
            sdy = -arrow_len * math.sin(rad)
            
            ex = sx + sdx
            ey = sy + sdy
            
            pygame.draw.line(self.screen, color_dir, (sx, sy), (ex, ey), 2)
            
            # Draw arrowhead
            arrow_size = 5
            back_rad1 = rad + math.radians(150)
            back_rad2 = rad - math.radians(150)
            ax1 = ex + arrow_size * math.cos(back_rad1)
            ay1 = ey - arrow_size * math.sin(back_rad1)
            ax2 = ex + arrow_size * math.cos(back_rad2)
            ay2 = ey - arrow_size * math.sin(back_rad2)
            pygame.draw.polygon(self.screen, color_dir, [(ex, ey), (ax1, ay1), (ax2, ay2)])
            
        # Draw text label (x, y) alongside the component
        angle_text = ""
        if hasattr(item, 'rotation'):
            angle_text += f"{int(item.rotation)}°"
        if hasattr(item, 'directionAngle') and item.directionAngle != 0:
            angle_text += f" (dir: {int(item.directionAngle)}°)"
            
        lbl_str = f"({x:.1f}, {y:.1f})"
        if angle_text:
            lbl_str += f" {angle_text}"
            
        if hasattr(self, 'small_font'):
            text_surf = self.small_font.render(lbl_str, True, (180, 180, 180))
            self.screen.blit(text_surf, (sx + 10, sy + 10))

    def _draw_selection_overlay(self, item):
        from config_parser import Source, Sink
        GREEN = (0, 255, 0)

        def _screen_rect_verts(cx, cy, hw, hh, rotation_deg):
            """Compute 4 screen-space corners matching pygame.transform.rotate(surf, rotation).
            Pygame CCW rotation by `a = rotation`:
            x' = x*cos(a) + y*sin(a)
            y' = -x*sin(a) + y*cos(a)
            """
            rot = math.radians(rotation_deg)
            cos_r, sin_r = math.cos(rot), math.sin(rot)
            corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
            return [(int(cx + lx*cos_r + ly*sin_r), int(cy - lx*sin_r + ly*cos_r))
                    for lx, ly in corners]

        if hasattr(item, 'type') and getattr(item, 'type') == 'linear':
            if getattr(item, 'shape', 'rectangle') == 'quadrilateral':
                if not getattr(item, 'vertexs', None):
                    item.vertexs = get_default_quad_vertices(item.length, item.beltWidth)
                screen_verts = [self._world_to_screen(wx, wy) for wx, wy in get_belt_polygon(item)]
                pygame.draw.polygon(self.screen, GREEN, screen_verts, 2)
                for sv in screen_verts:
                    pygame.draw.circle(self.screen, GREEN, sv, 6)
            else:
                cx, cy = self._world_to_screen(item.x, item.y)
                hw = self._scale_val(item.length) / 2
                hh = self._scale_val(item.beltWidth) / 2
                screen_verts = _screen_rect_verts(cx, cy, hw, hh, item.rotation)
                pygame.draw.polygon(self.screen, GREEN, screen_verts, 2)
                for sv in screen_verts:
                    pygame.draw.circle(self.screen, GREEN, sv, 5)

        elif hasattr(item, 'sensorType'):
            cx, cy = self._world_to_screen(item.x, item.y)
            hw = self._scale_val(item.width) / 2
            hh = self._scale_val(item.height) / 2
            screen_verts = _screen_rect_verts(cx, cy, hw, hh, item.rotation)
            pygame.draw.polygon(self.screen, GREEN, screen_verts, 2)
            for sv in screen_verts:
                pygame.draw.circle(self.screen, GREEN, sv, 5)

        elif isinstance(item, ControlButton):
            cx, cy = self._world_to_screen(item.x, item.y)
            hw = self._scale_val(item.width) / 2
            hh = self._scale_val(item.height) / 2
            screen_verts = _screen_rect_verts(cx, cy, hw, hh, item.rotation)
            pygame.draw.polygon(self.screen, GREEN, screen_verts, 2)
            for sv in screen_verts:
                pygame.draw.circle(self.screen, GREEN, sv, 5)

        elif isinstance(item, LedIndicator):
            pygame.draw.circle(self.screen, GREEN, self._world_to_screen(item.x, item.y), max(5, self._scale_val(item.radius)), 2)

        elif 'Source' in str(type(item)):
            w = item.maxWidth if hasattr(item, 'maxWidth') else 0.5
            h = item.maxHeight if hasattr(item, 'maxHeight') else 0.5
            cx, cy = self._world_to_screen(item.x, item.y)
            hw, hh = self._scale_val(w)/2, self._scale_val(h)/2
            # screen_verts = _screen_rect_verts(cx, cy, hw, hh, 0)
            screen_verts = _screen_rect_verts(cx, cy, hw, hh, item.rotation)
            pygame.draw.polygon(self.screen, GREEN, screen_verts, 2)

        elif 'Sink' in str(type(item)):
            cx, cy = self._world_to_screen(item.x, item.y)
            hw, hh = self._scale_val(item.width)/2, self._scale_val(item.height)/2
            screen_verts = _screen_rect_verts(cx, cy, hw, hh, 0)
            pygame.draw.polygon(self.screen, GREEN, screen_verts, 2)

        elif hasattr(item, 'type') and getattr(item, 'type') == 'curved':
            # Highlight center and arc endpoints
            c_sx, c_sy = self._world_to_screen(item.x, item.y)
            pygame.draw.circle(self.screen, GREEN, (c_sx, c_sy), 5)
            r = item.radius
            for ang in [item.startAngle, item.endAngle]:
                rad = math.radians(ang)
                px = item.x + r * math.cos(rad)
                py = item.y + r * math.sin(rad)
                pygame.draw.circle(self.screen, GREEN, self._world_to_screen(px, py), 5)

    def draw_parcels(self, physics_engine):
        if not physics_engine: return
        # Plain data (physics.ParcelView), from this process's engine or a
        # simulation process's snapshot (sim_process.py).
        views = physics_engine.parcel_views()
        self._parcel_count = len(views)
        outline = 1 if self.zoom * self.scale < 60 else 2
        for parcel in views:
            color = self._hex_to_rgb(parcel.color)
            points = [self._world_to_screen(x, y) for x, y in parcel.points]
            if len(points) >= 3:
                pygame.draw.polygon(self.screen, color, points)
                pygame.draw.polygon(self.screen, hmi.PARCEL_EDGE, points, outline)

            # [V] shows actual vs. belt speed per parcel (red when it lags
            # more than 10 %) - a check, not something to look at all day.
            if self.show_values:
                actual, target = parcel.actual_speed, parcel.target_speed
                if target is not None:
                    text = f"{actual:.2f}/{target:.2f}"
                    lagging = actual < target * 0.9
                else:
                    text, lagging = f"{actual:.2f}", False
                sx, sy = self._world_to_screen(parcel.x, parcel.y)
                self._queue_label(text, (sx, sy), self.PRIORITY_DEVICE, hmi.ALARM if lagging else hmi.TEXT)

    def _recent_ratio(self, sim_time: float) -> float:
        """Simulated seconds per wall second over the last ~2 s (0 while
        paused) - follows a speed change at once, unlike a since-start mean."""
        now = time.perf_counter()
        samples = self._ratio_samples
        if samples and sim_time < samples[-1][1]:
            samples.clear()                     # reset: time went back to 0
        samples.append((now, sim_time))
        while len(samples) > 2 and now - samples[0][0] > 2.0:
            samples.popleft()
        (t0, s0), (t1, s1) = samples[0], samples[-1]
        return (s1 - s0) / (t1 - t0) if t1 - t0 > 0.2 else 0.0

    def track_play_time(self) -> float:
        """Accumulate wall time spent playing; call once per loop iteration.
        Returns the total."""
        now = time.perf_counter()
        if self.playing:
            self.play_real_time += now - self._play_clock_last
        self._play_clock_last = now
        return self.play_real_time

    def render(self, physics_engine=None):
        if self.headless:
            return

        if self.zoom != self._surface_cache_zoom:
            # Every cached surface's (w, h) was derived from the OLD zoom -
            # drop them all at once instead of keeping one dead generation
            # of shapes per zoom level.
            self._surface_cache.clear()
            self._surface_cache_zoom = self.zoom

        # The scene reads parcels and component states the simulation thread
        # writes: draw it under the physics lock (a few ms), everything else
        # (panels, status bar, flip, frame pacing) without it.
        with self.sim_lock:
            self._draw_scene(physics_engine)
        self.editor_ui.sync_sim_buttons(self.playing, hmi.THEME, self.show_drawing, speed_label(self.speed))
        self.manager.draw_ui(self.screen)
        self._draw_status_bar(physics_engine)
        self._draw_hint_bar()

        pygame.display.flip()
        self.clock.tick(self.fps)

    def _draw_scene(self, physics_engine=None):
        self.screen.fill(hmi.BG)
        self.draw_grid()
        self._draw_background_drawing()
        self._labels = []

        for sink in self.config.sinks:
            self.draw_sink(sink)
        for belt in self.config.belts:
            self.draw_belt(belt)
            if belt.fault:
                self._draw_fault_outline(belt)
        for source in self.config.sources:
            self.draw_source(source)
        for sensor in self.config.sensors:
            self.draw_sensor(sensor, physics_engine)
        for button in self.config.buttons:
            self.draw_button(button)
        for led in self.config.leds:
            self.draw_led(led)
        if physics_engine:
            self.draw_parcels(physics_engine)

        # The selected item: outline, origin/heading and its coordinates.
        if self.selected_item is not None:
            self._draw_selection_overlay(self.selected_item)
            self.draw_origin_and_direction(self.selected_item)
        self._flush_labels()

        if physics_engine:
            self.editor_ui.update_sensor_state(physics_engine)

    def load_drawing(self) -> bool:
        """(Re)read the layout's DXF background; the View > drawing button is
        only enabled when there is one."""
        self._drawing_loaded = True
        ok = self.drawing.load(self.config.background, self.config_path)
        if self.drawing.error:
            print(f"[Sim] {self.drawing.error}")
        elif ok:
            print(f"[Sim] Drawing: {len(self.drawing.lines):,} lines from {self.drawing.loaded_from}")
        if not self.headless:
            (self.editor_ui.drawing_btn.enable if self.drawing.has_drawing else self.editor_ui.drawing_btn.disable)()
        return ok

    def _draw_background_drawing(self):
        if not self._drawing_loaded:
            self.load_drawing()
        if self.show_drawing:
            self.drawing.draw(self.screen, self.camera_x, self.camera_y, self.scale * self.zoom)

    def _draw_status_bar(self, physics_engine):
        """One quiet strip on top: run state, time, real-time ratio, flow
        counts, exchange link, zoom, frame rate."""
        bar = pygame.Rect(0, 0, self.width, TOP_BAR)
        pygame.draw.rect(self.screen, hmi.PANEL_DARK, bar)
        pygame.draw.line(self.screen, hmi.PANEL_BORDER, (0, TOP_BAR - 1), (self.width, TOP_BAR - 1))
        x = 10
        title = self._cached_text(self.hud_bold, "CONVEYOR SIMULATOR", hmi.TEXT_DIM)
        self.screen.blit(title, (x, (TOP_BAR - title.get_height()) // 2))
        x += title.get_width() + 18

        state, color = ("RUNNING", hmi.RUNNING) if self.playing else ("PAUSED", hmi.WARNING)
        pill_text = self._cached_text(self.hud_bold, state, (20, 22, 26))
        pill = pygame.Rect(x, 5, pill_text.get_width() + 18, TOP_BAR - 10)
        pygame.draw.rect(self.screen, color, pill, border_radius=pill.height // 2)
        self.screen.blit(pill_text, pill_text.get_rect(center=pill.center))
        x = pill.right + 16

        fields = []
        if physics_engine:
            self.track_play_time()
            ratio = self._recent_ratio(physics_engine.sim_time)
            fields.append(("TIME", f"{physics_engine.sim_time:7.1f} s", hmi.TEXT))
            fields.append(("SPEED", speed_label(self.speed), hmi.TEXT))
            # Amber when the simulation can't keep the speed asked for.
            behind = self.playing and self.speed is not None and ratio < 0.9 * self.speed
            fields.append(("REAL", f"{ratio:4.2f}x", hmi.WARNING if behind else hmi.TEXT))
            fields.append(("PARCELS", f"{getattr(self, '_parcel_count', 0)}", hmi.TEXT))
            delivered = sum(getattr(physics_engine, "delivered", {}).values())
            fields.append(("DELIVERED", f"{delivered}", hmi.TEXT))
            faults = sum(1 for b in self.config.belts if b.fault)
            if faults:
                fields.append(("FAULTS", f"{faults}", hmi.ALARM))
        exchange = getattr(self, "exchange", None)
        if exchange is not None and getattr(exchange, "_enabled", False):
            try:
                stats = exchange.stats()
            except Exception:  # noqa: BLE001 - status display only
                stats = {}
            clients = stats.get("clients")
            if clients is not None:
                fields.append(("PLC", f"{clients} link" + ("" if clients == 1 else "s"),
                               hmi.RUNNING if clients else hmi.WARNING))
            if stats.get("rx_size_errors"):
                fields.append(("LAYOUT", "mismatch", hmi.ALARM))
        for name, value, value_color in fields:
            n = self._cached_text(self.hud_small, name, hmi.TEXT_FAINT)
            v = self.hud_font.render(value, True, value_color)
            self.screen.blit(n, (x, (TOP_BAR - n.get_height()) // 2 + 1))
            x += n.get_width() + 6
            self.screen.blit(v, (x, (TOP_BAR - v.get_height()) // 2))
            x += v.get_width() + 18

        right = f"ZOOM {self.zoom:.2f}x   {self.clock.get_fps():3.0f} FPS"
        r = self.hud_small.render(right, True, hmi.TEXT_FAINT)
        self.screen.blit(r, (self.width - r.get_width() - 12, (TOP_BAR - r.get_height()) // 2 + 1))

    def _draw_hint_bar(self):
        bar = pygame.Rect(0, self.height - BOTTOM_BAR, self.width, BOTTOM_BAR)
        pygame.draw.rect(self.screen, hmi.PANEL_DARK, bar)
        pygame.draw.line(self.screen, hmi.PANEL_BORDER, bar.topleft, bar.topright)
        hint = self._cached_text(
            self.hud_small,
            "SPACE start / pause    [ ] speed    C clear parcels    R reset    V parcel speeds "
            + ("on" if self.show_values else "off")
            + "    D drawing    T theme    wheel zoom    drag pan    click a button to press it", hmi.TEXT_FAINT)
        self.screen.blit(hint, (12, bar.y + (BOTTOM_BAR - hint.get_height()) // 2))

    def quit(self):
        if not self.headless:
            pygame.quit()
