import pygame
import pygame_gui
import math
import time
from typing import Tuple, Optional, List
from config_parser import SimulatorConfig, Belt, Sensor, ControlButton, LedIndicator
from math_utils import get_rect_vertices, point_in_polygon, point_in_arc, get_belt_polygon, get_default_quad_vertices
from id_utils import next_indexed_id
from physics import parcel_speed_info
from surface import belt_contains
import devices
from cad.checks import belt_footprint
from cad.ports import centerline_points
from editor_ui import EditorUI


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
            pygame.display.set_caption("Conveyor Flow Simulator - Python")
            self.font = pygame.font.SysFont(None, 24)
            self.small_font = pygame.font.SysFont(None, 16)

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
        self.dragging_vertex_idx = None
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
            self.manager = pygame_gui.UIManager((self.width, self.height))
            self.editor_ui = EditorUI(self.manager, self.width, self.height, self.config)
            
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
            if ui_cmd:
                cmd = ui_cmd
                if isinstance(ui_cmd, tuple) and ui_cmd[0] == "SELECT_ITEM":
                    self.selected_item = ui_cmd[1]
                    continue
                
            # A button's keyboard key (ControlButton.key) presses it while held.
            if event.type in (pygame.KEYDOWN, pygame.KEYUP) and self._button_for_key(event) is not None:
                if self.playing:
                    button = self._button_for_key(event)
                    (devices.press if event.type == pygame.KEYDOWN else devices.release)(button)
                    if self.selected_item == button:
                        self.editor_ui.refresh_button_state()
                continue

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.playing = not self.playing
                    cmd = 'TOGGLE_PLAY'
                elif event.key == pygame.K_r:
                    cmd = 'RESET'
                elif event.key == pygame.K_c:
                    cmd = 'CLEAR'
                    
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: # Left click
                    # Check if clicked on UI panels
                    if self.editor_ui.is_ui_blocking_point(event.pos):
                        continue
                        
                    # Check if clicked a vertex handle of a selected quadrilateral belt
                    handle_clicked = False
                    if self.selected_item and isinstance(self.selected_item, Belt) and self.selected_item.type == "linear" and getattr(self.selected_item, "shape", "rectangle") == "quadrilateral":
                        belt = self.selected_item
                        if not belt.vertexs:
                            belt.vertexs = get_default_quad_vertices(belt.length, belt.beltWidth)

                        for idx, (wx, wy) in enumerate(get_belt_polygon(belt)):
                            sx, sy = self._world_to_screen(wx, wy)
                            if math.hypot(event.pos[0] - sx, event.pos[1] - sy) < 10.0:
                                self.dragging_vertex_idx = idx
                                self.is_dragging = True
                                handle_clicked = True
                                break

                    if handle_clicked:
                        continue
                        
                    # Hit testing logic for selection
                    wx, wy = self._screen_to_world(event.pos[0], event.pos[1])
                    clicked_item = self._hit_test(wx, wy)
                    self.selected_item = clicked_item
                    self.editor_ui.show_props_for(clicked_item)

                    if isinstance(clicked_item, ControlButton):
                        if  self.playing:
                            devices.press(clicked_item)  # momentary/latching, NO/NC contact
                            self.pressed_button = clicked_item
                            self.editor_ui.show_props_for(clicked_item)
                            self.editor_ui.refresh_button_state()
                            self.editor_ui._refresh_composer_table()
                            continue
                        else:
                            print(f"[Sim] '{clicked_item.label or clicked_item.id}' is selected for editing, not pressed - "
                                  f"press Space (or the Play control) to resume the simulation so button presses register.")

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
                        devices.release(self.pressed_button)
                        if self.selected_item == self.pressed_button:
                            self.editor_ui.show_props_for(self.pressed_button)
                            self.editor_ui.refresh_button_state()
                        self.editor_ui._refresh_composer_table()
                        self.pressed_button = None
                    self.is_dragging = False
                    self.dragging_vertex_idx = None
                    
            elif event.type == pygame.MOUSEMOTION:
                if self.is_dragging and self.dragging_vertex_idx is not None and self.selected_item:
                    belt = self.selected_item
                    mwx, mwy = self._screen_to_world(event.pos[0], event.pos[1])
                    rad = math.radians(belt.rotation)
                    cos_a = math.cos(rad)
                    sin_a = math.sin(rad)
                    dx = mwx - belt.x
                    dy = mwy - belt.y
                    lx = dx * cos_a + dy * sin_a
                    ly = -dx * sin_a + dy * cos_a
                    belt.vertexs[self.dragging_vertex_idx] = {'x': lx, 'y': ly}
                    self.editor_ui.update_props_values(belt)
                elif self.is_dragging and not self.selected_item:
                    # Pan camera if nothing selected
                    dx = event.pos[0] - self.drag_start[0]
                    dy = event.pos[1] - self.drag_start[1]
                    self.camera_x = self.camera_start[0] + dx
                    self.camera_y = self.camera_start[1] + dy
                elif self.is_dragging and self.selected_item:
                    # Move selected item incrementally
                    wx1, wy1 = self._screen_to_world(self.drag_start[0], self.drag_start[1])
                    wx2, wy2 = self._screen_to_world(event.pos[0], event.pos[1])
                    self.selected_item.x += (wx2 - wx1)
                    self.selected_item.y += (wy2 - wy1)
                    self.drag_start = event.pos
                    # Update properties UI
                    self.editor_ui.update_props_values(self.selected_item)
            
            # File Dialog handling
            elif event.type == pygame_gui.UI_FILE_DIALOG_PATH_PICKED:
                try:
                    with open(event.text, "w", encoding="utf-8") as f:
                        f.write(self.config.model_dump_json(indent=2))
                    print(f"Saved config to {event.text}")
                except Exception as e:
                    print(f"Failed to save: {e}")
                    
        self.manager.update(time_delta)
        
        # Handle custom UI commands
        if isinstance(cmd, tuple) and cmd[0] == "PLACE_COMPONENT":
            _, comp_cmd, wx, wy = cmd
            if comp_cmd == "ADD_LINEAR":
                new_belt = Belt(
                    id=next_indexed_id((s.id for s in self.config.belts), "belt"),
                    type="linear",
                    x=wx, y=wy, length=5, beltWidth=1, rotation=0, speed=1.0, directionAngle=0, color="#cccccc"
                )
                self.config.belts.append(new_belt)
                self.selected_item = new_belt
                self.editor_ui.show_props_for(new_belt)
            elif comp_cmd == "ADD_CURVED":
                new_belt = Belt(
                    id=next_indexed_id((s.id for s in self.config.belts), "belt"),
                    type="curved", shape="arc",
                    x=wx, y=wy, radius=2, beltWidth=1, startAngle=0, endAngle=90, rotation=0, speed=1.0, directionAngle=0, color="#cccccc"
                )
                self.config.belts.append(new_belt)
                self.selected_item = new_belt
                self.editor_ui.show_props_for(new_belt)
            elif comp_cmd == "ADD_SENSOR":
                new_sensor = Sensor(
                    id=next_indexed_id((s.id for s in self.config.sensors), "sensor"),
                    x=wx, y=wy, width=0.1, height=1.0, rotation=0, label="SENSOR", color="#ff0000", sensorType="ir"
                )
                self.config.sensors.append(new_sensor)
                self.selected_item = new_sensor
                self.editor_ui.show_props_for(new_sensor)
            elif comp_cmd == "ADD_BUTTON":
                new_button = ControlButton(
                    id=next_indexed_id((s.id for s in self.config.buttons), "button"),
                    x=wx, y=wy, width=0.7, height=0.25, rotation=0, label="START", color="#3b82f6"
                )
                self.config.buttons.append(new_button)
                self.selected_item = new_button
                self.editor_ui.show_props_for(new_button)
            elif comp_cmd == "ADD_LED":
                new_led = LedIndicator(
                    id=next_indexed_id((s.id for s in self.config.leds), "led"),
                    x=wx, y=wy, radius=0.2, label="AUTO", color="#22c55e"
                )
                self.config.leds.append(new_led)
                self.selected_item = new_led
                self.editor_ui.show_props_for(new_led)
            elif comp_cmd == "ADD_SOURCE":
                from config_parser import Source
                new_source = Source(
                    id=next_indexed_id((s.id for s in self.config.sources), "source"),
                    x=wx, y=wy, rotation=0, interval=2.0, minWidth=0.4, maxWidth=0.6, minHeight=0.4, maxHeight=0.6, label="SRC"
                )
                self.config.sources.append(new_source)
                self.selected_item = new_source
                self.editor_ui.show_props_for(new_source)
            elif comp_cmd == "ADD_SINK":
                from config_parser import Sink
                new_sink = Sink(
                    id=next_indexed_id((s.id for s in self.config.sinks), "sink"),
                    x=wx, y=wy, width=1.0, height=1.0, label="SNK"
                )
                self.config.sinks.append(new_sink)
                self.selected_item = new_sink
                self.editor_ui.show_props_for(new_sink)
                
        elif cmd == "SAVE_DIALOG":
            pygame_gui.windows.UIFileDialog(
                rect=pygame.Rect(160, 50, 440, 500),
                manager=self.manager,
                window_title='Save Config',
                initial_file_path='test_config.json',
                allow_existing_files_only=False
            )
        elif cmd == "OPEN_MANAGER":
            self.editor_ui.open_manager_panel()
        return cmd

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

    def _draw_fault_outline(self, belt: Belt):
        """A tripped drive (drive.py: overload or injected fault): red outline."""
        shape = belt_footprint(belt)
        if shape.is_empty or shape.geom_type != "Polygon":
            return
        points = [self._world_to_screen(x, y) for x, y in shape.exterior.coords]
        if len(points) >= 3:
            pygame.draw.polygon(self.screen, (239, 68, 68), points, 3)

    def draw_belt(self, belt: Belt):
        color = self._hex_to_rgb(belt.color)
        if belt.type == "path":
            # Drawn belt: the offset footprint (cad/path.py), a loop's inner
            # hole cut back out, and the centerline the parcels follow.
            shape = belt_footprint(belt)
            if shape.is_empty:
                return
            pygame.draw.polygon(self.screen, color, [self._world_to_screen(x, y) for x, y in shape.exterior.coords])
            for hole in shape.interiors:
                pygame.draw.polygon(self.screen, (30, 30, 30), [self._world_to_screen(x, y) for x, y in hole.coords])
            line = [self._world_to_screen(x, y) for x, y in centerline_points(belt)]
            if len(line) >= 2:
                pygame.draw.lines(self.screen, (255, 140, 0), False, line, 1)
            return
        if belt.type == "linear":
            if getattr(belt, 'shape', 'rectangle') == 'quadrilateral':
                screen_verts = [self._world_to_screen(wx, wy) for wx, wy in get_belt_polygon(belt)]
                pygame.draw.polygon(self.screen, color, screen_verts)
            else:
                w = self._scale_val(belt.length)
                h = self._scale_val(belt.beltWidth)
                if w <= 0 or h <= 0: return
                
                # Rotate surface (pygame rotation is counter-clockwise, matching World CCW)
                rotated_surf = self._cached_rotated_fill(w, h, (*color, 255), belt.rotation)
                rect = rotated_surf.get_rect(center=self._world_to_screen(belt.x, belt.y))
                self.screen.blit(rotated_surf, rect)
        elif belt.type == "curved":
            # Better arc drawing using polygon segments
            center = self._world_to_screen(belt.x, belt.y)
            radius = belt.radius
            thickness = belt.beltWidth
            if radius <= 0 or thickness <= 0: return
            
            # Draw using segments to support proper thickness
            angle_diff = belt.endAngle - belt.startAngle
            if angle_diff <= 0:
                angle_diff += 360
                
            segments = max(10, int(radius * math.radians(angle_diff) / 5))
            points = []
            inner_r = radius - thickness / 2.0
            outer_r = radius + thickness / 2.0
            
            s_rad = math.radians(belt.startAngle)
            e_rad = math.radians(belt.startAngle + angle_diff)
            
            # outer curve
            for i in range(segments + 1):
                angle = s_rad + (e_rad - s_rad) * (i / segments)
                wx = belt.x + outer_r * math.cos(angle)
                wy = belt.y + outer_r * math.sin(angle)
                points.append(self._world_to_screen(wx, wy))
                
            # inner curve (reverse)
            for i in range(segments, -1, -1):
                angle = s_rad + (e_rad - s_rad) * (i / segments)
                wx = belt.x + inner_r * math.cos(angle)
                wy = belt.y + inner_r * math.sin(angle)
                points.append(self._world_to_screen(wx, wy))
                
            if len(points) >= 3:
                pygame.draw.polygon(self.screen, color, points)

    def draw_sensor(self, sensor: Sensor, physics_engine=None):
        if sensor.sensorType == "laser_banner":
            color = (255, 50, 50) # distinctly red
        else:
            color = self._hex_to_rgb(sensor.color)
        w = self._scale_val(sensor.width)
        h = self._scale_val(sensor.height)
        if w <= 0 or h <= 0: return
        
        # Use direct rotation (World CCW = Pygame CCW); semi-transparent fill.
        angle = sensor.rotation
        rotated_surf = self._cached_rotated_fill(w, h, (*color, 100), angle)
        rect = rotated_surf.get_rect(center=self._world_to_screen(sensor.x, sensor.y))
        self.screen.blit(rotated_surf, rect)

        # Draw laser beams if laser_banner
        laser_count_val = getattr(sensor, 'laserCount', 16)
        laser_count = int(laser_count_val) if laser_count_val is not None else 16
        beam_dist_val = getattr(sensor, 'beamDistance', 0.05)
        beam_dist = float(beam_dist_val) if beam_dist_val is not None else 0.05
        
        if sensor.sensorType == "laser_banner" and laser_count > 0:
            total_span = beam_dist * (laser_count - 1)
            rot_rad = math.radians(angle)
            # In unrotated sensor (width<height), the beams spread along local Y axis.
            # Local +Y is DOWN on screen, UP in world.
            # When rotated by 'angle' CCW:
            # The local Y vector (0, 1) in World becomes (-sin(a), cos(a)).
            # dx_unit, dy_unit are WORLD vectors.
            dx_unit = -math.sin(rot_rad)
            dy_unit = math.cos(rot_rad)
            
            state_str = None
            if physics_engine:
                state_str = physics_engine.sensor_states.get(sensor.id)
                
            for i in range(laser_count):
                t = -total_span / 2 + i * beam_dist
                bx = sensor.x + t * dx_unit
                by = sensor.y + t * dy_unit
                
                is_blocked = False
                if isinstance(state_str, str) and i < len(state_str):
                    is_blocked = state_str[i] == '1'
                    
                sx, sy = self._world_to_screen(bx, by)
                dot_color = (255, 0, 0) if is_blocked else (0, 255, 0)
                pygame.draw.circle(self.screen, dot_color, (sx, sy), max(1, int(3 * self.zoom)))

    def draw_button(self, button: ControlButton):
        color = self._hex_to_rgb(button.color)
        if button.sensor_states:
            color = tuple(max(0, int(channel * 0.65)) for channel in color)
        w = self._scale_val(button.width)
        h = self._scale_val(button.height)
        if w <= 0 or h <= 0:
            return
        border = max(1, int(2 * self.zoom))
        key = ("button", w, h, color, border, button.label, button.rotation)
        rotated_surf = self._surface_cache.get(key)
        if rotated_surf is None:
            surf = pygame.Surface((w, h), pygame.SRCALPHA)
            surf.fill((*color, 220))
            pygame.draw.rect(surf, (20, 20, 20), surf.get_rect(), border)
            label = self._cached_text(self.font, button.label, (255, 255, 255))
            label_rect = label.get_rect(center=(w // 2, h // 2))
            surf.blit(label, label_rect)
            rotated_surf = pygame.transform.rotate(surf, button.rotation)
            self._surface_cache[key] = rotated_surf
        rect = rotated_surf.get_rect(center=self._world_to_screen(button.x, button.y))
        self.screen.blit(rotated_surf, rect)

    def draw_led(self, led: LedIndicator):
        cx, cy = self._world_to_screen(led.x, led.y)
        radius = max(3, self._scale_val(led.radius))
        base_color = self._hex_to_rgb(led.color)
        color = base_color if led.isOn else tuple(max(15, int(channel * 0.25)) for channel in base_color)
        pygame.draw.circle(self.screen, color, (cx, cy), radius)
        pygame.draw.circle(self.screen, (20, 20, 20), (cx, cy), radius, max(1, int(2 * self.zoom)))
        if led.label:
            label = self._cached_text(self.font, led.label, (220, 220, 220))
            self.screen.blit(label, (cx + radius + 6, cy - label.get_height() // 2))

    def draw_source(self, source):
        # Draw source as a blue box
        w = self._scale_val(source.maxWidth if hasattr(source, 'maxWidth') else 0.5)
        h = self._scale_val(source.maxHeight if hasattr(source, 'maxHeight') else 0.5)
        if w <= 0 or h <= 0: return
        angle = source.rotation
        rotated_surf = self._cached_rotated_fill(w, h, (50, 100, 255, 150), angle)
        rect = rotated_surf.get_rect(center=self._world_to_screen(source.x, source.y))
        self.screen.blit(rotated_surf, rect)

    def draw_sink(self, sink):
        # Draw sink as an orange box
        w = self._scale_val(sink.width)
        h = self._scale_val(sink.height)
        if w <= 0 or h <= 0: return
        surf = self._cached_rotated_fill(w, h, (255, 150, 50, 150), 0.0)
        rect = surf.get_rect(center=self._world_to_screen(sink.x, sink.y))
        self.screen.blit(surf, rect)

    def draw_grid(self):
        grid_spacing = getattr(self.config, 'gridSize', 0.5)
        if grid_spacing <= 0:
            grid_spacing = 0.5

        # Prevent dense grid line drawing bottleneck when zoomed out
        pixel_spacing = grid_spacing * self.scale * self.zoom
        if pixel_spacing < 15.0:
            # Scale up grid spacing dynamically so lines are at least 15 pixels apart
            multiplier = math.ceil(15.0 / pixel_spacing)
            grid_spacing *= multiplier

        w_min_x, w_max_y = self._screen_to_world(0, 0)
        w_max_x, w_min_y = self._screen_to_world(self.width, self.height)

        start_x = math.floor(w_min_x / grid_spacing) * grid_spacing
        end_x = math.ceil(w_max_x / grid_spacing) * grid_spacing
        
        start_y = math.floor(w_min_y / grid_spacing) * grid_spacing
        end_y = math.ceil(w_max_y / grid_spacing) * grid_spacing
        
        grid_color = (50, 50, 50)
        
        # Vertical lines (including Y-axis)
        x = start_x
        while x <= end_x:
            sx, _ = self._world_to_screen(x, 0)
            is_axis = abs(x) < 0.001
            color = (150, 50, 50) if is_axis else grid_color
            thickness = 2 if is_axis else 1
            pygame.draw.line(self.screen, color, (sx, 0), (sx, self.height), thickness)
            x += grid_spacing
            
        # Horizontal lines (including X-axis)
        y = start_y
        while y <= end_y:
            _, sy = self._world_to_screen(0, y)
            is_axis = abs(y) < 0.001
            color = (50, 150, 50) if is_axis else grid_color
            thickness = 2 if is_axis else 1
            pygame.draw.line(self.screen, color, (0, sy), (self.width, sy), thickness)
            y += grid_spacing

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
        # Snapshot: with --sim-thread the runner thread adds/removes parcels concurrently.
        for parcel in list(physics_engine.parcels):
            color = self._hex_to_rgb(parcel.color)
            
            # pymunk Poly vertices
            points = []
            for v in parcel.shape.get_vertices():
                wv = parcel.body.position + v.rotated(parcel.body.angle)
                points.append(self._world_to_screen(wv.x, wv.y))
                
            if len(points) >= 3:
                pygame.draw.polygon(self.screen, color, points)
                pygame.draw.polygon(self.screen, (0, 0, 0), points, 2) # Black outline

            # Actual vs. the belt's configured speed - a direct, always-on
            # verification of "is this parcel really moving at the speed
            # its belt is set to" rather than trusting a feeling.
            if hasattr(self, 'small_font'):
                actual, target = parcel_speed_info(parcel)
                if target is not None:
                    label_text = f"{actual:.2f}/{target:.2f}"
                    lagging = actual < target * 0.9
                else:
                    label_text = f"{actual:.2f}"
                    lagging = False
                label_color = (255, 90, 90) if lagging else (230, 230, 230)
                sx, sy = self._world_to_screen(parcel.body.position.x, parcel.body.position.y)
                label = self.small_font.render(label_text, True, label_color)
                self.screen.blit(label, label.get_rect(center=(sx, sy)))

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
            # once zoom changes, none of them will ever be looked up again
            # under their old cache keys, so they're just dead weight from
            # here on. Drop them all at once instead of letting the cache
            # grow by one full extra "generation" of shapes per zoom level
            # for the rest of the session.
            self._surface_cache.clear()
            self._surface_cache_zoom = self.zoom

        self.screen.fill((30, 30, 30)) # Dark background
        
        # Draw grid and axes
        self.draw_grid()

        # Draw belts
        for belt in self.config.belts:
            self.draw_belt(belt)
            if belt.fault:
                self._draw_fault_outline(belt)
            if belt == self.selected_item:
                self._draw_selection_overlay(belt)

        # Draw sensors
        for sensor in self.config.sensors:
            self.draw_sensor(sensor, physics_engine)
            if sensor == self.selected_item:
                self._draw_selection_overlay(sensor)

        for button in self.config.buttons:
            self.draw_button(button)
            if button == self.selected_item:
                self._draw_selection_overlay(button)

        for led in self.config.leds:
            self.draw_led(led)
            if led == self.selected_item:
                self._draw_selection_overlay(led)

        # Draw sources
        for source in self.config.sources:
            self.draw_source(source)
            if source == self.selected_item:
                self._draw_selection_overlay(source)
                
        # Draw sinks
        for sink in self.config.sinks:
            self.draw_sink(sink)
            if sink == self.selected_item:
                self._draw_selection_overlay(sink)

        # Draw origin and direction marks on top of components
        for belt in self.config.belts:
            self.draw_origin_and_direction(belt)
        # for sensor in self.config.sensors:
        #     self.draw_origin_and_direction(sensor)
        # for button in self.config.buttons:
        #     self.draw_origin_and_direction(button)
        # for led in self.config.leds:
        #     self.draw_origin_and_direction(led)
        # for source in self.config.sources:
        #     self.draw_origin_and_direction(source)
        # for sink in self.config.sinks:
        #     self.draw_origin_and_direction(sink)

        # Draw parcels
        if physics_engine:
            self.draw_parcels(physics_engine)
            self.editor_ui.update_sensor_state(physics_engine)

        # Draw overlays
        if not self.headless:
            self.manager.draw_ui(self.screen)
            status_text = "PLAYING" if self.playing else "PAUSED"
            status_color = (0, 255, 0) if self.playing else (255, 255, 0)

            texts = [
                self.font.render(f"Status: {status_text}", True, status_color),
                self.font.render(f"Zoom: {self.zoom:.2f}x", True, (255, 255, 255)),
                self.font.render(f"FPS: {self.clock.get_fps():.1f}", True, (0, 255, 0)),
            ]
            if physics_engine:
                # sim_time is the sum of every dt physics has actually been
                # stepped by; comparing it to wall-clock time elapsed since
                # this Renderer was created verifies the simulation is
                # keeping pace with real time rather than silently falling
                # behind under load - "does the parcel really move at the
                # speed it's set to" starts with "is time itself correct".
                real_elapsed = max(1e-6, self.track_play_time())
                ratio = physics_engine.sim_time / real_elapsed
                ratio_color = (150, 255, 150) if abs(ratio - 1.0) <= 0.1 else (255, 200, 0)
                dropped = getattr(physics_engine, "dropped_time", 0.0)
                texts.append(self.font.render(
                    f"Sim/Real: {ratio:.2f}x (sim={physics_engine.sim_time:.1f}s real={real_elapsed:.1f}s "
                    f"dropped={dropped:.2f}s)",
                    True, ratio_color,
                ))
            texts.append(self._cached_text(
                self.font,
                "Controls: [Space] Play/Pause | [R] Reset | [C] Clear | Scroll to Zoom | Click & Drag to Pan",
                (150, 150, 150),
            ))

            for i, txt in enumerate(texts):
                self.screen.blit(txt, (140, 10 + i * 30))

        pygame.display.flip()
        self.clock.tick(self.fps)

    def quit(self):
        if not self.headless:
            pygame.quit()
