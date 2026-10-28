"""The simulator window's panels: the toolbar folders, a read-only inspector
of the selected component and the Components tree. Layouts are designed in
the layout editor (layout_editor.py, "Edit in Designer"); this window only
runs and watches them."""
import pygame
import hmi_style as hmi
import pygame_gui
from config_parser import Belt, Sensor, Source, Sink, ControlButton, LedIndicator
from physics import PhysicsEngine


TOP_BAR = 30     # px, simulator status bar
BOTTOM_BAR = 24  # px, key hint bar


TOOLBAR_W = 150
# The toolbar's folders: (title, [(attribute, button text[, object id])]).
TOOLBAR_FOLDERS = (
    ("SIMULATION", (("sim_play_btn", "Start", "#sim_primary"), ("sim_clear_btn", "Clear Parcels"),
                    ("sim_reset_btn", "Reset"))),
    ("LAYOUT", (("manager_btn", "Components"), ("designer_btn", "Edit in Designer"))),
    ("VIEW", (("theme_btn", "Light Theme"),)),
)

# What the inspector lists for each kind of component (sorted into the
# editor's sections by hmi.group_sort_key).
INSPECTOR_FIELDS = {
    Belt: ("id", "x", "y", "rotation", "speed", "directionAngle", "transition_momentum_duration"),
    Sensor: ("id", "label", "sensorType", "x", "y", "rotation", "width", "height"),
    Source: ("id", "label", "x", "y", "rotation", "interval", "minWidth", "maxWidth", "minHeight", "maxHeight"),
    Sink: ("id", "label", "x", "y", "width", "height"),
    ControlButton: ("id", "label", "x", "y", "rotation", "width", "height"),
    LedIndicator: ("id", "label", "x", "y", "radius"),
}
BELT_GEOMETRY = {
    "linear": ("length", "beltWidth", "beltShape"),
    "curved": ("radius", "beltWidth", "startAngle", "endAngle"),
    "path": ("beltWidth",),
}
ROW_H = 24
PANEL_W = 250


def _kind_name(item) -> str:
    if isinstance(item, Belt):
        return f"{item.type} belt"
    return {Sensor: "sensor", Source: "source", Sink: "sink", ControlButton: "push button",
            LedIndicator: "LED"}.get(type(item), "component")


def format_value(attr: str, value) -> str:
    """A property value as the editor shows it: rounded, with its unit."""
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return f"{value:.3f}".rstrip("0").rstrip(".") + hmi.PROPERTY_UNITS.get(attr, "")
    if value in (None, ""):
        return "-"
    return str(value) + hmi.PROPERTY_UNITS.get(attr, "")


class EditorUI:
    def __init__(self, manager: pygame_gui.UIManager, width: int, height: int, config=None):
        self.manager = manager
        self.width = width
        self.height = height
        self.config = config

        # Toolbar (Left): folders of tools, each header collapses its folder.
        self.toolbar_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(10, TOP_BAR + 8, TOOLBAR_W, 200),
            manager=self.manager
        )
        self._toolbar_folders = []   # [header button, [tool buttons], collapsed, title]
        for title, tools in TOOLBAR_FOLDERS:
            header = pygame_gui.elements.UIButton(
                relative_rect=pygame.Rect(4, 0, TOOLBAR_W - 10, 22),
                text=f"–  {title}",
                manager=self.manager,
                container=self.toolbar_panel,
                object_id="#folder_header",
            )
            buttons = []
            for attr, text, *style in tools:
                button = pygame_gui.elements.UIButton(
                    relative_rect=pygame.Rect(16, 0, TOOLBAR_W - 28, 24),
                    text=text,
                    manager=self.manager,
                    container=self.toolbar_panel,
                    object_id=style[0] if style else None,
                )
                setattr(self, attr, button)
                buttons.append(button)
            self._toolbar_folders.append([header, buttons, False, title])
        self._layout_toolbar()

        # Inspector (Right): read-only, rows built for the selected item.
        self.prop_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(self.width - PANEL_W, TOP_BAR, PANEL_W, self._panel_height()),
            manager=self.manager
        )
        self.prop_rows = {}       # attr -> (name label, value label)
        self.live_rows = {}       # live readout name -> (name label, value label)
        self.prop_title = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(8, 8, PANEL_W - 16, 18), text="", manager=self.manager,
            container=self.prop_panel, object_id="#prop_kind")
        self.prop_hint = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(8, 8, PANEL_W - 16, 40),
            text="Click a component to inspect it", manager=self.manager,
            container=self.prop_panel, object_id="#prop_name")
        self.selected_item = None
        self.show_props_for(None)

        self.manager_window = None
        self.manager_list = None
        self.manager_details = None
        self.manager_select_btn = None
        self.manager_refresh_btn = None
        self.manager_items = {}
        self.manager_collapsed = set()   # kinds folded up in the manager tree
        self.manager_selected_key = None

    # -- inspector -----------------------------------------------------------------

    def _group_header(self, group: str):
        """The section heading of one property group (created on first use)."""
        headers = self.__dict__.setdefault("_group_headers", {})
        if group not in headers:
            headers[group] = pygame_gui.elements.UILabel(
                relative_rect=pygame.Rect(8, 0, PANEL_W - 16, 20), text=group.upper(), manager=self.manager,
                container=self.prop_panel, object_id="#prop_group")
        return headers[group]

    def _row(self, rows: dict, key: str, name: str):
        if key not in rows:
            rows[key] = (
                pygame_gui.elements.UILabel(pygame.Rect(8, 0, 96, ROW_H), name, self.manager,
                                            container=self.prop_panel, object_id="#prop_name"),
                pygame_gui.elements.UILabel(pygame.Rect(110, 0, PANEL_W - 122, ROW_H), "", self.manager,
                                            container=self.prop_panel, object_id="#prop_value"),
            )
        return rows[key]

    def hide_all_props(self):
        for header in self.__dict__.get("_group_headers", {}).values():
            header.hide()
        for rows in (self.prop_rows, self.live_rows):
            for name, value in rows.values():
                name.hide()
                value.hide()

    def inspector_fields(self, item) -> list:
        fields = list(INSPECTOR_FIELDS.get(type(item), ("id", "x", "y")))
        if isinstance(item, Belt):
            fields += BELT_GEOMETRY.get(item.type, ())
        elif isinstance(item, Sensor) and item.sensorType == "laser_banner":
            fields += ["laserCount", "beamDistance"]
        fields.sort(key=hmi.group_sort_key)
        return fields

    @staticmethod
    def _value_of(item, attr: str):
        if attr == "beltShape":
            return getattr(item, "shape", None)
        return getattr(item, attr, None)

    def show_props_for(self, item):
        self.selected_item = item
        self.hide_all_props()
        self._live_shown = {}
        if not item:
            self.prop_title.hide()
            self.prop_hint.show()
            return
        self.prop_hint.hide()
        self.prop_title.set_text(_kind_name(item).upper())
        self.prop_title.show()
        y = 32
        last_group = None
        for attr in self.inspector_fields(item):
            group = hmi.property_group(attr)
            if group != last_group:
                last_group = group
                y = self._place_header(group, y)
            name, value = self._row(self.prop_rows, attr, hmi.property_label(attr))
            value.set_text(format_value(attr, self._value_of(item, attr)))
            y = self._place_row(name, value, y)
        live = self._live_values(item)
        if live:
            y = self._place_header("Live", y)
            for key in live:
                name, value = self._row(self.live_rows, key, key)
                y = self._place_row(name, value, y)
        self.update_live()

    def _place_header(self, group: str, y: int) -> int:
        header = self._group_header(group)
        header.set_relative_position((8, y + 6))
        header.show()
        return y + 28

    @staticmethod
    def _place_row(name, value, y: int) -> int:
        name.set_relative_position((8, y))
        value.set_relative_position((110, y))
        name.show()
        value.show()
        return y + ROW_H

    def _live_values(self, item, physics_engine: PhysicsEngine = None) -> dict:
        """Readouts that change while the simulation runs: {name: text}."""
        if isinstance(item, Belt):
            drive = "FAULT" if item.fault else ("running" if item.running else "stopped")
            return {"Drive": drive, "Actual speed": f"{item.actualSpeed:.2f} m/s"}
        if isinstance(item, Sensor):
            # The conditioned output (delays, NC) as the PLC gets it: True/False
            # or a "0"/"1" string, one character per beam.
            state = item.sensor_states
            bits = state if isinstance(state, str) else ("1" if state else "0")
            if item.sensorType == "laser_banner":
                return {"Beams": f"{bits.count('1')} of {len(bits)} blocked"}
            return {"State": "BLOCKED" if "1" in bits else "clear"}
        if isinstance(item, ControlButton):
            return {"Contact": "PRESSED" if getattr(item, "sensor_states", False) else "released"}
        if isinstance(item, LedIndicator):
            return {"Lamp": "ON" if item.isOn else "off"}
        return {}

    def update_live(self, physics_engine: PhysicsEngine = None):
        """Refresh the Live section's readouts (text only changes when the value does)."""
        if not self.selected_item:
            return
        shown = self.__dict__.setdefault("_live_shown", {})
        for key, text in self._live_values(self.selected_item, physics_engine).items():
            if key in self.live_rows and shown.get(key) != text:
                shown[key] = text
                self.live_rows[key][1].set_text(text)

    def update_sensor_state(self, physics_engine: PhysicsEngine):
        """Called every frame by the renderer."""
        self.update_live(physics_engine)

    def refresh_button_state(self):
        self.update_live()

    def is_ui_blocking_point(self, pos) -> bool:
        for panel in (self.prop_panel, self.toolbar_panel):
            if panel and panel.rect.collidepoint(pos):
                return True
        window = self.manager_window
        return bool(window and window.alive() and window.rect.collidepoint(pos))

    # -- components tree -----------------------------------------------------------

    def open_manager_panel(self):
        if not self.config:
            return
        if self.manager_window and self.manager_window.alive():
            self.manager_window.focus()
            self.refresh_manager_panel()
            return

        self.manager_window = pygame_gui.elements.UIWindow(
            rect=pygame.Rect(170, 70, 520, 520),
            manager=self.manager,
            window_display_title="Components",
            resizable=False
        )
        container = self.manager_window.get_container()
        self.manager_list = pygame_gui.elements.UISelectionList(
            relative_rect=pygame.Rect(10, 10, 240, 395),
            item_list=[],
            manager=self.manager,
            container=container
        )
        self.manager_details = pygame_gui.elements.UITextBox(
            html_text="Select a component.",
            relative_rect=pygame.Rect(260, 10, 240, 395),
            manager=self.manager,
            container=container
        )
        self.manager_select_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 420, 110, 32),
            text="Inspect",
            manager=self.manager,
            container=container
        )
        self.manager_refresh_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(130, 420, 110, 32),
            text="Refresh",
            manager=self.manager,
            container=container
        )
        self.refresh_manager_panel()

    def refresh_manager_panel(self):
        if not self.manager_list or not self.config:
            return
        # A tree: a folder row per kind ("-  Belts (16)"; click to fold it
        # up) and its components indented under it.
        self.manager_items = {}
        self.manager_folder_rows = {}
        rows = []
        groups = [
            ("belt", "Belts", self.config.belts),
            ("sensor", "Sensors", self.config.sensors),
            ("source", "Sources", self.config.sources),
            ("sink", "Sinks", self.config.sinks),
            ("button", "Buttons", self.config.buttons),
            ("led", "LEDs", self.config.leds),
        ]
        for kind, title, items in groups:
            if not items:
                continue
            collapsed = kind in self.manager_collapsed
            folder = f"{'+' if collapsed else '–'}  {title} ({len(items)})"
            rows.append(folder)
            self.manager_folder_rows[folder] = kind
            if collapsed:
                continue
            for idx, item in enumerate(items):
                text = (getattr(item, "label", "") or "").strip()
                label = f"      {item.id}" + (f"  ·  {text}" if text and text != item.id else "")
                rows.append(label)
                self.manager_items[label] = (kind, idx, item)
        self.manager_list.set_item_list(rows)
        if self.manager_selected_key not in self.manager_items:
            self.manager_selected_key = None
        self._update_manager_details()

    def _update_manager_details(self):
        if not self.manager_details:
            return
        if not self.manager_selected_key or self.manager_selected_key not in self.manager_items:
            self.manager_details.set_text("Select a component.")
            return
        kind, _, item = self.manager_items[self.manager_selected_key]
        fields = []
        for name, value in item.model_dump(exclude_none=True).items():
            fields.append(f"<b>{name}</b>: {value}")
        self.manager_details.set_text(f"<b>{kind.upper()}</b><br>" + "<br>".join(fields))

    # -- toolbar -------------------------------------------------------------------

    def _layout_toolbar(self):
        """Stack the toolbar folders: header, then its tools unless collapsed."""
        y = 6
        for header, buttons, collapsed, title in self._toolbar_folders:
            header.set_relative_position((4, y))
            header.set_text(f"{'+' if collapsed else '–'}  {title}")
            y += 24
            for button in buttons:
                if collapsed:
                    button.hide()
                    continue
                button.show()
                button.set_relative_position((16, y))
                y += 28
            y += 4
        self.toolbar_panel.set_dimensions((TOOLBAR_W, y + 4))

    def sync_sim_buttons(self, playing: bool, theme: str):
        """Start <-> Pause (green / amber) and the theme button's wording."""
        if getattr(self, "_shown_playing", None) != playing:
            self._shown_playing = playing
            self.sim_play_btn.set_text("Pause" if playing else "Start")
            self.sim_play_btn.change_object_id("#sim_pause" if playing else "#sim_primary")
        label = "Dark Theme" if theme == "light" else "Light Theme"
        if self.theme_btn.text != label:
            self.theme_btn.set_text(label)

    def toggle_toolbar_folder(self, title: str):
        for folder in self._toolbar_folders:
            if folder[3] == title:
                folder[2] = not folder[2]
        self._layout_toolbar()

    def _panel_height(self) -> int:
        return max(200, self.height - TOP_BAR - BOTTOM_BAR)

    def handle_resize(self, width: int, height: int):
        self.width = width
        self.height = height
        self.prop_panel.set_position((width - PANEL_W, TOP_BAR))
        self.prop_panel.set_dimensions((PANEL_W, self._panel_height()))
        # Keep the Components window from being stranded off-screen
        if self.manager_window and self.manager_window.alive():
            rect = self.manager_window.rect
            self.manager_window.set_position((max(0, min(rect.x, width - rect.width)),
                                              max(0, min(rect.y, height - rect.height))))

    def handle_event(self, event):
        """Returns a command ("TOGGLE_PLAY", "OPEN_DESIGNER", ("SELECT_ITEM", item), ...) or None."""
        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            for header, _buttons, _collapsed, title in self._toolbar_folders:
                if event.ui_element == header:
                    self.toggle_toolbar_folder(title)
                    return None
            for button, command in ((self.sim_play_btn, "TOGGLE_PLAY"), (self.sim_clear_btn, "CLEAR"),
                                    (self.sim_reset_btn, "RESET"), (self.theme_btn, "TOGGLE_THEME"),
                                    (self.manager_btn, "OPEN_MANAGER"), (self.designer_btn, "OPEN_DESIGNER")):
                if event.ui_element == button:
                    return command
            if self.manager_select_btn and event.ui_element == self.manager_select_btn:
                if self.manager_selected_key in self.manager_items:
                    _, _, item = self.manager_items[self.manager_selected_key]
                    self.show_props_for(item)
                    return ("SELECT_ITEM", item)
            elif self.manager_refresh_btn and event.ui_element == self.manager_refresh_btn:
                self.refresh_manager_panel()

        if event.type == pygame_gui.UI_SELECTION_LIST_NEW_SELECTION:
            if event.ui_element == self.manager_list:
                kind = getattr(self, "manager_folder_rows", {}).get(event.text)
                if kind is not None:             # a folder row: fold / unfold it
                    self.manager_collapsed ^= {kind}
                    self.refresh_manager_panel()
                    return None
                self.manager_selected_key = event.text
                self._update_manager_details()

        if event.type == pygame_gui.UI_WINDOW_CLOSE:
            if event.ui_element == self.manager_window:
                self.manager_window = None

        return None
