"""The simulator window's panels: the toolbar under the status bar, an inspector of the
selected component and the Components tree. Layouts are designed in the
layout editor (layout_editor.py, "Edit in Designer"); this window runs and
watches them. The inspector's properties are read-only; its PLC I/O section
(io_signals.py) shows the component's PLC signals live and lets the
operator drive its commands: Source picks where they come from (auto /
plc / manual), and in manual its commands - a sensor: its output - are set
right there."""
import pygame
import hmi_style as hmi
import io_signals
import pygame_gui
from config_parser import Belt, Sensor, Source, Sink, ControlButton, LedIndicator, Pallet, Stopper, RobotArm, Elevator
from config_parser import Accumulator, Diverter, GuardDoor, SafetyCircuit, Scanner, Transfer, Turntable
from physics import PhysicsEngine


TOP_BAR = 30     # px, simulator status bar
TOOLBAR_H = 34   # px, the tool bar under it
BOTTOM_BAR = 24  # px, key hint bar
CANVAS_TOP = TOP_BAR + TOOLBAR_H


# The tool bar's groups, left to right: (title, [(attribute, text, width[, object id])]).
TOOLBAR_GROUPS = (
    ("SIMULATION", (("sim_play_btn", "Start", 76, "#sim_primary"), ("sim_speed_btn", "Speed 1x", 92),
                    ("sim_clear_btn", "Clear Parcels", 108), ("sim_reset_btn", "Reset", 64))),
    ("LAYOUT", (("manager_btn", "Components", 100), ("designer_btn", "Edit in Designer", 126))),
    ("VIEW", (("view3d_btn", "3D View", 76), ("drawing_btn", "Hide Drawing", 110), ("theme_btn", "Light Theme", 104))),
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
    Pallet: ("id", "label", "x", "y", "rotation", "width", "height", "mass", "capacity", "level"),
    Stopper: ("id", "label", "x", "y", "rotation", "width", "depth", "level", "holdTime", "releaseTime"),
    RobotArm: ("id", "label", "x", "y", "reach", "pickX", "pickY", "placeX", "placeY", "cycleTime", "pickSensorId"),
    Elevator: ("id", "label", "x", "y", "rotation", "width", "height", "levels", "levelHeight", "liftSpeed",
               "beltId"),
    Scanner: ("id", "label", "scannerType", "x", "y", "rotation", "width", "height", "level"),
    Diverter: ("id", "label", "diverterType", "x", "y", "rotation", "width", "height", "divertAngle", "speed",
               "destination"),
    Turntable: ("id", "label", "x", "y", "radius", "beltId", "turnSpeed"),
    Accumulator: ("id", "label", "x", "y", "mode"),
    Transfer: ("id", "label", "x", "y", "rotation", "width", "height", "crossAngle", "speed", "stop"),
    GuardDoor: ("id", "label", "x", "y", "rotation", "width"),
    SafetyCircuit: ("id", "label", "x", "y"),
}
BELT_GEOMETRY = {
    "linear": ("length", "beltWidth", "beltShape"),
    "curved": ("radius", "beltWidth", "startAngle", "endAngle"),
    "path": ("beltWidth",),
}
ROW_H = 24
PANEL_W = 250
SCROLLBAR_W = 16
# Tooltips of the PLC I/O section's Source buttons (a component's `control`).
SOURCE_TIPS = {
    "auto": "Its own controller sets the commands",
    "plc": "The PLC data exchange sets the commands",
    "manual": "Set the commands here; PLC frames leave them alone",
    "sim": "The simulation decides the output",
}


def io_text(value) -> str:
    """A PLC signal's value as the PLC would show it."""
    if isinstance(value, bool):
        return "TRUE" if value else "FALSE"
    if isinstance(value, float):
        return f"{value:.3f}"
    return "-" if value is None else str(value)


def _kind_name(item) -> str:
    if isinstance(item, Belt):
        return f"{item.type} belt"
    return {Sensor: "sensor", Source: "source", Sink: "sink", ControlButton: "push button",
            LedIndicator: "LED", Pallet: "pallet", Stopper: "stopper", RobotArm: "robot arm",
            Elevator: "elevator", Scanner: "scanner", Diverter: "diverter", Turntable: "turntable",
            Accumulator: "accumulation conveyor", GuardDoor: "guard door", Transfer: "90-degree transfer",
            SafetyCircuit: "safety circuit"}.get(type(item), "component")


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

        # Tool bar (top, under the status bar): small group titles, then buttons.
        self.toolbar_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(0, TOP_BAR, self.width, TOOLBAR_H),
            manager=self.manager, object_id="#toolbar",
        )
        self.toolbar_groups = []   # (title label, [buttons])
        x = 10
        for title, tools in TOOLBAR_GROUPS:
            label = pygame_gui.elements.UILabel(
                pygame.Rect(x, 0, 8 + 7 * len(title), TOOLBAR_H - 2), title, self.manager,
                container=self.toolbar_panel, object_id="#tool_group")
            x += label.relative_rect.width + 4
            buttons = []
            for attr, text, w, *style in tools:
                button = pygame_gui.elements.UIButton(
                    relative_rect=pygame.Rect(x, 4, w, TOOLBAR_H - 10),
                    text=text,
                    manager=self.manager,
                    container=self.toolbar_panel,
                    object_id=style[0] if style else None,
                )
                setattr(self, attr, button)
                buttons.append(button)
                x += w + 4
            self.toolbar_groups.append((label, buttons))
            x += 18
        if not (config is not None and config.background is not None):
            self.drawing_btn.disable()   # nothing to show / hide

        # Inspector (Right): read-only, rows built for the selected item.
        self.prop_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(self.width - PANEL_W, CANVAS_TOP, PANEL_W, self._panel_height()),
            manager=self.manager
        )
        # Everything in it scrolls (a robot's PLC signals don't fit a short window).
        self.prop_scroll = pygame_gui.elements.UIScrollingContainer(
            pygame.Rect(0, 0, PANEL_W - 4, self._panel_height() - 4), manager=self.manager,
            container=self.prop_panel, allow_scroll_x=False)
        self.prop_rows = {}       # attr -> (name label, value label)
        self.live_rows = {}       # live readout name -> (name label, value label)
        self.io_rows = {}         # PLC signal name -> (Signal, value label / editor)
        self.io_source_buttons = {}   # Source button -> its `control` value
        self._io_widgets = []
        self._io_shown = {}
        self.prop_title = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(8, 8, PANEL_W - 16, 18), text="", manager=self.manager,
            container=self.prop_scroll, object_id="#prop_kind")
        self.prop_hint = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(8, 8, PANEL_W - 16, 40),
            text="Click a component to inspect it", manager=self.manager,
            container=self.prop_scroll, object_id="#prop_name")
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
                container=self.prop_scroll, object_id="#prop_group")
        return headers[group]

    def _row(self, rows: dict, key: str, name: str):
        if key not in rows:
            rows[key] = (
                pygame_gui.elements.UILabel(pygame.Rect(8, 0, 96, ROW_H), name, self.manager,
                                            container=self.prop_scroll, object_id="#prop_name"),
                pygame_gui.elements.UILabel(pygame.Rect(110, 0, PANEL_W - 122 - SCROLLBAR_W, ROW_H), "",
                                            self.manager, container=self.prop_scroll, object_id="#prop_value"),
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
        self._clear_io()
        self._live_shown = {}
        if not item:
            self.prop_title.hide()
            self.prop_hint.show()
            self.prop_scroll.set_scrollable_area_dimensions((PANEL_W - 4 - SCROLLBAR_W, 60))
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
        y = self._place_io(item, y)
        self.prop_scroll.set_scrollable_area_dimensions((PANEL_W - 4 - SCROLLBAR_W, y + 8))
        self.update_live()

    # -- PLC I/O -------------------------------------------------------------------

    def _clear_io(self):
        for widget in self._io_widgets:
            widget.kill()
        self._io_widgets = []
        self.io_rows = {}
        self.io_source_buttons = {}
        self._io_shown = {}

    def _io_widget(self, widget):
        self._io_widgets.append(widget)
        return widget

    def _place_io(self, item, y: int) -> int:
        """The PLC I/O section: Source (where the commands come from), then
        the commands and the feedback by their PLC names. In manual, a
        command (a sensor: its output) is an editor - a toggle for a BOOL,
        a text box for a number (Enter sets it) - else a live readout."""
        signals = io_signals.signals(item)
        if not signals:
            return y
        control = getattr(item, "control", None)
        options = io_signals.control_options(item)
        scroll = self.prop_scroll
        if options:
            y = self._place_header("Command source", y)
            width = (PANEL_W - 20 - SCROLLBAR_W) // len(options)
            for i, option in enumerate(options):
                button = self._io_widget(pygame_gui.elements.UIButton(
                    pygame.Rect(8 + i * width, y + 1, width - 2, ROW_H - 2), option, self.manager,
                    container=scroll, tool_tip_text=SOURCE_TIPS.get(option)))
                if option == control:
                    button.select()
                self.io_source_buttons[button] = option
            y += ROW_H + 2
        for direction, title in (("command", "PLC commands"), ("feedback", "PLC feedback")):
            rows = [s for s in signals if s.direction == direction]
            if not rows:
                continue
            y = self._place_header(title, y)
            for signal in rows:
                self._io_widget(pygame_gui.elements.UILabel(
                    pygame.Rect(8, y, 100, ROW_H), signal.name, self.manager, container=scroll,
                    object_id="#prop_name"))
                rect = pygame.Rect(110, y + 1, PANEL_W - 122 - SCROLLBAR_W, ROW_H - 2)
                if control == "manual" and signal.settable:
                    if signal.count == 1 and signal.byte_type == io_signals.ByteType.BIT:
                        widget = pygame_gui.elements.UIButton(rect, "", self.manager, container=scroll,
                                                              object_id="#io_toggle")
                    else:
                        widget = pygame_gui.elements.UITextEntryLine(rect, self.manager, container=scroll,
                                                                     object_id="#io_entry")
                else:
                    widget = pygame_gui.elements.UILabel(rect, "", self.manager, container=scroll,
                                                         object_id="#prop_value")
                self.io_rows[signal.name] = (signal, self._io_widget(widget))
                y += ROW_H
        return y

    def _update_io(self):
        item = self.selected_item
        for name, (signal, widget) in self.io_rows.items():
            text = io_text(io_signals.value_of(item, signal))
            if isinstance(widget, pygame_gui.elements.UITextEntryLine):
                if widget.is_focused:
                    continue                       # the operator is typing in it
            if self._io_shown.get(name) == text:
                continue
            self._io_shown[name] = text
            if isinstance(widget, pygame_gui.elements.UIButton):
                widget.set_text("ON" if text == "TRUE" else "OFF")
                (widget.select if text == "TRUE" else widget.unselect)()
            else:
                widget.set_text(text)

    def typing(self) -> bool:
        """Whether a PLC I/O text box has the keyboard (no shortcuts then)."""
        return any(isinstance(w, pygame_gui.elements.UITextEntryLine) and w.is_focused
                   for _, w in self.io_rows.values())

    def _io_event(self, event):
        """("SET_IO", component id, signal or "control", value) for a PLC
        I/O edit, else None."""
        item = self.selected_item
        if item is None:
            return None
        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            option = self.io_source_buttons.get(event.ui_element)
            if option is not None:
                return ("SET_IO", item.id, "control", option)
            for name, (signal, widget) in self.io_rows.items():
                if widget is event.ui_element:
                    return ("SET_IO", item.id, name, not io_signals.value_of(item, signal))
        if event.type == pygame_gui.UI_TEXT_ENTRY_FINISHED:
            for name, (signal, widget) in self.io_rows.items():
                if widget is event.ui_element:
                    text = widget.get_text().strip()
                    self._io_shown.pop(name, None)            # show the value it really got
                    try:
                        if signal.count > 1:
                            value = text
                        elif signal.byte_type == io_signals.ByteType.FLOAT:
                            value = float(text)
                        else:
                            value = int(float(text))
                    except ValueError:
                        return None
                    return ("SET_IO", item.id, name, value)
        return None

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
        if isinstance(item, Pallet):
            return {"Loads": f"{item.loadCount} of {item.capacity}"}
        if isinstance(item, Stopper):
            return {"Bar": "UP" if item.isUp else "down", "Waiting": "yes" if item.occupied else "no"}
        if isinstance(item, RobotArm):
            if item.fault:
                state = f"FAULT {item.faultCode}"
            else:
                state = "holding a part" if item.holding else ("moving" if item.busy else "idle")
            return {"Robot": state, "Cycles": str(item.cycles),
                    "Joints": " ".join(f"{getattr(item, f'j{i}'):.0f}" for i in range(1, 7)),
                    "TCP": f"{item.tcpX:.2f}, {item.tcpY:.2f}, {item.tcpZ:.2f}"}
        if isinstance(item, Elevator):
            where = f"to level {item.targetLevel}" if item.moving else f"at level {item.level}"
            return {"Lift": where, "Platform": "loaded" if item.occupied else "empty"}
        if isinstance(item, Scanner):
            last = "no read" if item.noRead else f"{item.code} -> {item.destination}"
            out = {"Reads": str(item.readCount), "Last": last if item.readCount else "-"}
            if item.scannerType == "dws" and item.readCount:
                out["Measured"] = (f"{item.weight:.2f} kg  {item.measuredLength:.2f} x {item.measuredWidth:.2f}"
                                   f" x {item.measuredHeight:.2f} m")
            return out
        if isinstance(item, Diverter):
            return {"Diverter": "ACTIVE" if item.active else "idle", "Diverted": str(item.diverted)}
        if isinstance(item, Turntable):
            return {"Table": "turning" if item.turning else f"position {item.index}",
                    "Platform": "loaded" if item.occupied else "empty"}
        if isinstance(item, Transfer):
            state = "UP - carrying across" if item.up else ("pallet centred" if item.centered else "down")
            return {"Transfer": state, "Transferred": str(item.transferred)}
        if isinstance(item, Accumulator):
            return {"Zones": format(item.zones, "b")[::-1] or "0", "Full": "yes" if item.full else "no"}
        if isinstance(item, Sink) and item.capacity:
            return {"Chute": f"{item.count} / {item.capacity}", "Full": "yes" if item.full else "no"}
        if isinstance(item, GuardDoor):
            return {"Door": "closed" if item.closed else "OPEN", "Lock": "locked" if item.locked else "free"}
        if isinstance(item, SafetyCircuit):
            return {"Safety": "OK" if item.ok else ("tripped" if item.tripped else "stop")}
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
        self._update_io()

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
            ("pallet", "Pallets", self.config.pallets),
            ("stopper", "Stoppers", self.config.stoppers),
            ("robot", "Robot Arms", self.config.robots),
            ("elevator", "Elevators", self.config.elevators),
            ("scanner", "Scanners", self.config.scanners),
            ("diverter", "Diverters", self.config.diverters),
            ("turntable", "Turntables", self.config.turntables),
            ("transfer", "Transfers", self.config.transfers),
            ("accumulator", "Accumulation", self.config.accumulators),
            ("door", "Guard Doors", self.config.doors),
            ("safety", "Safety Circuits", self.config.safety),
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

    def sync_sim_buttons(self, playing: bool, theme: str, drawing_shown: bool = True, speed_text: str = "1x"):
        """Start <-> Pause (green / amber), the speed, theme and drawing buttons' wording."""
        if getattr(self, "_shown_playing", None) != playing:
            self._shown_playing = playing
            self.sim_play_btn.set_text("Pause" if playing else "Start")
            self.sim_play_btn.change_object_id("#sim_pause" if playing else "#sim_primary")
        label = "Dark Theme" if theme == "light" else "Light Theme"
        if self.theme_btn.text != label:
            self.theme_btn.set_text(label)
        label = "Hide Drawing" if drawing_shown else "Show Drawing"
        if self.drawing_btn.text != label:
            self.drawing_btn.set_text(label)
        label = f"Speed {speed_text}"
        if self.sim_speed_btn.text != label:
            self.sim_speed_btn.set_text(label)

    def _panel_height(self) -> int:
        return max(200, self.height - CANVAS_TOP - BOTTOM_BAR)

    def handle_resize(self, width: int, height: int):
        self.width = width
        self.height = height
        self.toolbar_panel.set_dimensions((width, TOOLBAR_H))
        self.prop_panel.set_position((width - PANEL_W, CANVAS_TOP))
        self.prop_panel.set_dimensions((PANEL_W, self._panel_height()))
        self.prop_scroll.set_dimensions((PANEL_W - 4, self._panel_height() - 4))
        # Keep the Components window from being stranded off-screen
        if self.manager_window and self.manager_window.alive():
            rect = self.manager_window.rect
            self.manager_window.set_position((max(0, min(rect.x, width - rect.width)),
                                              max(0, min(rect.y, height - rect.height))))

    def handle_event(self, event):
        """Returns a command ("TOGGLE_PLAY", "OPEN_DESIGNER", ("SELECT_ITEM", item),
        ("SET_IO", id, name, value), ...) or None."""
        io = self._io_event(event)
        if io is not None:
            return io
        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            for button, command in ((self.sim_play_btn, "TOGGLE_PLAY"), (self.sim_speed_btn, "CYCLE_SPEED"),
                                    (self.sim_clear_btn, "CLEAR"),
                                    (self.sim_reset_btn, "RESET"), (self.theme_btn, "TOGGLE_THEME"),
                                    (self.manager_btn, "OPEN_MANAGER"), (self.designer_btn, "OPEN_DESIGNER"),
                                    (self.drawing_btn, "TOGGLE_DRAWING"), (self.view3d_btn, "OPEN_3D")):
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
