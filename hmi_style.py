"""
hmi_style.py - one look for both windows (the pygame simulator and the Qt
layout editor), in the spirit of an industrial operator panel (ISA-101
"high-performance HMI"): a calm grey canvas, equipment in muted greys,
colour only where it means something (running, blocked, alarm, selection),
one small sans-serif typeface, and labels only where they fit.

Colours are (r, g, b) tuples; qcolor()/hex_() convert for Qt.
"""
import os

# -- palettes -----------------------------------------------------------------
#
# Two themes with the same meaning per colour. set_theme() swaps the module
# globals below, so code that reads hmi.BG etc. at draw time follows it.

THEMES = {
    "dark": {
        "BG": (31, 35, 41),            # canvas
        "GRID": (39, 44, 51),          # minor grid
        "GRID_MAJOR": (47, 53, 61),    # every 5th line
        "DRAWING": (84, 92, 104),      # the layout's DXF background
        "AXIS_X": (62, 92, 70),        # world X axis (y = 0)
        "AXIS_Y": (92, 62, 66),        # world Y axis (x = 0)
        "PANEL": (36, 40, 47),         # side panels, status bar
        "PANEL_DARK": (27, 30, 36),
        "PANEL_BORDER": (56, 62, 71),
        "TEXT": (214, 219, 225),
        "TEXT_DIM": (139, 148, 158),
        "TEXT_FAINT": (98, 106, 116),
        "ACCENT": (56, 152, 214),      # selection, focus
        "RUNNING": (64, 178, 104),     # motor running, sensor clear, OK
        "WARNING": (226, 168, 44),     # paused, lagging
        "ALARM": (222, 76, 64),        # fault, blocked beam, stop
        "BELT_EDGE": (20, 23, 28),
        "FLOW": (236, 239, 242),       # flow chevrons on a running belt
        "SOURCE": (70, 120, 200),
        "SINK": (200, 128, 60),
        "PARCEL_EDGE": (18, 20, 24),
        "TAG_BG": (22, 25, 30, 205),   # label backdrop (with alpha)
        "ON_ACCENT": (255, 255, 255),  # text on an accent / state colour
    },
    "light": {
        "BG": (226, 229, 233),
        "GRID": (215, 219, 224),
        "GRID_MAJOR": (201, 206, 212),
        "DRAWING": (150, 157, 168),
        "AXIS_X": (118, 168, 128),
        "AXIS_Y": (190, 122, 126),
        "PANEL": (244, 245, 247),
        "PANEL_DARK": (232, 234, 237),
        "PANEL_BORDER": (196, 201, 208),
        "TEXT": (31, 35, 41),
        "TEXT_DIM": (84, 92, 103),
        "TEXT_FAINT": (132, 139, 149),
        "ACCENT": (28, 116, 196),
        "RUNNING": (32, 148, 78),
        "WARNING": (196, 132, 18),
        "ALARM": (204, 50, 40),
        "BELT_EDGE": (88, 94, 102),
        "FLOW": (36, 40, 46),
        "SOURCE": (56, 108, 196),
        "SINK": (196, 118, 44),
        "PARCEL_EDGE": (60, 64, 70),
        "TAG_BG": (250, 251, 252, 225),
        "ON_ACCENT": (255, 255, 255),
    },
}
THEME = "dark"
globals().update(THEMES[THEME])


def set_theme(name: str) -> str:
    """Switch every colour to the "dark" or "light" palette."""
    global THEME
    if name not in THEMES:
        raise ValueError(f"unknown theme {name!r} (dark, light)")
    THEME = name
    globals().update(THEMES[name])
    return name


def other_theme() -> str:
    return "light" if THEME == "dark" else "dark"


# -- settings (shared by both windows) ------------------------------------------

def settings_path() -> str:
    """$CONVEYOR_SIM_SETTINGS, else ~/.conveyor_sim/settings.json."""
    return os.environ.get("CONVEYOR_SIM_SETTINGS") or os.path.join(
        os.path.expanduser("~"), ".conveyor_sim", "settings.json")


def load_settings() -> dict:
    import json
    try:
        with open(settings_path(), "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save_settings(**changes) -> None:
    import json
    data = load_settings()
    data.update(changes)
    path = settings_path()
    try:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except OSError:
        pass  # a preference, not worth failing over


def apply_saved_theme() -> str:
    """Use the theme saved last time (dark if none)."""
    theme = load_settings().get("theme", "dark")
    return set_theme(theme if theme in THEMES else "dark")


# -- property panels: one grouping for both windows -------------------------------

# The groups follow how a machine is built, layer by layer (as virtual
# commissioning tools split a digital twin into its physical, electrical and
# automation layers): where it stands, its mechanics, its drive (motor and
# inverter), how it moves, its signals, what it does, how it looks - and
# last, its PLC I/O (the editor's own section).
PROPERTY_GROUP_ORDER = ("Identity", "Layout", "Mechanics", "Devices", "Drive", "Motion", "Signal", "Operation",
                        "Display", "AX / PLC geometry")
PROPERTY_GROUPS = {
    # What it is.
    "id": "Identity", "label": "Identity", "sensorType": "Identity", "diverterType": "Identity",
    "scannerType": "Identity",
    # Where it stands and its footprint.
    "x": "Layout", "y": "Layout", "rotation": "Layout", "level": "Layout",
    "length": "Layout", "beltWidth": "Layout", "mitre.end": "Layout", "mitre.start": "Layout",
    "port.inX": "AX / PLC geometry", "port.inY": "AX / PLC geometry", "port.outX": "AX / PLC geometry",
    "port.outY": "AX / PLC geometry", "cornerRadius": "Layout", "width": "Layout", "height": "Layout",
    "radius": "Layout", "startAngle": "Layout", "endAngle": "Layout", "beltShape": "Layout",
    "beltVertexs": "Layout", "depth": "Layout",
    # The mechanical design: carrying surface, transmission, lift, arm.
    "surface": "Mechanics", "rollerPitch": "Mechanics",
    "drive.gearRatio": "Mechanics", "drive.transmissionRatio": "Mechanics", "drive.drumDiameter": "Mechanics",
    "mass": "Mechanics", "capacity": "Mechanics", "levels": "Mechanics", "levelHeight": "Mechanics",
    "reach": "Mechanics", "pickRadius": "Mechanics", "mountHeight": "Mechanics", "toolLength": "Mechanics",
    "divertAngle": "Mechanics",
    # a wheel deck's wheels: they set its PLC commands (a speed and an angle per row with row control)
    "rowControl": "Mechanics", "wheelRows": "Mechanics", "wheelColumns": "Mechanics",
    "wheelPitchAlong": "Mechanics", "wheelPitchAcross": "Mechanics",
    # The devices built into it: a control panel's buttons, lamps and buzzers.
    "elements": "Devices",
    # The drive: how it is commanded, the motor, the inverter, the encoder.
    "commandFormat": "Drive", "drive.motorPreset": "Drive", "drive.motorRatedSpeed": "Drive",
    "drive.motorRatedFrequency": "Drive", "drive.maxFrequency": "Drive", "drive.rampUpTime": "Drive",
    "drive.rampDownTime": "Drive", "drive.motorMaxSpeed": "Drive", "drive.motorMaxAccel": "Drive", "drive.encoderCountsPerRev": "Drive", "drive.summary": "Drive",
    # How it moves.
    "speed": "Motion", "directionAngle": "Motion", "transition_momentum_duration": "Motion",
    "liftSpeed": "Motion", "cycleTime": "Motion", "actuationTime": "Motion", "speedOverride": "Motion",
    "turnSpeed": "Motion", "crossAngle": "Motion", "liftTime": "Motion",
    # Its signals: what a sensor sees and how it reports it.
    "laserCount": "Signal", "beamDistance": "Signal",
    "onDelay": "Signal", "offDelay": "Signal", "invert": "Signal", "packFormat": "Signal",
    # What it does: production, operator controls, the machine's program and links.
    "interval": "Operation", "minWidth": "Operation", "maxWidth": "Operation", "minHeight": "Operation",
    "maxHeight": "Operation",
    "buttonType": "Operation", "normallyClosed": "Operation", "key": "Operation", "isOn": "Operation",
    "pickX": "Operation", "pickY": "Operation", "placeX": "Operation", "placeY": "Operation",
    "pickLevel": "Operation", "placeLevel": "Operation", "placeTarget": "Operation", "control": "Operation",
    "holdTime": "Operation", "releaseTime": "Operation", "beltId": "Operation", "pickSensorId": "Operation",
    "autoFrom": "Operation", "autoTo": "Operation", "destination": "Operation", "angles": "Operation",
    "stop": "Operation",
    "color": "Display",
}

# Choices shown with words instead of the stored value: {field: {value: text}}.
COMBO_LABELS = {
    "commandFormat": {
        "speed": "Setpoint m/s",
        "vfd": "VFD, m/s",
        "frequency": "Inverter, Hz",
    },
}


# Decimals a number box shows, where 3 (mm) would be noise.
PROPERTY_DECIMALS = {"drive.motorRatedSpeed": 0, "drive.motorRatedFrequency": 1, "drive.maxFrequency": 1,
                     "drive.gearRatio": 2, "drive.transmissionRatio": 2, "drive.rampUpTime": 1,
                     "drive.rampDownTime": 1, "drive.motorMaxSpeed": 0,
                     "drive.motorMaxAccel": 0}


def combo_label(attr: str, value) -> str:
    return COMBO_LABELS.get(attr, {}).get(value, str(value))


def combo_value(attr: str, label: str):
    for value, text in COMBO_LABELS.get(attr, {}).items():
        if text == label:
            return value
    return label


# Row labels the simulator's panel shares with the editor's property grid.
PROPERTY_LABELS = {
    "id": "Id", "label": "Label", "x": "X", "y": "Y", "rotation": "Rotation",
    "length": "Length", "beltWidth": "Width", "mitre.end": "End angle", "mitre.start": "Start angle", "port.inX": "Inlet X",
    "port.inY": "Inlet Y", "port.outX": "Outlet X", "port.outY": "Outlet Y", "width": "Width", "height": "Height",
    "radius": "Radius", "cornerRadius": "Corner radius", "startAngle": "Start angle", "endAngle": "End angle", "beltShape": "Shape",
    "beltVertexs": "Corners", "sensorType": "Type", "laserCount": "Beams", "beamDistance": "Beam pitch",
    "minWidth": "Min width", "maxWidth": "Max width", "minHeight": "Min length", "maxHeight": "Max length",
    "speed": "Speed", "directionAngle": "Direction", "transition_momentum_duration": "Transition",
    "interval": "Interval", "isOn": "On", "color": "Color",
    "level": "Level", "depth": "Depth", "reach": "Reach", "pickRadius": "Pick radius", "mass": "Mass",
    "capacity": "Capacity", "levels": "Levels", "levelHeight": "Level height", "liftSpeed": "Lift speed",
    "cycleTime": "Cycle time", "actuationTime": "Actuation", "pickX": "Pick X", "pickY": "Pick Y",
    "placeX": "Place X", "placeY": "Place Y", "pickLevel": "Pick level", "placeLevel": "Place level",
    "placeTarget": "Place on", "control": "Control", "holdTime": "Hold time", "releaseTime": "Release time",
    "beltId": "Platform belt", "pickSensorId": "Pick sensor", "autoFrom": "Auto from", "autoTo": "Auto to",
    "commandFormat": "Drive type", "drive.motorPreset": "Motor", "drive.summary": "Result",
    "surface": "Surface", "rollerPitch": "Roller pitch",
    "drive.motorRatedSpeed": "Motor speed", "drive.motorRatedFrequency": "Rated frequency",
    "drive.gearRatio": "Gear ratio", "drive.transmissionRatio": "Chain ratio", "drive.drumDiameter": "Drum Ø",
    "drive.maxFrequency": "Max frequency", "drive.rampUpTime": "Ramp up", "drive.rampDownTime": "Ramp down",
    "drive.motorMaxSpeed": "Motor max speed", "drive.motorMaxAccel": "Motor max accel",
    "drive.encoderCountsPerRev": "Encoder/rev",
}


PROPERTY_UNITS = {
    "x": " m", "y": " m", "rotation": "\u00b0", "directionAngle": "\u00b0", "startAngle": "\u00b0",
    "endAngle": "\u00b0", "mitre.end": "\u00b0", "mitre.start": "\u00b0", "port.inX": " m", "port.inY": " m",
    "port.outX": " m", "port.outY": " m", "length": " m", "beltWidth": " m", "width": " m", "height": " m", "radius": " m",
    "beamDistance": " m", "cornerRadius": " m", "minWidth": " m", "maxWidth": " m", "minHeight": " m", "maxHeight": " m",
    "speed": " m/s", "transition_momentum_duration": " s", "interval": " s", "onDelay": " s", "offDelay": " s",
    "depth": " m", "reach": " m", "pickRadius": " m", "levelHeight": " m", "pickX": " m", "pickY": " m",
    "placeX": " m", "placeY": " m", "mass": " kg", "liftSpeed": " m/s", "cycleTime": " s", "actuationTime": " s",
    "speedOverride": " %", "mountHeight": " m", "toolLength": " m",
    "holdTime": " s", "releaseTime": " s",
    "drive.motorRatedSpeed": " rpm", "drive.motorRatedFrequency": " Hz", "drive.drumDiameter": " m",
    "drive.maxFrequency": " Hz", "drive.rampUpTime": " s", "drive.rampDownTime": " s",
    "drive.motorMaxSpeed": " rpm", "drive.motorMaxAccel": " rpm/s",
}


def property_label(attr: str) -> str:
    return PROPERTY_LABELS.get(attr, attr.capitalize())


def property_group(attr: str) -> str:
    return PROPERTY_GROUPS.get(attr, "Display")


def group_sort_key(attr: str) -> int:
    return PROPERTY_GROUP_ORDER.index(property_group(attr))


# -- type ---------------------------------------------------------------------

UI_FONT_FAMILY = "Segoe UI"  # Qt widgets (falls back to the platform sans)
LABEL_PX = 11                # canvas labels, screen pixels, at the default zoom
LABEL_MIN_PX, LABEL_MAX_PX = 9, 14   # ... scaled with the zoom within these
HUD_PX = 13


def bundled_font_path(bold: bool = False) -> str:
    """Noto Sans shipped with pygame_gui (always there, any platform)."""
    import pygame_gui
    name = "NotoSans-Bold.ttf" if bold else "NotoSans-Regular.ttf"
    return os.path.join(os.path.dirname(pygame_gui.__file__), "data", name)


def hex_(rgb) -> str:
    return "#{:02x}{:02x}{:02x}".format(*rgb[:3])


def qcolor(rgb, alpha: int = 255):
    from PySide6.QtGui import QColor
    return QColor(rgb[0], rgb[1], rgb[2], rgb[3] if len(rgb) > 3 else alpha)


def shade(rgb, factor: float):
    """Darker (< 1) or lighter (> 1) version of a colour."""
    if factor < 1:
        return tuple(max(0, int(c * factor)) for c in rgb[:3])
    return tuple(min(255, int(c + (255 - c) * (factor - 1))) for c in rgb[:3])


# -- Qt widget style ----------------------------------------------------------

def qt_stylesheet() -> str:
    """Dark, compact style for the layout editor's widgets."""
    t, d, f = hex_(TEXT), hex_(TEXT_DIM), hex_(TEXT_FAINT)
    p, pd, b, a = hex_(PANEL), hex_(PANEL_DARK), hex_(PANEL_BORDER), hex_(ACCENT)
    return f"""
    QMainWindow, QDialog, QWidget {{ background: {p}; color: {t}; font-family: "{UI_FONT_FAMILY}"; font-size: 9pt; }}
    QMainWindow::separator {{ background: {pd}; width: 3px; height: 3px; }}
    QDockWidget {{ titlebar-close-icon: none; }}
    QDockWidget::title {{ background: {pd}; color: {d}; padding: 4px 8px; text-transform: uppercase;
                          font-size: 8pt; font-weight: 600; letter-spacing: 1px; border-bottom: 1px solid {b}; }}
    QMenuBar {{ background: {pd}; color: {t}; border-bottom: 1px solid {b}; }}
    QMenuBar::item {{ padding: 4px 10px; background: transparent; }}
    QMenuBar::item:selected, QMenu::item:selected {{ background: {a}; color: white; }}
    QMenu {{ background: {pd}; border: 1px solid {b}; padding: 4px 0; }}
    QMenu::item {{ padding: 4px 24px 4px 18px; }}
    QMenu::separator {{ height: 1px; background: {b}; margin: 4px 8px; }}
    QToolBar {{ background: {pd}; border: none; border-bottom: 1px solid {b}; spacing: 1px; padding: 1px 4px; }}
    QToolBar::separator {{ background: {b}; width: 1px; height: 1px; margin: 4px 3px; }}
    QToolButton {{ color: {t}; background: transparent; border: 1px solid transparent; border-radius: 3px;
                   padding: 2px; }}
    QToolButton:hover {{ border-color: {b}; background: {p}; }}
    QToolButton:checked {{ background: {a}; color: white; }}
    QToolButton:disabled, QLabel:disabled {{ color: {f}; }}
    QPushButton {{ background: {pd}; color: {t}; border: 1px solid {b}; border-radius: 3px; padding: 4px 10px; }}
    QPushButton:hover {{ border-color: {a}; }}
    QPushButton:pressed {{ background: {a}; color: white; }}
    QPushButton:disabled {{ color: {f}; border-color: {pd}; }}
    QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{ background: {pd}; color: {t}; border: 1px solid {b};
                   border-radius: 2px; padding: 2px 4px; selection-background-color: {a}; }}
    QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border-color: {a}; }}
    QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {{ color: {f}; }}
    QComboBox QAbstractItemView {{ background: {pd}; border: 1px solid {b}; selection-background-color: {a}; }}
    QListWidget, QTreeWidget, QTableWidget {{ background: {pd}; border: 1px solid {b}; outline: none; }}
    QTableWidget {{ alternate-background-color: {p}; gridline-color: {b}; }}
    QHeaderView::section {{ background: {p}; color: {d}; border: none; border-right: 1px solid {b};
                           border-bottom: 1px solid {b}; padding: 2px 6px; }}
    QListWidget::item {{ padding: 3px 4px; }}
    QListWidget::item:selected {{ background: {a}; color: white; }}
    QListWidget::item:hover:!selected {{ background: {p}; }}
    QGroupBox {{ border: 1px solid {b}; border-radius: 3px; margin-top: 14px; padding-top: 6px; }}
    QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 4px; color: {d}; font-weight: 600; }}
    QCheckBox {{ spacing: 6px; }}
    QCheckBox::indicator, QTableWidget::indicator {{ width: 13px; height: 13px; border: 1px solid {b};
                                                     border-radius: 3px; background: {pd}; }}
    QCheckBox::indicator:hover, QTableWidget::indicator:hover {{ border-color: {a}; }}
    QCheckBox::indicator:checked, QTableWidget::indicator:checked {{ background: {a}; border-color: {a}; }}
    QStatusBar {{ background: {pd}; color: {d}; border-top: 1px solid {b}; }}
    QStatusBar QLabel {{ color: {d}; padding: 0 8px; }}
    QScrollBar:vertical, QScrollBar:horizontal {{ background: {pd}; border: none; width: 10px; height: 10px; }}
    QScrollBar::handle {{ background: {b}; border-radius: 4px; min-height: 24px; min-width: 24px; }}
    QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
    QToolTip {{ background: {pd}; color: {t}; border: 1px solid {b}; padding: 4px; }}
    QGraphicsView {{ border: none; background: {hex_(BG)}; }}
    QSplitter::handle {{ background: {pd}; }}
    """


# -- pygame_gui theme -----------------------------------------------------------------

def pygame_gui_theme() -> dict:
    """The simulator window's widget theme for the current palette."""
    h = hex_
    base = {
        "normal_bg": h(PANEL_DARK), "hovered_bg": h(PANEL), "disabled_bg": h(PANEL_DARK),
        "selected_bg": h(ACCENT), "active_bg": h(ACCENT), "dark_bg": h(PANEL),
        "normal_text": h(TEXT), "hovered_text": h(TEXT), "selected_text": h(ON_ACCENT),
        "active_text": h(ON_ACCENT), "disabled_text": h(TEXT_FAINT),
        "normal_border": h(PANEL_BORDER), "hovered_border": h(ACCENT), "selected_border": h(ACCENT),
        "active_border": h(ACCENT), "disabled_border": h(PANEL_BORDER),
        "filled_bar": h(ACCENT), "unfilled_bar": h(PANEL_DARK), "text_cursor": h(TEXT),
        "link_text": h(ACCENT), "link_hover": h(ACCENT), "link_selected": h(ACCENT),
    }
    flat = {"shape": "rectangle", "border_width": "0", "text_horiz_alignment": "left"}
    return {
        "defaults": {"colours": base, "font": {"name": "noto_sans", "size": "12"},
                     "misc": {"shape": "rounded_rectangle", "shape_corner_radius": "3", "border_width": "1",
                              "shadow_width": "0"}},
        "panel": {"colours": {"dark_bg": h(PANEL), "normal_border": h(PANEL_BORDER)},
                  "misc": {"shape": "rectangle", "border_width": "1", "shadow_width": "0"}},
        "label": {"colours": {"dark_bg": "#00000000", "normal_text": h(TEXT_DIM)},
                  "font": {"name": "noto_sans", "size": "12"}},
        "text_entry_line": {"colours": {"dark_bg": h(PANEL_DARK), "normal_border": h(PANEL_BORDER),
                                        "selected_border": h(ACCENT)},
                            "misc": {"shape": "rectangle", "padding": "4,2"}},
        "window": {"colours": {"dark_bg": h(PANEL), "normal_border": h(PANEL_BORDER)},
                   "misc": {"shape": "rectangle", "shadow_width": "0", "title_bar_height": "24"}},
        "window.#title_bar": {"colours": {"normal_bg": h(PANEL_DARK), "hovered_bg": h(PANEL_DARK),
                                          "active_bg": h(PANEL_DARK), "normal_text": h(TEXT_DIM)},
                              "misc": {"shape": "rectangle", "text_horiz_alignment": "left",
                                       "text_horiz_alignment_padding": "8"}},
        "drop_down_menu": {"misc": {"shape": "rectangle"}},
        "#folder_header": {"colours": {"normal_bg": "#00000000", "hovered_bg": h(PANEL_DARK),
                                       "active_bg": h(PANEL_DARK), "selected_bg": h(PANEL_DARK),
                                       "normal_text": h(TEXT_DIM), "hovered_text": h(TEXT),
                                       "active_text": h(TEXT), "normal_border": "#00000000",
                                       "hovered_border": "#00000000", "active_border": "#00000000"},
                           "font": {"name": "noto_sans", "size": "11", "bold": "1"},
                           "misc": {**flat, "text_horiz_alignment_padding": "4"}},
        "#prop_group": {"colours": {"dark_bg": "#00000000", "normal_text": h(ACCENT)},
                        "font": {"name": "noto_sans", "size": "11", "bold": "1"},
                        "misc": {"text_horiz_alignment": "left"}},
        "#toolbar": {"colours": {"dark_bg": h(PANEL), "normal_border": h(PANEL_BORDER)},
                     "misc": {"shape": "rectangle", "border_width": "1", "shadow_width": "0"}},
        "#tool_group": {"colours": {"dark_bg": "#00000000", "normal_text": h(TEXT_FAINT)},
                        "font": {"name": "noto_sans", "size": "10", "bold": "1"},
                        "misc": {"text_horiz_alignment": "left"}},
        "#prop_kind": {"colours": {"dark_bg": "#00000000", "normal_text": h(TEXT_DIM)},
                       "font": {"name": "noto_sans", "size": "11", "bold": "1"},
                       "misc": {"text_horiz_alignment": "left"}},
        "#prop_name": {"colours": {"dark_bg": "#00000000", "normal_text": h(TEXT_DIM)},
                       "font": {"name": "noto_sans", "size": "12"},
                       "misc": {"text_horiz_alignment": "left"}},
        "#prop_value": {"colours": {"dark_bg": "#00000000", "normal_text": h(TEXT)},
                        "font": {"name": "noto_sans", "size": "12"},
                        "misc": {"text_horiz_alignment": "left"}},
        "#sim_primary": {"colours": {"normal_bg": h(RUNNING), "hovered_bg": h(shade(RUNNING, 1.15)),
                                     "active_bg": h(shade(RUNNING, 0.85)), "normal_text": h(ON_ACCENT),
                                     "hovered_text": h(ON_ACCENT), "normal_border": h(shade(RUNNING, 0.8))},
                         "font": {"name": "noto_sans", "size": "12", "bold": "1"}},
        "#sim_pause": {"colours": {"normal_bg": h(WARNING), "hovered_bg": h(shade(WARNING, 1.15)),
                                   "active_bg": h(shade(WARNING, 0.85)), "normal_text": h(ON_ACCENT),
                                   "hovered_text": h(ON_ACCENT), "normal_border": h(shade(WARNING, 0.8))},
                       "font": {"name": "noto_sans", "size": "12", "bold": "1"}},
        "selection_list.#item_list_item": {"misc": {**flat, "text_horiz_alignment_padding": "6"},
                                           "colours": {"normal_bg": h(PANEL_DARK), "hovered_bg": h(PANEL),
                                                       "selected_bg": h(ACCENT), "normal_text": h(TEXT),
                                                       "selected_text": h(ON_ACCENT)}},
    }
