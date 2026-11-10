"""
control_panel.py - an operator control panel (ControlPanel): one enclosure
whose push buttons, pilot lamps and buzzer are its properties (`elements`),
not components placed one by one.

Each element becomes an ordinary ControlButton / LedIndicator ("child"),
id `<panel id>_<element name>`, laid out in a grid on the panel face and
turned with it. Everything that knows buttons and lamps - the PLC exchange,
the generated ST, the mock PLC's lines, the simulator's clicks and keys -
works with them unchanged. The children are derived: refresh() rebuilds
them from the panels (on load and after an edit), and a saved layout keeps
only the panel with its elements (config_parser.SimulatorConfig).

A buzzer is a lamp-like output (`LedIndicator`, indicator "buzzer"): the
PLC switches it like a lamp; the simulator shows it sounding.
"""
import math
from typing import List, Tuple

from config_parser import ControlButton, ControlPanel, LedIndicator, PanelElement, SimulatorConfig

MAX_COLUMNS = 4


def child_id(panel: ControlPanel, element: PanelElement) -> str:
    return f"{panel.id}_{element.name}"


def _cells(panel: ControlPanel) -> List[Tuple[float, float, float]]:
    """Per element: (local x, local y, cell size) - a grid of up to
    MAX_COLUMNS per row, first row at the top, centred on the panel."""
    n = len(panel.elements)
    if n == 0:
        return []
    columns = min(n, MAX_COLUMNS)
    rows = math.ceil(n / columns)
    cw, ch = panel.width / columns, panel.height / rows
    cells = []
    for i in range(n):
        r, c = divmod(i, columns)
        cells.append((-panel.width / 2.0 + cw * (c + 0.5), panel.height / 2.0 - ch * (r + 0.5), min(cw, ch)))
    return cells


def _to_world(panel: ControlPanel, lx: float, ly: float) -> Tuple[float, float]:
    a = math.radians(panel.rotation)
    return (panel.x + lx * math.cos(a) - ly * math.sin(a), panel.y + lx * math.sin(a) + ly * math.cos(a))


def children(panel: ControlPanel) -> Tuple[List[ControlButton], List[LedIndicator]]:
    """The panel's elements as buttons and lamps, placed on its face."""
    buttons, lamps = [], []
    for element, (lx, ly, cell) in zip(panel.elements, _cells(panel)):
        x, y = _to_world(panel, lx, ly)
        label = element.label or element.name.upper()
        if element.kind == "button":
            size = cell * 0.55
            buttons.append(ControlButton(
                id=child_id(panel, element), x=x, y=y, width=size, height=size, rotation=panel.rotation,
                label=label, color=element.color, buttonType=element.buttonType,
                normallyClosed=element.normallyClosed, key=element.key, packFormat=element.packFormat,
                panel=panel.id))
        else:
            lamps.append(LedIndicator(
                id=child_id(panel, element), x=x, y=y, radius=cell * 0.25, label=label, color=element.color,
                packFormat=element.packFormat, control=panel.control, panel=panel.id,
                indicator="buzzer" if element.kind == "buzzer" else "lamp"))
    return buttons, lamps


def refresh(config: SimulatorConfig) -> None:
    """Rebuild every panel's children in config.buttons / config.leds: the
    old ones (of any panel, also of a deleted one) out, the current ones in,
    keeping the runtime state of those that stay (a held button, a lamp's
    mode)."""
    old_buttons = {b.id: b for b in config.buttons if b.panel}
    old_lamps = {lamp.id: lamp for lamp in config.leds if lamp.panel}
    config.buttons[:] = [b for b in config.buttons if not b.panel]
    config.leds[:] = [lamp for lamp in config.leds if not lamp.panel]
    for panel in config.controlPanels:
        buttons, lamps = children(panel)
        for b in buttons:
            old = old_buttons.get(b.id)
            if old is not None:
                b.sensor_states, b.pressed = old.sensor_states, old.pressed
        for lamp in lamps:
            old = old_lamps.get(lamp.id)
            if old is not None:
                lamp.isOn, lamp.mode = old.isOn, old.mode
        config.buttons.extend(buttons)
        config.leds.extend(lamps)


def default_elements() -> List[PanelElement]:
    """A typical operator panel: START, STOP (NC), RESET; RUN and FAULT
    lamps; a horn."""
    return [
        PanelElement(kind="button", name="start", label="START", color="#16a34a"),
        PanelElement(kind="button", name="stop", label="STOP", color="#dc2626", normallyClosed=True),
        PanelElement(kind="button", name="reset", label="RESET", color="#2563eb"),
        PanelElement(kind="lamp", name="run", label="RUN", color="#22c55e"),
        PanelElement(kind="lamp", name="fault", label="FAULT", color="#ef4444"),
        PanelElement(kind="buzzer", name="horn", label="HORN", color="#f59e0b"),
    ]
