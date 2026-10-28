"""
scene3d.py - the running simulation as 3D data, for the 3D view (view3d.py
serves it; view3d_web/viewer.js draws it with three.js).

The simulation is 2D seen from above (x, y, heading); this adds the third
dimension from the layout's floor levels and a few standard heights:

  floor of level L         L * levelHeight (heights.py)
  belt top                 floor + BELT_TOP
  a parcel / pallet        bottom on its level's belt top
  a pallet's load          on the pallet deck (PALLET_H above)
  on a travelling lift     on the platform, at the lift's height
  in a gripper             hanging from the robot's tool centre point (the
                           6-axis arm's own TCP height, robot6.py)

so every moving thing goes out with a full 3D pose (x, y, z, yaw) and size.
Machines that will really move things up and down later (mobile robots,
lifting arms) only need to report their z here; the viewer draws whatever
it gets.

  static_scene(config, config_path)  -> the layout: belts as outlines to
                                        extrude, sensors, control panels
                                        (operator buttons and lamps grouped),
                                        machines, the DXF floor plan (once)
  dynamic_state(config, views, ...)  -> moving parts and live states (sent
                                        ~30 times a second)
"""
import math
import re
from typing import Dict, List, Optional, Sequence

import robot6
from config_parser import SimulatorConfig
from heights import BELT_TOP, PALLET_H, PARCEL_H, level_height, parcel_height, surface_z  # noqa: F401 (re-exported)

BELT_THICKNESS = 0.12
SHAFT_LEVEL = -1000        # machines.SHAFT_LEVEL: elevator i's riders are on SHAFT_LEVEL - i
MAX_DRAWING_POINTS = 600_000
PANEL_GAP = 1.0            # m: operator devices this close belong to one control panel,
PANEL_TAG_GAP = 4.0        # and those of one line (label "L1 ...") this close
E_STOP = re.compile(r"e[-_ ]?stop|emergency|estop|not[-_ ]?aus", re.I)


def _r(v: float) -> float:
    return round(float(v), 4)


# ---------------------------------------------------------------------------
# The layout (sent once per layout)
# ---------------------------------------------------------------------------

def _belt_outline(belt):
    from cad.checks import belt_footprint
    poly = belt_footprint(belt)
    if poly.is_empty:
        return None
    if poly.geom_type == "MultiPolygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    outline = [[_r(x), _r(y)] for x, y in list(poly.exterior.coords)[:-1]]
    holes = [[[_r(x), _r(y)] for x, y in list(ring.coords)[:-1]] for ring in poly.interiors]
    return outline, holes


def _drawing_polylines(config: SimulatorConfig, config_path: Optional[str]) -> List[List[float]]:
    if config.background is None or not config.background.visible:
        return []
    try:
        from sim_drawing import DrawingLayer
        layer = DrawingLayer()
        if not layer.load(config.background, config_path):
            return []
    except Exception as exc:        # the 3D view works without its floor plan
        print(f"[3D] No floor plan: {exc}")
        return []
    out, total = [], 0
    for line in layer.lines:
        if total + len(line) > MAX_DRAWING_POINTS:
            break
        out.append([_r(v) for v in line.reshape(-1)])
        total += len(line)
    return out


def _device_style(device, kind: str) -> str:
    """How an operator device looks on a panel."""
    if kind == "led":
        return "lamp"
    text = f"{device.label} {device.id}"
    if E_STOP.search(text) or (getattr(device, "buttonType", "") == "latching" and device.normallyClosed):
        return "estop"
    if getattr(device, "buttonType", "") == "latching":
        return "selector"
    return "button"


def _line_tag(device) -> str:
    """The line a device belongs to by its label ("L2 START" -> "L2"), or ""."""
    words = (device.label or "").split()
    return words[0].upper() if len(words) >= 2 else ""


ROW_ORDER = {"lamp": 0, "button": 1, "selector": 2, "estop": 2}


def control_panels(config: SimulatorConfig) -> List[dict]:
    """The operator buttons and lamps as control panels (consoles). Devices
    within PANEL_GAP of each other share a console, and so do devices of
    one line (same first label word, "L1 START" / "L1 AUTO") within
    PANEL_TAG_GAP. Laid out like a real panel: pilot lamps on the top row,
    pushbuttons below, selectors and the emergency stop at the bottom, each
    row in layout order. The console faces away from the nearest belt: the
    operator stands in the aisle."""
    devices = [(b, "button") for b in config.buttons] + [(l, "led") for l in config.leds]
    if not devices:
        return []
    parent = list(range(len(devices)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, (a, _ka) in enumerate(devices):
        for j in range(i + 1, len(devices)):
            b = devices[j][0]
            d = math.hypot(a.x - b.x, a.y - b.y)
            same_line = _line_tag(a) and _line_tag(a) == _line_tag(b)
            if d <= PANEL_GAP or (same_line and d <= PANEL_TAG_GAP):
                parent[root(i)] = root(j)
    groups: Dict[int, list] = {}
    for i, d in enumerate(devices):
        groups.setdefault(root(i), []).append(d)
    belt_points = [p for belt in config.belts for p in (_belt_outline(belt) or ([], []))[0]]
    panels = []
    for k, members in enumerate(sorted(groups.values(), key=lambda g: (g[0][0].x, g[0][0].y))):
        cx = sum(d.x for d, _ in members) / len(members)
        cy = sum(d.y for d, _ in members) / len(members)
        yaw = -90.0
        if belt_points:
            bx, by = min(belt_points, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
            if math.hypot(cx - bx, cy - by) > 1e-6:
                yaw = math.degrees(math.atan2(cy - by, cx - bx))     # facing the aisle
        rows: Dict[int, list] = {}
        for d, kind in members:                                   # config order within a row
            style = _device_style(d, kind)
            rows.setdefault(ROW_ORDER[style], []).append((d, kind, style))
        laid = []
        for r, key in enumerate(sorted(rows)):
            row = sorted(rows[key], key=lambda m: m[2] == "estop")    # the e-stop at the end
            for c, (d, kind, style) in enumerate(row):
                laid.append({"id": d.id, "kind": kind, "style": style, "color": d.color, "label": d.label or d.id,
                             "row": r, "col": c, "cols": len(row)})
        tags = {_line_tag(d) for d, _ in members if _line_tag(d)}
        panels.append({"id": f"panel_{k + 1}", "x": _r(cx), "y": _r(cy), "yaw": _r(yaw),
                       "title": " / ".join(sorted(tags)) or f"Panel {k + 1}",
                       "rows": len(rows), "cols": max(len(v) for v in rows.values()), "devices": laid})
    return panels


def static_scene(config: SimulatorConfig, config_path: Optional[str] = None, version: int = 0) -> dict:
    lh = level_height(config)
    elevator_belts = {e.beltId: e.id for e in config.elevators if e.beltId}
    belts = []
    xs, ys = [], []
    for belt in config.belts:
        shape = _belt_outline(belt)
        if shape is None:
            continue
        outline, holes = shape
        xs += [p[0] for p in outline]
        ys += [p[1] for p in outline]
        belts.append({
            "id": belt.id, "color": belt.color, "level": belt.level, "outline": outline, "holes": holes,
            "elevator": elevator_belts.get(belt.id),
            # Straight belts get a moving surface texture along this heading.
            "dir": _r(belt.rotation + belt.directionAngle) if belt.type == "linear" and not belt.vertexs else None,
        })
    sensors = [{"id": s.id, "x": s.x, "y": s.y, "w": s.width, "h": s.height, "rot": s.rotation, "level": s.level,
                "type": s.sensorType, "beams": int(s.laserCount or 16), "pitch": float(s.beamDistance or 0.05),
                "label": s.label} for s in config.sensors]
    for group in (config.sources, config.sinks, config.buttons, config.leds, config.stoppers, config.robots,
                  config.elevators, config.pallets):
        for c in group:
            xs.append(c.x)
            ys.append(c.y)
    bounds = [min(xs), min(ys), max(xs), max(ys)] if xs else [-5.0, -5.0, 5.0, 5.0]
    return {
        "version": version,
        "levelHeight": lh, "beltTop": BELT_TOP, "beltThickness": BELT_THICKNESS, "palletHeight": PALLET_H,
        "bounds": bounds,
        "belts": belts,
        "sensors": sensors,
        "sources": [{"id": s.id, "x": s.x, "y": s.y, "w": s.maxWidth, "h": s.maxHeight, "rot": s.rotation,
                     "level": s.level, "label": s.label} for s in config.sources],
        "sinks": [{"id": s.id, "x": s.x, "y": s.y, "w": s.width, "h": s.height, "label": s.label}
                  for s in config.sinks],
        "panels": control_panels(config),
        "stoppers": [{"id": s.id, "x": s.x, "y": s.y, "w": s.width, "d": s.depth, "rot": s.rotation,
                      "level": s.level, "color": s.color} for s in config.stoppers],
        "robots": [{"id": r.id, "x": r.x, "y": r.y, "z": _r(lh * r.level), "level": r.level, "color": r.color,
                    "geometry": robot6.Geometry(r.reach, r.mountHeight, r.toolLength).as_dict(),
                    "pick": [r.pickX, r.pickY, surface_z(config, r.pickLevel, lh)],
                    "place": [r.placeX, r.placeY, surface_z(config, r.placeLevel, lh)], "label": r.label}
                   for r in config.robots],
        "elevators": [{"id": e.id, "x": e.x, "y": e.y, "w": e.width, "h": e.height, "rot": e.rotation,
                       "levels": e.levels, "levelHeight": e.levelHeight, "color": e.color, "beltId": e.beltId,
                       "label": e.label} for e in config.elevators],
        "drawing": _drawing_polylines(config, config_path),
    }


# ---------------------------------------------------------------------------
# The moving parts (sent every frame)
# ---------------------------------------------------------------------------

def dynamic_state(config: SimulatorConfig, views: Sequence, sim_time: float = 0.0, version: int = 0) -> dict:
    lh = level_height(config)
    lifts = {i: e for i, e in enumerate(config.elevators)}

    def base_z(level: int) -> float:
        if level <= SHAFT_LEVEL:                       # on a travelling lift
            elevator = lifts.get(SHAFT_LEVEL - level)
            return elevator.position * lh + BELT_TOP if elevator is not None else BELT_TOP
        return surface_z(config, level, lh)

    # Robots: the six joints (the viewer builds the arm from them) and the
    # gripper's state; a gripped part hangs from the tool centre point.
    robots: Dict[str, list] = {}
    tools = {}
    for r in config.robots:
        tools[(round(r.tcpX, 6), round(r.tcpY, 6))] = r.tcpZ
        robots[r.id] = [_r(r.j1), _r(r.j2), _r(r.j3), _r(r.j4), _r(r.j5), _r(r.j6), bool(r.holding), bool(r.fault)]

    bodies = []
    for v in views:
        p = v.points
        if len(p) < 4:
            continue
        (x0, y0), (x1, y1), (x2, y2) = p[0], p[1], p[2]
        w, l = math.hypot(x1 - x0, y1 - y0), math.hypot(x2 - x1, y2 - y1)
        yaw = math.degrees(math.atan2(y1 - y0, x1 - x0))
        kind, carrier = v.kind, getattr(v, "carrier", "")
        h = PALLET_H if kind == "pallet" else parcel_height(v.id)
        if carrier == "held":
            tcp_z = tools.get((round(v.x, 6), round(v.y, 6)))
            z = tcp_z - h if tcp_z is not None else base_z(v.level)
        elif carrier == "load":
            z = base_z(v.level) + PALLET_H
        else:
            z = base_z(v.level)
        bodies.append([v.id, kind, _r(v.x), _r(v.y), _r(z), _r(yaw), _r(w), _r(l), _r(h), v.color])

    lift_of_belt = {e.beltId: e for e in config.elevators if e.beltId}
    belts = {}
    for b in config.belts:
        elevator = lift_of_belt.get(b.id)
        z = elevator.position * lh + BELT_TOP if elevator is not None else None
        belts[b.id] = [_r(b.actualSpeed), z if z is None else _r(z)]

    def on(value) -> bool:
        if isinstance(value, str):
            return "1" in value
        return bool(value)

    return {
        "t": round(sim_time, 3), "layout": version,
        "bodies": bodies,
        "belts": belts,
        "sensors": {s.id: (s.sensor_states if isinstance(s.sensor_states, str) and len(s.sensor_states) > 1
                           else on(s.sensor_states)) for s in config.sensors},
        "leds": {l.id: bool(l.isOn) for l in config.leds},
        "buttons": {b.id: bool(b.pressed) or (not b.normallyClosed and on(b.sensor_states)) for b in config.buttons},
        "stoppers": {s.id: bool(s.isUp) for s in config.stoppers},
        "robots": robots,
        "elevators": {e.id: [_r(e.position * lh), bool(e.moving)] for e in config.elevators},
    }
