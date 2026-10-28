"""
scene3d.py - the running simulation as 3D data, for the 3D view (view3d.py
serves it; view3d_web/viewer.js draws it with three.js).

The simulation is 2D seen from above (x, y, heading); this adds the third
dimension from the layout's floor levels and a few standard heights:

  floor of level L         L * levelHeight (the elevators' levelHeight;
                           1.0 m when the layout has none)
  belt top                 floor + BELT_TOP
  a parcel / pallet        bottom on its level's belt top
  a pallet's load          on the pallet deck (PALLET_H above)
  on a travelling lift     on the platform, at the lift's height
  in a gripper             the robot's tool height (lifted between pick
                           and place, from its cycle phase)

so every moving thing goes out with a full 3D pose (x, y, z, yaw) and size.
Machines that will really move things up and down later (mobile robots,
lifting arms) only need to report their z here; the viewer draws whatever
it gets.

  static_scene(config, config_path)  -> the layout: belts as outlines to
                                        extrude, sensors, panels, machines,
                                        the DXF floor plan (sent once)
  dynamic_state(config, views, ...)  -> moving parts and live states (sent
                                        ~30 times a second)
"""
import math
import zlib
from typing import Dict, List, Optional, Sequence

from config_parser import SimulatorConfig

BELT_TOP = 0.75            # m above its floor: a conveyor's working height
BELT_THICKNESS = 0.12
PALLET_H = 0.14            # pallet deck height
PARCEL_H = (0.18, 0.42)    # parcel heights, picked per parcel id (stable)
TOOL_LIFT = 0.5            # a robot lifts its tool this much above pick / place
SHAFT_LEVEL = -1000        # machines.SHAFT_LEVEL: elevator i's riders are on SHAFT_LEVEL - i
MAX_DRAWING_POINTS = 600_000


def level_height(config: SimulatorConfig) -> float:
    for elevator in config.elevators:
        if elevator.levelHeight > 0:
            return float(elevator.levelHeight)
    return 1.0


def surface_z(config: SimulatorConfig, level: int, lh: Optional[float] = None) -> float:
    """Top of the belts on floor `level`."""
    return (lh if lh is not None else level_height(config)) * level + BELT_TOP


def parcel_height(part_id: str) -> float:
    lo, hi = PARCEL_H
    return lo + (zlib.crc32(part_id.encode("utf-8")) % 1000) / 999.0 * (hi - lo)


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
        "buttons": [{"id": b.id, "x": b.x, "y": b.y, "w": b.width, "h": b.height, "rot": b.rotation,
                     "color": b.color, "label": b.label} for b in config.buttons],
        "leds": [{"id": l.id, "x": l.x, "y": l.y, "r": l.radius, "color": l.color, "label": l.label}
                 for l in config.leds],
        "stoppers": [{"id": s.id, "x": s.x, "y": s.y, "w": s.width, "d": s.depth, "rot": s.rotation,
                      "level": s.level, "color": s.color} for s in config.stoppers],
        "robots": [{"id": r.id, "x": r.x, "y": r.y, "reach": r.reach, "level": r.level, "color": r.color,
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

def _smooth(t: float) -> float:
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return t * t * (3.0 - 2.0 * t)


def _pallet_under(views, x: float, y: float) -> bool:
    from math_utils import point_in_polygon
    return any(v.kind == "pallet" and point_in_polygon(x, y, v.points) for v in views)


def tool_z(robot, zp: float, zq: float) -> float:
    """A robot's tool height (the gripped part's bottom) from its phase:
    down at the pick (second half of reaching it), up, across, down at the
    place, up on the way home; `zp` / `zq` = the part's bottom at the pick /
    place point."""
    top = max(zp, zq) + TOOL_LIFT
    t, phase = robot.phaseT, robot.phase
    if phase == "to_pick":
        return top + (zp - top) * _smooth((t - 0.5) / 0.5)
    if phase == "to_place":
        if t < 0.3:
            return zp + (top - zp) * _smooth(t / 0.3)
        if t > 0.7:
            return top + (zq - top) * _smooth((t - 0.7) / 0.3)
        return top
    if phase == "home":
        return zq + (top - zq) * _smooth(t / 0.5)
    return top


def dynamic_state(config: SimulatorConfig, views: Sequence, sim_time: float = 0.0, version: int = 0) -> dict:
    lh = level_height(config)
    lifts = {i: e for i, e in enumerate(config.elevators)}

    def base_z(level: int) -> float:
        if level <= SHAFT_LEVEL:                       # on a travelling lift
            elevator = lifts.get(SHAFT_LEVEL - level)
            return elevator.position * lh + BELT_TOP if elevator is not None else BELT_TOP
        return surface_z(config, level, lh)

    robots: Dict[str, list] = {}
    tools = {}
    for r in config.robots:
        zp = surface_z(config, r.pickLevel, lh) + (PALLET_H if _pallet_under(views, r.pickX, r.pickY) else 0.0)
        zq = surface_z(config, r.placeLevel, lh) + (PALLET_H if _pallet_under(views, r.placeX, r.placeY) else 0.0)
        z = tool_z(r, zp, zq)
        tools[(round(r.toolX, 6), round(r.toolY, 6))] = z
        robots[r.id] = [_r(r.toolX), _r(r.toolY), _r(z), bool(r.holding), 0.0]

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
            z = tools.get((round(v.x, 6), round(v.y, 6)), base_z(v.level))
            for rid, row in robots.items():
                if row[3] and abs(row[0] - v.x) < 1e-3 and abs(row[1] - v.y) < 1e-3:
                    row[4] = _r(h)
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
