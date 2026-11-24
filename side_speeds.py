"""
side_speeds.py - the speed a belt or a wheel diverter needs where it meets a
line at an angle: its speed along that line must be the line's, so it runs

    v = v_line / cos(angle)          (30 degrees: 1.1547 x; 2 m/s -> 2.309 m/s)

  a side join      a belt whose end lies on another belt's side (a side
                   induction's last belt), and every belt end to end before
                   it on the same chain (metering, sync section)
  a branch         a belt that starts on another belt's side (the belt after
                   a wheel diverter), and every belt end to end after it on
                   the same heading
  a wheel diverter its wheels steer at divertAngle on the belt they sit in

The line's speed is its own design speed (or its expected one, when it is a
side belt itself). expected_speeds(config) gives each such belt or diverter
its speed; problems(config) lists the ones that differ (Tools > Check
Layout); the mock PLC (plc_merge.py) runs them at it.
"""
import math
from typing import Dict, List, Tuple

import ax_geometry
from config_parser import SimulatorConfig

MIN_ANGLE_DEG = 5.0       # a join below this is end to end, not at an angle
JOIN_TOL_M = 0.03         # an end within this of another belt's surface or start
SPEED_TOL = 0.02          # a speed within 2 % of the expected one is right


def _wrap(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _on(point, g) -> Tuple[bool, float]:
    """Is the point on line belt g's surface (or its edge): (on, s along it)."""
    (x0, y0, _), (x1, y1, _) = g["start"], g["end"]
    length = math.hypot(x1 - x0, y1 - y0)
    if length < 1e-9:
        return False, 0.0
    ux, uy = (x1 - x0) / length, (y1 - y0) / length
    s = (point[0] - x0) * ux + (point[1] - y0) * uy
    lateral = abs(-(point[0] - x0) * uy + (point[1] - y0) * ux)
    return (-JOIN_TOL_M <= s <= length + JOIN_TOL_M and lateral <= g["width"] / 2.0 + JOIN_TOL_M), s


def _heading(g) -> float:
    (x0, y0, _), (x1, y1, _) = g["start"], g["end"]
    return math.degrees(math.atan2(y1 - y0, x1 - x0))


def expected_speeds(config: SimulatorConfig, line_speeds: Dict[str, float] = None) -> Dict[str, Tuple[float, str]]:
    """Belt or diverter id -> (its speed, why), for those meeting a line at an
    angle. `line_speeds` overrides the design speed of a belt (the mock PLC's
    running speed)."""
    geo = {b["id"]: b for b in ax_geometry.layout_geometry(config)["belts"]}
    belts = {b.id: b for b in config.belts}
    lines = {i: g for i, g in geo.items() if g["shape"] in ("line", "cut") and i in belts}
    own = {b.id: abs(float((line_speeds or {}).get(b.id, b.speed) or 0.0)) for b in config.belts}
    # A wheel deck with no belt under it is a piece of the line itself: a straight carrier along its
    # travel, at the pace of the belt feeding it (a belt after it branches off its side).
    decks = {}
    for d in config.diverters:
        if getattr(d, "diverterType", "") != "wheels":
            continue
        if any(_on((d.x, d.y), g)[0] for g in lines.values()):
            continue                                     # wheels in a belt: the belt is the line
        a = math.radians(d.rotation)
        half = float(d.width) / 2.0
        start = (d.x - half * math.cos(a), d.y - half * math.sin(a), 0.0)
        end = (d.x + half * math.cos(a), d.y + half * math.sin(a), 0.0)
        feeder = next((i for i, g in lines.items() if math.dist(g["end"][:2], start[:2]) <= JOIN_TOL_M
                       and abs(_wrap(g["heading_end"] - d.rotation)) < MIN_ANGLE_DEG), None)
        decks[d.id] = {"id": d.id, "shape": "line", "start": start, "end": end, "width": float(d.height),
                       "heading_start": d.rotation, "heading_end": d.rotation}
        own[d.id] = own[feeder] if feeder is not None else             abs(float(d.speed or 0.0)) * math.cos(math.radians(d.divertAngle))
    lines = {**lines, **decks}
    expected: Dict[str, Tuple[float, str]] = {}

    def speed_of(i):
        return expected[i][0] if i in expected else own[i]

    joins, branches, ends = [], [], []        # (a, b, angle): a meets b
    for a, ga in geo.items():
        if a not in belts:
            continue
        for b, gb in lines.items():
            if b == a:
                continue
            on_end, s_end = _on(ga["end"], gb)
            angle_end = _wrap(ga["heading_end"] - _heading(gb))
            if on_end and abs(angle_end) >= MIN_ANGLE_DEG and s_end > JOIN_TOL_M:
                joins.append((a, b, angle_end))
            on_start, s_start = _on(ga["start"], gb)
            angle_start = _wrap(ga["heading_start"] - _heading(gb))
            if on_start and abs(angle_start) >= MIN_ANGLE_DEG and s_start < math.hypot(
                    gb["end"][0] - gb["start"][0], gb["end"][1] - gb["start"][1]) - JOIN_TOL_M:
                branches.append((a, b, angle_start))
            if math.dist(ga["end"][:2], gb["start"][:2]) <= JOIN_TOL_M and \
                    abs(_wrap(ga["heading_end"] - gb["heading_start"])) < MIN_ANGLE_DEG:
                ends.append((a, b))
    joining = {j[0] for j in joins}
    for _ in range(len(geo) + 2):             # to a fixed point: chains carry the speed along
        before = dict(expected)
        for a, b, angle in joins:
            expected[a] = (speed_of(b) / math.cos(math.radians(angle)), f"joins {b}'s side at {abs(angle):.0f} deg")
        for a, b, angle in branches:
            if a not in expected or "joins" not in expected[a][1]:
                expected[a] = (speed_of(b) / math.cos(math.radians(angle)),
                               f"branches off {b}'s side at {abs(angle):.0f} deg")
        for a, b in ends:                     # a chain: the belt before a side belt, the belt after a branch
            if b in expected and "joins" in expected[b][1] and a not in joining:
                why = expected[b][1] if expected[b][1].startswith("is on the chain") else \
                    f"is on the chain of {b}, which {expected[b][1]}"
                expected[a] = (expected[b][0], why)
            if a in expected and "branches" in expected[a][1] and b not in expected:
                why = expected[a][1] if expected[a][1].startswith("is on the chain") else \
                    f"is on the chain of {a}, which {expected[a][1]}"
                expected[b] = (expected[a][0], why)
        if expected == before:
            break
    for d in config.diverters:
        if getattr(d, "diverterType", "") != "wheels" or abs(d.divertAngle) < MIN_ANGLE_DEG:
            continue
        under = next((i for i, g in lines.items() if i != d.id and _on((d.x, d.y), g)[0]), None)
        if under is None and d.id in decks:
            expected[d.id] = (own[d.id] / math.cos(math.radians(d.divertAngle)),
                              f"steers at {abs(d.divertAngle):.0f} deg, the line's pace along its travel")
        elif under is not None:
            expected[d.id] = (speed_of(under) / math.cos(math.radians(d.divertAngle)),
                              f"steers at {abs(d.divertAngle):.0f} deg on {under}")
    return expected


def problems(config: SimulatorConfig) -> List[Tuple[str, float, float, str]]:
    """(id, its speed, the expected one, why) for each that differs by more than 2 %."""
    actual = {b.id: abs(float(b.speed or 0.0)) for b in config.belts}
    actual.update({d.id: abs(float(d.speed or 0.0)) for d in config.diverters})
    out = []
    for ident, (speed, why) in expected_speeds(config).items():
        if speed > 0 and abs(actual.get(ident, 0.0) - speed) > SPEED_TOL * speed:
            out.append((ident, actual.get(ident, 0.0), speed, why))
    return out


def beyond_drive(config: SimulatorConfig) -> List[Tuple[str, float, float]]:
    """(belt id, the expected speed, its drive's speedMax) where the drive cannot run it."""
    belts = {b.id: b for b in config.belts}
    out = []
    for ident, (speed, _why) in expected_speeds(config).items():
        belt = belts.get(ident)
        top = getattr(getattr(belt, "drive", None), "speedMax", None) if belt is not None else None
        if top and speed > float(top) * (1.0 + 1e-6):
            out.append((ident, speed, float(top)))
    return out
