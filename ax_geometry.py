"""
ax_geometry.py - the layout's geometry the way a SIMATIC AX program states it.

The designer places a component by its middle (x, y), a rotation, a length
and a width. The AX network classes (AxDev.Logistics.ConveyorPath, the
carriers) state a carrier by its path instead: where loads enter and leave
it, in 3D, as one straight line or one arc, and every eye or device by how
far along that path it is. This module turns one into the other:

  belt_path(config, belt)      start / end (x, y, z), headings, length; an arc's
                               centre and direction; an outline belt's inlet and
                               outlet: the middles of its start and end edges, or
                               where they were set by hand (Belt.inletPoint /
                               outletPoint), and the line between them
  check(config)                what an AX program would get wrong from this layout:
                               eyes at a joint, a source behind its induction eye,
                               outline ports off the belt or joined to nothing
  along(config, x, y)          the belt under a point, and how far along it (m)
  layout_geometry(config)      every belt, eye, source, sink and device
  st_snippet(config)           ST for Configure: Path.AddLine / AddArc, AddEye

z is the belt top: heights.surface_z (BELT_TOP above the belt's floor level).
Angles are degrees here, CCW from +X as in the designer; the ST uses radians.

    python ax_geometry.py merge_demo.json              a table
    python ax_geometry.py merge_demo.json --st out.st  the ST snippet too
    python ax_geometry.py merge_demo.json --json g.json
"""
import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from typing import List, Optional, Tuple

import heights
from cad.belts import cut_of
from math_utils import get_belt_polygon
from cad.path import belt_world_path, segments
from cad.ports import belt_ports, curved_ccw
from config_parser import Belt, SimulatorConfig, load_config

Point3 = Tuple[float, float, float]


@dataclass
class Segment:
    kind: str                       # "line" or "arc"
    start: Point3
    end: Point3
    length: float                   # m, along the centreline
    centre: Optional[Point3] = None  # an arc's
    ccw: Optional[bool] = None


@dataclass
class BeltPath:
    id: str
    shape: str                      # "line", "arc", "merge" (a cut end), "path" (several segments), "outline"
    start: Point3
    end: Point3
    heading_start: float            # degrees: the travel where loads enter
    heading_end: float              # ... and where they leave
    length: float
    width: float
    segments: List[Segment] = field(default_factory=list)
    end_angle: Optional[float] = None   # a cut belt's end edge, degrees to the travel
    note: str = ""
    start_angle: Optional[float] = None  # ... and its start edge


def _r(v: float, nd: int = 6) -> float:          # 1 um: what AX keeps (LINT um)
    return round(v + 0.0, nd)


def _p(config, belt, x, y) -> Point3:
    return (_r(x), _r(y), _r(heights.surface_z(config, belt.level)))


def belt_path(config: SimulatorConfig, belt: Belt) -> BeltPath:
    """The belt's path as AX states it (see the module doc)."""
    z_of = lambda x, y: _p(config, belt, x, y)               # noqa: E731
    if belt.type == "linear" and (belt.shape == "quadrilateral" or belt.inletPoint or belt.outletPoint):
        # an outline belt - or a straight one whose start / end is set by hand (the designer: by
        # input, drag or a pick): from its inlet to its outlet (outline_ports), a straight line
        (sx, sy), (ex, ey), how = outline_ports(belt)
        h = math.degrees(math.atan2(ey - sy, ex - sx)) % 360.0
        seg = Segment("line", z_of(sx, sy), z_of(ex, ey), _r(math.hypot(ex - sx, ey - sy)))
        cut = cut_of(belt)
        notes = []
        if how != "shape":
            notes.append(f"ports set by hand ({how})")
        if cut is None and belt.shape == "quadrilateral":
            notes.append("an outline belt: inlet / outlet at the middles of its start / end edges")
        flow = (belt.rotation + belt.directionAngle) % 360.0
        if abs(((h - flow + 180.0) % 360.0) - 180.0) > 1.0:
            notes.append(f"its path runs at {h:.1f} deg, its travel at {flow:.1f} deg")
        kind = "cut" if cut else ("outline" if belt.shape == "quadrilateral" else "line")
        return BeltPath(belt.id, kind, seg.start, seg.end, _r(h), _r(h), seg.length,
                        _r(cut["width"] if cut else belt.beltWidth), [seg],
                        _r(cut["end_deg"]) if cut else None, "; ".join(notes),
                        _r(cut["start_deg"]) if cut else None)
    ports = belt_ports(belt)
    if belt.type == "path" and belt.path:
        pts, bulges = belt_world_path(belt)
        segs = []
        for s in segments(pts, bulges, belt.closed):
            (x0, y0), (x1, y1) = s.p0, s.p1
            if s.kind == "arc":
                segs.append(Segment("arc", z_of(x0, y0), z_of(x1, y1), _r(s.length), z_of(*s.center), s.sweep > 0))
            else:
                segs.append(Segment("line", z_of(x0, y0), z_of(x1, y1), _r(s.length)))
        hs = ports.inlet.heading if ports else 0.0
        he = ports.outlet.heading if ports else 0.0
        return BeltPath(belt.id, "path" if len(segs) > 1 else segs[0].kind, segs[0].start, segs[-1].end,
                        _r(hs % 360), _r(he % 360), _r(sum(s.length for s in segs)), _r(belt.beltWidth), segs,
                        note="several segments: one AX carrier each" if len(segs) > 1 else "")
    if ports is None:
        return BeltPath(belt.id, "outline", z_of(belt.x, belt.y), z_of(belt.x, belt.y), 0.0, 0.0, 0.0,
                        _r(belt.beltWidth), note="no inlet / outlet (its flow runs across it)")
    start, end = z_of(ports.inlet.x, ports.inlet.y), z_of(ports.outlet.x, ports.outlet.y)
    if belt.type == "curved":
        sweep = (belt.endAngle - belt.startAngle) % 360.0 or 360.0
        length = belt.radius * math.radians(sweep)
        seg = Segment("arc", start, end, _r(length), z_of(belt.x, belt.y), curved_ccw(belt))
        return BeltPath(belt.id, "arc", start, end, _r(ports.inlet.heading % 360), _r(ports.outlet.heading % 360),
                        seg.length, _r(belt.beltWidth), [seg])
    seg = Segment("line", start, end, _r(belt.length))
    return BeltPath(belt.id, "line", start, end, _r(ports.inlet.heading % 360), _r(ports.outlet.heading % 360),
                    seg.length, _r(belt.beltWidth), [seg])


def to_world(belt: Belt, local) -> Tuple[float, float]:
    r = math.radians(belt.rotation)
    return (belt.x + local[0] * math.cos(r) - local[1] * math.sin(r),
            belt.y + local[0] * math.sin(r) + local[1] * math.cos(r))


def to_local(belt: Belt, x: float, y: float) -> List[float]:
    r = math.radians(belt.rotation)
    dx, dy = x - belt.x, y - belt.y
    return [round(dx * math.cos(r) + dy * math.sin(r), 6), round(-dx * math.sin(r) + dy * math.cos(r), 6)]


CROSS_DEG = 20.0      # an outline edge further than this from the travel is part of a start or end edge


def shape_ports(belt: Belt) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """An outline belt's inlet and outlet from its shape: the middles of its start
    and end edges - the edges across its travel at its back and its front (one
    edge, or a chain of them; a cut edge too). Exact for any cut belt, also one
    drawn by hand; a shape with no clear start or end edge falls back to the line
    through the middle of its width."""
    poly = get_belt_polygon(belt)
    a = math.radians(belt.rotation + belt.directionAngle)
    ux, uy = math.cos(a), math.sin(a)
    n = len(poly)
    u = [x * ux + y * uy for x, y in poly]
    cross = []
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        du, dv = (x1 - x0) * ux + (y1 - y0) * uy, -(x1 - x0) * uy + (y1 - y0) * ux
        cross.append(math.degrees(math.atan2(abs(dv), abs(du))) > CROSS_DEG)
    chains, i0 = [], next((i for i in range(n) if not cross[i]), None)
    if i0 is not None:
        cur = []
        for k in range(1, n + 1):
            i = (i0 + k) % n
            if cross[i]:
                cur.append(i)
            elif cur:
                chains.append(cur)
                cur = []
        if cur:
            chains.append(cur)
    if len(chains) == 2:
        ends = []
        for chain in chains:
            p0, p1 = poly[chain[0]], poly[(chain[-1] + 1) % n]
            mid = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
            ends.append((mid[0] * ux + mid[1] * uy, mid))
        ends.sort()
        return ends[0][1], ends[1][1]
    # no two clear ends: through the middle of its width, from its back to its front
    v = [-x * uy + y * ux for x, y in poly]
    vc = (min(v) + max(v)) / 2.0
    return ((min(u) * ux - vc * uy, min(u) * uy + vc * ux), (max(u) * ux - vc * uy, max(u) * uy + vc * ux))


def outline_ports(belt: Belt):
    """(inlet, outlet, how): each set by hand (Belt.inletPoint / outletPoint, in the
    belt's own frame) or from its shape (shape_ports); how = "shape", "inlet",
    "outlet" or "inlet, outlet"."""
    auto_in, auto_out = shape_ports(belt)
    inlet = to_world(belt, belt.inletPoint) if belt.inletPoint else auto_in
    outlet = to_world(belt, belt.outletPoint) if belt.outletPoint else auto_out
    how = ", ".join(n for n, v in (("inlet", belt.inletPoint), ("outlet", belt.outletPoint)) if v) or "shape"
    return inlet, outlet, how


def _along_segment(seg: Segment, x: float, y: float) -> Tuple[float, float]:
    """(s from the segment's start, signed offset across: + left of the travel), metres."""
    (x0, y0, _), (x1, y1, _) = seg.start, seg.end
    if seg.kind == "line":
        L = math.hypot(x1 - x0, y1 - y0) or 1e-12
        ux, uy = (x1 - x0) / L, (y1 - y0) / L
        return (x - x0) * ux + (y - y0) * uy, -(x - x0) * uy + (y - y0) * ux
    cx, cy, _ = seg.centre
    r = math.hypot(x0 - cx, y0 - cy)
    a0, a = math.atan2(y0 - cy, x0 - cx), math.atan2(y - cy, x - cx)
    d = (a - a0) % (2 * math.pi) if seg.ccw else (a0 - a) % (2 * math.pi)
    radial = math.hypot(x - cx, y - cy) - r
    return r * d, (-radial if seg.ccw else radial)


def along(config: SimulatorConfig, x: float, y: float, paths: Optional[List[BeltPath]] = None):
    """(belt id, s along its path in m, offset across in m) of the belt under (x, y), or None."""
    paths = paths if paths is not None else [belt_path(config, b) for b in config.belts]
    best = None
    for path in paths:
        acc = 0.0
        for seg in path.segments:
            s, off = _along_segment(seg, x, y)
            if -1e-6 <= s <= seg.length + 1e-6 and abs(off) <= path.width / 2.0 + 1e-6:
                # on a joint (the end of one belt, the start of the next): the belt it
                # ends, as ON_JOINT says - decided here, not by the offset's rounding
                at_end = abs(path.length - (acc + s)) <= ON_JOINT
                cand = (_r(abs(off), 6), not at_end, path.id, _r(acc + s, 6), _r(off, 6))
                if best is None or cand < best:
                    best = cand
            acc += seg.length
    return None if best is None else best[2:]


def layout_geometry(config: SimulatorConfig) -> dict:
    """Every belt's path, and every placed device with the belt under it."""
    paths = [belt_path(config, b) for b in config.belts]
    devices = []
    for attr, kind in (("sensors", "eye"), ("sources", "source"), ("sinks", "sink"), ("scanners", "scanner"),
                       ("diverters", "diverter"), ("stoppers", "stopper")):
        for d in getattr(config, attr, []):
            on = along(config, d.x, d.y, paths)
            what = "banner" if kind == "eye" and getattr(d, "sensorType", "ir") == "laser_banner" else kind
            entry = {"id": d.id, "kind": what, "x": _r(d.x), "y": _r(d.y),
                     "z": _r(heights.surface_z(config, getattr(d, "level", 0))),
                     "belt": on[0] if on else None, "s": on[1] if on else None,
                     "lateral": on[2] if on else None}
            if kind == "diverter":                          # its zone along its own belt, begin to end
                begin, end, _how = zone_ends(d)
                own = [p for p in paths if on and p.id == on[0]] or paths
                for name, (x, y) in (("begin", begin), ("end", end)):
                    at = along(config, x, y, own)
                    entry[f"{name}_s"] = at[1] if at else None
            devices.append(entry)
    for panel in getattr(config, "controlPanels", []):
        devices.append({"id": panel.id, "kind": "panel", "x": _r(panel.x), "y": _r(panel.y), "z": 1.2,
                        "belt": None, "s": None, "lateral": None})
    return {"belts": [asdict(p) for p in paths], "devices": devices}


JOINT_MARGIN = 0.05    # m: an eye this close to its belt's start or end sits on a joint
ON_JOINT = 0.001       # m: an eye this close to its belt's end is AT the joint, on purpose: it belongs to that belt
PORT_TOLERANCE = 0.005  # m: a port this far off its belt's outline, or from the belt it joins, is off


def check(config: SimulatorConfig, only: Optional[str] = None) -> List[str]:
    """What an AX program built from this layout would get wrong (`only`: the
    problems about that component)."""
    from shapely.geometry import Point, Polygon
    paths = {b.id: belt_path(config, b) for b in config.belts}
    plist = list(paths.values())
    out = []
    # eyes: on a belt, clear of its ends (an eye on a joint belongs to neither)
    devices = layout_geometry(config)["devices"]
    for d in devices:
        if d["kind"] not in ("eye", "banner"):
            continue
        if not d["belt"]:
            out.append((d["id"], f"{d['id']} is over no belt"))
            continue
        L = paths[d["belt"]].length
        if abs(L - d["s"]) <= ON_JOINT:
            continue                    # exactly at its belt's end: a gate where the next belt begins (deliberate)
        if d["s"] < JOINT_MARGIN or L - d["s"] < JOINT_MARGIN:
            out.append((d["id"], f"{d['id']} sits on a joint ({d['s']:.3f} m along {d['belt']}, "
                                 f"{L:.3f} m long): place it clearly on one belt"))
    # sources: their loads must pass the belt's first eye (the induction eye)
    for src in [d for d in devices if d["kind"] == "source" and d["belt"]]:
        eyes = [d for d in devices if d["kind"] in ("eye", "banner") and d["belt"] == src["belt"]]
        if eyes and all(e["s"] < src["s"] for e in eyes):
            out.append((src["id"], f"{src['id']} drops loads at {src['s']:.3f} m on {src['belt']}, after its eye(s) "
                                   f"{', '.join(e['id'] for e in eyes)}: they are never seen, never created"))
        elif eyes:
            first = min((e for e in eyes if e["s"] > src["s"]), key=lambda e: e["s"])
            behind = [e["id"] for e in eyes if e["s"] < src["s"]]
            if behind:
                out.append((src["id"], f"{src['id']} drops loads after {', '.join(behind)}: the first eye they "
                                       f"pass is {first['id']}"))
    # outline belts: ports on the belt, and joined to a belt (or onto a belt's side)
    belts = {b.id: b for b in config.belts}
    for b in config.belts:
        if not (b.type == "linear" and b.shape == "quadrilateral"):
            continue
        poly = Polygon(get_belt_polygon(b))
        path = paths[b.id]
        for name, p in (("inlet", path.start), ("outlet", path.end)):
            off = poly.exterior.distance(Point(p[0], p[1]))
            if off > PORT_TOLERANCE:
                out.append((b.id, f"{b.id}: its {name} ({p[0]:.3f}, {p[1]:.3f}) is {off * 1000:.0f} mm off its outline"))
            joined = False
            for other in plist:
                if other.id == b.id:
                    continue
                q = other.end if name == "inlet" else other.start
                if math.dist(p[:2], q[:2]) <= PORT_TOLERANCE:
                    joined = True
                    break
                # onto (or off) another belt's side: the port on that belt's edge
                ob = belts[other.id]
                edge = Polygon(get_belt_polygon(ob)).exterior if not (ob.type == "curved" or ob.type == "path") else None
                if edge is not None and edge.distance(Point(p[0], p[1])) <= PORT_TOLERANCE:
                    joined = True
                    break
            if not joined:                               # onto / off a wheel deck (a carrier of its own)
                for d in config.diverters:
                    if d.diverterType != "wheels":
                        continue
                    a = math.radians(d.rotation)
                    dx, dy = p[0] - d.x, p[1] - d.y
                    u, v = dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)
                    if abs(u) <= d.width / 2 + PORT_TOLERANCE and abs(v) <= d.height / 2 + PORT_TOLERANCE:
                        joined = True
                        break
            if not joined and name == "outlet":          # into a chute / sink
                joined = any(abs(p[0] - s.x) <= s.width / 2 + PORT_TOLERANCE and abs(p[1] - s.y) <= s.height / 2 + PORT_TOLERANCE
                             for s in config.sinks)
            if not joined and not any(d["belt"] == b.id and d["kind"] == "source" for d in devices):
                out.append((b.id, f"{b.id}: its {name} ({p[0]:.3f}, {p[1]:.3f}) meets no belt's "
                                  f"{'outlet' if name == 'inlet' else 'inlet'} and no belt's edge"))
    return [text for who, text in out if only is None or who == only]


# -- ST ----------------------------------------------------------------------------------------

def _st_point(p: Point3) -> str:
    return f"Point3Of(LREAL#{p[0]:.6f}, LREAL#{p[1]:.6f}, LREAL#{p[2]:.6f})"


def st_path_lines(path: BeltPath, name: Optional[str] = None) -> List[str]:
    """`name.Path.AddLine(...)` / `AddArc(...)` for one belt (one call per segment)."""
    name = name or path.id
    out = []
    for k, seg in enumerate(path.segments):
        target = name if len(path.segments) == 1 else f"{name}_{k + 1}"
        if seg.kind == "line":
            out.append(f"{target}.Path.AddLine(startPoint := {_st_point(seg.start)},\n"
                       f"    endPoint := {_st_point(seg.end)});")
        else:
            out.append(f"{target}.Path.AddArc(startPoint := {_st_point(seg.start)},\n"
                       f"    endPoint := {_st_point(seg.end)},\n"
                       f"    center := {_st_point(seg.centre)}, ccw := {'TRUE' if seg.ccw else 'FALSE'});")
    if path.width:
        out.append(f"{name}.Width_m := REAL#{path.width:g};")
    return out


def st_snippet(config: SimulatorConfig) -> str:
    """The positions part of an AX Configure: each belt's path and width, each
    eye on the belt under it (s along its path, in um)."""
    geo = layout_geometry(config)
    paths = {b.id: b for b in (belt_path(config, b) for b in config.belts)}
    lines = ["// Generated by ax_geometry.py from the layout: positions only (paths, widths, eyes).",
             "// Paste into Configure; links, kinds, leads and drives are the program's own."]
    for path in paths.values():
        lines.append(f"// {path.id}: {path.shape}, {path.length:.3f} m, heading {path.heading_start:g} -> "
                     f"{path.heading_end:g} deg" + (f", cut: start edge {path.start_angle:g} deg, end edge "
                                                    f"{path.end_angle:g} deg" if path.end_angle else "")
                     + (f" ({path.note})" if path.note else ""))
        lines += st_path_lines(path)
    for problem in check(config):
        lines.append(f"// CHECK: {problem}")
    for d in geo["devices"]:
        if d["kind"] == "eye" and d["belt"]:
            lines.append(f"{d['belt']}.AddEye(s_um := LINT#{round(d['s'] * 1e6)}, use := EyeUse#Occupied);"
                         f"   // {d['id']}")
        elif d["kind"] == "banner" and d["belt"]:
            s = next(x for x in config.sensors if x.id == d["id"])
            lines.append(f"{d['belt']}.AddEye(s_um := LINT#{round(d['s'] * 1e6)}, use := EyeUse#Presence);"
                         f"   // {d['id']}: a laser banner ({s.laserCount or 16} beams, {s.beamDistance or 0.05:g} m"
                         f" apart): its any-beam signal; its beams go to a measuring station")
        elif d["kind"] in ("eye", "banner"):
            lines.append(f"// {d['id']}: not over a belt ({d['x']}, {d['y']})")
    return "\n".join(lines) + "\n"


def describe(config: SimulatorConfig, belt: Belt) -> str:
    """A few lines for the designer's property panel."""
    p = belt_path(config, belt)
    rows = [f"start ({p.start[0]:.3f}, {p.start[1]:.3f}, {p.start[2]:.3f})",
            f"end   ({p.end[0]:.3f}, {p.end[1]:.3f}, {p.end[2]:.3f})",
            f"heading {p.heading_start:g}° → {p.heading_end:g}°, {p.length:.3f} m {p.shape}"]
    for seg in p.segments:
        if seg.kind == "arc":
            rows.append(f"centre ({seg.centre[0]:.3f}, {seg.centre[1]:.3f}), {'CCW' if seg.ccw else 'CW'}")
    if p.end_angle is not None:
        rows.append(f"cut belt: start edge {p.start_angle:g}\u00b0, end edge {p.end_angle:g}\u00b0 to the travel")
    if p.note:
        rows.append(p.note)
    paths = [belt_path(config, b) for b in config.belts if b.id != belt.id]
    feeds = [o.id for o in paths if math.dist(o.end[:2], p.start[:2]) <= PORT_TOLERANCE]
    leads = [o.id for o in paths if math.dist(o.start[:2], p.end[:2]) <= PORT_TOLERANCE]
    if feeds or leads:
        rows.append("joins: " + ", ".join([f"{i} \u2192 here" for i in feeds] + [f"here \u2192 {i}" for i in leads]))
    rows += [f"\u26a0 {text}" for text in check(config, only=belt.id)]
    return "\n".join(rows)


def zone_ends(model) -> Tuple[Tuple[float, float], Tuple[float, float], str]:
    """A diverter's zone for the AX program: (begin, end, how) in world metres - each set by
    hand (Diverter.axStart / axEnd) or the ends of its box along its travel (`width` along
    `rotation`); how = "box", "begin", "end" or "begin, end"."""
    a = math.radians(getattr(model, "rotation", 0.0))
    half = getattr(model, "width", 0.0) / 2.0
    box_begin = (model.x - half * math.cos(a), model.y - half * math.sin(a))
    box_end = (model.x + half * math.cos(a), model.y + half * math.sin(a))
    start = tuple(model.axStart) if getattr(model, "axStart", None) else box_begin
    end = tuple(model.axEnd) if getattr(model, "axEnd", None) else box_end
    how = ", ".join(n for n, v in (("begin", getattr(model, "axStart", None)),
                                   ("end", getattr(model, "axEnd", None))) if v) or "box"
    return start, end, how


def describe_point(config: SimulatorConfig, model) -> str:
    on = along(config, model.x, model.y)
    z = heights.surface_z(config, getattr(model, "level", 0))
    text = f"at ({model.x:.3f}, {model.y:.3f}, {z:.3f})"
    if on:
        text += f"\non {on[0]}: {on[1]:.3f} m along, {on[2]:+.3f} m across"
    if hasattr(model, "axStart"):                           # a diverter: its zone, begin to end
        begin, end, how = zone_ends(model)
        own = [belt_path(config, b) for b in config.belts if on and b.id == on[0]]    # along its own belt
        for name, (x, y) in (("begin", begin), ("end", end)):
            at = along(config, x, y, own or None)
            text += f"\nzone {name} ({x:.3f}, {y:.3f})" + (f": {at[0]} {at[1]:.3f} m" if at else "")
        if how != "box":
            text += f"\n(set by hand: {how})"
    problems = check(config, only=model.id)
    return "\n".join([text] + [f"\u26a0 {x}" for x in problems])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("config")
    ap.add_argument("--st", help="write the ST snippet here")
    ap.add_argument("--json", help="write the geometry as JSON here")
    args = ap.parse_args(argv)
    config = load_config(args.config)
    geo = layout_geometry(config)
    print(f"{'belt':<14} {'shape':<7} {'start (x, y, z)':<26} {'end (x, y, z)':<26} {'heading':>13} {'length':>7}")
    for b in geo["belts"]:
        s, e = b["start"], b["end"]
        print(f"{b['id']:<14} {b['shape']:<7} ({s[0]:7.3f}, {s[1]:7.3f}, {s[2]:5.3f})  "
              f"({e[0]:7.3f}, {e[1]:7.3f}, {e[2]:5.3f})  {b['heading_start']:5g} -> {b['heading_end']:<5g} "
              f"{b['length']:7.3f}")
    print(f"\n{'device':<16} {'kind':<9} {'x':>7} {'y':>7}  on belt        s (m)")
    for d in geo["devices"]:
        print(f"{d['id']:<16} {d['kind']:<9} {d['x']:7.3f} {d['y']:7.3f}  {d['belt'] or '-':<14} "
              f"{'' if d['s'] is None else format(d['s'], '.3f')}")
    problems = check(config)
    print(f"\n{len(problems)} check(s):" if problems else "\nchecks: none")
    for text in problems:
        print(f"  {text}")
    if args.st:
        with open(args.st, "w", encoding="utf-8") as f:
            f.write(st_snippet(config))
        print(f"\nwrote {args.st}")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(geo, f, indent=2)
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
