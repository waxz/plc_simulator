"""
logistics_draw.py - the logistics equipment (logistics.py) in the
simulator window (visualizer.Renderer): scanners, diverters, turntables,
accumulation zones, chutes, guard doors and safety circuits, drawn from
their live state; and which of them is under the mouse.
"""
import math
from typing import Optional

import pygame

import equipment_geometry as geo
import hmi_style as hmi
from logistics import divert_angle
from math_utils import get_rect_vertices, point_in_polygon


def _color(renderer, hex_color):
    return renderer._hex_to_rgb(hex_color)


def _dashed(screen, color, a, b, dash=6):
    (x0, y0), (x1, y1) = a, b
    length = math.hypot(x1 - x0, y1 - y0)
    if length < 1:
        return
    n = max(1, int(length // dash))
    for i in range(0, n, 2):
        t0, t1 = i / n, min(1.0, (i + 1) / n)
        pygame.draw.line(screen, color, (x0 + (x1 - x0) * t0, y0 + (y1 - y0) * t0),
                         (x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1), 1)


def _polygon_dashed(screen, color, pts):
    for a, b in zip(pts, pts[1:] + pts[:1]):
        _dashed(screen, color, a, b)


def draw_logistics(r) -> None:
    c = r.config
    screen = r.screen
    for sink in c.sinks:
        if sink.capacity:
            _draw_chute(r, sink)
    for acc in c.accumulators:
        _draw_accumulator(r, acc)
    for table in c.turntables:
        _draw_turntable(r, table)
    for t in c.transfers:
        _draw_transfer(r, t)
    for d in c.diverters:
        _draw_diverter(r, d)
    for s in c.scanners:
        _draw_scanner(r, s)
    for door in c.doors:
        _draw_door(r, door)
    for circuit in c.safety:
        _draw_safety(r, circuit)
    # PLC values are not drawn: the designer's Exchange Table lists them, the tag
    # monitors and the 3D page's charts show them
    for w in getattr(c, "watchdogs", []):
        _draw_watchdog(r, w)
    _ = screen


def watchdog_text(w) -> str:
    state = "TRIPPED" if w.tripped else ("PLC alive" if w.plcAlive else "waiting for the PLC")
    return f"{w.label or w.id}: {state}  ({w.trips} trips)"


def _draw_watchdog(r, w) -> None:
    """The PLC link's watchdog: green while the PLC's heartbeat moves, red
    while tripped (the PLC's commands at rest), grey before the first beat."""
    color = hmi.ALARM if w.tripped else (_color(r, w.color) if w.plcAlive else hmi.TEXT_FAINT)
    pts = [r._world_to_screen(x, y) for x, y in get_rect_vertices(w.x, w.y, w.width, w.height, 0.0)]
    overlay = pygame.Surface(r.screen.get_size(), pygame.SRCALPHA)
    pygame.draw.polygon(overlay, (*color, 50), pts)
    r.screen.blit(overlay, (0, 0))
    pygame.draw.polygon(r.screen, color, pts, 2)
    cx = sum(p[0] for p in pts) / 4
    cy = sum(p[1] for p in pts) / 4
    r._queue_label(watchdog_text(w), (cx, cy), r.PRIORITY_ALARM if w.tripped else r.PRIORITY_DEVICE,
                   hmi.ALARM if w.tripped else hmi.TEXT, force=True)


def format_plc_value(v) -> str:
    """A PLC value as shown: its label, the value to its decimals, its unit."""
    try:
        number = f"{float(v.value):.{max(0, min(6, int(v.decimals)))}f}"
    except (TypeError, ValueError):
        number = str(v.value)
    return f"{v.label or v.id}  {number}{(' ' + v.unit) if v.unit else ''}"


def _draw_plc_value(r, v) -> None:
    """A readout of a value the PLC shows (config_parser.PlcValue): a tinted
    box with its text; it drives nothing."""
    color = _color(r, v.color)
    pts = [r._world_to_screen(x, y) for x, y in get_rect_vertices(v.x, v.y, v.width, v.height, v.rotation)]
    overlay = pygame.Surface(r.screen.get_size(), pygame.SRCALPHA)
    pygame.draw.polygon(overlay, (*color, 45), pts)
    r.screen.blit(overlay, (0, 0))
    _polygon_dashed(r.screen, color, pts)
    cx = sum(p[0] for p in pts) / 4
    cy = sum(p[1] for p in pts) / 4
    r._queue_label(format_plc_value(v), (cx, cy), r.PRIORITY_DEVICE, hmi.TEXT, force=True)


def draw_rollers(r, belt) -> None:
    """A roller conveyor: a line per roller across it - along radii on a
    curve (every second / fourth one when zoomed out)."""
    if belt.type == "curved":
        _draw_arc_rollers(r, belt)
        return
    a = math.radians(belt.rotation)
    ca, sa = math.cos(a), math.sin(a)
    pitch = max(0.02, belt.rollerPitch)
    step = 1
    while r._scale_val(pitch * step) < 5 and step < 64:
        step *= 2
    n = int(belt.length / pitch)
    hw = belt.beltWidth / 2.0 * 0.92
    color = hmi.shade(r._hex_to_rgb(belt.color), 1.45)
    for i in range(0, n + 1, step):
        u = -belt.length / 2.0 + (i + 0.5) * pitch
        if u > belt.length / 2.0:
            break
        cx, cy = belt.x + u * ca, belt.y + u * sa
        p0 = r._world_to_screen(cx - hw * -sa, cy - hw * ca)
        p1 = r._world_to_screen(cx + hw * -sa, cy + hw * ca)
        pygame.draw.line(r.screen, color, p0, p1, 1)


def _draw_arc_rollers(r, belt) -> None:
    from cad.ports import arc_angles
    start, end = arc_angles(belt)
    pitch = max(0.02, belt.rollerPitch)
    n = max(1, int(math.radians(end - start) * belt.radius / pitch))
    step = 1
    while r._scale_val(pitch * step) < 5 and step < 64:
        step *= 2
    r0, r1 = belt.radius - belt.beltWidth / 2.0 * 0.92, belt.radius + belt.beltWidth / 2.0 * 0.92
    color = hmi.shade(r._hex_to_rgb(belt.color), 1.45)
    for i in range(0, n, step):
        a = math.radians(start + (end - start) * (i + 0.5) / n)
        c, s = math.cos(a), math.sin(a)
        pygame.draw.line(r.screen, color, r._world_to_screen(belt.x + r0 * c, belt.y + r0 * s),
                         r._world_to_screen(belt.x + r1 * c, belt.y + r1 * s), 1)


def _draw_transfer(r, t) -> None:
    """A right-angle transfer: its chain strands across the travel (bright
    while lifted) and an arrow to the side it transfers to."""
    color = _color(r, t.color)
    pts = r._rect_screen(t.x, t.y, t.width, t.height, t.rotation)
    a = math.radians(t.rotation)
    ca, sa = math.cos(a), math.sin(a)
    c = math.radians(t.rotation + t.crossAngle)
    strand = hmi.WARNING if t.up else hmi.shade(color, 0.8)
    for k in (-0.3, 0.0, 0.3):
        u = k * t.width
        cx, cy = t.x + u * ca, t.y + u * sa
        hw = t.height / 2.0 * 0.95
        p0 = r._world_to_screen(cx - hw * math.cos(c), cy - hw * math.sin(c))
        p1 = r._world_to_screen(cx + hw * math.cos(c), cy + hw * math.sin(c))
        pygame.draw.line(r.screen, strand, p0, p1, 3 if t.up else 2)
    pygame.draw.polygon(r.screen, color, pts, 1)
    x0, y0 = r._world_to_screen(t.x, t.y)
    reach = min(t.width, t.height) * 0.42
    x1, y1 = r._world_to_screen(t.x + reach * math.cos(c), t.y + reach * math.sin(c))
    pygame.draw.line(r.screen, strand, (x0, y0), (x1, y1), 2)
    head = math.atan2(y1 - y0, x1 - x0)
    for side in (-0.5, 0.5):
        pygame.draw.line(r.screen, strand, (x1, y1), (x1 - 7 * math.cos(head + side), y1 - 7 * math.sin(head + side)), 2)


def _draw_scanner(r, s) -> None:
    color = _color(r, s.color)
    pts = r._rect_screen(s.x, s.y, s.width, s.height, s.rotation)
    pygame.draw.polygon(r.screen, color, pts, 2)
    # The read line across the belt: lit while a parcel is in the zone.
    mid_a = ((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2)
    mid_b = ((pts[2][0] + pts[3][0]) / 2, (pts[2][1] + pts[3][1]) / 2)
    pygame.draw.line(r.screen, hmi.ALARM if s.present else hmi.shade(color, 0.6), mid_a, mid_b, 2 if s.present else 1)
    kind = {"barcode": "SCAN", "rfid": "RFID", "dws": "DWS"}[s.scannerType]
    if s.readCount == 0:
        text, tint = f"{s.label or kind}", hmi.TEXT_DIM
    elif s.noRead:
        text, tint = f"{s.label or kind}  NO READ", hmi.ALARM
    else:
        text, tint = f"{s.label or kind}  {s.code} -> {s.destination}", hmi.TEXT
        if s.scannerType == "dws":
            text += f"  {s.weight:.1f} kg"
    cx = sum(p[0] for p in pts) / 4
    r._queue_label(text, (cx, max(p[1] for p in pts) + 2), r.PRIORITY_DEVICE, tint, align="below")


def _draw_diverter(r, d) -> None:
    color = _color(r, d.color)
    pts = r._rect_screen(d.x, d.y, d.width, d.height, d.rotation)
    angle = divert_angle(d)                 # where it is told to send them: commandAngle
    if d.active:
        overlay = pygame.Surface(r.screen.get_size(), pygame.SRCALPHA)
        pygame.draw.polygon(overlay, (*color, 70), pts)
        r.screen.blit(overlay, (0, 0))
        pygame.draw.polygon(r.screen, color, pts, 2)
    else:
        _polygon_dashed(r.screen, color, pts)
    # The machine itself (equipment_geometry.py): the pusher beside the belt,
    # its plate across while active; or the wheel module's wheels, where
    # they point now (swivelling to the divert angle while active).
    rot = math.radians(d.rotation)
    ca, sa = math.cos(rot), math.sin(rot)

    def screen(lx, ly):
        return r._world_to_screen(d.x + lx * ca - ly * sa, d.y + lx * sa + ly * ca)

    def rect(cx, cy, sx, sy):
        return [screen(cx + u * sx / 2, cy + v * sy / 2) for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1))]

    px_per_m = math.dist(screen(0, 0), screen(1, 0))
    if d.diverterType == "pusher":
        parts = geo.pusher_parts(d.width, d.height, d.divertAngle, 1.0 if d.active else 0.0)
        body = rect(*parts.body)
        pygame.draw.polygon(r.screen, (58, 64, 73), body)
        pygame.draw.polygon(r.screen, hmi.TEXT_DIM, body, 1)
        for a, b in parts.rods:
            pygame.draw.line(r.screen, (154, 163, 173), screen(*a), screen(*b), max(1, int(0.022 * px_per_m)))
        plate = rect(*parts.plate)
        pygame.draw.polygon(r.screen, color, plate)
        pygame.draw.polygon(r.screen, hmi.shade(color, 0.5), plate, 1)
    elif px_per_m >= 25:                                   # wheels too small to see further out
        steer = math.radians(d.wheelAngle)
        dx, dy = geo.WHEEL_R * math.cos(steer), geo.WHEEL_R * math.sin(steer)
        wheel = hmi.shade(color, 0.35) if d.active else (43, 48, 56)   # dark on the lit deck
        width = max(1, int(geo.WHEEL_T * px_per_m))
        rows = list(getattr(d, "rowWheelAngles", None) or [])
        us, vs = geo.grid_of(d)
        for i, x in enumerate(us):
            if rows and i < len(rows):                     # a deck of rows: each row points its own way
                a = math.radians(rows[i])
                dx, dy = geo.WHEEL_R * math.cos(a), geo.WHEEL_R * math.sin(a)
            for y in vs:
                pygame.draw.line(r.screen, wheel, screen(x - dx, y - dy), screen(x + dx, y + dy), width)
        if rows:                                           # the rows' borders
            for lo, _hi in geo.row_bounds(d.width, us)[1:]:
                pygame.draw.line(r.screen, hmi.shade(color, 0.6), screen(lo, -d.height / 2.0),
                                 screen(lo, d.height / 2.0), 1)
    # The divert direction.
    a = math.radians(d.rotation + angle)
    reach = max(d.width, d.height) * 0.45
    x0, y0 = r._world_to_screen(d.x, d.y)
    x1, y1 = r._world_to_screen(d.x + reach * math.cos(a), d.y + reach * math.sin(a))
    pygame.draw.line(r.screen, color, (x0, y0), (x1, y1), 3 if d.active else 1)
    head = math.atan2(y1 - y0, x1 - x0)
    for side in (-0.5, 0.5):
        pygame.draw.line(r.screen, color, (x1, y1), (x1 - 8 * math.cos(head + side), y1 - 8 * math.sin(head + side)),
                         2)
    cx = sum(p[0] for p in pts) / 4
    r._queue_label(f"{d.label or d.id}  ->{d.destination}  ({d.diverted})", (cx, max(p[1] for p in pts) + 2),
                   r.PRIORITY_DEVICE, hmi.TEXT if d.active else hmi.TEXT_DIM, align="below")


def _draw_turntable(r, t) -> None:
    color = _color(r, t.color)
    cx, cy = r._world_to_screen(t.x, t.y)
    radius = max(3, r._scale_val(t.radius))
    pygame.draw.circle(r.screen, color, (cx, cy), radius, 3 if t.turning else 2)
    a = math.radians(t.angle)
    tip = r._world_to_screen(t.x + t.radius * math.cos(a), t.y + t.radius * math.sin(a))
    pygame.draw.line(r.screen, color, (cx, cy), tip, 2)
    state = "turning" if t.turning else f"pos {t.index}"
    r._queue_label(f"{t.label or t.id}  {state}", (cx, cy + radius + 2), r.PRIORITY_DEVICE, hmi.TEXT, align="below")


def _draw_accumulator(r, acc) -> None:
    """A photo-eye mark at each zone's end: filled while occupied."""
    from cad.ports import belt_ports
    belts = {b.id: b for b in r.config.belts}
    color = _color(r, acc.color)
    for i, belt_id in enumerate(acc.beltIds):
        belt = belts.get(belt_id)
        ports = belt_ports(belt) if belt is not None else None
        if ports is None:
            continue
        a = math.radians(ports.outlet.heading)
        ex, ey = ports.outlet.x - 0.15 * math.cos(a), ports.outlet.y - 0.15 * math.sin(a)
        sx, sy = r._world_to_screen(ex, ey)
        size = max(4, r._scale_val(0.12))
        rect = pygame.Rect(int(sx - size / 2), int(sy - size / 2), size, size)
        if acc.zones >> i & 1:
            pygame.draw.rect(r.screen, color, rect)
        pygame.draw.rect(r.screen, color, rect, 1)
    sx, sy = r._world_to_screen(acc.x, acc.y)
    r._queue_label(f"{acc.label or acc.id}{'  FULL' if acc.full else ''}", (sx, sy), r.PRIORITY_DEVICE,
                   hmi.WARNING if acc.full else hmi.TEXT_DIM)


def _draw_chute(r, sink) -> None:
    pts = r._rect_screen(sink.x, sink.y, sink.width, sink.height, 0.0)
    pygame.draw.polygon(r.screen, hmi.ALARM if sink.full else hmi.SINK, pts, 2)
    # Fill level as a bar along the bottom.
    x0 = min(p[0] for p in pts)
    x1 = max(p[0] for p in pts)
    y1 = max(p[1] for p in pts)
    frac = min(1.0, sink.count / max(1, sink.capacity))
    pygame.draw.rect(r.screen, hmi.ALARM if sink.full else hmi.SINK,
                     pygame.Rect(int(x0), int(y1) - 5, int((x1 - x0) * frac), 4))
    r._queue_label(f"{sink.count}/{sink.capacity}{'  FULL' if sink.full else ''}", ((x0 + x1) / 2, y1 + 2),
                   r.PRIORITY_DEVICE, hmi.ALARM if sink.full else hmi.TEXT, align="below")


def _door_ends(door):
    a = math.radians(door.rotation)
    hx, hy = door.width / 2.0 * math.cos(a), door.width / 2.0 * math.sin(a)
    return (door.x - hx, door.y - hy), (door.x + hx, door.y + hy)


def _draw_door(r, door) -> None:
    color = _color(r, door.color)
    (ax, ay), (bx, by) = _door_ends(door)
    a = r._world_to_screen(ax, ay)
    b = r._world_to_screen(bx, by)
    # Posts at both ends; the leaf closed across, or swung open from the first post.
    for p in (a, b):
        pygame.draw.circle(r.screen, hmi.TEXT_DIM, p, 3)
    if door.closed:
        pygame.draw.line(r.screen, color, a, b, 4)
    else:
        swing = math.radians(door.rotation + 70.0)
        open_end = r._world_to_screen(ax + door.width * math.cos(swing), ay + door.width * math.sin(swing))
        pygame.draw.line(r.screen, hmi.ALARM, a, open_end, 3)
        _dashed(r.screen, hmi.ALARM, a, b)
    text = f"{door.label or door.id}  {'closed' if door.closed else 'OPEN'}{'  locked' if door.locked else ''}"
    r._queue_label(text, ((a[0] + b[0]) / 2, max(a[1], b[1]) + 4), r.PRIORITY_DEVICE,
                   hmi.TEXT if door.closed else hmi.ALARM, align="below")


def _draw_safety(r, circuit) -> None:
    sx, sy = r._world_to_screen(circuit.x, circuit.y)
    size = 18
    rect = pygame.Rect(sx - size // 2, sy - size // 2, size, size)
    color = hmi.RUNNING if circuit.ok else hmi.ALARM
    pygame.draw.rect(r.screen, color, rect, border_radius=3)
    pygame.draw.rect(r.screen, hmi.TEXT, rect, 1, border_radius=3)
    text = "SAFE" if circuit.ok else ("TRIPPED - reset" if circuit.tripped else "STOP")
    r._queue_label(f"{circuit.label or circuit.id}  {text}", (sx, sy + size // 2 + 2), r.PRIORITY_DEVICE,
                   hmi.TEXT if circuit.ok else hmi.ALARM, align="below")


def hit_logistics(config, wx: float, wy: float) -> Optional[object]:
    """The logistics device at a world point (doors first: they are clicked)."""
    for door in config.doors:
        verts = get_rect_vertices(door.x, door.y, door.width, 0.3, door.rotation)
        if point_in_polygon(wx, wy, verts):
            return door
    for circuit in config.safety:
        if math.hypot(wx - circuit.x, wy - circuit.y) <= 0.3:
            return circuit
    for s in config.scanners:
        if point_in_polygon(wx, wy, get_rect_vertices(s.x, s.y, s.width, s.height, s.rotation)):
            return s
    for d in config.diverters:
        if point_in_polygon(wx, wy, get_rect_vertices(d.x, d.y, d.width, d.height, d.rotation)):
            return d
    for t in config.turntables:
        if math.hypot(wx - t.x, wy - t.y) <= t.radius:
            return t
    for t in config.transfers:
        if point_in_polygon(wx, wy, get_rect_vertices(t.x, t.y, t.width, t.height, t.rotation)):
            return t
    for acc in config.accumulators:
        if math.hypot(wx - acc.x, wy - acc.y) <= 0.4:
            return acc
    for v in getattr(config, "plcValues", []):
        if point_in_polygon(wx, wy, get_rect_vertices(v.x, v.y, v.width, v.height, v.rotation)):
            return v
    for w in getattr(config, "watchdogs", []):
        if point_in_polygon(wx, wy, get_rect_vertices(w.x, w.y, w.width, w.height, 0.0)):
            return w
    return None
