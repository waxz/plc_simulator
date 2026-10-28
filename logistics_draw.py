"""
logistics_draw.py - the logistics equipment (logistics.py) in the
simulator window (visualizer.Renderer): scanners, diverters, turntables,
accumulation zones, chutes, guard doors and safety circuits, drawn from
their live state; and which of them is under the mouse.
"""
import math
from typing import Optional

import pygame

import hmi_style as hmi
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
    for d in c.diverters:
        _draw_diverter(r, d)
    for s in c.scanners:
        _draw_scanner(r, s)
    for door in c.doors:
        _draw_door(r, door)
    for circuit in c.safety:
        _draw_safety(r, circuit)
    _ = screen


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
    if d.active:
        overlay = pygame.Surface(r.screen.get_size(), pygame.SRCALPHA)
        pygame.draw.polygon(overlay, (*color, 70), pts)
        r.screen.blit(overlay, (0, 0))
        pygame.draw.polygon(r.screen, color, pts, 2)
    else:
        _polygon_dashed(r.screen, color, pts)
    # The divert direction.
    a = math.radians(d.rotation + d.divertAngle)
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
    for acc in config.accumulators:
        if math.hypot(wx - acc.x, wy - acc.y) <= 0.4:
            return acc
    return None
