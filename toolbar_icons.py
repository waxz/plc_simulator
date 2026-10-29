"""
toolbar_icons.py - the layout editor's tool bar icons, drawn with QPainter
(no image files), in the theme's colours.

Every icon is line art on a 24 x 24 grid: `icon(name)` renders it in the
current hmi_style colours (TEXT, with ACCENT / RUNNING / ALARM for the
highlights). IconBinder remembers which action shows which icon and
redraws them all when the theme changes.
"""
import math
from typing import Callable, Dict, List, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QIcon, QPainter, QPainterPath, QPen, QPixmap, QPolygonF

import hmi_style as hmi

P = QPointF


class _Pen:
    """Drawing helpers on the 24-unit grid."""

    def __init__(self, painter: QPainter):
        self.p = painter
        self.fg = hmi.qcolor(hmi.TEXT)
        self.accent = hmi.qcolor(hmi.ACCENT)
        self.go = hmi.qcolor(hmi.RUNNING)
        self.stop = hmi.qcolor(hmi.ALARM)
        self.dim = hmi.qcolor(hmi.TEXT_DIM)
        self.use(self.fg)

    def use(self, color: QColor, width: float = 1.7, dash: bool = False):
        pen = QPen(color, width)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        if dash:
            pen.setDashPattern([1.5, 2.0])
        self.p.setPen(pen)
        self.p.setBrush(Qt.NoBrush)
        return self

    def line(self, *xy):
        pts = [P(xy[i], xy[i + 1]) for i in range(0, len(xy), 2)]
        for a, b in zip(pts, pts[1:]):
            self.p.drawLine(a, b)

    def poly(self, *xy, fill: QColor = None, closed: bool = True):
        poly = QPolygonF([P(xy[i], xy[i + 1]) for i in range(0, len(xy), 2)])
        if fill is not None:
            self.p.setBrush(QBrush(fill))
        if closed:
            self.p.drawPolygon(poly)
        else:
            self.p.drawPolyline(poly)
        self.p.setBrush(Qt.NoBrush)

    def rect(self, x, y, w, h, fill: QColor = None, r: float = 0.0):
        if fill is not None:
            self.p.setBrush(QBrush(fill))
        self.p.drawRoundedRect(QRectF(x, y, w, h), r, r)
        self.p.setBrush(Qt.NoBrush)

    def circle(self, cx, cy, r, fill: QColor = None):
        if fill is not None:
            self.p.setBrush(QBrush(fill))
        self.p.drawEllipse(P(cx, cy), r, r)
        self.p.setBrush(Qt.NoBrush)

    def arc(self, cx, cy, r, start_deg, span_deg):
        """Counter-clockwise from start_deg (0 = east, y up on screen)."""
        self.p.drawArc(QRectF(cx - r, cy - r, 2 * r, 2 * r), int(start_deg * 16), int(span_deg * 16))

    def arrow_head(self, x, y, angle_deg, size=3.2):
        """A filled head at (x, y) pointing along angle_deg (screen y down)."""
        a = math.radians(angle_deg)
        back = [(x - size * math.cos(a) + size * 0.7 * math.sin(a), y - size * math.sin(a) - size * 0.7 * math.cos(a)),
                (x - size * math.cos(a) - size * 0.7 * math.sin(a), y - size * math.sin(a) + size * 0.7 * math.cos(a))]
        color = self.p.pen().color()
        self.poly(x, y, *back[0], *back[1], fill=color)

    def dot(self, x, y, r=1.6, color: QColor = None):
        c = color or self.p.pen().color()
        old = self.p.pen()
        self.p.setPen(Qt.NoPen)
        self.p.setBrush(QBrush(c))
        self.p.drawEllipse(P(x, y), r, r)
        self.p.setBrush(Qt.NoBrush)
        self.p.setPen(old)

    def grip(self, x, y, color: QColor = None):
        old = self.p.pen()
        self.use(color or self.accent, 1.2)
        self.rect(x - 1.8, y - 1.8, 3.6, 3.6, fill=hmi.qcolor(hmi.PANEL))
        self.p.setPen(old)

    def page(self):
        self.poly(5, 3, 14, 3, 19, 8, 19, 21, 5, 21)
        self.line(14, 3, 14, 8, 19, 8)


# -- the icons ---------------------------------------------------------------------------

def _open(d):
    d.poly(3, 6, 9, 6, 11, 8, 20, 8, 20, 19, 3, 19)
    d.use(d.accent).line(3, 12, 21, 12)


def _save(d):
    d.poly(4, 4, 17, 4, 20, 7, 20, 20, 4, 20)
    d.rect(8, 4, 8, 5)
    d.use(d.accent).rect(7, 13, 10, 7)


def _u_turn(d, mirror: bool):
    path = QPainterPath()
    if mirror:
        path.moveTo(17, 9)
        path.lineTo(10, 9)
        path.arcTo(QRectF(5, 9, 10, 10), 90, 180)
        path.lineTo(16, 19)
    else:
        path.moveTo(7, 9)
        path.lineTo(14, 9)
        path.arcTo(QRectF(9, 9, 10, 10), 90, -180)
        path.lineTo(8, 19)
    d.p.drawPath(path)
    d.arrow_head(20, 9, 0) if mirror else d.arrow_head(4, 9, 180)


def _undo(d):
    _u_turn(d, False)


def _redo(d):
    _u_turn(d, True)


def _import_dxf(d):
    d.page()
    d.use(d.accent).line(12, 9, 12, 17)
    d.arrow_head(12, 18, 90)


def _export_dxf(d):
    d.page()
    d.use(d.accent).line(12, 18, 12, 10)
    d.arrow_head(12, 9, -90)


def _bg_place(d):
    """A ship's anchor: the drawing's placement point."""
    d.use(d.accent)
    d.circle(12, 5, 2)
    d.line(12, 7, 12, 20)
    d.line(8, 10, 16, 10)
    d.arc(12, 13, 7, 180, 180)
    d.arrow_head(5, 13, -100, 2.6)
    d.arrow_head(19, 13, -80, 2.6)


def _bg_settings(d):
    for y, x in ((7, 15), (12, 8), (17, 13)):
        d.use(d.fg).line(4, y, 20, y)
        d.use(d.accent).circle(x, y, 2, fill=hmi.qcolor(hmi.PANEL))


def _bg_crop(d):
    d.line(7, 3, 7, 17, 21, 17)
    d.line(3, 7, 17, 7, 17, 21)
    d.use(d.accent, dash=True).rect(9.5, 9.5, 5, 5)


def _bg_remove(d):
    d.page()
    d.use(d.stop).line(9, 12, 15, 18)
    d.line(15, 12, 9, 18)


def _pan(d):
    d.line(12, 3, 12, 21)
    d.line(3, 12, 21, 12)
    for x, y, a in ((12, 3, -90), (12, 21, 90), (3, 12, 180), (21, 12, 0)):
        d.arrow_head(x, y, a, 2.8)


def _measure(d):
    d.p.save()
    d.p.translate(12, 12)
    d.p.rotate(-35)
    d.rect(-10, -3.5, 20, 7, r=1)
    for i, x in enumerate(range(-7, 9, 3)):
        d.line(x, -3.5, x, -1 if i % 2 else 0.5)
    d.p.restore()


def _clear_measure(d):
    d.use(d.dim).line(4, 16, 20, 16)
    d.line(4, 13, 4, 19)
    d.line(20, 13, 20, 19)
    d.use(d.stop).line(9, 4, 15, 10)
    d.line(15, 4, 9, 10)


def _fit(d):
    for x, y, sx, sy in ((3, 3, 1, 1), (21, 3, -1, 1), (3, 21, 1, -1), (21, 21, -1, -1)):
        d.line(x + 5 * sx, y, x, y, x, y + 5 * sy)
    d.use(d.accent).rect(8, 8, 8, 8)


def _ports(d):
    d.rect(6, 8, 12, 8)
    d.use(d.go).line(1, 12, 5, 12)
    d.arrow_head(6, 12, 0, 2.6)
    d.use(d.stop).line(18, 12, 22, 12)
    d.arrow_head(23, 12, 0, 2.6)


def _run(d):
    d.use(d.go).poly(7, 4, 20, 12, 7, 20, fill=d.go)


def _theme(d):
    d.circle(12, 12, 8)
    path = QPainterPath()
    path.moveTo(12, 4)
    path.arcTo(QRectF(4, 4, 16, 16), 90, 180)
    path.closeSubpath()
    d.p.fillPath(path, QBrush(d.fg))


def _line(d):
    d.line(5, 19, 19, 5)
    d.grip(5, 19)
    d.grip(19, 5)


def _arc(d):
    d.arc(12, 17, 9, 0, 180)
    d.grip(3, 17)
    d.grip(21, 17)
    d.use(d.accent).dot(12, 17, 1.4)


def _arc_center(d):
    """An arc from its center: the center first, then start and end."""
    d.arc(12, 17, 9, 0, 180)
    d.use(d.dim).line(12, 17, 3, 17)
    d.use(d.dim).line(12, 17, 18.4, 10.6)
    d.use(d.accent).dot(12, 17, 2.0)
    d.grip(3, 17)
    d.grip(18.4, 10.6)


def _divide(d):
    """A line cut into equal belts."""
    _band(d, [(2, 14), (8, 14)])
    _band(d, [(9, 14), (15, 14)], color=d.accent)
    _band(d, [(16, 14), (22, 14)])
    for x in (8.5, 15.5):
        d.use(d.fg).line(x, 8, x, 20)


def _rectangle(d):
    d.rect(4, 6, 16, 12)
    d.grip(4, 18)
    d.grip(20, 6)


def _circle(d):
    d.circle(12, 12, 8)
    d.use(d.accent).dot(12, 12, 1.5)


def _polygon(d):
    pts = []
    for k in range(6):
        a = math.radians(60 * k + 30)
        pts += [12 + 8.5 * math.cos(a), 12 + 8.5 * math.sin(a)]
    d.poly(*pts)


def _clear(d):
    d.line(5, 7, 19, 7)
    d.line(10, 7, 10, 4, 14, 4, 14, 7)
    d.poly(7, 7, 8, 20, 16, 20, 17, 7, closed=False)
    d.use(d.dim).line(10.5, 10, 10.5, 17)
    d.line(13.5, 10, 13.5, 17)


def _band(d, path_pts, width=5.0, color=None):
    """A belt: a thick band along a polyline with chevrons."""
    pen = QPen(color or d.dim, width)
    pen.setCapStyle(Qt.FlatCap)
    pen.setJoinStyle(Qt.RoundJoin)
    d.p.setPen(pen)
    d.p.drawPolyline(QPolygonF([P(x, y) for x, y in path_pts]))
    d.use(d.fg)


def _belt_pick(d):
    _band(d, [(2, 16), (16, 16)])
    d.use(d.accent).line(2, 16, 16, 16)
    d.use(d.fg).poly(14, 4, 14, 15, 17, 12, 19.5, 17, 21, 16.2, 18.5, 11.5, 22, 11.2, fill=hmi.qcolor(hmi.PANEL))


def _roller_pick(d):
    """Rollers across a conveyor, picked."""
    d.use(d.dim).rect(2, 12, 15, 8)
    for x in (4, 7, 10, 13, 16):
        d.use(d.accent).line(x, 12, x, 20)
    d.use(d.fg).poly(14, 4, 14, 15, 17, 12, 19.5, 17, 21, 16.2, 18.5, 11.5, 22, 11.2, fill=hmi.qcolor(hmi.PANEL))


def _belt_path(d):
    _band(d, [(3, 18), (11, 18), (11, 7), (21, 7)])
    d.use(d.accent).poly(3, 18, 11, 18, 11, 7, 21, 7, closed=False)
    for x, y in ((3, 18), (11, 18), (11, 7), (21, 7)):
        d.grip(x, y, d.fg)


def _curved_belt(d):
    pen = QPen(d.dim, 5)
    pen.setCapStyle(Qt.FlatCap)
    d.p.setPen(pen)
    d.arc(4, 20, 13, 0, 90)
    d.use(d.accent).arc(4, 20, 13, 0, 90)
    d.arrow_head(4, 7, 180, 2.8)


def _outline_belt(d):
    d.use(d.accent, dash=True).poly(3, 8, 21, 8, 21, 16, 3, 16)
    d.use(d.fg)
    for x, y in ((3, 8), (21, 8), (21, 16), (3, 16)):
        d.grip(x, y, d.fg)


def _connect(d):
    d.rect(2, 9, 8, 6)
    d.rect(14, 9, 8, 6)
    d.use(d.go).line(10, 12, 14, 12)
    d.dot(10, 12, 1.6)
    d.dot(14, 12, 1.6)


def _disconnect(d):
    d.rect(2, 9, 7, 6)
    d.rect(15, 9, 7, 6)
    d.use(d.stop).line(10.5, 8, 13.5, 16)


def _fillet_belts(d):
    _band(d, [(4, 21), (4, 13)])
    _band(d, [(12, 4), (21, 4)])
    pen = QPen(d.dim, 5)
    pen.setCapStyle(Qt.FlatCap)
    d.p.setPen(pen)
    d.arc(12, 13, 8, 90, 90)
    d.use(d.accent).arc(12, 13, 8, 90, 90)


def _solve(d):
    d.use(d.accent).arc(12, 12, 7, 30, 150)
    d.arrow_head(5.5, 15.5, 100, 2.8)
    d.arc(12, 12, 7, 210, 150)
    d.arrow_head(18.5, 8.5, -80, 2.8)
    d.use(d.fg).dot(12, 12, 1.6)


def _check(d):
    d.circle(12, 12, 9)
    d.use(d.go, 2.2).line(7.5, 12.5, 10.5, 15.5, 16.5, 8.5)


def _trim(d):
    d.line(15, 3, 15, 21)
    d.line(3, 12, 15, 12)
    d.use(d.stop, dash=True).line(15, 12, 21, 12)


def _extend(d):
    d.line(20, 3, 20, 21)
    d.line(3, 12, 10, 12)
    d.use(d.accent, dash=True).line(10, 12, 17, 12)
    d.arrow_head(19, 12, 0, 2.8)


def _break(d):
    d.line(3, 15, 10, 9)
    d.line(14, 15, 21, 9)
    d.use(d.accent).dot(10, 9)
    d.dot(14, 15)


def _offset(d):
    d.arc(12, 20, 8, 20, 140)
    d.use(d.accent).arc(12, 20, 13, 20, 140)


def _fillet(d):
    d.line(5, 21, 5, 12)
    d.line(12, 5, 21, 5)
    d.use(d.accent).arc(12, 12, 7, 90, 90)


def _join(d):
    d.line(3, 18, 12, 9)
    d.use(d.accent).line(12, 9, 21, 15)
    d.use(d.fg).dot(12, 9, 2)


def _explode(d):
    d.line(10, 10, 4, 4)
    d.line(14, 10, 20, 6)
    d.line(12, 14, 12, 21)
    d.use(d.accent).line(4, 13, 6, 13)
    d.line(18, 15, 20, 17)


def _move(d):
    d.use(d.dim, dash=True).rect(3, 11, 8, 8)
    d.use(d.fg).rect(13, 5, 8, 8)
    d.use(d.accent).line(8, 14, 15, 8)
    d.arrow_head(16, 7, -40, 2.8)


def _copy(d):
    d.rect(3, 9, 10, 10)
    d.use(d.accent).rect(10, 4, 10, 10, fill=hmi.qcolor(hmi.PANEL))


def _rotate(d):
    d.use(d.dim, dash=True).rect(7, 7, 10, 10)
    d.use(d.accent).arc(12, 12, 8, -20, 250)
    d.arrow_head(19.3, 15.2, 70, 2.8)
    d.use(d.fg).dot(12, 12, 1.5)


def _mirror(d):
    d.use(d.dim, dash=True).line(12, 2, 12, 22)
    d.use(d.fg).poly(10, 6, 10, 18, 3, 18)
    d.use(d.accent).poly(14, 6, 14, 18, 21, 18)


def _scale(d):
    d.rect(3, 3, 18, 18)
    d.use(d.accent).rect(3, 13, 8, 8)
    d.line(11, 13, 17, 7)
    d.arrow_head(18, 6, -45, 2.8)


ICONS: Dict[str, Callable[[_Pen], None]] = {
    "open": _open, "save": _save, "undo": _undo, "redo": _redo,
    "import_dxf": _import_dxf, "export_dxf": _export_dxf,
    "bg_place": _bg_place, "bg_settings": _bg_settings, "bg_crop": _bg_crop, "bg_remove": _bg_remove,
    "pan": _pan, "measure": _measure, "clear_measure": _clear_measure, "fit": _fit, "ports": _ports,
    "run": _run, "theme": _theme,
    "line": _line, "arc": _arc, "arc_center": _arc_center, "divide": _divide, "rectangle": _rectangle, "circle": _circle, "polygon": _polygon, "clear": _clear,
    "belt_pick": _belt_pick, "roller_pick": _roller_pick, "belt_path": _belt_path, "curved_belt": _curved_belt, "outline_belt": _outline_belt,
    "connect": _connect, "disconnect": _disconnect, "fillet_belts": _fillet_belts, "solve": _solve,
    "check": _check,
    "trim": _trim, "extend": _extend, "break": _break, "offset": _offset, "fillet": _fillet, "join": _join,
    "explode": _explode, "move": _move, "copy": _copy, "rotate": _rotate, "mirror": _mirror, "scale": _scale,
}


def pixmap(name: str, px: int = 24, enabled: bool = True) -> QPixmap:
    pm = QPixmap(px, px)
    pm.fill(Qt.transparent)
    painter = QPainter(pm)
    painter.setRenderHint(QPainter.Antialiasing)
    painter.scale(px / 24.0, px / 24.0)
    if not enabled:
        painter.setOpacity(0.35)
    ICONS[name](_Pen(painter))
    painter.end()
    return pm


def icon(name: str) -> QIcon:
    """The icon in the current theme's colours, sharp at 1x and 2x."""
    result = QIcon()
    for px in (24, 48):
        result.addPixmap(pixmap(name, px), QIcon.Normal)
        result.addPixmap(pixmap(name, px, enabled=False), QIcon.Disabled)
    return result


class IconBinder:
    """Which action shows which icon; refresh() redraws them (theme change)."""

    def __init__(self):
        self.bound: List[Tuple[object, str]] = []

    def bind(self, action, name: str):
        if name not in ICONS:
            raise KeyError(name)
        self.bound.append((action, name))
        action.setIcon(icon(name))

    def refresh(self):
        cache = {}
        for action, name in self.bound:
            if name not in cache:
                cache[name] = icon(name)
            try:
                action.setIcon(cache[name])
            except RuntimeError:
                pass          # the action is gone with its window
