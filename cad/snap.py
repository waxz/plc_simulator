"""
cad/snap.py - object snap (OSNAP) and picking against drawing geometry.
No Qt; everything in world metres.

SnapIndex.query(x, y, tol) finds the point a CAD user means when clicking
near geometry, in priority order:

  endpoint   a vertex of a line/polyline/arc, or a component key point
  center     the center of an arc or circle
  midpoint   the middle of a straight segment
  nearest    the closest point on any segment (straight or flattened arc)

Within one priority the closest candidate wins; a higher-priority candidate
inside the tolerance beats a closer lower-priority one (as in AutoCAD).

DrawingPicker answers "what did the user click on" for building belts from
an imported layout:
  pick_line()   - the drawing entity (line/polyline/arc) under the cursor
  pick_region() - the smallest closed area containing a point, formed by any
                  combination of entities (separate LINEs work, not just
                  closed polylines) via Shapely polygonize. The areas are
                  found once, lazily; start_regions() does it on a worker
                  thread so a big drawing doesn't freeze the editor.
"""
import math
import threading
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
from shapely import STRtree
from shapely.geometry import LineString, MultiLineString, Point
from shapely.ops import polygonize, unary_union

from cad.path import flatten, reverse_polyline, segments

PRIORITY = {"endpoint": 0, "center": 1, "midpoint": 2, "nearest": 3}


@dataclass(frozen=True)
class Snap:
    x: float
    y: float
    kind: str        # endpoint | center | midpoint | nearest | grid
    distance: float


class SnapIndex:
    def __init__(self, points: Sequence[Tuple[float, float, str]] = (),
                 segments_: Sequence[Tuple[float, float, float, float]] = ()):
        self.pts = np.array([(x, y) for x, y, _ in points], dtype=float).reshape(-1, 2)
        self.prio = np.array([PRIORITY[k] for _, _, k in points], dtype=int)
        self.kinds = [k for _, _, k in points]
        self.seg = np.array(segments_, dtype=float).reshape(-1, 4)

    @classmethod
    def from_shapes(cls, shapes, flatten_tolerance: float = 0.005) -> "SnapIndex":
        """Snap targets of drawing shapes (anything with .points/.bulges/.closed)."""
        points: List[Tuple[float, float, str]] = []
        segs: List[Tuple[float, float, float, float]] = []
        for shape in shapes:
            pts = shape.points
            for x, y in pts:
                points.append((x, y, "endpoint"))
            for seg in segments(pts, shape.bulges, shape.closed):
                if seg.kind == "line":
                    points.append(((seg.p0[0] + seg.p1[0]) / 2, (seg.p0[1] + seg.p1[1]) / 2, "midpoint"))
                    segs.append((*seg.p0, *seg.p1))
                else:
                    points.append((*seg.center, "center"))
                    points.append((*seg.point_at(0.5), "midpoint"))
                    flat, _ = flatten([seg], flatten_tolerance)
                    segs.extend((*a, *b) for a, b in zip(flat, flat[1:]))
        return cls(points, segs)

    def __len__(self):
        return len(self.pts) + len(self.seg)

    def query(self, x: float, y: float, tol: float) -> Optional[Snap]:
        best: Optional[Snap] = None
        if len(self.pts):
            d = np.hypot(self.pts[:, 0] - x, self.pts[:, 1] - y)
            near = np.nonzero(d <= tol)[0]
            if near.size:
                i = near[np.lexsort((d[near], self.prio[near]))[0]]
                best = Snap(float(self.pts[i, 0]), float(self.pts[i, 1]), self.kinds[i], float(d[i]))
        if best is None and len(self.seg):
            ax, ay, bx, by = self.seg.T
            dx, dy = bx - ax, by - ay
            len2 = dx * dx + dy * dy
            t = np.clip(np.where(len2 > 0, ((x - ax) * dx + (y - ay) * dy) / np.where(len2 > 0, len2, 1), 0.0), 0, 1)
            px, py = ax + t * dx, ay + t * dy
            d = np.hypot(px - x, py - y)
            i = int(np.argmin(d))
            if d[i] <= tol:
                best = Snap(float(px[i]), float(py[i]), "nearest", float(d[i]))
        return best


def best_snap(*candidates: Optional[Snap]) -> Optional[Snap]:
    found = [c for c in candidates if c is not None]
    if not found:
        return None
    return min(found, key=lambda s: (PRIORITY.get(s.kind, 9), s.distance))


class DrawingPicker:
    """Pick drawing entities and closed areas (world metres)."""

    def __init__(self, shapes):
        self.shapes = list(shapes)
        self._lines = [LineString(flatten(segments(s.points, s.bulges, s.closed), 0.005)[0]) for s in self.shapes]
        self._line_tree = STRtree(self._lines) if self._lines else None
        self._regions = None
        self._region_tree = None
        self._region_lock = threading.Lock()
        self._region_thread: Optional[threading.Thread] = None

    def pick_line(self, x: float, y: float, tol: float):
        """The shape whose outline passes within `tol` of (x, y), or None."""
        if self._line_tree is None:
            return None
        p = Point(x, y)
        i = self._line_tree.query_nearest(p, max_distance=tol)
        return self.shapes[int(i[0])] if len(i) else None

    def _compute_regions(self):
        try:
            noded = unary_union(MultiLineString([list(l.coords) for l in self._lines])) if self._lines else None
            regions = [r for r in polygonize(noded)] if noded is not None else []
        except Exception:  # noqa: BLE001 - GEOS gave up on odd geometry: no areas, lines still pick
            regions = []
        tree = STRtree(regions) if regions else None
        with self._region_lock:
            if self._regions is None:
                self._region_tree, self._regions = tree, regions

    def start_regions(self) -> bool:
        """Find the closed areas on a worker thread (Shapely's GEOS calls
        release the GIL). True when they are already available."""
        with self._region_lock:
            if self._regions is not None:
                return True
            if self._region_thread is None:
                self._region_thread = threading.Thread(target=self._compute_regions, name="regions", daemon=True)
                self._region_thread.start()
        return False

    def regions_ready(self) -> bool:
        return self._regions is not None

    def regions(self):
        """Every closed area bounded by the drawing's lines (computed once;
        waits for a running start_regions())."""
        if self._regions is None:
            thread = self._region_thread
            if thread is not None:
                thread.join()
            else:
                self._compute_regions()
        return self._regions

    def pick_region(self, x: float, y: float, wait: bool = True):
        """The smallest closed area containing (x, y), or None. With
        wait=False and the areas still being found, None at once."""
        if not wait and not self.start_regions():
            return None
        regions = self.regions()
        if not regions:
            return None
        p = Point(x, y)
        hits = [regions[int(i)] for i in self._region_tree.query(p, predicate="within")]
        return min(hits, key=lambda r: r.area) if hits else None


def polygon_vertices(polygon, tolerance: float = 1e-6) -> List[Tuple[float, float]]:
    """Exterior vertices of a picked area, without the repeated closing point
    and without collinear in-between points that noding adds."""
    coords = list(polygon.exterior.coords)[:-1]
    out = []
    n = len(coords)
    for i in range(n):
        a, b, c = coords[i - 1], coords[i], coords[(i + 1) % n]
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        if abs(cross) > tolerance * max(1.0, math.hypot(c[0] - a[0], c[1] - a[1])):
            out.append(b)
    return out


def chain_from(shapes, start, tolerance: float = 1e-3):
    """Join `start` with every open shape connected to it end-to-end - the
    "connect lines/curves into one" step. Walks both ways from `start` while
    exactly one unused shape continues the chain (a branch point stops it),
    reversing shapes as needed. Returns (points, bulges, closed); closed if
    the chain comes back to where it began."""
    if start.closed:
        return list(start.points), list(start.bulges), True
    pts, bul = list(start.points), list(start.bulges) + [0.0] * (len(start.points) - len(start.bulges))
    used = {id(start)}
    candidates = [s for s in shapes if not s.closed and len(s.points) >= 2]

    def near(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1]) <= tolerance

    def extend(pts, bul):
        while True:
            end = pts[-1]
            nxt = [s for s in candidates if id(s) not in used and (near(s.points[0], end) or near(s.points[-1], end))]
            if len(nxt) != 1 or near(pts[0], end) and len(pts) > 2:
                return pts, bul
            shape = nxt[0]
            used.add(id(shape))
            s_pts, s_bul = list(shape.points), list(shape.bulges) + [0.0] * (len(shape.points) - len(shape.bulges))
            if not near(s_pts[0], end):
                s_pts, s_bul = reverse_polyline(s_pts, s_bul)
            pts = pts + s_pts[1:]
            bul = bul[:-1] + s_bul

    pts, bul = extend(pts, bul)
    if not (len(pts) > 2 and near(pts[0], pts[-1])):
        r_pts, r_bul = reverse_polyline(pts, bul)
        r_pts, r_bul = extend(r_pts, r_bul)
        pts, bul = reverse_polyline(r_pts, r_bul)
    closed = len(pts) > 2 and near(pts[0], pts[-1])
    if closed:
        pts, bul = pts[:-1], bul[:-1]
    return pts, bul, closed
