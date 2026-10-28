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
                  thread so a big drawing doesn't freeze the editor. Small
                  gaps between line ends are bridged first (heal_lines).
chain_from() / join_shapes() join lines and arcs end to end into one chain,
also across small gaps and ignoring shapes drawn twice.
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
    """Pick drawing entities and closed areas (world metres).

    Real drawings rarely join exactly: line ends miss each other by a few
    millimetres, stop short of the line they meet, or an arc's computed end
    differs from the next line's start in the last digit. `join_gap` (metres)
    bridges such gaps, both when finding closed areas and when joining
    lines/arcs into one chain."""

    def __init__(self, shapes, join_gap: float = 0.0):
        self.shapes = list(shapes)
        self._lines = [LineString(flatten(segments(s.points, s.bulges, s.closed), 0.005)[0]) for s in self.shapes]
        self._line_tree = STRtree(self._lines) if self._lines else None
        self.join_gap = max(0.0, join_gap)
        self._regions = None
        self._region_tree = None
        self._region_lock = threading.Lock()
        self._region_thread: Optional[threading.Thread] = None
        self._generation = 0
        self._end_tree = None
        self._end_owner: List[int] = []

    def set_join_gap(self, gap: float):
        """Change the gap bridged when finding areas (they are found again)."""
        gap = max(0.0, gap)
        with self._region_lock:
            if abs(gap - self.join_gap) <= 1e-12:
                return
            self.join_gap = gap
            self._generation += 1
            self._regions = self._region_tree = self._region_thread = None

    def chain_tolerance(self) -> float:
        return max(DEFAULT_CHAIN_TOLERANCE, self.join_gap)

    def _ends_near(self, point, tolerance: float):
        if self._end_tree is None:
            owners, ends = [], []
            for k, shape in enumerate(self.shapes):
                if not shape.closed and len(shape.points) >= 2:
                    for p in (shape.points[0], shape.points[-1]):
                        owners.append(k)
                        ends.append(Point(p))
            self._end_owner = owners
            self._end_tree = STRtree(ends) if ends else False
        if self._end_tree is False:
            return []
        hits = self._end_tree.query(Point(point), predicate="dwithin", distance=tolerance)
        return [self.shapes[self._end_owner[int(i)]] for i in sorted(set(int(h) for h in hits))]

    def chain(self, start):
        """chain_from(start) over this drawing, across gaps up to join_gap."""
        return chain_from(self.shapes, start, self.chain_tolerance(), near_ends=self._ends_near)

    def pick_line(self, x: float, y: float, tol: float):
        """The shape whose outline passes within `tol` of (x, y), or None."""
        if self._line_tree is None:
            return None
        p = Point(x, y)
        i = self._line_tree.query_nearest(p, max_distance=tol)
        return self.shapes[int(i[0])] if len(i) else None

    def _compute_regions(self, generation: Optional[int] = None):
        if generation is None:
            generation = self._generation
        try:
            healed = heal_lines([list(l.coords) for l in self._lines], self.join_gap)
            noded = unary_union(MultiLineString(healed)) if healed else None
            regions = [r for r in polygonize(noded)] if noded is not None else []
        except Exception:  # noqa: BLE001 - GEOS gave up on odd geometry: no areas, lines still pick
            regions = []
        tree = STRtree(regions) if regions else None
        with self._region_lock:
            if self._regions is None and generation == self._generation:
                self._region_tree, self._regions = tree, regions

    def start_regions(self) -> bool:
        """Find the closed areas on a worker thread (Shapely's GEOS calls
        release the GIL). True when they are already available."""
        with self._region_lock:
            if self._regions is not None:
                return True
            if self._region_thread is None:
                self._region_thread = threading.Thread(target=self._compute_regions, args=(self._generation,),
                                                       name="regions", daemon=True)
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
            if self._regions is None:  # no thread, or it ran for an older join gap
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
    and without collinear in-between points that noding adds. Points closer
    than `tolerance` (rounding noise where two shapes meet, e.g. 40.6 and
    40.599999999999994) are merged first - otherwise a corner next to its
    own near-copy looks collinear and is dropped with it."""
    coords = []
    for p in list(polygon.exterior.coords)[:-1]:
        if not coords or math.hypot(p[0] - coords[-1][0], p[1] - coords[-1][1]) > tolerance:
            coords.append(p)
    while len(coords) > 1 and math.hypot(coords[0][0] - coords[-1][0], coords[0][1] - coords[-1][1]) <= tolerance:
        coords.pop()
    out = []
    n = len(coords)
    for i in range(n):
        a, b, c = coords[i - 1], coords[i], coords[(i + 1) % n]
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        if abs(cross) > tolerance * max(1.0, math.hypot(c[0] - a[0], c[1] - a[1])):
            out.append(b)
    return out


DEFAULT_CHAIN_TOLERANCE = 1e-3  # metres: line ends this close count as joined


def heal_lines(lines: List[List[Tuple[float, float]]], gap: float) -> List[List[Tuple[float, float]]]:
    """Close small gaps between line ends so polygonize finds the areas a
    person sees: ends within `gap` of each other are merged to one point,
    and an end within `gap` of another line (stopping short of it) is
    extended onto it, slightly past so noding sees the crossing (the dangle
    is dropped by polygonize). A tiny gap is always bridged, for ends that
    differ only by rounding (e.g. an arc's computed end)."""
    coords = [list(c) for c in lines if len(c) >= 2]
    if not coords:
        return []
    xs = [x for c in coords for x, _ in c]
    ys = [y for c in coords for _, y in c]
    size = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    gap = max(gap, 1e-9 * size)

    # 1. Merge ends that (nearly) coincide.
    ends = [(i, w) for i, c in enumerate(coords) for w in (0, -1)
            if c[0] != c[-1] and LineString(c).length > gap]
    if not ends:
        return coords
    pts = np.array([coords[i][w] for i, w in ends], dtype=float)
    parent = list(range(len(ends)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    end_points = [Point(p) for p in pts]
    pairs = STRtree(end_points).query(end_points, predicate="dwithin", distance=gap)
    for a, b in pairs.T:
        if a < b:
            ra, rb = find(int(a)), find(int(b))
            if ra != rb:
                parent[ra] = rb
    clusters = {}
    for k in range(len(ends)):
        clusters.setdefault(find(k), []).append(k)
    dangling = []
    for members in clusters.values():
        if len(members) == 1:
            dangling.append(members[0])
            continue
        cx, cy = pts[members].mean(axis=0)
        for k in members:
            i, w = ends[k]
            coords[i][w] = (float(cx), float(cy))

    # 2. Extend ends that stop short of (or end exactly on) another line.
    if dangling:
        geoms = [LineString(c) for c in coords]
        tree = STRtree(geoms)
        for k in dangling:
            i, w = ends[k]
            end = Point(coords[i][w])
            best, best_d = None, gap
            for j in tree.query(end, predicate="dwithin", distance=gap):
                if int(j) == i:
                    continue
                d = geoms[int(j)].distance(end)
                if d <= best_d:
                    best, best_d = int(j), d
            if best is None:
                continue
            target = geoms[best].interpolate(geoms[best].project(end))
            prev = coords[i][1] if w == 0 else coords[i][-2]
            dx, dy = coords[i][w][0] - prev[0], coords[i][w][1] - prev[1]
            norm = math.hypot(dx, dy)
            if norm <= 0:
                continue
            over = max(gap * 0.05, 1e-7 * size)
            coords[i][w] = (target.x + dx / norm * over, target.y + dy / norm * over)
    return coords


def _same_shape(a, b, tolerance: float) -> bool:
    """Two drawing shapes on top of each other (either direction)."""
    if a.closed != b.closed or len(a.points) != len(b.points):
        return False

    def near(p, q):
        return math.hypot(p[0] - q[0], p[1] - q[1]) <= tolerance
    if all(near(p, q) for p, q in zip(a.points, b.points)):
        return True
    return all(near(p, q) for p, q in zip(a.points, reversed(b.points)))


def chain_from(shapes, start, tolerance: float = DEFAULT_CHAIN_TOLERANCE, used: Optional[set] = None,
               near_ends=None):
    """Join `start` with every open shape connected to it end-to-end - the
    "connect lines/curves into one" step. Walks both ways from `start` while
    exactly one unused shape continues the chain (a branch point stops it),
    reversing shapes as needed. Ends within `tolerance` count as joined (the
    gap is closed); a shape drawn twice on top of itself is not a branch.
    Returns (points, bulges, closed); closed if the chain comes back to where
    it began. `used`, if given, receives the id() of every shape joined.
    `near_ends(point, tolerance)`, if given, returns the shapes with an end
    near a point (a spatial index; otherwise every shape is checked)."""
    used = used if used is not None else set()
    used.add(id(start))
    if start.closed:
        return list(start.points), list(start.bulges), True
    pts, bul = list(start.points), list(start.bulges) + [0.0] * (len(start.points) - len(start.bulges))
    candidates = [s for s in shapes if not s.closed and len(s.points) >= 2]
    joined = [start]

    def near(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1]) <= tolerance

    def extend(pts, bul):
        while True:
            end = pts[-1]
            pool = candidates if near_ends is None else [s for s in near_ends(end, tolerance)
                                                         if not s.closed and len(s.points) >= 2]
            nxt = [s for s in pool if id(s) not in used and (near(s.points[0], end) or near(s.points[-1], end))]
            # Duplicates (of each other or of what is already joined) are one shape.
            unique = []
            for s in nxt:
                if any(_same_shape(s, t, tolerance) for t in joined):
                    used.add(id(s))
                elif not any(_same_shape(s, u, tolerance) for u in unique):
                    unique.append(s)
                else:
                    used.add(id(s))
            nxt = unique
            if len(nxt) != 1 or near(pts[0], end) and len(pts) > 2:
                return pts, bul
            shape = nxt[0]
            used.add(id(shape))
            joined.append(shape)
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


def join_shapes(selected, tolerance: float = DEFAULT_CHAIN_TOLERANCE):
    """Join exactly the selected shapes (in any order and direction) into one
    chain: (points, bulges, closed). ValueError if they don't form one."""
    selected = list(selected)
    if not selected:
        raise ValueError("Nothing selected.")
    used = set()
    result = chain_from(selected, selected[0], tolerance, used)
    if any(id(s) not in used for s in selected):
        raise ValueError("The selected lines don't join end to end into one chain (a gap, or a branch).")
    return result
