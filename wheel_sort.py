"""
wheel_sort.py - the mock PLC's control of a wheel diverter (pop-up wheels:
enable, and a speed and an angle per row of wheels) by a plan of directions
and the parcels tracked onto it.

  plan      one direction per parcel, in the order they come: from the
            scanner upstream - each read appends the angle for its
            destination (the diverter's divertAngle for its destination, 0 =
            straight on for any other, a no-read included). E.g. 0, -30, 0,
            0, -30, -30.
  tracking  the IR eye just before the deck sees each parcel: its front
            (rising edge) takes the next direction from the plan, its tail
            (falling edge) gives its length. From there its position is
            estimated - along the line by the line belt's encoder up to the
            deck, then by the wheels under it (each row's commanded speed and
            angle, with the swivel's rate), across by the same.
  rows      the deck is `rows` rows of wheels one after another (row 0
            first), each swivelling on its own (Diverter.rows; one row: the
            whole deck). A row points the way of the oldest parcel not yet
            clear of it, and never turns while that one is on it: the moment
            it is clear, the row turns to the next parcel's direction - so a
            parcel carried off the side keeps the rows it still covers, while
            the rows it has left already turn for the parcel behind.
  clear     every corner of a parcel past the row's end, or carried off the
            deck's side: the parcel as a rectangle - its measured length, its
            width taken as the deck's (the eye does not measure it) - square
            to the travel or turned to its direction (a diverted parcel turns
            with the wheels), with margins for what the estimate cannot know
            (a parcel carried sideways lags the wheels).

Everything is measured along the diverter's travel (u, m, 0 at the deck's
centre) and across it (v, + left). Times from the plant's clock (the scan's
dt), distances from the encoder (design rule C7).
"""
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional

import equipment_geometry
from device_kinds import wheel_rows
from logistics import WHEEL_SWIVEL
from plc_logic import occupied

EYE_REACH_M = 1.5          # an IR eye on the line at most this far before the deck is its eye
SCANNER_REACH_M = 6.0      # a scanner on the line at most this far before the deck feeds its plan
CLEAR_ALONG_M = 0.05       # clear of a row: past its end by this ...
CLEAR_ACROSS_M = 0.20      # ... or past the deck's side by this: a parcel carried off the side lags the
                           # wheels (up to 0.19 m behind the estimate on merge_demo)
SAMPLES = (0.1, 0.3, 0.5, 0.7, 0.9)   # where along a parcel the wheels under it are taken


@dataclass
class Tracked:
    ident: int
    angle: float                      # its direction (deg to the travel)
    head: float                       # its front, along the travel (u, m)
    length: Optional[float] = None    # measured at the eye (None: still on it)
    lateral: float = 0.0              # carried across by the wheels (m, + left)


@dataclass
class WheelSorter:
    diverter: object
    eye: object                       # the IR sensor before the deck
    scanner: Optional[object]         # where the plan comes from (None: everything straight on)
    line_belt: object                 # the belt carrying the parcels to the deck (its encoder)
    eye_u: float                      # the eye along the travel (m, 0 = the deck's centre)
    spot: float = 0.05                # the eye's beam width
    plan: Deque[float] = field(default_factory=deque)
    items: List[Tracked] = field(default_factory=list)
    row_cmd: List[float] = field(default_factory=list)     # each row's angle, as commanded
    row_est: List[float] = field(default_factory=list)     # where each row points, as the PLC reckons
    row_speed: List[float] = field(default_factory=list)   # each row's speed, as commanded
    was_blocked: bool = False
    last_read: Optional[int] = None
    next_id: int = 1
    switches: int = 0
    late: int = 0
    off_deck: int = 0
    no_plan: int = 0

    def __post_init__(self):
        n = self.rows
        self.row_cmd, self.row_est, self.row_speed = [0.0] * n, [0.0] * n, [0.0] * n

    # the deck, along / across the travel
    @property
    def rows(self) -> int:
        return wheel_rows(self.diverter)

    @property
    def deck_start(self) -> float:
        return -float(self.diverter.width) / 2.0

    @property
    def deck_end(self) -> float:
        return float(self.diverter.width) / 2.0

    @property
    def half_across(self) -> float:
        return float(self.diverter.height) / 2.0

    @property
    def bounds(self):
        """Each row's strip along the travel (equipment_geometry.row_bounds; one row: the deck)."""
        if self.rows == 1:
            return [(self.deck_start, self.deck_end)]
        return equipment_geometry.row_bounds(float(self.diverter.width), equipment_geometry.grid_of(self.diverter)[0])

    def row_end(self, i: int) -> float:
        return self.bounds[i][1]

    def row_at(self, u: float) -> Optional[int]:
        return equipment_geometry.row_at(self.bounds, u)

    @property
    def angle_cmd(self) -> float:
        """Row 0's angle: where the deck sends what comes on it next."""
        return self.row_cmd[0]

    def angle_for(self, destination: int) -> float:
        d = self.diverter
        return equipment_geometry.wheel_steer(d.divertAngle) if destination and destination == d.destination else 0.0

    def _reach(self, item: Tracked):
        """(along, across) half extents of its footprint: square or turned to its direction."""
        half_l, half_w = (item.length or 0.0) / 2.0, self.half_across
        reach_u = reach_v = 0.0
        for deg in {0.0, item.angle}:
            c, s = abs(math.cos(math.radians(deg))), abs(math.sin(math.radians(deg)))
            reach_u = max(reach_u, half_l * c + half_w * s)
            reach_v = max(reach_v, half_l * s + half_w * c)
        return reach_u, reach_v

    def clear_of(self, item: Tracked, row: int) -> bool:
        """Every corner of it past the row's end, or carried off the deck's side (with margins)."""
        if item.length is None:
            return False
        reach_u, reach_v = self._reach(item)
        centre = item.head - item.length / 2.0
        return (centre - reach_u >= self.row_end(row) + CLEAR_ALONG_M
                or abs(item.lateral) - reach_v >= self.half_across + CLEAR_ACROSS_M)

    def is_off(self, item: Tracked) -> bool:
        return self.clear_of(item, self.rows - 1)

    def on_row(self, item: Tracked, row: int) -> bool:
        """Any of it on the row now (its front past the row's start, not clear of it)."""
        return item.head > self.bounds[row][0] and not self.clear_of(item, row)

    def _velocity(self, item: Tracked, line_v: float):
        """(along, across) the wheels and the belt move it at: the average over points along it."""
        vu = vv = 0.0
        length = item.length if item.length is not None else max(0.0, item.head - self.eye_u)
        for f in SAMPLES:
            row = self.row_at(item.head - f * length)
            if row is None:
                vu += line_v
                continue
            a = math.radians(self.row_est[row])
            vu += self.row_speed[row] * math.cos(a)
            vv += self.row_speed[row] * math.sin(a)
        return vu / len(SAMPLES), vv / len(SAMPLES)

    def step(self, dt: float, travel: float, line_speed: float, running: bool, events: List[str]) -> None:
        """One scan: `travel` the line belt's encoder travel since the last scan (m), `line_speed`
        its speed now - the pace along the travel the wheels keep."""
        d = self.diverter
        n = self.rows
        # the plan: one direction per read
        if self.scanner is not None and self.scanner.readCount != self.last_read:
            if self.last_read is not None and self.scanner.readCount:
                self.plan.append(self.angle_for(self.scanner.destination))
            self.last_read = self.scanner.readCount
        # the estimate of the wheels: each row swivels at its rate, turns at its commanded speed
        turn = WHEEL_SWIVEL * dt
        for i in range(n):
            self.row_est[i] += max(-turn, min(turn, self.row_cmd[i] - self.row_est[i]))
        # the parcels move: before the deck by the belt's encoder, on it by the wheels under them
        line_v = travel / dt if dt > 0 else abs(line_speed)
        for item in self.items:
            if item.head <= self.deck_start:
                item.head += travel
            else:
                vu, vv = self._velocity(item, line_v)
                item.head += vu * dt
                item.lateral += vv * dt
        # the eye: a front takes the next direction, a tail gives the length
        blocked = occupied(self.eye)
        half = abs(travel) / 2.0                     # the edge came sometime in the scan: half of it
        if blocked and not self.was_blocked:
            if self.plan:
                angle = self.plan.popleft()
            else:
                angle = 0.0
                self.no_plan += 1
                events.append(f"wheels {d.id}: a parcel at {self.eye.id} without a plan - straight on")
            # the beam is `spot` wide: blocked as the front reaches its near edge
            self.items.append(Tracked(self.next_id, angle, self.eye_u - self.spot / 2.0 + half))
            self.next_id += 1
        elif not blocked and self.was_blocked and self.items:
            item = next((i for i in self.items if i.length is None), None)
            if item is not None:
                item.length = max(0.05, item.head - (self.eye_u + self.spot / 2.0 + half))   # clear past its far edge
        self.was_blocked = blocked
        # forget what is off the deck
        for item in [i for i in self.items if self.is_off(i)]:
            self.items.remove(item)
            self.off_deck += 1
        # each row: the way of the oldest parcel not yet clear of it (held while it is on it), else
        # the way of the next one planned
        for i in range(n):
            owner = next((it for it in self.items if not self.clear_of(it, i)), None)
            target = owner.angle if owner is not None else (self.plan[0] if self.plan else self.row_cmd[i])
            if target != self.row_cmd[i]:
                if owner is not None and self.on_row(owner, i) and abs(self.row_est[i] - target) > 1.0:
                    self.late += 1                   # it reached the row first: that part went the way before
                self.row_cmd[i] = target
                if i == 0:
                    self.switches += 1
        # the commands: enabled while running, each row at the line's pace along the travel
        speeds = [abs(line_speed) / max(math.cos(math.radians(a)), 0.2) for a in self.row_cmd]
        self.row_speed = speeds if running else [0.0] * n
        d.enable = bool(running)
        if n > 1:
            d.rowAngles, d.rowSpeeds = list(self.row_cmd), list(speeds)
        d.commandAngle, d.commandSpeed = self.row_cmd[0], speeds[0]

    def plan_text(self) -> str:
        coming = [f"{i.angle:+.0f}" for i in self.items] + [f"{a:+.0f}" for a in self.plan]
        return ", ".join(coming) or "-"

    def summary(self) -> str:
        rows = "" if self.rows == 1 else f", rows [{', '.join(f'{a:+.0f}' for a in self.row_cmd)}]"
        return (f"{self.diverter.id}: plan [{self.plan_text()}]{rows}, {len(self.items)} tracked, "
                f"{self.switches} turns, {self.late} late, {self.no_plan} without a plan")


def along(d, x: float, y: float):
    """(u, v) of a point in the diverter's frame: u along its travel from its centre, v across."""
    a = math.radians(d.rotation)
    dx, dy = x - d.x, y - d.y
    return dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)


def sorters_for(config, line_belt) -> List[WheelSorter]:
    """A sorter per wheel diverter under PLC control with an IR eye on its line just before it
    (and the nearest scanner further back for its plan), found from the geometry."""
    out = []
    for d in config.diverters:
        if d.diverterType != "wheels" or getattr(d, "control", "plc") != "plc":
            continue
        start = -float(d.width) / 2.0

        def before(item, reach):
            u, v = along(d, item.x, item.y)
            return start - reach <= u <= start + 1e-6 and abs(v) <= float(d.height) / 2.0, u

        eyes = [(u, s) for s in config.sensors if getattr(s, "sensorType", "ir") == "ir"
                for ok, u in [before(s, EYE_REACH_M)] if ok]
        if not eyes:
            continue
        eye_u, eye = max(eyes, key=lambda e: e[0])                # the nearest before the deck
        scanners = [(u, s) for s in config.scanners for ok, u in [before(s, SCANNER_REACH_M)] if ok and u < eye_u]
        scanner = max(scanners, key=lambda e: e[0])[1] if scanners else None
        out.append(WheelSorter(d, eye, scanner, line_belt, eye_u, spot=float(getattr(eye, "width", 0.05) or 0.05)))
    return out
