"""
logistics.py - parcel identity and the logistics equipment of a layout
(config_parser.Scanner / Diverter / Turntable / Accumulator / GuardDoor /
SafetyCircuit, and Sink capacity). PhysicsEngine owns one LogisticsRuntime
and calls it each fixed step:

  before_drives(h)  safety circuits hold their belts stopped; accumulation
                    zones and auto turntables set their belts' speeds;
                    chutes empty; turntables turn (carrying what is on them)
  steer()           after the belt drive set every parcel's motion: active
                    diverters steer the parcels in their zone onto the branch
  after_step(h)     after the solver: scanners read the parcels entering
                    their zone; diverter / turntable feedback

Parcel identity: a source gives each parcel a barcode (its next number), a
destination from its mix and, at its no-read rate, an unreadable label
(identify). Scanners report them; auto diverters sort by them.

Each device runs itself (control "auto") or follows the PLC's command fields
(control "plc"); the decisions are the functions below (accumulator_runs,
turntable_auto), which plc_logic.py uses too.
"""
import math
import random
import zlib
from typing import Dict, List, Optional, Tuple

from config_parser import Accumulator, Diverter, SimulatorConfig, Transfer, Turntable
from math_utils import get_rect_vertices, point_in_polygon

EYE_ZONE = 0.3              # m: an accumulation zone's photo-eye sees this far back from the zone's end
TABLE_LOADED_S = 0.3        # auto turntable: a parcel on board this long -> stop the belt and turn
TABLE_EMPTY_S = 0.5         # ... the table empty this long -> unloaded


# ---------------------------------------------------------------------------
# Decisions shared with the PLC program (plc_logic.py)
# ---------------------------------------------------------------------------

def accumulator_runs(occupied: List[bool], mode: str, release: bool) -> List[bool]:
    """Which zones run: an empty zone always (to take the next parcel); an
    occupied one when the zone after it is free (singulation) or also
    moving (train); the last one while released."""
    n = len(occupied)
    runs = [True] * n
    for i in range(n - 1, -1, -1):
        if not occupied[i]:
            runs[i] = True
        elif i == n - 1:
            runs[i] = release
        else:
            runs[i] = not occupied[i + 1] or (mode == "train" and runs[i + 1])
    return runs


def turntable_auto(table: Turntable, state: dict, h: float) -> Tuple[int, bool]:
    """Auto mode: (target index, run the table's belt). Loads at autoFrom
    (belt runs until a parcel has been on board a moment), turns to autoTo,
    unloads (belt runs until the table has been empty a moment) and turns
    back."""
    if not table.inPosition:
        return state.get("target", table.index), False
    if table.index == table.autoFrom:
        state["on_s"] = state.get("on_s", 0.0) + h if table.occupied else 0.0
        if state["on_s"] >= TABLE_LOADED_S:
            state["target"], state["on_s"] = table.autoTo, 0.0
            return table.autoTo, False
        return table.autoFrom, True
    state["off_s"] = state.get("off_s", 0.0) + h if not table.occupied else 0.0
    if state["off_s"] >= TABLE_EMPTY_S:
        state["target"], state["off_s"] = table.autoFrom, 0.0
        return table.autoFrom, False
    return table.index, True


def _in_rect(x: float, y: float, cx: float, cy: float, along: float, across: float, rotation: float) -> bool:
    a = math.radians(rotation)
    dx, dy = x - cx, y - cy
    u = dx * math.cos(a) + dy * math.sin(a)
    v = -dx * math.sin(a) + dy * math.cos(a)
    return abs(u) <= along / 2.0 and abs(v) <= across / 2.0


class LogisticsRuntime:
    def __init__(self, engine):
        self.engine = engine
        self.config: SimulatorConfig = engine.config
        self._serial: Dict[str, int] = {}          # per source: parcels made
        self._scanners: Dict[str, dict] = {}
        self._tables: Dict[str, dict] = {}
        self._accumulators: Dict[str, dict] = {}
        self._sinks: Dict[str, dict] = {}
        self._safety: Dict[str, dict] = {}
        self._diverting: List[Tuple[Diverter, float, float]] = []
        self._transfers: Dict[str, dict] = {}
        # Parcels moved by transfers this step: (parcels, vx, vy) - (0, 0) holds them.
        self._moves: List[Tuple[list, float, float]] = []

    @property
    def active(self) -> bool:
        c = self.config
        return bool(c.scanners or c.diverters or c.turntables or c.accumulators or c.doors or c.safety
                    or c.transfers or any(s.capacity for s in c.sinks))

    # -- lifecycle ---------------------------------------------------------------------

    def reset(self) -> None:
        c = self.config
        self._serial = {}
        self._scanners = {s.id: {"seen": set()} for s in c.scanners}
        for s in c.scanners:
            s.code = s.destination = s.readCount = 0
            s.noRead = s.present = False
            s.weight = s.measuredLength = s.measuredWidth = s.measuredHeight = 0.0
        for d in c.diverters:
            d.active = d.present = False
            d.presentCode = d.diverted = 0
        belts = {b.id: b for b in c.belts}
        for state in self._tables.values():                  # put the platform belts back
            belt = state["belt"]
            if belt is not None:
                belt.rotation = state["belt_rotation"]
                belt.x, belt.y = state["belt_xy"]
        self._tables = {}
        for t in c.turntables:
            belt = belts.get(t.beltId) if t.beltId else None
            start = max(0, min(len(t.angles) - 1, t.autoFrom if t.control == "auto" else t.targetIndex))
            t.index = t.targetIndex = start
            t.turning, t.inPosition, t.occupied = False, True, False
            t.angle = t.angles[start] if t.angles else 0.0
            state = {"belt": belt, "riders": [], "auto": {},
                     "design_speed": (belt.speed if belt is not None and belt.speed else 1.0),
                     "belt_rotation": belt.rotation if belt is not None else 0.0,
                     "belt_xy": (belt.x, belt.y) if belt is not None else (0.0, 0.0)}
            self._tables[t.id] = state
            self._place_table_belt(t, state)
        self._accumulators = {}
        for a in c.accumulators:
            zone_belts = [belts[i] for i in a.beltIds if i in belts]
            for b in zone_belts:
                b._zone_hold = False
            self._accumulators[a.id] = {"belts": zone_belts}
            a.zones, a.full = 0, False
        self._transfers = {t.id: {"phase": "down", "t": 0.0, "settle": 0.0, "empty": 0.0} for t in c.transfers}
        for t in c.transfers:
            t.present = t.centered = t.up = False
            t.transferred, t.lift = 0, 0.0
        self._sinks = {s.id: {"full_s": 0.0} for s in c.sinks if s.capacity}
        for s in c.sinks:
            s.count, s.full = 0, False
        for d in c.doors:
            d.closed, d.locked = True, False
        self._safety = {}
        for b in c.belts:
            b._safety_stop = False
        for s in c.safety:
            s.ok, s.tripped = True, False

    # -- parcel identity ------------------------------------------------------------------

    def identify(self, parcel, source) -> None:
        """A new parcel from `source`: its barcode, destination and label."""
        n = self._serial.get(source.id, 0)
        self._serial[source.id] = n + 1
        parcel.barcode = source.barcodeStart + n
        parcel.destination = 0
        if source.destinations:
            weights = source.destinationWeights if len(source.destinationWeights) == len(source.destinations) \
                else None
            parcel.destination = int(random.choices(source.destinations, weights=weights)[0])
        parcel.readable = not (source.noReadRate > 0 and random.random() < source.noReadRate)
        # Its weight in the source's mass range (what a scale reads; the
        # kinematic belt model moves every body alike). From the barcode, so
        # the random sequence - parcel sizes - stays as without identity.
        k = (zlib.crc32(str(parcel.barcode).encode()) % 1000) / 999.0
        parcel.weight = source.minMass + k * (source.maxMass - source.minMass)

    # -- per step --------------------------------------------------------------------------

    def before_drives(self, h: float) -> None:
        c = self.config
        if c.safety:
            for belt in c.belts:
                belt._safety_stop = False
            for circuit in c.safety:
                self._step_safety(circuit)
        for door in c.doors:
            door.locked = bool(door.lock) and door.closed
        for sink in c.sinks:
            if sink.capacity:
                self._step_chute(sink, h)
        for acc in c.accumulators:
            self._step_accumulator(acc)
        for table in c.turntables:
            self._step_table(table, h)
        self._diverting = []
        for d in c.diverters:
            self._step_diverter(d)
        self._moves = []
        for t in c.transfers:
            self._step_transfer(t, h)

    def steer(self) -> None:
        """Active diverters: the parcels in their zone head for the branch;
        transfers hold a pallet centred or carry it across."""
        for parcels, vx, vy in self._moves:
            for parcel in parcels:
                bd = parcel.body.__dict__
                if bd.get("target_v_enable"):
                    bd["target_v"] = (vx, vy)
                    bd["target_w"] = 0.0
                else:
                    parcel.body.velocity = (vx, vy)
                    parcel.body.angular_velocity = 0.0
        if not self._diverting:
            return
        for d, vx, vy in self._diverting:
            for parcel in self._parcels_in(d.x, d.y, d.width, d.height, d.rotation, d.level):
                bd = parcel.body.__dict__
                if bd.get("target_v_enable"):
                    bd["target_v"] = (vx, vy)
                    bd["target_w"] = 0.0
                else:
                    parcel.body.velocity = (vx, vy)
                if not bd.get("_diverted_by") == d.id:
                    bd["_diverted_by"] = d.id
                    d.diverted = (d.diverted + 1) & 0xFFFF

    def after_step(self, h: float) -> None:
        for scanner in self.config.scanners:
            self._step_scanner(scanner)
        for table in self.config.turntables:
            state = self._tables.get(table.id)
            if state is not None:
                table.occupied = bool(self._on_table(table)) or bool(state["riders"])

    # -- helpers -------------------------------------------------------------------------------

    def _parcels_in(self, x, y, along, across, rotation, level) -> List:
        return [p for p in self.engine.parcels
                if p.level == level and _in_rect(*p.body.position, x, y, along, across, rotation)]

    # -- scanners -----------------------------------------------------------------------------

    def _step_scanner(self, scanner) -> None:
        state = self._scanners.setdefault(scanner.id, {"seen": set()})
        inside = self._parcels_in(scanner.x, scanner.y, scanner.width, scanner.height, scanner.rotation,
                                  scanner.level)
        scanner.present = bool(inside)
        seen = state["seen"]
        for parcel in inside:
            if id(parcel) in seen:
                continue
            seen.add(id(parcel))
            readable = getattr(parcel, "readable", True) or scanner.scannerType == "rfid"
            code = getattr(parcel, "barcode", 0)
            scanner.noRead = not readable or not code
            scanner.code = 0 if scanner.noRead else code
            scanner.destination = 0 if scanner.noRead else getattr(parcel, "destination", 0)
            scanner.readCount = (scanner.readCount + 1) & 0xFFFF
            if scanner.scannerType == "dws":
                import heights
                xs = [v.x for v in parcel.shape.get_vertices()]
                ys = [v.y for v in parcel.shape.get_vertices()]
                size = sorted((max(xs) - min(xs), max(ys) - min(ys)), reverse=True)
                scanner.weight = round(getattr(parcel, "weight", None) or parcel.body.mass, 3)
                scanner.measuredLength, scanner.measuredWidth = round(size[0], 3), round(size[1], 3)
                scanner.measuredHeight = round(heights.part_height(parcel), 3)
        # Forget parcels that left (keeps the set small).
        state["seen"] = {i for i in seen if any(id(p) == i for p in inside)}

    # -- diverters ----------------------------------------------------------------------------

    def _step_diverter(self, d: Diverter) -> None:
        inside = self._parcels_in(d.x, d.y, d.width, d.height, d.rotation, d.level)
        d.present = bool(inside)
        first = inside[0] if inside else None
        d.presentCode = getattr(first, "barcode", 0) if first is not None and getattr(first, "readable", True) else 0
        if d.control == "plc":
            d.active = bool(d.divert)
        else:
            d.active = any(getattr(p, "readable", True) and getattr(p, "destination", 0) == d.destination
                           for p in inside)
        if d.active:
            a = math.radians(d.rotation + d.divertAngle)
            self._diverting.append((d, d.speed * math.cos(a), d.speed * math.sin(a)))

    # -- right-angle transfers ----------------------------------------------------------------

    def _step_transfer(self, t, h: float) -> None:
        """down -> (something centred, or the PLC's transfer) lifting -> up:
        the chains carry it across until the zone is empty (auto) or the
        command drops (plc) -> lowering -> down. Down or lifting, what
        reaches the zone's centre is held there (`stop`)."""
        state = self._transfers.setdefault(t.id, {"phase": "down", "t": 0.0, "settle": 0.0, "empty": 0.0})
        inside = self._parcels_in(t.x, t.y, t.width, t.height, t.rotation, t.level)
        a = math.radians(t.rotation)
        ca, sa = math.cos(a), math.sin(a)
        ahead = [p for p in inside
                 if (p.body.position[0] - t.x) * ca + (p.body.position[1] - t.y) * sa >= -0.02]
        t.present = bool(inside)
        t.centered = bool(ahead) and state["phase"] == "down"
        phase = state["phase"]
        if phase == "down":
            state["settle"] = state["settle"] + h if ahead else 0.0
            want = state["settle"] >= 0.15 if t.control == "auto" else bool(t.transfer)
            if want and (t.control == "plc" or ahead):
                state["phase"], state["t"] = "lifting", 0.0
                if inside:
                    t.transferred = (t.transferred + 1) & 0xFFFF
        elif phase == "lifting":
            state["t"] += h
            t.lift = min(1.0, state["t"] / max(1e-3, t.liftTime))
            if t.lift >= 1.0:
                state["phase"], state["empty"] = "up", 0.0
        elif phase == "up":
            state["empty"] = state["empty"] + h if not inside else 0.0
            done = state["empty"] >= 0.1 if t.control == "auto" else not t.transfer
            if done:
                state["phase"], state["t"] = "lowering", 0.0
        else:                                                    # lowering
            state["t"] += h
            t.lift = max(0.0, 1.0 - state["t"] / max(1e-3, t.liftTime))
            if t.lift <= 0.0:
                state["phase"] = "down"
                state["settle"] = 0.0
        t.up = state["phase"] == "up"
        if t.up:
            c = math.radians(t.rotation + t.crossAngle)
            self._moves.append((inside, t.speed * math.cos(c), t.speed * math.sin(c)))
        elif t.stop and ahead:
            self._moves.append((ahead, 0.0, 0.0))                # the positioning stop

    # -- turntables ---------------------------------------------------------------------------

    def _place_table_belt(self, table: Turntable, state: dict) -> None:
        """The platform belt turned by the table's angle about its centre."""
        belt = state["belt"]
        if belt is None:
            return
        bx, by = state["belt_xy"]
        a = math.radians(table.angle)
        dx, dy = bx - table.x, by - table.y
        belt.x = table.x + dx * math.cos(a) - dy * math.sin(a)
        belt.y = table.y + dx * math.sin(a) + dy * math.cos(a)
        belt.rotation = state["belt_rotation"] + table.angle

    def _on_table(self, table: Turntable) -> List:
        """The parcels on the platform: on its belt (as it is turned now) -
        not the ones waiting at its edge."""
        state = self._tables.get(table.id)
        belt = state["belt"] if state is not None else None
        if belt is not None:
            from surface import belt_contains
            return [p for p in self.engine.parcels if p.level == table.level and belt_contains(belt, *p.body.position)]
        r2 = (table.radius * 0.8) ** 2
        return [p for p in self.engine.parcels
                if p.level == table.level and (p.body.position[0] - table.x) ** 2
                + (p.body.position[1] - table.y) ** 2 <= r2]

    def _step_table(self, table: Turntable, h: float) -> None:
        state = self._tables.get(table.id)
        if state is None or not table.angles:
            return
        belt = state["belt"]
        if table.control == "auto":
            target, run = turntable_auto(table, state["auto"], h)
            table.targetIndex = target
            if belt is not None:
                belt.speed = state["design_speed"] if run else 0.0
                if belt.commandFormat == "vfd":
                    belt.run = run
        target = max(0, min(len(table.angles) - 1, int(table.targetIndex)))
        goal = table.angles[target]
        if not table.turning:
            if target == table.index:
                return
            state["riders"] = self._on_table(table)
            table.turning, table.inPosition = True, False
        step = table.turnSpeed * h
        delta = goal - table.angle
        turn = delta if abs(delta) <= step else math.copysign(step, delta)
        table.angle += turn
        a = math.radians(turn)
        for part in state["riders"]:                         # what is on it turns with it
            px, py = part.body.position
            dx, dy = px - table.x, py - table.y
            part.body.position = (table.x + dx * math.cos(a) - dy * math.sin(a),
                                  table.y + dx * math.sin(a) + dy * math.cos(a))
            part.body.angle += a
            part.body.velocity = (0.0, 0.0)
            part.body.angular_velocity = 0.0
            part.body.__dict__["target_v_enable"] = False
        self._place_table_belt(table, state)
        if abs(goal - table.angle) < 1e-9:
            table.index = target
            table.turning, table.inPosition = False, True
            state["riders"] = []

    # -- accumulation -----------------------------------------------------------------------------

    def _zone_occupied(self, belt) -> bool:
        from cad.ports import belt_ports
        ports = belt_ports(belt)
        if ports is None:
            return False
        out = ports.outlet
        a = math.radians(out.heading)
        cx, cy = out.x - math.cos(a) * EYE_ZONE / 2.0, out.y - math.sin(a) * EYE_ZONE / 2.0
        return bool(self._parcels_in(cx, cy, EYE_ZONE, belt.beltWidth, out.heading, belt.level))

    def _step_accumulator(self, acc: Accumulator) -> None:
        state = self._accumulators.get(acc.id)
        if state is None or not state["belts"]:
            return
        occupied = [self._zone_occupied(b) for b in state["belts"]]
        acc.zones = sum(1 << i for i, o in enumerate(occupied) if o) & 0xFFFF
        acc.full = all(occupied)
        release = True if acc.control == "auto" else bool(acc.release)
        # A held zone stops its belt (its drive's setpoint); a running one
        # moves at whatever the belt is commanded - by the PLC, or its design
        # speed in a layout without one.
        for belt, run in zip(state["belts"], accumulator_runs(occupied, acc.mode, release)):
            belt._zone_hold = not run

    # -- chutes ---------------------------------------------------------------------------------

    def chute_takes(self, sink) -> bool:
        """A parcel reaches `sink`: True if it goes in (and is counted)."""
        if not sink.capacity:
            return True
        if sink.count >= sink.capacity:
            sink.full = True
            return False
        sink.count += 1
        sink.full = sink.count >= sink.capacity
        return True

    def _step_chute(self, sink, h: float) -> None:
        state = self._sinks.setdefault(sink.id, {"full_s": 0.0})
        emptied = sink.empty and not state.get("last_empty", False)
        state["last_empty"] = sink.empty
        if sink.full and sink.emptyTime > 0:
            state["full_s"] += h
            if state["full_s"] >= sink.emptyTime:
                emptied = True
        if emptied:
            sink.count, sink.full, state["full_s"] = 0, False, 0.0

    # -- safety ---------------------------------------------------------------------------------

    def _step_safety(self, circuit) -> None:
        c = self.config
        state = self._safety.setdefault(circuit.id, {"last_reset": False})
        buttons = {b.id: b for b in c.buttons}
        doors = {d.id: d for d in c.doors}
        sensors = {s.id: s for s in c.sensors}

        # An emergency stop trips while pressed (a latching one: until pulled out).
        cause = any(buttons[i].pressed for i in circuit.stopButtonIds if i in buttons) \
            or any(not doors[i].closed for i in circuit.doorIds if i in doors) \
            or any(_blocked(sensors[i]) for i in circuit.curtainIds if i in sensors)
        reset_button = buttons.get(circuit.resetButtonId) if circuit.resetButtonId else None
        reset = bool(circuit.reset) or (reset_button is not None and bool(reset_button.pressed))
        edge = reset and not state["last_reset"]
        state["last_reset"] = reset
        if cause:
            circuit.tripped = True
        elif circuit.tripped and edge:
            circuit.tripped = False
        circuit.ok = not circuit.tripped and not cause
        if not circuit.ok:
            scope = set(circuit.beltIds)
            for belt in c.belts:
                if not scope or belt.id in scope:
                    belt._safety_stop = True

    # -- the operator ---------------------------------------------------------------------------

    def toggle_door(self, door_id: str) -> Optional[bool]:
        """Open / close a guard door (a locked one stays shut). Its new
        `closed`, or None if there is no such door."""
        door = next((d for d in self.config.doors if d.id == door_id), None)
        if door is None:
            return None
        if door.closed and door.locked:
            return True
        door.closed = not door.closed
        return door.closed


def _blocked(sensor) -> bool:
    """A light curtain is interrupted: its output says blocked (any beam)."""
    value = sensor.sensor_states
    on = ("1" in value) if isinstance(value, str) else bool(value)
    return on != bool(sensor.invert)


def scanner_zone(scanner) -> List[Tuple[float, float]]:
    return get_rect_vertices(scanner.x, scanner.y, scanner.width, scanner.height, scanner.rotation)


def contains(scanner_like, x: float, y: float) -> bool:
    return point_in_polygon(x, y, scanner_zone(scanner_like))
