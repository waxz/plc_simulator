"""
plc_merge.py - a merge line's inductions by a slot train: the PLC program
the AX library will carry (ax_dev design 4.7 item 4, 4.8, reviews 35-36),
written here first to check the workflow against the simulator's physics.

Like every program of the mock PLC (plc_logic.py) it sees only its I/O image
- eyes, banners, the belts' speed feedback, buttons - never the parcels.

  the train    the merge line's slots (cars) a pitch apart, riding the line:
               every scan each moves by the line's travel - its measured
               speed (actualSpeed) times the scan - a car couples on at the
               line's start as the last moves half a pitch on, uncouples
               past the end (SlotTrain)
  a client     one per induction (a gate sensor `<chain>_gate` over the joint of
               the metering belt `<chain>_a` and the sync section `<chain>_b` ...
               `<chain>`; for a single belt, that belt): a load is admitted
               across the gate only while the section is free, crossing with
               both belts at one speed (never held on the joint); its
               detection ends when the gate clears (its length: the travel
               from front to tail); it stops wholly on the section and asks
               for a car it can reach; once granted, the sync tracker drives
               the section so the load reaches the port in its car at the
               line's speed along it; at the commit point it is on its car or
               it is stopped (missed: it asks again at rest); past the port it
               has landed and its car is occupied. Meanwhile the metering belt
               holds the next load short of the gate (InductionClient)
  the tracker  belt_control's law (github.com/waxz/belt_control, 9889cdc): the
               trigger when the gap to the target crosses 0.75 dv^2 / a, a
               smoothstep ramp over 1.5 dv / a, a PID on the position error
               (Kp 8, Ki 0, Kd 0: its gains) (SyncTracker)

What the PLC tracks is shown in the layout's track-box rows (trackedLoads by
role): each client's car (slot), the load it drives (sync), and the loads in
occupied cars (load).

    python plc_mock.py merge_demo.json --program merge-train ...
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import ax_geometry
import devices
from config_parser import SimulatorConfig
from plc_logic import (DEFAULT_SPEED, MachineControl, _command_frequency, _design_speed, banner_profiles,
                       measure_banners, occupied)

PITCH_M = 1.0               # a car: the longest load (0.4 m) and the gap to the next
LOAD_M = 0.4                # a load's nominal (largest) length
# Head to head: a load's front is aligned with this point of its car, ahead of the car's centre
# (the largest load then sits centred; a shorter one nearer the front). Only the front is needed:
# the gate sees it the moment the load arrives, so a load asks for its car while it crosses.
FRONT_IN_CAR_M = LOAD_M / 2.0
INDUCT_MPS = 1.0            # the metering belt (human-assisted induction) and a crossing load
KP, KI, KD = 8.0, 0.0, 0.0  # belt_control's gains
COMMIT_TOL_M = 0.10         # at the commit point the load within this of its car's centre
LEAD_MARGIN_M = 0.15        # a car granted at least this beyond the ramp's need
# How far downstream of where the chain's axis meets the line's edge a side
# load really lands: a commissioning value, measured from where the loads land
# in their cars (track boxes, tracking check). merge_demo.json's 30-degree ports
# need none: their loads land within 31 mm of their car's centre (2026-11-15;
# the 0.09 - 0.17 m seen before came from loads held across the gate's joint).
SIDE_LAND_OFFSET_M = 0.0


def required_gap(v0: float, vf: float, a_max: float) -> float:
    dv = vf - v0
    return 0.0 if dv <= 0.0 else 0.75 * dv * dv / a_max


def smoothstep(v0: float, vf: float, a_max: float, t: float):
    """(v, a, x) of the C1 ramp from v0 to vf, t after the trigger."""
    dv = vf - v0
    if dv <= 0.0:
        return vf, 0.0, vf * max(t, 0.0)
    T = 1.5 * dv / a_max
    if t <= 0.0:
        return v0, 0.0, 0.0
    if t >= T:
        return vf, 0.0, (v0 + vf) / 2.0 * T + vf * (t - T)
    s = t / T
    return (v0 + dv * (3 * s * s - 2 * s ** 3), dv / T * (6 * s - 6 * s * s),
            v0 * t + dv * t ** 3 / T ** 2 - dv * t ** 4 / (2 * T ** 3))


@dataclass
class SyncTracker:
    """belt_control's ConveyorSyncController with a PID on the position error;
    positions along the sync section towards the port (the port at 0)."""
    kp: float = KP
    ki: float = KI
    kd: float = KD
    active: bool = False
    t_trigger: float = 0.0
    x_trigger: float = 0.0
    v_trigger: float = 0.0
    prev: Optional[tuple] = None          # (t, gap, x, v) of the last scan
    integral: float = 0.0
    last_error: float = 0.0
    x_ref: float = 0.0
    v_ref: float = 0.0
    error: float = 0.0

    def reset(self):
        self.active, self.prev, self.integral, self.last_error, self.error = False, None, 0.0, 0.0, 0.0

    def update(self, t, target_x, target_v, x, v, a_max, v_max):
        gap = x - target_x                         # > 0: its car still behind it (it waits for it)
        if self.prev is None:
            self.prev = (t, gap, x, v)
        t0, gap0, x0, v0 = self.prev
        dt = max(t - t0, 0.0)
        if not self.active:
            need = required_gap(v, target_v, a_max)
            if gap <= need:                        # crossed (interpolated in the scan), or already inside
                alpha = 0.0 if gap0 <= need or abs(gap0 - gap) < 1e-9 else                     min(max((gap0 - need) / (gap0 - gap), 0.0), 1.0)
                self.active = True
                self.t_trigger = t0 + alpha * (t - t0)
                self.x_trigger = x0 + alpha * (x - x0)
                self.v_trigger = v0 + alpha * (v - v0)
        self.prev = (t, gap, x, v)
        if not self.active:
            # belt_control keeps a moving parcel at its speed until the trigger; here the
            # load waits at rest for its car (its lead was planned from rest at the gate)
            return 0.0
        v_ref, _a, x_rel = smoothstep(self.v_trigger, target_v, a_max, max(t - self.t_trigger, 0.0))
        self.x_ref, self.v_ref = self.x_trigger + x_rel, v_ref
        self.error = self.x_ref - x
        command = v_ref + self.kp * self.error
        if self.ki and 0.0 < command < v_max:      # no wind-up while clamped
            self.integral += self.error * dt
        command += self.ki * self.integral + (self.kd * (self.error - self.last_error) / dt if dt > 0 else 0.0)
        self.last_error = self.error
        return min(max(command, 0.0), v_max)


@dataclass
class Car:
    s: float                      # its centre along the line, m
    state: str = "free"           # free, granted, occupied
    client: Optional[str] = None
    load: int = 0
    length: float = LOAD_M        # its load's (measured at the gate)
    width: float = 0.3            # its load's, across it
    yaw: Optional[float] = None   # its load's heading (deg, world): its chain's - it keeps it on the line
    centre_offset: float = 0.0    # its load's centre to the car's centre, along the line, m
    lateral: float = 0.0          # ... and across the line (+ left): where it came in from the side


class SlotTrain:
    """The merge line's cars, riding it (the AX SlotTrain, TrainAdvance, TrainServe)."""

    def __init__(self, length: float, pitch: float = PITCH_M, before: float = 0.0):
        self.length, self.pitch = length, pitch
        self.before = before      # the train reaches this far back before the line's start (a straight induction's lead)
        self.cars: List[Car] = [Car(0.0)]
        self.speed = 0.0
        self.grants = self.landings = self.misses = 0
        self.empty_out = 0        # cars that left the line free: capacity no induction used

    def advance(self, step: float, speed: float):
        self.speed = speed
        for car in self.cars:
            car.s += step
        self.empty_out += sum(1 for c in self.cars if c.s > self.length + self.pitch / 2.0 and c.state == "free")
        self.cars = [c for c in self.cars if c.s <= self.length + self.pitch / 2.0]
        while not self.cars or self.cars[-1].s - self.pitch >= -self.before - self.pitch / 2.0:
            self.cars.append(Car((self.cars[-1].s if self.cars else self.pitch) - self.pitch))

    def grant(self, client: "InductionClient") -> Optional[Car]:
        best = None
        for car in self.cars:
            d = client.port + client.land_offset - client.car_point(car)      # its target point to the port
            best_d = None if best is None else client.port + client.land_offset - client.car_point(best)
            if car.state == "free" and d >= client.min_lead and (best is None or d < best_d):
                best = car
        if best is not None:
            best.state, best.client, best.load = "granted", client.name, client.head.ident
            self.grants += 1
        return best


@dataclass
class QueuedLoad:
    ident: int
    front: float = 0.0            # its front past the gate, along the chain, m
    length: float = LOAD_M        # measured at the gate: the travel from its front to its tail passing
    width: float = 0.3            # ... and across: a banner gate's widest blocked span (an IR eye: nominal)

    @property
    def centre(self) -> float:
        return self.front - self.length / 2.0


@dataclass
class InductionClient:
    name: str                     # the chain: belt_2
    gate: object                  # its gate sensor
    metering: Optional[object]    # the belt before the gate (None: the gate is on the sync belt)
    section: List[object]         # the sync section's belts, in order
    to_port: float                # gate -> the port along the chain, m
    port: float                   # the port along the line, m
    angle: float                  # the chain to the line, rad
    origin: tuple                 # the gate's point (x, y) and the chain's heading there (rad)
    a_max: float
    v_max: float
    queue: List[QueuedLoad] = field(default_factory=list)
    state: str = "idle"           # idle, requesting, tracking, committed, stopping (after a miss)
    car: Optional[Car] = None
    tracker: SyncTracker = field(default_factory=SyncTracker)
    command: float = 0.0
    was_blocked: bool = False
    landed: int = 0
    missed: int = 0
    errors: List[float] = field(default_factory=list)   # the load to its car at the commit point, m
    v_line: float = 1.0           # the line's speed (the last scan's)
    v_now: float = 0.0            # its section's speed (the last scan's)
    land_offset: float = 0.0      # how far downstream of the port its loads really land (commissioned), m
    align: str = "head"           # head: the load's front to its car's front target; center: centre to centre
    max_length: float = PITCH_M   # a load longer than this (the gate still blocked) gets no car: it is held
    gate_spot: float = 0.0        # the gate's own size along the travel (an IR eye's spot; a banner's beams: 0)
    oversize: int = 0             # loads held for their length
    checked: bool = False         # its car checked against its measured length (once its tail is past)
    released: int = 0             # cars given back: out of reach once the length was known
    short_ok: bool = False        # its car checked in time for the short ramp (once, at rest)
    spans: List[tuple] = field(default_factory=list)   # (belt, from, to) along the chain, the gate at 0
    follow_command: float = 0.0   # the section's belts behind the leading load: the next load's command

    def belt_under(self, x: float):
        """The chain's belt at x (along it from the gate): the one carrying a load whose centre is there."""
        for belt, a, b in self.spans:
            if a <= x < b:
                return belt
        return self.spans[0][0] if x < self.spans[0][1] else self.spans[-1][0]

    @property
    def follower(self):
        """The load behind the leading one (on the gate or past it), if any."""
        return self.queue[1] if len(self.queue) > 1 else None

    @property
    def leader_driven(self) -> bool:
        """The leading load is being driven into its car (its ramp started): the next may come on."""
        return self.car is not None and self.head is not self.crossing and \
            (self.tracker.active or self.short_t0 is not None)
    induct: Optional[object] = None   # the eye at the chain's start: loads coming to the gate
    induct_to_gate: float = 0.0       # along the metering belt, m
    induct_was: bool = False
    approaching: List[float] = field(default_factory=list)  # fronts before the gate (< 0), m
    crossing: Optional[QueuedLoad] = None   # the load passing the gate now (its detection)
    v_admit: float = 1.0          # both belts while a load crosses the gate: the metering belt's speed
    on_gate: int = 0              # loads that reached the gate behind another still on the chain (no error)
    unscheduled: int = 0          # loads carried past the port behind the one ahead, without a car
    pos_error: float = 0.0        # tracking error: reference - load, along the line, m (+: behind it); 0 until driven
    speed_error: float = 0.0      # ... reference speed - its speed, along the line, m/s; 0 until driven
    short: Optional[float] = None # the granted load is too close for the whole ramp: the speed it reaches
    short_t0: Optional[float] = None
    short_x0: float = 0.0

    @property
    def head(self) -> Optional[QueuedLoad]:
        return self.queue[0] if self.queue else None

    @property
    def min_lead(self) -> float:
        """How far before the port a car's front target must be for the head load to make it: from
        where it comes to rest (it may still be crossing the gate, moving)."""
        stop = self.v_now * self.v_now / (2.0 * self.a_max)
        # while it crosses it runs on through the gate: it can be controlled once its tail is past,
        # its front a (nominal) load past the gate at least
        extra = max(LOAD_M - self.head.front, 0.0) if self.head is self.crossing else 0.0
        # ... and its car comes on at the line's speed meanwhile: crossing, then braking
        until = (extra / max(self.v_now, 0.1) if extra > 0.0 else 0.0) + self.v_now / self.a_max
        return self.lead_at(self.load_point(self.head) + extra + stop) + self.v_line * until

    def load_point(self, load) -> float:
        """The point of a load the control aligns, past the gate along the chain: its front (head),
        or its centre (center: its length measured, once its tail has passed the gate)."""
        return load.front if self.align == "head" else load.front - load.length / 2.0

    def car_point(self, car) -> float:
        """The point of a car its load's point is aligned with, along the line."""
        return car.s + FRONT_IN_CAR_M if self.align == "head" else car.s

    def lead_at(self, front: float) -> float:
        """How far before the port its car's front target must be for a load at rest with its
        front here: its distance and the ramp's gap; a load too close for the whole ramp
        (short_speed) needs its car the ramp's time away."""
        d_load = self.to_port - front
        vf = self.cos_speed(self.v_line)
        short = self.short_speed(d_load)
        if short is None:
            chain = d_load + required_gap(0.0, vf, self.a_max)
        else:
            chain = vf * self.time_to_port(short, d_load) + self.catch_up(short)
        return math.cos(self.angle) * chain + LEAD_MARGIN_M

    def time_to_port(self, short: float, d_load: float) -> float:
        """From rest at constant acceleration to `short`, then (when that is the line's speed)
        cruising the rest of the way: its time to the port."""
        vf = self.cos_speed(self.v_line)
        return short / self.a_max + max(d_load - short * short / (2.0 * self.a_max), 0.0) / max(vf, 1e-6)

    def catch_up(self, short: float) -> float:
        """What its car gains, chain-wise, while a load at the port at `short` accelerates on to the
        line's speed: (vf - short)^2 / (2 a). The car is that far short of the port when it arrives."""
        vf = self.cos_speed(self.v_line)
        return max(vf - short, 0.0) ** 2 / (2.0 * self.a_max)

    def short_speed(self, d_load: float) -> Optional[float]:
        """When the whole ramp (0.75 v^2 / a) does not fit before the port: the speed a load at rest
        this far from it reaches there at constant acceleration, sqrt(2 a d) (the line's at most);
        it goes on accelerating after the port to the line's speed (catch_up). None: the whole
        ramp fits."""
        vf = self.cos_speed(self.v_line)
        if d_load >= 0.75 * vf * vf / self.a_max:
            return None
        return min(math.sqrt(max(d_load, 0.0) * 2.0 * self.a_max), vf)

    def lead_at_gate(self) -> float:
        """The most it can need: a front just at the gate (a centre half a load behind it)."""
        return self.lead_at(0.0 if self.align == "head" else -LOAD_M / 2.0)

    def cos_speed(self, line_speed: float) -> float:
        return line_speed / max(math.cos(self.angle), 0.2)


class MergeTrainController:
    """The mock PLC's program for a merge line fed by inductions with gates
    (merge_demo.json): the train, a client per induction, every other belt
    at its design speed while running."""

    def __init__(self, config: SimulatorConfig, jam_time: float = 5.0, autostart: bool = False,
                 land_offsets: Optional[Dict[str, float]] = None, align: str = "head",
                 max_length: float = PITCH_M, induct_speed: float = INDUCT_MPS):
        self.config = config
        self.running = autostart
        self.machines = MachineControl(config)
        self.banners = banner_profiles(config)
        self.speeds = {b.id: _design_speed(b) for b in config.belts}
        # a belt or wheel diverter meeting a line at an angle: the line's speed / cos(angle)
        import side_speeds
        self.angled = {i: v for i, (v, _why) in side_speeds.expected_speeds(config).items()}
        for ident, speed in self.angled.items():
            if ident in self.speeds:
                self.speeds[ident] = speed if self.speeds[ident] >= 0 else -speed
        self.belts = {b.id: b for b in config.belts}
        geo = ax_geometry.layout_geometry(config)
        self.geo = {b["id"]: b for b in geo["belts"]}
        places = {d["id"]: d for d in geo["devices"]}
        self.clients: List[InductionClient] = []
        line_id = None
        # The inductions, found from the geometry (not the names): each gate sensor (an id with
        # "gate"); the belt before its joint the metering belt; the belts end to end after it, up
        # to the one that joins the merge line, the sync section; an "induct" eye on the
        # metering belt (else on the gate's belt, before the gate) - see _chains.
        line_id = None
        chains = self._chains(config, places)
        self.chain_belts = {b.id for ch in chains for b in ch["section"] + ([ch["metering"]] if ch["metering"] else [])}
        import drivetrain
        for ch in chains:
            sensor, place, metering, section = ch["gate"], ch["place"], ch["metering"], ch["section"]
            on = self.belts[place["belt"]]
            name = section[-1].id
            end = self.geo[section[-1].id]["end"]
            line_id, s_port, angle = self._port_of(end, self.geo[section[-1].id]["heading_end"], {b.id for b in section})
            heading = math.radians(self.geo[on.id]["heading_end"])
            # the section's limits: each drive's ramp and speedMax, and its motor's (drivetrain)
            accel = min(min(float(getattr(b.drive, "accel", 2.0) or 2.0), drivetrain.motor_limits(b)[1])
                        for b in section)
            vmax = min(min(float(getattr(b.drive, "speedMax", 2.5) or 2.5), drivetrain.motor_limits(b)[0])
                       for b in section)
            client = InductionClient(
                name=name, gate=sensor, metering=metering, section=section, to_port=ch["to_port"], port=s_port,
                angle=angle, origin=(place["x"], place["y"], heading), a_max=0.8 * accel, v_max=vmax)
            client.align, client.max_length = align, max_length
            # the chain's belts along it, the gate at 0: the gate's belt starts s before it
            on_len, s_gate = self.geo[on.id]["length"], place["s"]
            spans = []
            if metering is not None and metering is not on:
                spans.append((metering, -s_gate - self.geo[metering.id]["length"], -s_gate))
            x = -s_gate
            for b in ([on] if on not in section else []) + section:
                if b is on and metering is on:
                    spans.append((on, x, x + on_len))
                    x += on_len
                    continue
                spans.append((b, x, x + self.geo[b.id]["length"]))
                x += self.geo[b.id]["length"]
            client.spans = spans
            client.land_offset = (land_offsets or {}).get(
                name, SIDE_LAND_OFFSET_M if abs(angle) > math.radians(5.0) else 0.0)
            # the induction speed: the metering belt (human-assisted: slow) and the section with it
            # while a load crosses the gate - its length measured at a steady speed; past the gate
            # the section speeds up (the tracker)
            client.v_admit = min(abs(self.speeds.get((metering or on).id, DEFAULT_SPEED)), induct_speed)
            client.gate_spot = float(sensor.width) if getattr(sensor, "sensorType", "ir") == "ir" else 0.0
            # a load asks as its front reaches the gate, and may have to wait for its car: it must be
            # able to stop within half the gate's distance to the port (a straight belt at 2 m/s, its
            # gate 1.25 m before the port, could not - its loads ran onto the line without their car)
            client.v_admit = min(client.v_admit, math.sqrt(client.a_max * client.to_port / 2.0))
            induct = ch["induct"]
            if induct is not None:
                client.induct, client.induct_to_gate = induct, ch["induct_to_gate"]
                if metering is not None:
                    # admitted no faster than the metering belt can stop the next load within half
                    # the induct eye's distance to the gate (2.309 m/s needs 1.3 m at 2 m/s^2 - all
                    # of it: the next load was carried across behind the one admitted)
                    m_accel = min(float(getattr(metering.drive, "accel", 2.0) or 2.0),
                                  drivetrain.motor_limits(metering)[1])
                    client.v_admit = min(client.v_admit, math.sqrt(m_accel * client.induct_to_gate / 2.0))
            self.clients.append(client)
        self.line = self.belts.get(line_id) if line_id else None
        # the train reaches back as far as a load waiting at a gate needs its car before the port
        if line_id:                                             # the leads at the line's design speed
            for c in self.clients:
                c.v_line = abs(self.speeds.get(line_id, DEFAULT_SPEED))
        before = max([0.0] + [c.lead_at_gate() - c.port for c in self.clients])
        self.train = SlotTrain(self.geo[line_id]["length"], before=before) if line_id else None
        self.next_id = 1
        self.t = None
        self.dt = self.wall_dt = 0.0
        self.plant_t = 0.0
        self._load_rows: Dict[int, int] = {}          # a landed load's car -> its track row
        self._counts: Dict[str, int] = {}
        self.travel: Dict[str, float] = {}
        self._pressed: Dict[str, bool] = {}
        # the wheel diverters after the line: a plan of directions, the parcels tracked onto them
        import wheel_sort
        self.sorters = wheel_sort.sorters_for(config, self.line) if self.line is not None else []
        self.machines.own_programs = {s.diverter.id for s in self.sorters}
        self._write_outputs()

    def _chains(self, config, places) -> List[dict]:
        """Each induction from the geometry: its gate, metering belt (or None: the gate is on a
        single belt), sync section (to the belt that joins the line) and induct eye."""
        geo = self.geo

        def next_of(bid):                                       # the belt end to end after it, same heading
            g = geo[bid]
            return next((o for o, h in geo.items() if o != bid and o in self.belts
                         and math.dist(g["end"][:2], h["start"][:2]) <= 0.03
                         and abs((g["heading_end"] - h["heading_start"] + 180.0) % 360.0 - 180.0) < 5.0), None)

        def prev_of(bid):
            return next((o for o in geo if o != bid and o in self.belts and next_of(o) == bid), None)

        import side_speeds
        lines = set()                                           # belts something joins at an angle: merge lines
        for a, ga in geo.items():
            for b, gb in geo.items():
                if a != b and gb["shape"] in ("line", "cut"):
                    on, s = side_speeds._on(ga["end"], gb)
                    angle = side_speeds._wrap(ga["heading_end"] - side_speeds._heading(gb))
                    if on and abs(angle) >= side_speeds.MIN_ANGLE_DEG and s > side_speeds.JOIN_TOL_M:
                        lines.add(b)
        out = []
        for sensor in config.sensors:
            if "gate" not in sensor.id.lower() or sensor.id not in places:
                continue
            place = places[sensor.id]
            on = place["belt"]
            length, s = geo[on]["length"], place["s"]
            after = next_of(on)
            if s >= length - 0.05 and after is not None and after not in lines:
                metering, first = on, after                     # the gate over the joint, at the end of `on`
            elif s <= 0.05 and prev_of(on) is not None:
                metering, first = prev_of(on), on               # ... at the start of `on`
            else:
                metering, first = None, on                      # in the middle of a single belt
            section = [first]
            while True:
                nxt = next_of(section[-1])
                if nxt is None or nxt in lines or nxt in section or len(section) > 20:
                    break
                section.append(nxt)
            # the gate to the section's end
            to_port = (length - s) + sum(geo[b]["length"] for b in section if b != on)
            belts_ = [self.belts[b] for b in section]
            eye_belt = metering if metering is not None else on
            # the gate's place along the eye's belt (on the next belt's start: past the metering belt's end)
            gate_s = geo[metering]["length"] + s if metering is not None and on != metering else s
            inducts = [(places[x.id]["s"], x) for x in config.sensors
                       if "induct" in x.id.lower() and x.id in places and places[x.id]["belt"] == eye_belt]
            ups = [(gate_s - es, x) for es, x in inducts if es < gate_s - 0.05]
            near = min(ups, key=lambda u: u[0]) if ups else None
            out.append({"gate": sensor, "place": place, "metering": self.belts[metering] if metering else None,
                        "section": belts_, "to_port": to_port,
                        "induct": near[1] if near else None, "induct_to_gate": near[0] if near else 0.0})
        return out

    def _port_of(self, point, heading_deg, own):
        """The belt the chain ends on: its id, the port's distance along it, the chain's angle to it."""
        best = None
        for bid, g in self.geo.items():
            if bid in own or bid in self.chain_belts or g["shape"] != "line":
                continue
            (x0, y0, _), (x1, y1, _) = g["start"], g["end"]
            L = math.hypot(x1 - x0, y1 - y0)
            ux, uy = (x1 - x0) / L, (y1 - y0) / L
            s = (point[0] - x0) * ux + (point[1] - y0) * uy
            lateral = abs(-(point[0] - x0) * uy + (point[1] - y0) * ux)
            if -0.05 <= s <= L and lateral <= g["width"] / 2.0 + 0.05:
                if best is None or lateral < best[3]:
                    angle = math.radians(heading_deg) - math.atan2(uy, ux)
                    best = (bid, max(s, 0.0), abs(math.atan2(math.sin(angle), math.cos(angle))), lateral)
        if best is None:
            raise ValueError(f"no belt at the end of the chain at {point}")
        return best[:3]

    def roles(self) -> Dict[str, Optional[str]]:
        out = {"merge_line": self.line.id if self.line else None}
        for c in self.clients:
            out[c.name] = (f"gate {c.gate.id}, section {'+'.join(b.id for b in c.section)}, "
                           f"port {c.port:.2f} m at {math.degrees(c.angle):.0f} deg (lands {c.land_offset:+.3f} m, "
                           f"{c.align} aligned), "
                           f"{c.to_port:.2f} m from the gate")
        return out

    # -- the scan --------------------------------------------------------------------------

    def _travel(self, belt) -> float:
        """The belt's travel since the last scan, m, from its encoder (encoderCount, the plant's
        own measure - design rule C7); its speed x the scan when it has none."""
        counts = getattr(belt, "encoderCount", None)
        per_m = float(getattr(belt.drive, "countsPerM", 0.0) or 0.0) if belt.drive is not None else 0.0
        if counts is None or per_m <= 0.0:
            return float(belt.actualSpeed or 0.0) * self.wall_dt
        last = self._counts.get(belt.id)
        self._counts[belt.id] = int(counts)
        if last is None:
            return 0.0
        step = (int(counts) - last) & 0xFFFFFFFF
        if step >= 0x80000000:                                  # backwards (a UDINT wraps)
            step -= 0x100000000
        return step / per_m

    def step(self, now: Optional[float] = None) -> List[str]:
        """One scan. Every distance comes from the belts' encoders and every time from the plant:
        the line's travel over its measured speed (a simulator running slower or faster than real
        time, or a PLC scan that jitters, then moves nothing out of place - with the CPU's clock
        the cars drifted from the parcels: loads landed in occupied cars)."""
        import time
        now = time.monotonic() if now is None else now
        events: List[str] = []
        self.wall_dt = 0.0 if self.t is None else max(0.0, now - self.t)
        self.t = now
        self.travel = {b.id: self._travel(b) for b in self.config.belts}
        if self.line is not None and abs(float(self.line.actualSpeed or 0.0)) > 0.2:
            self.dt = abs(self.travel[self.line.id] / float(self.line.actualSpeed))     # the plant's clock
        else:
            self.dt = self.wall_dt
        self.plant_t += self.dt
        now = self.plant_t
        self._buttons(events)
        measure_banners(self.banners, self.config, self.dt,
                        lambda belt: self.speeds.get(belt, 0.0) if self.running and belt else 0.0, events)
        if self.train is not None:
            v_line = float(self.line.actualSpeed or 0.0)
            self.train.advance(self.travel[self.line.id], v_line)
            for client in self.clients:
                self._client(client, now, events)
        self._write_outputs()
        self.machines.step(now, self.running, self.speeds, events)
        sorted_ids = set()
        for s in self.sorters:                                   # after the device programs: the plan decides
            s.step(self.dt, self.travel.get(s.line_belt.id, 0.0), float(s.line_belt.actualSpeed or 0.0),
                   self.running, events)
            sorted_ids.add(s.diverter.id)
        for d in self.config.diverters:                          # its wheels at the line's speed / cos(angle)
            if d.id in self.angled and d.id not in sorted_ids and getattr(d, "control", "plc") == "plc":
                # angled: the line's speed / cos(divertAngle); the wheels keep the line's pace at their angle now
                line_v = self.angled[d.id] * math.cos(math.radians(d.divertAngle))
                d.commandSpeed = line_v / math.cos(math.radians(d.commandAngle))
        self._show()                                            # after the device programs (they clear the rows)
        return events

    def _buttons(self, events):
        for b in self.config.buttons:
            pressed = devices.is_pressed_signal(b, b.sensor_states)
            if pressed and not self._pressed.get(b.id):
                name = b.id.lower()
                if "start" in name:
                    self.running = True
                    events.append("START")
                elif "stop" in name or "estop" in name:
                    self.running = False
                    events.append("STOP")
            self._pressed[b.id] = pressed

    def _client(self, c: InductionClient, now: float, events: List[str]):
        """One induction, every scan (the workflow as the user set it, 2026-11-15/18):
        1. a load is admitted across the gate only while the section after it
           is free; the gate sees its front: from then on it is tracked by its
           front and **asks for a car at once**, while it crosses (head to head:
           its front to its car's front target - no length needed); both belts
           move together while it crosses (with the section's command); its
           length is the travel from its front to its tail passing the gate;
        2. granted, the tracker drives it into its car; not yet, it stops (on
           the joint, both belts together) and waits;
        3. meanwhile the metering belt holds the next load short of the gate."""
        blocked = occupied(c.gate)
        v_section = float(c.section[0].actualSpeed or 0.0)
        v_metering = float(c.metering.actualSpeed or 0.0) if c.metering is not None else v_section
        d_section = self.travel[c.section[0].id]
        d_metering = self.travel[c.metering.id] if c.metering is not None else d_section
        for i in range(len(c.approaching)):
            c.approaching[i] += d_metering
        if c.crossing is not None:
            c.crossing.front += d_section
        for load in c.queue:
            if load is not c.crossing:                          # with the belt under its centre
                load.front += self.travel[c.belt_under(load.front - load.length / 2.0).id]
        if c.induct is not None:                                # a load on its way to the gate
            at_induct = occupied(c.induct)
            if at_induct and not c.induct_was:
                c.approaching.append(-c.induct_to_gate)
            c.induct_was = at_induct
        if blocked and not c.was_blocked:                       # a front at the gate: queued, it may ask now
            if c.queue:                                         # another waits: this one waits on the gate
                c.on_gate += 1
            # the front passed somewhere in this scan: half its travel, on average
            c.crossing = QueuedLoad(self.next_id, front=d_section / 2.0)
            c.queue.append(c.crossing)
            self.next_id += 1
            if c.approaching:
                c.approaching.remove(max(c.approaching))
        elif c.was_blocked and not blocked and c.crossing is not None:   # its tail: its length
            # the tail passed somewhere in this scan (half its travel, on average); the gate was
            # blocked for the load's length and its own spot along the travel
            c.crossing.front -= d_section / 2.0
            c.crossing.length = min(max(c.crossing.front - c.gate_spot, 0.05), PITCH_M)
            profile = self.banners.get(c.gate.id)               # a banner: its width too
            if profile is not None and profile.measured is not None:
                c.crossing.width = max(profile.measured[1], 0.05)
            c.crossing = None
        c.was_blocked = blocked
        c.v_line, c.v_now = self.train.speed, v_section
        if c.car is None:
            c.pos_error = c.speed_error = 0.0
        head = c.head
        if not self.running:
            c.command = 0.0
            return
        if head is None:                                        # free: receive the next load at the admit speed
            c.command = c.v_admit
            return
        d_front = c.to_port - head.front                        # its front to the port, along the chain
        d_load = c.to_port - c.load_point(head)                 # the point it is aligned by, to the port
        if head is not c.head or (c.state in ("idle", "requesting", "stopping") and d_front <= 0.0):
            # carried past the port behind the load ahead (its metering belt could not stop it
            # before the gate): it is on the line without a car - counted, and no longer waited for
            c.queue.pop(0)
            c.unscheduled += 1
            events.append(f"{c.name}: load {head.ident} reached the line without a car")
            c.state, c.car = "idle", None
            c.command = 0.0
            return
        crossing = head is c.crossing
        if crossing and head.front > c.max_length and c.state != "oversize":
            # the gate still blocked after the longest load: no car for it - held where it is
            if c.car is not None:
                c.car.state, c.car.client, c.car.load = "free", None, 0
                c.car = None
            c.state = "oversize"
            c.oversize += 1
            events.append(f"{c.name}: load {head.ident} longer than {c.max_length:.2f} m - held, no car: remove it")
        if c.state == "oversize":
            c.command = 0.0
            if not crossing:                                    # the gate clear again: it was taken away
                c.queue.pop(0)
                c.state = "idle"
            return
        if c.state == "stopping":                               # after a miss: at rest before it asks again
            c.command = 0.0
            if abs(v_section) > 0.02:
                return
            c.state = "idle"
        if c.state == "idle":                                   # its front seen: it asks at once (moving)
            c.state = "requesting"
        if c.state == "requesting":
            # it asks as the gate sees its front and runs on through the gate (never stopped on it);
            # center aligned it asks once its length is measured
            if not (c.align == "center" and crossing):
                c.car = self.train.grant(c)
            if c.car is None:
                c.command = c.v_admit if crossing else 0.0      # through the gate; then it waits off it
                return
            c.state = "tracking"
            c.tracker.reset()
            c.short, c.short_t0, c.checked, c.short_ok = None, None, False, False
        if crossing:                                            # granted: still through the gate first
            c.command = c.v_admit
            return
        if not c.checked:
            # its tail past the gate, its length known: its car checked once more - out of reach
            # (a longer load than planned for), it is given back and the load asks again
            c.checked = True
            if c.port + c.land_offset - c.car_point(c.car) < c.min_lead - LEAD_MARGIN_M:
                c.car.state, c.car.client, c.car.load = "free", None, 0
                c.car, c.state = None, "requesting"
                c.released += 1
                c.command = 0.0
                return
            stop_point = c.load_point(head) + v_section * v_section / (2.0 * c.a_max)
            c.short = c.short_speed(c.to_port - stop_point)
        cos = max(math.cos(c.angle), 0.2)
        if c.state in ("tracking", "committed") and c.short is not None and c.car in self.train.cars:
            # too close for the whole ramp: constant acceleration from rest - the front at the port
            # at `short`, then on to the line's speed; the car meanwhile gains catch_up, so it is
            # that far short of the port when the front arrives: aligned once both run at its speed
            d_slot = (c.port + c.land_offset - c.car_point(c.car)) / cos
            v_target = self.train.speed / cos
            ramp_t = c.time_to_port(c.short, d_load)
            if c.short_t0 is None:
                c.command = 0.0
                if abs(v_section) > 0.02:                       # it starts from rest
                    return
                c.short = c.short_speed(d_load) or c.short     # where it came to rest
                ramp_t = c.time_to_port(c.short, d_load)
                needed = v_target * ramp_t + c.catch_up(c.short)
                if not c.short_ok:
                    # checked once, as it comes to rest: its car already too close - given back.
                    # After that the car only comes nearer, so it starts at the first scan the car is
                    # in time (a scan may step past the exact moment: the tracker takes up the rest)
                    if d_slot < needed - 0.05:
                        c.car.state, c.car.client, c.car.load = "free", None, 0
                        c.car, c.state, c.short = None, "requesting", None
                        c.released += 1
                        return
                    c.short_ok = True
                if d_slot > needed:
                    return
                c.short_t0, c.short_x0 = now, -d_load
                c.state = "committed"                           # from here it cannot stop before the port
            tau = now - c.short_t0
            to_line = v_target / c.a_max                        # when it reaches the line's speed
            if tau <= to_line:
                v_ref, x_rel = c.a_max * tau, c.a_max * tau * tau / 2.0
            else:
                v_ref = v_target
                x_rel = v_target * v_target / (2.0 * c.a_max) + v_target * (tau - to_line)
            x_ref = c.short_x0 + x_rel
            c.pos_error, c.speed_error = (x_ref + d_load) * cos, (v_ref - v_section) * cos
            c.command = min(max(v_ref + KP * (x_ref + d_load), 0.0), c.v_max)
            if d_front <= -(head.length + 0.1) and head is not c.crossing:  # its tail off the chain
                self._land(c, head)
                c.landed += 1
                self.train.landings += 1
                c.queue.pop(0)
                c.state, c.car, c.short = "idle", None, None
            return
        if c.state in ("tracking", "committed"):
            if c.car not in self.train.cars:                    # its car left the line: missed
                c.state, c.car = "stopping", None
                c.missed += 1
                self.train.misses += 1
                c.command = 0.0
                return
            # the car's front target to the port, chain-wise; a load lands land_offset downstream of it
            d_slot = (c.port + c.land_offset - c.car_point(c.car)) / cos
            v_target = self.train.speed / cos
            c.command = c.tracker.update(now, -d_slot, v_target, -d_load, v_section, c.a_max, c.v_max)
            # the tracking error: the load against its reference (the ramp that ends on its car), along
            # the line - once the tracker drives it (before: it waits at rest for its car)
            c.pos_error, c.speed_error = (c.tracker.error * cos, (c.tracker.v_ref - v_section) * cos)                 if c.tracker.active else (0.0, 0.0)
            stop = v_section * v_section / (2.0 * c.a_max) + 0.05
            if c.state == "tracking" and d_load <= stop:        # the commit point
                error = (d_slot - d_load) * cos                 # along the line: + the load behind its car
                if abs(error) <= COMMIT_TOL_M:
                    c.state = "committed"
                    c.errors.append(error)
                else:
                    c.car.state, c.car.client, c.car.load = "free", None, 0
                    c.state, c.car = "stopping", None
                    c.missed += 1
                    self.train.misses += 1
                    c.command = 0.0
                    events.append(f"{c.name}: missed its car by {error * 1000:.0f} mm")
                    return
            if d_front <= -(head.length + 0.1) and head is not c.crossing:  # its tail off the chain            # off the chain, onto the line: landed
                self._land(c, head)
                c.landed += 1
                self.train.landings += 1
                c.queue.pop(0)
                c.state, c.car = "idle", None
                c.tracker.reset()

    def _section_commands(self, c: InductionClient) -> Dict[str, float]:
        """Each belt of the section: the leading load's command on the belts from its tail forward;
        behind it, once it is driven, the next load's - the induction speed while that one comes
        on and crosses the gate, stopping it short of the leading load's tail if it would close up
        (a single-belt induction: the one command)."""
        out = {b.id: c.command for b in c.section}
        head = c.head
        if c.metering is None or head is None or not c.leader_driven or not self.running:
            c.follow_command = c.command
            return out
        tail = head.front - head.length
        nxt = c.follower
        if nxt is None:
            follow = c.v_admit                                  # ready for the next load, at its speed
        else:
            v = float(c.belt_under(nxt.front - nxt.length / 2.0).actualSpeed or 0.0)
            room = tail - nxt.front - 0.15                      # its front to the leading tail, a gap kept
            follow = c.v_admit if room > v * v / (2.0 * c.a_max) + 0.05 else 0.0
        c.follow_command = follow
        for belt, a, b in c.spans:
            if belt is not c.metering and belt.id in out and b <= tail - 0.05:   # wholly behind its tail
                out[belt.id] = follow
        return out

    def _land(self, c: InductionClient, head) -> None:
        """Its load is on the line, in its car: the car occupied, with the load's pose - its size and
        its chain's heading, which a load keeps on the line (it is not straightened); its centre
        half its length behind its front (head aligned: the front on the car's target), along its
        own axis, projected onto the line (center aligned: on the car's centre)."""
        car = c.car
        car.state, car.length, car.width = "occupied", head.length, head.width
        line = self.geo[self.line.id]
        (x0, y0, _), (x1, y1, _) = line["start"], line["end"]
        line_heading = math.atan2(y1 - y0, x1 - x0)
        chain_heading = c.origin[2]
        car.yaw = math.degrees(chain_heading)
        rel = chain_heading - line_heading
        car.centre_offset = (FRONT_IN_CAR_M - head.length / 2.0 * math.cos(rel)) if c.align == "head" else 0.0
        # across: where its centre is as it leaves its chain (a side load stays on its side of the line)
        gx, gy, _h = c.origin
        along_chain = head.front - head.length / 2.0
        cx, cy = gx + along_chain * math.cos(chain_heading), gy + along_chain * math.sin(chain_heading)
        lateral = -(cx - x0) * math.sin(line_heading) + (cy - y0) * math.cos(line_heading)
        car.lateral = max(-line["width"] / 2.0, min(line["width"] / 2.0, lateral))

    def _metering(self, c: InductionClient) -> float:
        """The metering belt: with the section while a load crosses the gate;
        bringing the next load while the section is free; else holding it
        short of the gate (its stopping distance before it)."""
        if not self.running:
            return 0.0
        if c.crossing is not None:
            # with the section's first belt while a load crosses: the leading load's command, or the
            # next one's when it crosses behind a leading load being driven
            return c.follow_command if c.crossing is not c.head else c.command
        section_ready = abs(float(c.section[0].actualSpeed or 0.0) - c.v_admit) < 0.05
        first = c.spans[1] if c.spans and c.spans[0][0] is c.metering and len(c.spans) > 1 else None
        if c.queue and c.follower is None and c.leader_driven and first is not None:
            # the leading load driven and its tail past the section's first belt: the next may come
            head = c.head
            if head.front - head.length > first[2] + 0.05 and \
                    abs(float(first[0].actualSpeed or 0.0) - c.v_admit) < 0.05:
                return c.v_admit
        if not c.queue and section_ready:
            # free, and the section at the induction speed (after a load it may still be slowing from
            # the line's): both belts at one speed while the next load crosses - its length right
            return c.v_admit
        speed = c.v_admit                                       # it can stop the next load in time at this
        accel = float(getattr(c.metering.drive, "accel", 2.0) or 2.0)
        hold_at = -(speed * speed / (2.0 * accel) + 0.1)
        nearest = max(c.approaching, default=None)
        return speed if nearest is not None and nearest < hold_at else 0.0

    def _write_outputs(self):
        run = self.running
        driven = {}
        for c in self.clients:
            for b, cmd in self._section_commands(c).items():
                driven[b] = cmd
            if c.metering is not None:
                driven[c.metering.id] = self._metering(c)
        for belt in self.config.belts:
            nominal = driven.get(belt.id, self.speeds.get(belt.id, DEFAULT_SPEED))
            on = run and abs(nominal) > 1e-6
            if belt.commandFormat == "frequency":
                _command_frequency(belt, on, nominal if on else self.speeds.get(belt.id, DEFAULT_SPEED))
            elif belt.commandFormat == "vfd":
                belt.run = on
                belt.reverse = nominal < 0
                belt.speed = abs(nominal) if on else 0.0
            else:
                belt.speed = nominal if run else 0.0

    # -- what it shows ---------------------------------------------------------------------

    def _show(self):
        rows = {"load": [], "slot": [], "sync": []}
        for t in getattr(self.config, "trackedLoads", []):
            rows.setdefault(getattr(t, "role", "load"), []).append(t)
        for rows_of in rows.values():
            for t in rows_of:
                t.active = False
        frame = int(self.config.watchdogs[0].simHeartbeat) if getattr(self.config, "watchdogs", None) else 0
        line = self.geo.get(self.line.id) if self.line else None

        def on_line(s):
            (x0, y0, _), (x1, y1, _) = line["start"], line["end"]
            L = math.hypot(x1 - x0, y1 - y0)
            return x0 + (x1 - x0) * s / L, y0 + (y1 - y0) * s / L, math.degrees(math.atan2(y1 - y0, x1 - x0))

        def fill(t, ident, x, y, yaw, length, width):
            t.active, t.loadId, t.loadX, t.loadY, t.loadYaw = True, ident, x, y, yaw
            t.loadLength, t.loadWidth, t.frame = length, width, frame

        # a fixed row per feed for its car (slot) and its load (sync): a box stays the same box
        def row_of(kind, index):
            of = rows.get(kind, [])
            return of[index] if index < len(of) else None

        values = {v.id: v for v in getattr(self.config, "plcValues", [])}
        for index, c in enumerate(self.clients):
            pos, speed = values.get(f"sync_{c.name}_pos_mm"), values.get(f"sync_{c.name}_speed_mps")
            if pos is not None:
                pos.value = c.pos_error * 1000.0
            if speed is not None:
                speed.value = c.speed_error
            slot, sync = row_of("slot", index), row_of("sync", index)
            if c.car is not None and line is not None and slot is not None:
                x, y, yaw = on_line(c.car.s)
                fill(slot, c.car.load, x, y, yaw, self.train.pitch, line["width"] * 0.9)
            if c.head is not None and sync is not None:
                gx, gy, h = c.origin
                along = c.head.centre
                fill(sync, c.head.ident, gx + along * math.cos(h), gy + along * math.sin(h), math.degrees(h),
                     c.head.length, c.head.width)
        if self.train is not None and line is not None:
            # a landed load keeps its row while its car is on the line; a new one takes a free row
            shown = {id(car): car for car in self.train.cars
                     if car.state == "occupied" and 0.0 <= car.s <= self.train.length}
            self._load_rows = {k: r for k, r in self._load_rows.items() if k in shown}
            taken = set(self._load_rows.values())
            free = [i for i in range(len(rows.get("load", []))) if i not in taken]
            for key, car in sorted(shown.items(), key=lambda kv: -kv[1].s):
                if key not in self._load_rows and free:
                    self._load_rows[key] = free.pop(0)
                if key in self._load_rows:
                    # where its load is: the car's centre and the load's offset, along the line; the
                    # load's own heading (its chain's: a load keeps its turn), its measured size
                    x, y, yaw = on_line(car.s + car.centre_offset)
                    lx, ly = -math.sin(math.radians(yaw)), math.cos(math.radians(yaw))     # the line's left
                    x, y = x + car.lateral * lx, y + car.lateral * ly
                    fill(rows["load"][self._load_rows[key]], car.load, x, y,
                         car.yaw if car.yaw is not None else yaw, car.length, car.width)

    # what a study shows of metrics() by default (sim_study.py)
    HEADLINE = ("train.landings", "train.empty_cars", "train.misses")

    def metrics(self) -> Dict[str, float]:
        """Counters for a study (sim_study.py): the train's, each induction's, each wheel sorter's."""
        out: Dict[str, float] = {}
        if self.train is not None:
            t = self.train
            out.update({"train.grants": t.grants, "train.landings": t.landings, "train.misses": t.misses,
                        "train.empty_cars": t.empty_out})
        for c in self.clients:
            for name in ("landed", "missed", "released", "unscheduled", "oversize"):
                out[f"feed.{c.name}.{name}"] = getattr(c, name)
        for s in self.sorters:
            d = s.diverter.id
            out.update({f"sorter.{d}.turns": s.switches, f"sorter.{d}.late": s.late,
                        f"sorter.{d}.no_plan": s.no_plan})
        return out

    def summary(self) -> str:
        if self.train is None:
            return "no merge line"
        parts = [f"{c.name}: {c.state}, {len(c.queue)} queued, {c.landed} landed, {c.missed} missed"
                 + (f", {c.on_gate} came on behind another" if c.on_gate else "")
                 + (f", {c.unscheduled} without a car" if c.unscheduled else "")
                 + (f", {c.oversize} too long" if c.oversize else "")
                 + (f", {c.released} cars given back" if c.released else "")
                 for c in self.clients]
        return (f"{'RUN' if self.running else 'STOP'} - train {len(self.train.cars)} cars, "
                f"{self.train.grants} grants, {self.train.landings} landed, {self.train.misses} missed; "
                + "; ".join(parts + [s.summary() for s in self.sorters]))


# -- commissioning ---------------------------------------------------------------------------------

def measure_landings(config, seconds: float = 120.0, align: str = "head",
                     land_offsets: Optional[Dict[str, float]] = None, seed: int = 1,
                     dt: float = 0.01) -> Dict[str, List[float]]:
    """Run the layout's physics with this program (in this process) and measure each induction's
    landings: where each landed load's aligned point really is against its car's point, along the
    line, m (+: ahead of it). What a plant's commissioning measures with the track boxes."""
    import random
    from physics import PhysicsEngine
    random.seed(seed)
    engine = PhysicsEngine(config)
    c = MergeTrainController(config, autostart=True, land_offsets=land_offsets, align=align)
    line = c.geo[c.line.id]
    (x0, y0, _), (x1, y1, _) = line["start"], line["end"]
    length = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / length, (y1 - y0) / length
    seen: Dict[int, tuple] = {}
    t = 0.0
    for i in range(int(seconds / dt)):
        engine.update(dt)
        t += dt
        c.step(t)
        if i % 20:
            continue
        on = []
        for v in engine.parcel_views():
            s = (v.x - x0) * ux + (v.y - y0) * uy
            lateral = -(v.x - x0) * uy + (v.y - y0) * ux
            if 0.0 < s < length and abs(lateral) < line["width"] / 2.0 + 0.05:
                extent = max(math.dist(v.points[0], v.points[1]), math.dist(v.points[1], v.points[2]))
                on.append((s, extent))
        for car in c.train.cars:
            # a car well past every port (the last 40 % of the line): its load has settled
            if car.state == "occupied" and 0.6 * length <= car.s <= length - 0.6 and car.load not in seen:
                near = min(on, key=lambda e: abs(e[0] - car.s), default=None)
                if near is not None and abs(near[0] - car.s) < c.train.pitch / 2.0:
                    point = near[0] + (near[1] / 2.0 if align == "head" else 0.0)
                    client = next(cl for cl in c.clients if cl.name == car.client)
                    seen[car.load] = (car.client, point - client.car_point(car))
    out: Dict[str, List[float]] = {cl.name: [] for cl in c.clients}
    for name, error in seen.values():
        out.setdefault(name, []).append(error)
    return out


def commission(config, seconds: float = 120.0, align: str = "head",
               land_offsets: Optional[Dict[str, float]] = None) -> Dict[str, float]:
    """Each induction's landing offset: the mean of its loads' landing errors, added to the offset
    it ran with (so a second run refines the first)."""
    errors = measure_landings(config, seconds, align, land_offsets)
    return {name: round((land_offsets or {}).get(name, 0.0) + sum(e) / len(e), 3)
            for name, e in errors.items() if e}


def main(argv=None) -> int:
    import argparse
    from config_parser import load_config
    ap = argparse.ArgumentParser(description="plc_merge.py: commission a merge line's inductions - "
                                             "measure each one's landing offset in the physics")
    ap.add_argument("layout")
    ap.add_argument("--commission", type=float, default=120.0, metavar="SECONDS",
                    help="how long to run the line (default 120 s)")
    ap.add_argument("--align", choices=("head", "center"), default="head")
    ap.add_argument("--land-offset", action="append", default=[], metavar="FEED=METRES",
                    help="the offsets it runs with (from a run before): refined")
    args = ap.parse_args(argv)
    offsets = parse_offsets(args.land_offset)
    before = measure_landings(load_config(args.layout), args.commission, args.align, offsets)
    result = {name: round(offsets.get(name, 0.0) + sum(e) / len(e), 3) for name, e in before.items() if e}
    for name, e in before.items():
        if e:
            print(f"{name}: {len(e)} landings, error mean {1000 * sum(e) / len(e):+.0f} mm, "
                  f"{1000 * min(e):+.0f} .. {1000 * max(e):+.0f} mm -> offset {result[name]:+.3f} m")
    print("plc_mock.py options: " + " ".join(f"--land-offset {n}={v}" for n, v in result.items())
          + (f" --align {args.align}" if args.align != "head" else ""))
    return 0


def parse_offsets(items) -> Dict[str, float]:
    """["belt_2=-0.09", ...] -> {"belt_2": -0.09, ...}"""
    out = {}
    for item in items or []:
        name, _, value = item.partition("=")
        out[name.strip()] = float(value)
    return out


if __name__ == "__main__":
    raise SystemExit(main())
