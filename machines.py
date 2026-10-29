"""
machines.py - pallets, stoppers, robot arms and elevators in the simulation
(config_parser.Pallet / Stopper / RobotArm / Elevator). PhysicsEngine owns
one MachineRuntime and calls it each fixed step:

  before_step(h)  after the belt drives, before the belt transport and the
                  solver: stoppers move their bars, elevators move and run
                  their platform belts, robots step through their cycles
  after_step(h)   after the solver and the sinks: loads and held parts
                  follow their pallet / gripper, feedback flags

Pallets are parcels of kind "pallet" (physics.Parcel): the belts, sensors
and collisions treat them like any parcel. A part on a pallet (its load) or
in a gripper is taken out of the physics space and follows its carrier;
it is put back when it is set down.

Floor levels: every belt, sensor, source, pallet and parcel has a level (0
= ground). A belt moves only what is on its level, sensors only see their
level and only bodies on one level collide. An elevator carries what is on
its platform - and its platform belt - from one level to another; while
travelling they are on a level of their own (SHAFT_LEVEL - i), so nothing
else touches them.

Each machine follows its command fields: the PLC's (control "plc") or set
by hand ("manual"). The decisions a PLC makes for them are the small
functions below (stopper_wants_up, elevator_auto), which the PLC program,
plc_logic.py, uses.
"""
import math
from typing import Dict, List, Optional, Tuple

import pymunk

import heights
import robot6

from config_parser import Elevator, Pallet, RobotArm, SimulatorConfig, Stopper
from math_utils import get_rect_vertices, point_in_polygon

SHAFT_LEVEL = -1000          # elevator i's travellers are on SHAFT_LEVEL - i
NO_LEVEL = -10 ** 6          # a travelling platform belt: matches nothing
STOPPER_ZONE = 0.12          # m: the "something waiting" zone upstream of a stopper bar
ELEVATOR_INNER = 0.6         # a part counts as loaded once its centre is within this share of the platform
UNLOAD_CLEAR_S = 0.5         # the PLC program's elevator: platform empty this long -> unloaded


def is_multi_level(config: SimulatorConfig) -> bool:
    """Whether a layout uses more than the ground level (then belts,
    sensors and collisions are filtered by level)."""
    if config.elevators:
        return True
    for group in (config.belts, config.sensors, config.sources, config.pallets, config.stoppers):
        if any(getattr(c, "level", 0) for c in group):
            return True
    return any(r.level or r.pickLevel or r.placeLevel for r in config.robots)


def _smooth(t: float) -> float:
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return t * t * (3.0 - 2.0 * t)


def _rect(x, y, w, h, rotation):
    return get_rect_vertices(x, y, w, h, rotation)


# ---------------------------------------------------------------------------
# Decisions (shared with plc_logic.py)
# ---------------------------------------------------------------------------

def stopper_wants_up(stopper: Stopper, state: dict, h: float) -> bool:
    """Auto mode: hold whatever arrives for holdTime, then lower for
    releaseTime. `state` keeps the timers between calls."""
    phase = state.setdefault("phase", "hold")
    state["t"] = state.get("t", 0.0) + h
    if phase == "hold":
        if not stopper.occupied:
            state["t"] = 0.0
        elif state["t"] >= stopper.holdTime:
            state["phase"], state["t"] = "release", 0.0
            return False
        return True
    if state["t"] >= stopper.releaseTime:
        state["phase"], state["t"] = "hold", 0.0
        return True
    return False


def robot_should_start(robot: RobotArm, state: dict) -> bool:
    """A program begins when `start` turns on (the PLC's or set by hand)."""
    edge = robot.start and not state.get("last_start", False)
    state["last_start"] = robot.start
    return edge


def elevator_auto(elevator: Elevator, state: dict, h: float) -> Tuple[int, bool]:
    """Auto mode: (target level, run the platform belt). Loads at autoFrom
    (belt runs until a part is on the platform), carries it to autoTo,
    unloads (belt runs until the platform has been empty a moment) and
    goes back."""
    phase = state.setdefault("phase", "load")
    if not elevator.atLevel:
        return elevator.targetLevel, False
    if phase == "load":
        if elevator.level != elevator.autoFrom:
            return elevator.autoFrom, False
        if state.get("loaded"):
            state["phase"], state["empty_s"] = "unload", 0.0
            return elevator.autoTo, False
        return elevator.autoFrom, True
    # unload
    if elevator.level != elevator.autoTo:
        return elevator.autoTo, False
    state["empty_s"] = 0.0 if elevator.occupied else state.get("empty_s", 0.0) + h
    if state["empty_s"] >= UNLOAD_CLEAR_S:
        state["phase"] = "load"
        return elevator.autoFrom, False
    return elevator.autoTo, True


# ---------------------------------------------------------------------------
# The runtime
# ---------------------------------------------------------------------------

class MachineRuntime:
    def __init__(self, engine):
        self.engine = engine
        self.config: SimulatorConfig = engine.config
        self.pallets: List = []                     # physics.Parcel of kind "pallet"
        self._stoppers: Dict[str, dict] = {}
        self._robots: Dict[str, dict] = {}
        self._elevators: Dict[str, dict] = {}

    # -- lifecycle ---------------------------------------------------------------

    def reset(self) -> None:
        """Pallets back at their start, machines at rest (engine init / Reset)."""
        engine = self.engine
        for pallet in self.pallets:
            self._drop_loads(pallet)
            if pallet in engine.parcels:
                engine.parcels.remove(pallet)
                engine._remove_parcel(pallet)
        self.pallets = []
        for model in self.config.pallets:
            p = engine.add_parcel(model.x, model.y, model.width, model.height, model.rotation, model.mass,
                                  kind="pallet", level=model.level, label=model.label or model.id,
                                  color=model.color, parcel_id=model.id)
            p.model = model
            model.loadCount = 0
            self.pallets.append(p)
        for state in self._stoppers.values():
            if state.get("added"):
                engine.space.remove(state["body"], state["shape"])
        self._stoppers = {}
        for s in self.config.stoppers:
            s.isUp, s.occupied, s.release = True, False, False
        self._robots = {}
        for r in self.config.robots:
            self._reset_robot(r)
        self._elevators = {}
        belts = {b.id: b for b in self.config.belts}
        for i, e in enumerate(self.config.elevators):
            start = max(0, min(e.levels - 1, e.targetLevel))
            e.level = e.targetLevel = start
            e.position = float(start)
            e.moving, e.atLevel, e.occupied = False, True, False
            belt = belts.get(e.beltId) if e.beltId else None
            if belt is not None:
                belt.level = start
            self._elevators[e.id] = {"index": i, "belt": belt, "riders": []}

    def clear_parts(self) -> None:
        """Clear Parcels: pallets stay (empty), grippers let go."""
        for pallet in self.pallets:
            pallet.loads = []
            if pallet.model is not None:
                pallet.model.loadCount = 0
        for state in self._robots.values():
            state["held"] = None
        for r in self.config.robots:
            r.holding = False
        for state in self._elevators.values():
            state["riders"] = [p for p in state["riders"] if p.kind == "pallet"]

    def carried_parts(self) -> List:
        """Parts out of the space: pallet loads and gripped parts (drawn on top)."""
        return [part for part, _carrier in self.carried_parts_with_carrier()]

    def carried_parts_with_carrier(self) -> List[Tuple[object, str]]:
        """(part, "load" | "held") for every part out of the space."""
        out = [(load, "load") for pallet in self.pallets for load in pallet.loads]
        out += [(s["held"], "held") for s in self._robots.values() if s.get("held") is not None]
        return out

    # -- per step --------------------------------------------------------------------

    def before_step(self, h: float) -> None:
        for stopper in self.config.stoppers:
            self._step_stopper(stopper, h)
        for elevator in self.config.elevators:
            self._step_elevator(elevator, h)
        for robot in self.config.robots:
            self._step_robot(robot, h)

    def after_step(self, h: float) -> None:
        for pallet in self.pallets:
            self._follow(pallet)
        for robot in self.config.robots:
            state = self._robots.get(robot.id, {})
            held = state.get("held")
            if held is not None:
                self._hold_pose(robot, state, held)
        for elevator in self.config.elevators:
            state = self._elevators.get(elevator.id)
            if state is not None:
                elevator.occupied = bool(self._on_platform(elevator, self._platform_level(elevator, state)))

    # -- pallets and loads -------------------------------------------------------------

    def _follow(self, pallet) -> None:
        if not pallet.loads:
            return
        px, py = pallet.body.position
        a = pallet.body.angle
        c, s = math.cos(a), math.sin(a)
        for load in pallet.loads:
            dx, dy, da = load.offset
            load.body.position = (px + dx * c - dy * s, py + dx * s + dy * c)
            load.body.angle = a + da
            load.level = pallet.level

    def pallet_with_room_at(self, x: float, y: float, level: int):
        for pallet in self.pallets:
            if pallet.level != level or len(pallet.loads) >= pallet.model.capacity:
                continue
            if point_in_polygon(x, y, self.engine._parcel_vertices(pallet)):
                return pallet
        return None

    def put_on_pallet(self, part, pallet) -> None:
        """`part` (out of the space) becomes `pallet`'s load: centred, or in
        slots along the pallet for capacity > 1."""
        slots = max(1, pallet.model.capacity)
        k = len(pallet.loads)
        dx = (k + 0.5) / slots * pallet.model.width - pallet.model.width / 2.0 if slots > 1 else 0.0
        part.offset = (dx, 0.0, 0.0)
        part.level = pallet.level
        pallet.loads.append(part)
        pallet.model.loadCount = len(pallet.loads)
        self._follow(pallet)

    def take_loads(self, pallet) -> int:
        """A sink takes the pallet's loads; returns how many."""
        n = len(pallet.loads)
        pallet.loads = []
        if pallet.model is not None:
            pallet.model.loadCount = 0
        return n

    def _drop_loads(self, pallet) -> None:
        pallet.loads = []

    # -- stoppers ------------------------------------------------------------------------

    def _stopper_state(self, stopper: Stopper) -> dict:
        state = self._stoppers.get(stopper.id)
        geom = (stopper.x, stopper.y, stopper.width, stopper.depth, stopper.rotation)
        if state is None or state["geom"] != geom:
            engine = self.engine
            if state is not None and state.get("added"):
                engine.space.remove(state["body"], state["shape"])
            body = pymunk.Body(body_type=pymunk.Body.STATIC)
            body.position = (stopper.x, stopper.y)
            body.angle = math.radians(stopper.rotation)
            shape = pymunk.Poly.create_box(body, (stopper.depth, stopper.width))
            shape.friction, shape.elasticity = 0.0, 0.0
            shape.collision_type = engine.COLLISION_TYPE_WALL
            shape.filter = pymunk.ShapeFilter(categories=engine.CATEGORY_WALL, mask=engine.CATEGORY_PARCEL)
            body.__dict__["level"] = stopper.level
            # The waiting zone, just upstream of the bar.
            zone_body = pymunk.Body(body_type=pymunk.Body.STATIC)
            a = math.radians(stopper.rotation)
            back = stopper.depth / 2.0 + STOPPER_ZONE / 2.0
            zone_body.position = (stopper.x - back * math.cos(a), stopper.y - back * math.sin(a))
            zone_body.angle = a
            zone = pymunk.Poly.create_box(zone_body, (STOPPER_ZONE, stopper.width))
            zone.filter = pymunk.ShapeFilter(categories=engine.CATEGORY_WALL, mask=engine.CATEGORY_PARCEL)
            # The bodies are kept here: a pymunk shape holds its body only weakly,
            # so a bar out of the space (lowered) would lose it.
            state = {"geom": geom, "shape": shape, "body": body, "zone": zone, "zone_body": zone_body,
                     "added": False, "up_target": True, "t": 0.0}
            self._stoppers[stopper.id] = state
            if stopper.isUp:
                engine.space.add(body, shape)
                state["added"] = True
        return state

    def _touching(self, shape, level: int) -> bool:
        for info in self.engine.space.shape_query(shape):
            other = info.shape
            if other.filter.categories & self.engine.CATEGORY_PARCEL and other.body.__dict__.get("level", 0) == level:
                return True
        return False

    def _step_stopper(self, stopper: Stopper, h: float) -> None:
        state = self._stopper_state(stopper)
        stopper.occupied = self._touching(state["zone"], stopper.level)
        want_up = not stopper.release
        if want_up != state["up_target"]:
            state["up_target"], state["t"] = want_up, 0.0
        state["t"] += h
        if want_up == stopper.isUp or state["t"] < stopper.actuationTime:
            return
        engine = self.engine
        if want_up:
            # Rising under a pallet would shove it away: wait until the bar is clear.
            if self._touching(state["shape"], stopper.level):
                return
            engine.space.add(state["body"], state["shape"])
            state["added"] = True
        else:
            engine.space.remove(state["body"], state["shape"])
            state["added"] = False
        stopper.isUp = want_up

    # -- robots ----------------------------------------------------------------------------
    #
    # A robot runs a queue of motion segments (robot6.py kinematics):
    #   "joint"    synchronized joint move from -> to (smooth start / stop)
    #   "linear"   the tool in a straight line from -> to (inverse kinematics
    #              every step), its yaw kept
    #   "grip" / "release"   the gripper, at once
    # each with a duration and the cycle phase it belongs to (for drawing).

    def _geometry(self, robot: RobotArm) -> robot6.Geometry:
        return robot6.Geometry(robot.reach, robot.mountHeight, robot.toolLength)

    def _base(self, robot: RobotArm) -> Tuple[float, float, float]:
        return robot.x, robot.y, heights.floor_z(self.config, robot.level)

    def _ik(self, robot: RobotArm, x: float, y: float, z: float, yaw: float) -> robot6.Joints:
        bx, by, bz = self._base(robot)
        return robot6.inverse(self._geometry(robot), x - bx, y - by, z - bz, yaw)

    def _tcp(self, robot: RobotArm, q) -> Tuple[float, float, float]:
        bx, by, bz = self._base(robot)
        x, y, z = robot6.forward(self._geometry(robot), q)["tcp"]
        return x + bx, y + by, z + bz

    def _perch(self, robot: RobotArm) -> robot6.Joints:
        """Home between cycles: above the middle of pick and place."""
        mx, my = (robot.pickX + robot.placeX) / 2.0, (robot.pickY + robot.placeY) / 2.0
        top = max(heights.surface_z(self.config, robot.pickLevel), heights.surface_z(self.config, robot.placeLevel))
        for z in (top + 0.8, top + 0.6, top + 0.4):
            try:
                return self._ik(robot, mx, my, z, 0.0)
            except robot6.Unreachable:
                continue
        return robot6.HOME

    def _set_joints(self, robot: RobotArm, state: dict, q) -> None:
        state["q"] = tuple(q)
        robot.j1, robot.j2, robot.j3, robot.j4, robot.j5, robot.j6 = (round(v, 4) for v in q)
        robot.tcpX, robot.tcpY, robot.tcpZ = self._tcp(robot, q)
        robot.toolX, robot.toolY = robot.tcpX, robot.tcpY

    def _reset_robot(self, robot: RobotArm) -> None:
        state = self._robots[robot.id] = {"segs": [], "t": 0.0, "held": None, "cycle": False}
        robot.busy = robot.holding = robot.partAtPick = robot.fault = False
        robot.faultCode = robot.cycles = 0
        robot.phase, robot.phaseT = "idle", 0.0
        state["perch"] = self._perch(robot)
        self._set_joints(robot, state, state["perch"])
        robot.atHome, robot.inPosition = True, True

    def robot(self, robot_id: str) -> "RobotHandle":
        """Script access to a robot (tests, scenarios): queue moves and
        gripper actions, read joints / TCP / state."""
        model = next(r for r in self.config.robots if r.id == robot_id)
        if robot_id not in self._robots:
            self._reset_robot(model)
        return RobotHandle(self, model)

    def part_at(self, x: float, y: float, level: int, radius: float):
        """The part to pick at (x, y): a pallet's load first, else a free parcel."""
        best, best_d = None, radius
        for pallet in self.pallets:
            for load in pallet.loads:
                if load.level != level:
                    continue
                lx, ly = load.body.position
                d = math.hypot(lx - x, ly - y)
                if d <= best_d:
                    best, best_d = (load, pallet), d
        if best is not None:
            return best
        for part in self.engine.parcels:
            if part.kind != "parcel" or part.level != level:
                continue
            px, py = part.body.position
            d = math.hypot(px - x, py - y)
            if d <= best_d:
                best, best_d = (part, None), d
        return best

    def place_ready(self, robot: RobotArm) -> bool:
        """A pallet with room at the place point - or, placeTarget "any",
        just a free spot there."""
        if self.pallet_with_room_at(robot.placeX, robot.placeY, robot.placeLevel) is not None:
            return True
        if robot.placeTarget == "pallet":
            return False
        for part in self.engine.parcels:
            if part.level == robot.placeLevel:
                px, py = part.body.position
                if math.hypot(px - robot.placeX, py - robot.placeY) <= robot.pickRadius + part.max_extent:
                    return False
        return True

    def _scale(self, robot: RobotArm) -> float:
        """Time factor of the speed override (100 % = 1)."""
        return 100.0 / max(1, min(100, int(robot.speedOverride or 100)))

    def _cycle_program(self, robot: RobotArm, state: dict) -> List[dict]:
        """Program 1: pick and place, timed to cycleTime / override - the
        same thirds as always (grip at 1/3, let go at 2/3)."""
        third = robot.cycleTime * self._scale(robot) / 3.0
        clear = 0.25
        found = self.part_at(robot.pickX, robot.pickY, robot.pickLevel, robot.pickRadius)
        pick_z = heights.surface_z(self.config, robot.pickLevel)
        held_h, pick_yaw = heights.PARCEL_H[1], 0.0
        if found is not None:
            part, pallet = found
            held_h, pick_yaw = heights.part_height(part), math.degrees(part.body.angle)
            pick_z += heights.PALLET_H if pallet is not None else 0.0
        pick_z += held_h                                    # the TCP grips the part's top
        place_z = heights.surface_z(self.config, robot.placeLevel)
        pallet = self.pallet_with_room_at(robot.placeX, robot.placeY, robot.placeLevel)
        place_yaw = pick_yaw
        if pallet is not None:
            place_z += heights.PALLET_H
            place_yaw = math.degrees(pallet.body.angle)
        place_z += held_h                                   # the part's bottom on the surface
        above_pick = (robot.pickX, robot.pickY, pick_z + clear)
        at_pick = (robot.pickX, robot.pickY, pick_z)
        above_place = (robot.placeX, robot.placeY, place_z + clear)
        at_place = (robot.placeX, robot.placeY, place_z)
        q_above_pick = self._ik(robot, *above_pick, pick_yaw)
        q_above_place = self._ik(robot, *above_place, place_yaw)
        self._ik(robot, *at_pick, pick_yaw)                 # reachable? (raises)
        self._ik(robot, *at_place, place_yaw)
        return [
            _seg("joint", "to_pick", 0.0, 0.7, third * 0.7, state["q"], q_above_pick),
            _seg("linear", "to_pick", 0.7, 1.0, third * 0.3, above_pick, at_pick, pick_yaw),
            _seg("grip", "to_place", 0.0, 0.0, 0.0),
            _seg("linear", "to_place", 0.0, 0.2, third * 0.2, at_pick, above_pick, pick_yaw),
            _seg("joint", "to_place", 0.2, 0.8, third * 0.6, q_above_pick, q_above_place),
            _seg("linear", "to_place", 0.8, 1.0, third * 0.2, above_place, at_place, place_yaw),
            _seg("release", "home", 0.0, 0.0, 0.0),
            _seg("linear", "home", 0.0, 0.3, third * 0.3, at_place, above_place, place_yaw),
            _seg("joint", "home", 0.3, 1.0, third * 0.7, q_above_place, state["perch"]),
        ]

    def _joint_seg(self, robot: RobotArm, q_from, q_to, phase: str = "move") -> dict:
        duration = max(0.2, robot6.joint_move_time(q_from, q_to) * 1.3) * self._scale(robot)
        return _seg("joint", phase, 0.0, 1.0, duration, tuple(q_from), tuple(q_to))

    def _start(self, robot: RobotArm, state: dict, program: int) -> None:
        try:
            if program == 1:
                segs = self._cycle_program(robot, state)
            elif program == 2:
                segs = [self._joint_seg(robot, state["q"], state["perch"], "home")]
            elif program == 3:
                q = self._ik(robot, robot.targetX, robot.targetY, robot.targetZ, robot.targetYaw)
                segs = [self._joint_seg(robot, state["q"], q)]
            else:
                return
        except robot6.Unreachable:
            self._fault(robot, state, 1)
            return
        state["segs"], state["t"], state["cycle"] = segs, 0.0, program == 1
        robot.busy, robot.inPosition, robot.atHome, robot.partAtPick = True, False, False, False

    def _fault(self, robot: RobotArm, state: dict, code: int) -> None:
        robot.fault, robot.faultCode = True, code
        robot.busy, robot.ready, robot.inPosition = False, False, True
        state["segs"] = []
        robot.phase, robot.phaseT = "idle", 0.0

    def _step_robot(self, robot: RobotArm, h: float) -> None:
        state = self._robots.get(robot.id)
        if state is None:
            self._reset_robot(robot)
            state = self._robots[robot.id]
        # Fault reset and the gripper bit act on their edges.
        if robot.faultReset and not state.get("last_reset", False) and robot.fault:
            robot.fault, robot.faultCode = False, 0
        state["last_reset"] = robot.faultReset
        if not state["segs"]:
            robot.partAtPick = self.part_at(robot.pickX, robot.pickY, robot.pickLevel, robot.pickRadius) is not None
            robot.placeReady = self.place_ready(robot)
            robot.ready = robot.enable and not robot.fault and not robot.hold
            robot.phase, robot.phaseT = "idle", 0.0
            if robot.gripperClose != state.get("last_grip", False):
                (self._grip_at_tool if robot.gripperClose else self._release_at_tool)(robot, state)
            state["last_grip"] = robot.gripperClose
            if state.get("api"):
                state["segs"], state["t"], state["cycle"] = state.pop("api"), 0.0, False
                robot.busy, robot.inPosition, robot.atHome = True, False, False
            elif robot.ready and robot_should_start(robot, state):
                self._start(robot, state, int(robot.program))
            if not state["segs"]:
                return
        else:
            robot.ready = False
            state["last_start"] = robot.start
        if robot.hold:
            return                                              # *HOLD: stand still
        state["t"] += h
        while state["segs"]:
            seg = state["segs"][0]
            f = 1.0 if seg["duration"] <= 0 else min(1.0, state["t"] / seg["duration"])
            robot.phase = seg["phase"]
            robot.phaseT = seg["p0"] + (seg["p1"] - seg["p0"]) * f
            if seg["kind"] == "joint":
                self._set_joints(robot, state, robot6.interpolate(seg["from"], seg["to"], _smooth(f)))
            elif seg["kind"] == "linear":
                (x0, y0, z0), (x1, y1, z1) = seg["from"], seg["to"]
                s = _smooth(f)
                try:
                    q = self._ik(robot, x0 + (x1 - x0) * s, y0 + (y1 - y0) * s, z0 + (z1 - z0) * s, seg["yaw"])
                except robot6.Unreachable:
                    self._fault(robot, state, 1)
                    return
                self._set_joints(robot, state, q)
            elif seg["kind"] == "grip":
                (self._grip_at_tool if seg.get("at_tool") else self._grip)(robot, state)
            elif seg["kind"] == "release":
                (self._release_at_tool if seg.get("at_tool") else self._let_go)(robot, state)
            if f < 1.0:
                return
            state["segs"].pop(0)
            state["t"] = max(0.0, state["t"] - seg["duration"])
        robot.busy, robot.inPosition = False, True
        robot.phase, robot.phaseT = "idle", 0.0
        robot.atHome = max(abs(a - b) for a, b in zip(state["q"], state["perch"])) < 0.5
        if state.get("cycle"):
            robot.cycles = (robot.cycles + 1) & 0xFFFF
            state["cycle"] = False

    @staticmethod
    def _hold_pose(robot: RobotArm, state: dict, part) -> None:
        """A gripped part under the tool, turned with it."""
        part.body.position = (robot.tcpX, robot.tcpY)
        part.body.angle = math.radians(robot.j1 - robot.j6 + state.get("held_turn", 0.0))

    def _grip(self, robot: RobotArm, state: dict) -> None:
        found = self.part_at(robot.pickX, robot.pickY, robot.pickLevel, robot.pickRadius)
        if found is not None:
            self._take(robot, state, *found)

    def _take(self, robot: RobotArm, state: dict, part, pallet) -> None:
        if pallet is not None:
            pallet.loads.remove(part)
            pallet.model.loadCount = len(pallet.loads)
        else:
            self.engine.lift_out(part)
        state["held"] = part
        state["held_turn"] = math.degrees(part.body.angle) - (robot.j1 - robot.j6)
        robot.holding = True

    def _level_below(self, z: float) -> int:
        """The floor level whose belts are just below height z."""
        lh = heights.level_height(self.config)
        return max(0, int(math.floor((z - heights.BELT_TOP + 0.05) / lh))) if lh > 0 else 0

    def _grip_at_tool(self, robot: RobotArm, state: dict) -> None:
        """Grip the part right under the tool (PLC gripperClose, scripts)."""
        if state.get("held") is not None:
            return
        level = self._level_below(robot.tcpZ - heights.PARCEL_H[0])
        found = self.part_at(robot.tcpX, robot.tcpY, level, robot.pickRadius)
        if found is None:
            return
        part, pallet = found
        top = heights.surface_z(self.config, level) + (heights.PALLET_H if pallet else 0.0) + heights.part_height(part)
        if abs(top - robot.tcpZ) <= 0.1:
            self._take(robot, state, part, pallet)

    def _release_at_tool(self, robot: RobotArm, state: dict) -> None:
        """Let go where the tool is: onto a pallet below, else the belt / floor."""
        part = state.get("held")
        if part is None:
            return
        self._hold_pose(robot, state, part)
        state["held"] = None
        robot.holding = False
        level = self._level_below(robot.tcpZ - heights.part_height(part))
        pallet = self.pallet_with_room_at(robot.tcpX, robot.tcpY, level)
        if pallet is not None:
            self.put_on_pallet(part, pallet)
        else:
            part.level = level
            self.engine.set_down(part, robot.tcpX, robot.tcpY)

    def _let_go(self, robot: RobotArm, state: dict) -> None:
        part = state["held"]
        state["held"] = None
        robot.holding = False
        if part is None:
            return
        self._hold_pose(robot, state, part)
        pallet = self.pallet_with_room_at(robot.placeX, robot.placeY, robot.placeLevel)
        if pallet is not None:
            self.put_on_pallet(part, pallet)
        elif robot.placeTarget == "pallet":
            # The pallet left meanwhile: put the part back where it came from.
            part.level = robot.pickLevel
            self.engine.set_down(part, robot.pickX, robot.pickY)
        else:
            part.level = robot.placeLevel
            self.engine.set_down(part, robot.placeX, robot.placeY)

    # -- elevators -------------------------------------------------------------------------

    def _footprint(self, elevator: Elevator, share: float = 1.0):
        return _rect(elevator.x, elevator.y, elevator.width * share, elevator.height * share, elevator.rotation)

    def _on_platform(self, elevator: Elevator, level: int, share: float = 1.0) -> List:
        poly = self._footprint(elevator, share)
        out = []
        for part in self.engine.parcels:
            if part.level == level and point_in_polygon(*part.body.position, poly):
                out.append(part)
        return out

    def _platform_level(self, elevator: Elevator, state: dict) -> int:
        return SHAFT_LEVEL - state["index"] if elevator.moving else elevator.level

    def _step_elevator(self, elevator: Elevator, h: float) -> None:
        state = self._elevators.get(elevator.id)
        if state is None:
            return
        belt = state["belt"]
        target = max(0, min(elevator.levels - 1, int(elevator.targetLevel)))
        if not elevator.moving:
            if target == elevator.level:
                return
            # Depart: what is on the platform travels with it.
            shaft = SHAFT_LEVEL - state["index"]
            state["riders"] = self._on_platform(elevator, elevator.level)
            for part in state["riders"]:
                part.level = shaft
            if belt is not None:
                belt.level = NO_LEVEL
            elevator.moving, elevator.atLevel = True, False
        step = elevator.liftSpeed / max(1e-6, elevator.levelHeight) * h
        if abs(target - elevator.position) <= step:
            elevator.position = float(target)
            elevator.level = target
            elevator.moving, elevator.atLevel = False, True
            for part in state["riders"]:
                part.level = target
            state["riders"] = []
            if belt is not None:
                belt.level = target
        else:
            elevator.position += step if target > elevator.position else -step
            for part in state["riders"]:
                part.body.velocity = (0.0, 0.0)
                part.body.angular_velocity = 0.0


def _seg(kind: str, phase: str, p0: float, p1: float, duration: float, frm=None, to=None, yaw: float = 0.0) -> dict:
    return {"kind": kind, "phase": phase, "p0": p0, "p1": p1, "duration": duration, "from": frm, "to": to,
            "yaw": yaw}


class RobotHandle:
    """A robot for scripts (tests, scenarios). Moves are queued and run in
    simulated time when the robot is idle - step the engine to carry them out:

        arm = engine.machines.robot("robot_1")
        arm.move_to(1.0, 2.0, 1.4, yaw=90)          # joint move, tool pointing down
        arm.move_to(1.0, 2.0, 1.05, linear=True)    # straight down
        arm.grip().move_to(1.0, 2.0, 1.4, linear=True).move_to(2.0, 0.5, 1.4)
        arm.release().home()
        engine.run_for(arm.remaining())

    Positions are world metres (z up from the ground floor), yaw in degrees.
    move_to raises robot6.Unreachable for a target out of reach."""

    def __init__(self, runtime: MachineRuntime, robot: RobotArm):
        self.runtime, self.model = runtime, robot

    @property
    def _state(self) -> dict:
        return self.runtime._robots[self.model.id]

    def _queue(self) -> List[dict]:
        return self._state.setdefault("api", [])

    def _end(self):
        """Where the robot is once everything queued has run: (joints, tcp)."""
        rt, r = self.runtime, self.model
        for seg in reversed(self._state.get("segs", []) + self._queue()):
            if seg["kind"] == "joint":
                return seg["to"], rt._tcp(r, seg["to"])
            if seg["kind"] == "linear":
                return rt._ik(r, *seg["to"], seg["yaw"]), seg["to"]
        return self._state["q"], (r.tcpX, r.tcpY, r.tcpZ)

    def move_to(self, x: float, y: float, z: float, yaw: Optional[float] = None, linear: bool = False,
                speed: float = 0.5) -> "RobotHandle":
        """The TCP to (x, y, z), tool down, jaws across `yaw` (deg; None:
        as it is): a joint move, or with linear=True a straight line at
        `speed` m/s (both scaled by speedOverride)."""
        rt, r = self.runtime, self.model
        q_from, p_from = self._end()
        if yaw is None:
            yaw = q_from[0] - q_from[5]                      # the tool's yaw is J1 - J6
        q = rt._ik(r, x, y, z, yaw)
        if linear:
            duration = max(0.1, math.dist(p_from, (x, y, z)) / max(1e-3, speed)) * rt._scale(r)
            self._queue().append(_seg("linear", "move", 0.0, 1.0, duration, tuple(p_from), (x, y, z), yaw))
        else:
            self._queue().append(rt._joint_seg(r, q_from, q))
        return self

    def move_joints(self, q) -> "RobotHandle":
        q = tuple(float(v) for v in q)
        if len(q) != 6 or not robot6.within_limits(q):
            raise robot6.Unreachable("joint target past a limit")
        self._queue().append(self.runtime._joint_seg(self.model, self._end()[0], q))
        return self

    def grip(self) -> "RobotHandle":
        """Grip the part under the tool, when the queue gets here."""
        self._queue().append({**_seg("grip", "move", 1.0, 1.0, 0.0), "at_tool": True})
        return self

    def release(self) -> "RobotHandle":
        """Let go there: onto a pallet below, else the belt / floor."""
        self._queue().append({**_seg("release", "move", 1.0, 1.0, 0.0), "at_tool": True})
        return self

    def home(self) -> "RobotHandle":
        return self.move_joints(self._state["perch"])

    def remaining(self) -> float:
        """Seconds of motion still to run (queued and running), plus a step."""
        state = self._state
        total = sum(s["duration"] for s in state.get("segs", []) + self._queue()) - state.get("t", 0.0)
        return max(0.0, total) + 2 * self.runtime.config.physicsDt

    @property
    def joints(self) -> robot6.Joints:
        return self._state["q"]

    @property
    def tcp(self) -> Tuple[float, float, float]:
        return self.model.tcpX, self.model.tcpY, self.model.tcpZ

    @property
    def busy(self) -> bool:
        return bool(self._state.get("segs") or self._state.get("api"))

    @property
    def holding(self) -> bool:
        return self.model.holding
