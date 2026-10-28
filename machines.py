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

Each machine runs itself (control "auto") or follows the PLC's command
fields (control "plc"); the decisions are the small functions below
(stopper_wants_up, robot_should_start, elevator_auto), which plc_logic.py
uses too, so auto mode and the PLC program behave alike.
"""
import math
from typing import Dict, List, Optional, Tuple

import pymunk

from config_parser import Elevator, Pallet, RobotArm, SimulatorConfig, Stopper
from math_utils import get_rect_vertices, point_in_polygon

SHAFT_LEVEL = -1000          # elevator i's travellers are on SHAFT_LEVEL - i
NO_LEVEL = -10 ** 6          # a travelling platform belt: matches nothing
STOPPER_ZONE = 0.12          # m: the "something waiting" zone upstream of a stopper bar
ELEVATOR_INNER = 0.6         # a part counts as loaded once its centre is within this share of the platform
UNLOAD_CLEAR_S = 0.5         # auto elevator: platform empty this long -> unloaded


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
    """Auto: a part waits at the pick point and the place point is ready.
    PLC: `start` turned on."""
    if robot.control == "plc":
        edge = robot.start and not state.get("last_start", False)
        state["last_start"] = robot.start
        return edge
    return robot.partAtPick and robot.placeReady


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
                engine.space.remove(state["shape"].body, state["shape"])
        self._stoppers = {}
        for s in self.config.stoppers:
            s.isUp, s.occupied, s.release = True, False, False
        self._robots = {}
        for r in self.config.robots:
            r.busy = r.holding = r.partAtPick = False
            r.cycles = 0
            r.toolX, r.toolY = self._robot_home(r)
        self._elevators = {}
        belts = {b.id: b for b in self.config.belts}
        for i, e in enumerate(self.config.elevators):
            start = max(0, min(e.levels - 1, e.autoFrom if e.control == "auto" else e.targetLevel))
            e.level = e.targetLevel = start
            e.position = float(start)
            e.moving, e.atLevel, e.occupied = False, True, False
            belt = belts.get(e.beltId) if e.beltId else None
            if belt is not None:
                belt.level = start
            self._elevators[e.id] = {"index": i, "belt": belt, "riders": [],
                                     "design_speed": (belt.speed if belt is not None and belt.speed else 1.0)}

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
            held = self._robots.get(robot.id, {}).get("held")
            if held is not None:
                held.body.position = (robot.toolX, robot.toolY)
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
                engine.space.remove(state["shape"].body, state["shape"])
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
            state = {"geom": geom, "shape": shape, "zone": zone, "added": False, "up_target": True, "t": 0.0,
                     "auto": {}}
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
        if stopper.control == "plc":
            want_up = not stopper.release
        else:
            want_up = stopper_wants_up(stopper, state["auto"], h)
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
            engine.space.add(state["shape"].body, state["shape"])
            state["added"] = True
        else:
            engine.space.remove(state["shape"].body, state["shape"])
            state["added"] = False
        stopper.isUp = want_up

    # -- robots ----------------------------------------------------------------------------

    @staticmethod
    def _robot_home(robot: RobotArm) -> Tuple[float, float]:
        mx, my = (robot.pickX + robot.placeX) / 2.0, (robot.pickY + robot.placeY) / 2.0
        d = math.hypot(mx - robot.x, my - robot.y)
        if d < 1e-6:
            return robot.x + robot.reach * 0.4, robot.y
        r = min(d, robot.reach) * 0.5
        return robot.x + (mx - robot.x) / d * r, robot.y + (my - robot.y) / d * r

    def _robot_target(self, robot: RobotArm, phase: str) -> Tuple[float, float]:
        if phase == "to_pick":
            return robot.pickX, robot.pickY
        if phase == "to_place":
            return robot.placeX, robot.placeY
        return self._robot_home(robot)

    def _sweep(self, robot: RobotArm, a, b, t: float) -> Tuple[float, float]:
        """The tool between two points, swinging round the base."""
        ra = math.hypot(a[0] - robot.x, a[1] - robot.y)
        rb = math.hypot(b[0] - robot.x, b[1] - robot.y)
        aa = math.atan2(a[1] - robot.y, a[0] - robot.x)
        ab = math.atan2(b[1] - robot.y, b[0] - robot.x)
        da = (ab - aa + math.pi) % (2 * math.pi) - math.pi
        s = _smooth(t)
        r, ang = ra + (rb - ra) * s, aa + da * s
        return robot.x + r * math.cos(ang), robot.y + r * math.sin(ang)

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

    def _step_robot(self, robot: RobotArm, h: float) -> None:
        state = self._robots.setdefault(robot.id, {"phase": "idle", "t": 0.0, "held": None, "from": None})
        robot.phase, robot.phaseT = state["phase"], 0.0
        if state["phase"] == "idle":
            robot.partAtPick = self.part_at(robot.pickX, robot.pickY, robot.pickLevel, robot.pickRadius) is not None
            robot.placeReady = self.place_ready(robot)
            robot.toolX, robot.toolY = self._robot_home(robot)
            if robot_should_start(robot, state):
                self._robot_phase(robot, state, "to_pick")
            return
        if robot.control == "plc":
            state["last_start"] = robot.start
        leg = max(1e-3, robot.cycleTime / 3.0)
        state["t"] += h
        t = min(1.0, state["t"] / leg)
        robot.phaseT = t
        robot.toolX, robot.toolY = self._sweep(robot, state["from"], self._robot_target(robot, state["phase"]), t)
        if t < 1.0:
            return
        if state["phase"] == "to_pick":
            self._grip(robot, state)
            self._robot_phase(robot, state, "to_place")
        elif state["phase"] == "to_place":
            self._let_go(robot, state)
            self._robot_phase(robot, state, "home")
        else:
            state["phase"] = "idle"
            robot.busy = False
            robot.cycles = (robot.cycles + 1) & 0xFFFF

    def _robot_phase(self, robot: RobotArm, state: dict, phase: str) -> None:
        state["from"] = (robot.toolX, robot.toolY)
        state["phase"], state["t"] = phase, 0.0
        robot.busy = True
        robot.partAtPick = False if phase != "idle" else robot.partAtPick

    def _grip(self, robot: RobotArm, state: dict) -> None:
        found = self.part_at(robot.pickX, robot.pickY, robot.pickLevel, robot.pickRadius)
        if found is None:
            return
        part, pallet = found
        if pallet is not None:
            pallet.loads.remove(part)
            pallet.model.loadCount = len(pallet.loads)
        else:
            self.engine.lift_out(part)
        state["held"] = part
        robot.holding = True

    def _let_go(self, robot: RobotArm, state: dict) -> None:
        part = state["held"]
        state["held"] = None
        robot.holding = False
        if part is None:
            return
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
        if elevator.control == "auto":
            state.setdefault("auto", {})["loaded"] = bool(
                self._on_platform(elevator, elevator.level, ELEVATOR_INNER)) if elevator.atLevel else False
            target, run = elevator_auto(elevator, state["auto"], h)
            elevator.targetLevel = target
            if belt is not None:
                belt.speed = state["design_speed"] if run else 0.0
                if belt.commandFormat == "vfd":
                    belt.run = run
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
