from config_parser import Belt
from typing import Dict
from pymunk.vec2d import Vec2d
from contextlib import nullcontext
import math
import time
import random
import pymunk
import threading
from typing import List, Tuple, Optional
from config_parser import SimulatorConfig
from math_utils import point_in_polygon, get_rect_vertices
import devices
from drive import step_drive
from surface import belt_bounds, belt_contains, belt_surface_speed, curved_direction, geometry_frame, surface_velocity
from traction import FrictionTraction

class Parcel:
    def __init__(self, id: str, body: pymunk.Body, shape: pymunk.Poly, label: str, color: str):
        self.id = id
        self.body = body
        self.shape = shape
        self.label = label
        self.color = color
        self.max_extent = max(math.hypot(v.x, v.y) for v in shape.get_vertices()) if hasattr(shape, 'get_vertices') else 0.5
        # FrictionTraction state (carrier body + joints), None for kinematic parcels.
        self.traction: Optional[dict] = None
        self.belt_id: Optional[str] = None


def parcel_speed_info(parcel: Parcel) -> Tuple[float, Optional[float]]:
    """Returns (actual_speed, target_speed) for a parcel, in the same units
    as Belt.speed. `target_speed` is None when useSurfaceVelocity is off or
    the parcel isn't currently riding a belt (no single "setting" to
    compare against in that case). Shared by both GUI backends' "is the
    parcel actually moving at the configured speed" indicators, so there's
    one place that knows how to read this off a parcel's pymunk body."""
    actual = parcel.body.velocity.length
    target = None
    if parcel.traction is not None:
        # Friction traction: the target is the belt motion the carrier is
        # dragging the parcel toward (None on the floor).
        if parcel.belt_id is not None:
            target = parcel.traction["carrier"].velocity.length
        return actual, target
    if getattr(parcel.body, "target_v_enable", False):
        target_v = getattr(parcel.body, "target_v", None)
        if target_v is not None:
            target = math.hypot(target_v[0], target_v[1])
    return actual, target


def _smootherstep(t: float) -> float:
    """Ken Perlin's smootherstep: a monotonic S-curve on t in [0, 1] with
    zero first *and* second derivative at both endpoints. Used below to
    ease a belt-to-belt velocity transition instead of blending at one
    constant rate for the whole window and then snapping dead onto the
    target the instant the timer expires - a smoother, more natural
    "settling into the new belt's speed" feel, closer to how friction
    actually brings a sliding object's speed in line with a surface than a
    straight linear ramp is."""
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def dynamic_velocity_modifier(body, gravity, damping, dt):
    """Pymunk per-body velocity callback (assigned to `body.velocity_func`
    when `useSurfaceVelocity` is enabled). Drives a parcel toward
    `target_v`/`target_w` - the belt it's currently on, set each tick by
    `PhysicsEngine.apply_conveyor_kinematics` - easing between belts with
    different speeds instead of snapping instantly, the "like friction"
    transition on a belt handoff.

    `transition_momentum_v` is the velocity captured at the instant of a
    handoff; `transition_momentum_time` counts down from
    `transition_momentum_duration` (the belt's configured transition
    time, `Belt.transition_momentum_duration`) to 0 across the window.
    `t = momentum_time / momentum_duration` is therefore the fraction of
    the transition *remaining* (1 -> just handed off, 0 -> complete);
    `_smootherstep(t)` eases that fraction instead of using it as a raw
    linear blend weight, while still finishing exactly at
    `momentum_duration` either way.
    """
    pymunk.Body.update_velocity(body, gravity, damping, dt)

    target_v_enable = getattr(body, "target_v_enable", False)
    if not target_v_enable:
        return
    target_v = Vec2d(*getattr(body, "target_v", body.velocity))
    momentum_v = getattr(body, "transition_momentum_v", None)
    momentum_time = getattr(body, "transition_momentum_time", 0.0)
    momentum_duration = getattr(body, "transition_momentum_duration", 0.2)

    if momentum_v is not None and momentum_time > 0.0 and momentum_duration > 0.01:
        alpha = _smootherstep(momentum_time / momentum_duration)
        original_v = Vec2d(*momentum_v)
        body.velocity = original_v * alpha + target_v * (1.0 - alpha)
        body.transition_momentum_time = max(0.0, momentum_time - dt)
    else:
        body.velocity = target_v
        body.transition_momentum_v = None
        body.transition_momentum_time = 0.0

    target_w = getattr(body, "target_w", body.angular_velocity)
    remaining_w = getattr(body, "target_w_remaining", None)

    if remaining_w is not None and remaining_w > 0.0 and dt > 0.0 and abs(target_w) > 1e-9:
        clamped_w = min(abs(target_w), remaining_w / dt)
        body.angular_velocity = math.copysign(clamped_w, target_w)
        body.target_w_remaining = max(0.0, remaining_w - clamped_w * dt)
    else:
        body.angular_velocity = target_w

class PhysicsEngine:
    # ShapeFilter category bitmasks
    CATEGORY_WALL   = 0b01
    CATEGORY_PARCEL = 0b10
    # collision_type integers used by space.on_collision()
    COLLISION_TYPE_WALL   = 1
    COLLISION_TYPE_PARCEL = 2

    def __init__(self, config: SimulatorConfig, lock: Optional[threading.Lock] = None):
        self.config = config
        # Shared with MQTTSyncEngine (see main.py, which constructs both and
        # passes the same lock into each) so that update()'s reads/writes of
        # config.belts[i].speed, config.sensors[i].sensor_states, etc. can't
        # race against a background TCP recv thread's deserialize() writing
        # those same fields, or against sync()'s serialize() reading them.
        # Previously the lock only serialized the MQTT-side accesses against
        # each other, never against this engine's own main-thread mutation
        # of the identical fields - a real data race, just narrow enough to
        # rarely produce visibly-corrupted output. Defaults to a no-op
        # context manager so PhysicsEngine keeps working standalone (tests,
        # or any caller that never wires up MQTT) without needing a lock at
        # all.
        self._lock = lock if lock is not None else nullcontext()
        self.space = pymunk.Space()
        self.space.gravity = (0, 0)
        self.space.damping = 0.1 # High damping for top-down

        self.parcels: List[Parcel] = []
        self.static_walls: List[pymunk.Shape] = []
        self.parcel_id_counter = 0
        # Parcels that reached each sink (sink id -> count), for stats/tests.
        self.delivered = {}
        self.sensor_states = {}  # {sensor.id: bool | str}

        # Cumulative simulated seconds - the sum of every dt ever passed to
        # update(). Compare against a wall-clock measurement in the caller
        # to verify the sim is actually keeping pace with real time (a
        # GUI feeding update() a fixed nominal dt instead of the real
        # elapsed time between frames/ticks will silently fall behind
        # real-time under any load, without this ever being visible
        # otherwise - see visualizer.py's Renderer for the indicator built on
        # this, and main.py's [TIMING] line).
        self.sim_time = 0.0
        # Fixed-step accumulator: update(dt) runs whole physicsDt steps and
        # carries the remainder; dropped_time counts real time skipped when a
        # single update() would need more than maxStepsPerAdvance steps.
        self._accumulator = 0.0
        self.dropped_time = 0.0
        self.step_count = 0

        self.friction = FrictionTraction(self.space, config.floorFriction)

        # Sources timing — new sources added at runtime will be lazily initialised
        self.source_timers = {source.id: 100.0 for source in self.config.sources}
        
        # Performance caches
        self._overlap_cache = {}
        self._last_belts_sig = None
        self._sensor_shapes_cache = {}
        
        # self.build_walls()
        self._setup_collision_handlers()
        # Buttons start released: an NC contact (STOP, E-stop) reads 1.
        for button in self.config.buttons:
            devices.sync_button(button)

    def _setup_collision_handlers(self):
        """Install a parcel-parcel collision handler that eliminates bouncing.
        
        Uses space.on_collision(type_a, type_b) — the correct API for this
        pymunk version. Parcel shapes must have collision_type=COLLISION_TYPE_PARCEL.
        """
        def _pre_solve_parcel_parcel(arbiter: pymunk.Arbiter, space, data):
            # Zero restitution — no bounce between parcel bodies.
            # Low friction so touching parcels don’t spin each other.
            arbiter.restitution = 0.0
            arbiter.friction    = 0.1
            # process_collision=True keeps the physical separation response
            # (prevents overlap), but with zero bounce.
            arbiter.process_collision = True
        def _pre_solve_parcel_wall(arbiter: pymunk.Arbiter, space, data):
            # Zero restitution — no bounce between parcel bodies.
            # Low friction so touching parcels don’t spin each other.
            arbiter.restitution = 0.0
            arbiter.friction    = 0.1
            # process_collision=True keeps the physical separation response
            # (prevents overlap), but with zero bounce.
            arbiter.process_collision = True
        self.space.on_collision(
            self.COLLISION_TYPE_PARCEL,
            self.COLLISION_TYPE_PARCEL,
            pre_solve=_pre_solve_parcel_parcel
        )
        # self.space.on_collision(
        #     self.COLLISION_TYPE_PARCEL,
        #     self.COLLISION_TYPE_WALL,
        #     pre_solve=_pre_solve_parcel_wall
        # )

    def _belts_under(self, px: float, py: float, belts=None):
        """The belts (of `belts`, default all) whose surface holds the point -
        bounding box first, exact shape only for the few that pass."""
        boxes = getattr(self, "_belt_boxes", None)
        if boxes is None:
            boxes = [(belt, *belt_bounds(belt)) for belt in self.config.belts]
        wanted = None if belts is None else {id(b) for b in belts}
        return [belt for belt, x0, y0, x1, y1 in boxes
                if x0 <= px <= x1 and y0 <= py <= y1 and (wanted is None or id(belt) in wanted)
                and belt_contains(belt, px, py)]

    def _is_point_on_belt(self, px: float, py: float, belt) -> bool:
        """Returns True if the point (px, py) is physically on the given belt."""
        return belt_contains(belt, px, py)

    def _is_angle_in_arc(self, angle: float, start_angle: float, end_angle: float, tolerance: float = 0.0) -> bool:
        angle %= 360.0
        start_angle = (start_angle - tolerance) % 360.0
        end_angle = (end_angle + tolerance) % 360.0

        if start_angle <= end_angle:
            return start_angle <= angle <= end_angle
        return angle >= start_angle or angle <= end_angle

    def _arc_span_radians(self, start_angle: float, end_angle: float) -> float:
        span = end_angle - start_angle
        if span <= 0.0:
            span += 360.0
        return math.radians(span)

    def _get_belt_bounds(self, belt) -> Tuple[float, float, float, float]:
        return belt_bounds(belt)

    def _get_belts_signature(self):
        return tuple(
            (
                b.id,
                b.x,
                b.y,
                getattr(b, 'length', 0),
                getattr(b, 'radius', 0),
                b.beltWidth,
                b.rotation,
                getattr(b, 'startAngle', 0),
                getattr(b, 'endAngle', 0),
            )
            for b in self.config.belts
        )

    def _belts_have_area_overlap(self, belt_a, belt_b) -> bool:
        sig = self._get_belts_signature()
        if sig != self._last_belts_sig:
            self._overlap_cache.clear()
            self._last_belts_sig = sig
            
        cache_key = (belt_a.id, belt_b.id)
        if cache_key in self._overlap_cache:
            return self._overlap_cache[cache_key]

        a_min_x, a_min_y, a_max_x, a_max_y = self._get_belt_bounds(belt_a)
        b_min_x, b_min_y, b_max_x, b_max_y = self._get_belt_bounds(belt_b)
        min_x = max(a_min_x, b_min_x)
        min_y = max(a_min_y, b_min_y)
        max_x = min(a_max_x, b_max_x)
        max_y = min(a_max_y, b_max_y)

        if min_x >= max_x or min_y >= max_y:
            self._overlap_cache[cache_key] = False
            return False

        sample_count = 10
        for ix in range(sample_count):
            x = min_x + (ix + 0.5) * (max_x - min_x) / sample_count
            for iy in range(sample_count):
                y = min_y + (iy + 0.5) * (max_y - min_y) / sample_count
                if self._is_point_on_belt(x, y, belt_a) and self._is_point_on_belt(x, y, belt_b):
                    self._overlap_cache[cache_key] = True
                    return True
        self._overlap_cache[cache_key] = False
        return False

    def _get_belt_by_id(self, belt_id: str):
        for belt in self.config.belts:
            if belt.id == belt_id:
                return belt
        return None

    def build_walls(self):
        return # BYPASS BUILD WALLS FOR NOW

    def _get_belt_velocity_at(self, px: float, py: float):
        """Returns (vx, vy, vw) of the belt under point (px, py), or None if not on any belt."""
        vs = []
        hit = False
        for belt in self.config.belts:
            if self._is_point_on_belt(px, py, belt):
                hit = True
                sv = surface_velocity(belt, px, py)
                if sv is not None:
                    vs.append(sv)

        if hit and vs:
            vx = sum(v[0] for v in vs) / len(vs)
            vy = sum(v[1] for v in vs) / len(vs)
            vw = sum(v[2] for v in vs) / len(vs)
            return (vx, vy, vw)
        return None

    def handle_sources(self, dt: float):
        for source in self.config.sources:
            # Lazy-init timer for sources added at runtime via the editor
            self.source_timers[source.id] = self.source_timers.get(source.id, 0.0) + dt
            if self.source_timers[source.id] >= source.interval:
                # Time to spawn!
                w = random.uniform(
                    getattr(source, 'minWidth', 0.3),
                    getattr(source, 'maxWidth', 0.5)
                )
                h = random.uniform(
                    getattr(source, 'minHeight', 0.3),
                    getattr(source, 'maxHeight', 0.5)
                )
                
                # Check if spawn area is clear using exact shape overlap query
                sx, sy = source.x, source.y
                
                temp_body = pymunk.Body(body_type=pymunk.Body.STATIC)
                temp_body.position = (sx, sy)
                temp_shape = pymunk.Poly.create_box(temp_body, (w, h))
                
                query_results = self.space.shape_query(temp_shape)
                
                conflict = False
                for info in query_results:
                    if info.shape.filter.categories & self.CATEGORY_PARCEL:
                        conflict = True
                        break
                        
                if not conflict:
                    # Spawn
                    friction_mode = self.config.tractionModel == "friction"
                    mass = random.uniform(source.minMass, source.maxMass) if friction_mode else 1.0
                    moment = pymunk.moment_for_box(mass, (w, h))
                    body = pymunk.Body(mass, moment)
                    body.position = (sx, sy)
                    body.angle = math.radians(source.rotation)

                    if not friction_mode and getattr(self.config, 'useSurfaceVelocity', False):
                        body.velocity_func = dynamic_velocity_modifier

                    shape = pymunk.Poly.create_box(body, (w, h))
                    shape.friction       = 0.3
                    shape.elasticity     = 0.0   # no parcel-parcel bounce
                    shape.collision_type = self.COLLISION_TYPE_PARCEL
                    shape.filter = pymunk.ShapeFilter(
                        categories=self.CATEGORY_PARCEL,
                        mask=self.CATEGORY_WALL | self.CATEGORY_PARCEL
                    )
                    
                    self.space.add(body, shape)
                    self.parcel_id_counter += 1
                    parcel = Parcel(
                        id=f"p_{self.parcel_id_counter}",
                        body=body,
                        shape=shape,
                        label=getattr(source, 'label', 'PKG'),
                        color="#ffaa00"
                    )
                    if friction_mode:
                        parcel.traction = self.friction.attach(body, w / 2.0, h / 2.0)
                    self.parcels.append(parcel)
                    self.source_timers[source.id] = 0.0 # reset timer only if spawned

    def add_parcel(self, x: float, y: float, width: float, height: float,
                   angle_deg: float = 0.0, mass: float = 1.0) -> Parcel:
        """Place one parcel directly (tests, scripted scenarios), using the
        same body/shape/traction setup as a source spawn."""
        friction_mode = self.config.tractionModel == "friction"
        body = pymunk.Body(mass, pymunk.moment_for_box(mass, (width, height)))
        body.position = (x, y)
        body.angle = math.radians(angle_deg)
        if not friction_mode and self.config.useSurfaceVelocity:
            body.velocity_func = dynamic_velocity_modifier
        shape = pymunk.Poly.create_box(body, (width, height))
        shape.friction = 0.3
        shape.elasticity = 0.0
        shape.collision_type = self.COLLISION_TYPE_PARCEL
        shape.filter = pymunk.ShapeFilter(
            categories=self.CATEGORY_PARCEL,
            mask=self.CATEGORY_WALL | self.CATEGORY_PARCEL,
        )
        self.space.add(body, shape)
        self.parcel_id_counter += 1
        parcel = Parcel(id=f"p_{self.parcel_id_counter}", body=body, shape=shape, label="PKG", color="#ffaa00")
        if friction_mode:
            parcel.traction = self.friction.attach(body, width / 2.0, height / 2.0)
        self.parcels.append(parcel)
        return parcel

    def _remove_parcel(self, parcel: Parcel) -> None:
        if parcel.traction is not None:
            self.friction.detach(parcel.traction)
            parcel.traction = None
        self.space.remove(parcel.body, parcel.shape)

    def clear_parcels(self) -> int:
        """Remove every parcel (the simulator's Clear Parcels button);
        returns how many. Sources keep spawning."""
        with self._lock:
            parcels, self.parcels = list(self.parcels), []
            for parcel in parcels:
                self._remove_parcel(parcel)
        return len(parcels)

    def reset(self) -> None:
        """Back to the start (the Reset button): no parcels, time 0, sources
        due at once, delivery counts, sensors and drives back to rest."""
        self.clear_parcels()
        with self._lock:
            self.sim_time = 0.0
            self._accumulator = 0.0
            self.dropped_time = 0.0
            self.step_count = 0
            self.config.simTimeNs = 0
            self.delivered = {}
            self.source_timers = {source.id: 100.0 for source in self.config.sources}
            for sensor in self.config.sensors:
                sensor._filter = None
                sensor.risingEdges = 0
                sensor.blockedBeams = 0
                sensor.sensor_states = "0" * len(sensor.sensor_states) if isinstance(sensor.sensor_states, str) \
                    else sensor.sensor_states
            for belt in self.config.belts:
                belt.actualSpeed = 0.0
                belt.encoderCount = 0
                belt.fault = belt.running = belt.atSpeed = belt.reversing = False
                belt._drive_ref = 0.0
                belt._encoder_frac = 0.0
                belt._overload_s = 0.0

    def _play_area_bounds(self) -> Optional[Tuple[float, float, float, float]]:
        """Bounding box covering every belt/source/sink, padded generously -
        a safety net for handle_sinks(). Walls are currently disabled
        (build_walls() is a no-op), so nothing physically stops a parcel
        from drifting away from every belt without ever crossing a sink
        polygon - pushed off an edge by a collision, a layout where a belt
        dead-ends with no sink, or simply "tunneling" past a thin sink
        polygon in one sub-step. Without this, such a parcel stays in
        self.parcels forever, and since apply_conveyor_kinematics,
        evaluate_sensors, and handle_sinks itself all iterate the full
        parcel list every sub-step, per-tick cost grows without bound the
        longer the sim runs - eventually looking like the whole app has
        frozen, even though it's really just doing more and more work on
        parcels nobody can see anymore."""
        xs, ys = [], []
        for belt in self.config.belts:
            min_x, min_y, max_x, max_y = self._get_belt_bounds(belt)
            xs.extend([min_x, max_x])
            ys.extend([min_y, max_y])
        for source in self.config.sources:
            xs.append(source.x)
            ys.append(source.y)
        for sink in self.config.sinks:
            xs.extend([sink.x - sink.width / 2.0, sink.x + sink.width / 2.0])
            ys.extend([sink.y - sink.height / 2.0, sink.y + sink.height / 2.0])
        if not xs or not ys:
            return None
        margin = 20.0  # meters - generous, well beyond any legitimate belt-edge overshoot
        return (min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin)

    def handle_sinks(self):
        to_remove = []
        bounds = self._play_area_bounds()
        # Sinks are axis-aligned rectangles (rotation 0): a box test, built once.
        boxes = [(sink.id, sink.x - sink.width / 2.0, sink.y - sink.height / 2.0,
                  sink.x + sink.width / 2.0, sink.y + sink.height / 2.0) for sink in self.config.sinks]
        for parcel in self.parcels:
            px, py = parcel.body.position
            in_a_sink = False
            for sink_id, x0, y0, x1, y1 in boxes:
                if x0 <= px <= x1 and y0 <= py <= y1:
                    in_a_sink = True
                    self.delivered[sink_id] = self.delivered.get(sink_id, 0) + 1
                    break
            if in_a_sink:
                to_remove.append(parcel)
                continue
            if bounds is not None:
                min_x, min_y, max_x, max_y = bounds
                if not (min_x <= px <= max_x and min_y <= py <= max_y):
                    to_remove.append(parcel)

        for p in to_remove:
            self._remove_parcel(p)
            self.parcels.remove(p)

    def clear_all_parcels(self):
        for p in self.parcels:
            self._remove_parcel(p)
        self.parcels.clear()

    def apply_friction_traction(self):
        for parcel in self.parcels:
            if parcel.traction is not None:
                parcel.belt_id = self.friction.pre_step(parcel.body, parcel.traction, self.config.belts)

    def apply_conveyor_kinematics(self):
        """Drive each parcel using the belt velocity sampled at its center + 4 corners."""
        for parcel in self.parcels:
            if parcel.traction is not None:
                continue  # friction-traction parcel (see apply_friction_traction)
            pos = parcel.body.position
            sample_pts = [(pos.x, pos.y)]
            # for v in parcel.shape.get_vertices():
            #     wv = pos + v.rotated(parcel.body.angle)
            #     sample_pts.append((wv.x, wv.y))

            if getattr(self.config, 'useSurfaceVelocity', False):
                touching_belts:Dict[str, Belt] = {}
                for sp in sample_pts:
                    for belt in self._belts_under(sp[0], sp[1]):
                        touching_belts[belt.id] = belt
                touching_ids = set(touching_belts.keys())
                if len(touching_ids) > 1:
                    overlapping_ids = set()
                    touching_belt_list = list(touching_belts.values())
                    for i, belt_a in enumerate(touching_belt_list):
                        for belt_b in touching_belt_list[i + 1:]:
                            if self._belts_have_area_overlap(belt_a, belt_b):
                                overlapping_ids.add(belt_a.id)
                                overlapping_ids.add(belt_b.id)
                    if overlapping_ids:
                        parcel.body.momentum_intersection_ids = overlapping_ids
                            
                if touching_belts:
                    best_belt = None
                    best_score = -float('inf')
                    best_idx = 0
                    
                    for belt in touching_belts.values():
                        belt_speed = belt_surface_speed(belt)
                        if belt.type == "linear":
                            rad = math.radians(belt.rotation + belt.directionAngle)
                            cvx = belt_speed * math.cos(rad)
                            cvy = belt_speed * math.sin(rad)
                            cw = 0.0
                        elif belt.type == "path":
                            # Drawn belt: tangent of the nearest centerline
                            # piece, turning at speed * curvature on arcs.
                            sv = surface_velocity(belt, pos.x, pos.y)
                            if sv is None:
                                continue
                            cvx, cvy, cw = sv
                        elif belt.type == "curved":
                            dx = pos.x - belt.x
                            dy = pos.y - belt.y
                            dist = math.hypot(dx, dy)
                            direction = curved_direction(belt)
                            cw = direction * belt_speed / belt.radius if belt.radius > 0 else 0
                            speed_at_dist = cw * dist
                            if dist > 1e-6:
                                cvx = speed_at_dist * (-dy / dist)
                                cvy = speed_at_dist * (dx / dist)

                                radial_error = belt.radius - dist
                                radial_gain = abs(belt_speed) / max(belt.beltWidth * 0.5, 1e-6)
                                max_radial_speed = abs(belt_speed) * 0.75
                                radial_speed = max(
                                    -max_radial_speed,
                                    min(max_radial_speed, radial_error * radial_gain),
                                )
                                # cvx += radial_speed * dx / dist
                                # cvy += radial_speed * dy / dist
                            else:
                                cvx, cvy = 0.0, 0.0
                            # cvx, cvy = 0.0, 0.0
                        else:
                            continue

                        # Translation may begin while any parcel corner touches a
                        # curved belt, but parcel orientation should follow the
                        # centerline sweep. Otherwise a box starts rotating early
                        # and keeps rotating late, adding a few extra degrees.
                        if belt.type == "curved" and dist > 1e-6:
                            center_angle = math.degrees(math.atan2(dy, dx))
                            if center_angle < 0:
                                center_angle += 360.0
                            angular_sector_hit = self._is_angle_in_arc(
                                center_angle,
                                belt.startAngle,
                                belt.endAngle,
                            )
                        else:
                            angular_sector_hit = False

                        if belt.type == "curved" and not angular_sector_hit:
                            cw = 0.0
                            
                        v_len = parcel.body.velocity.length
                        if v_len < 0.1:
                            score = math.hypot(cvx, cvy)
                        else:
                            score = cvx * parcel.body.velocity.x + cvy * parcel.body.velocity.y
                            
                        if score > best_score:
                            best_score = score
                            best_belt = (cvx, cvy, cw, belt)

                    if best_belt:
                        selected_belt = best_belt[3]
                        # print(f"selected_belt: {selected_belt.id}")
                        remaining_w = getattr(parcel.body, "target_w_remaining", 0.0)
                        if remaining_w is None:
                            remaining_w = 0.0
                        momentum_intersection_ids = getattr(parcel.body, "momentum_intersection_ids", set())
                        momentum_source_id = getattr(parcel.body, "target_w_belt_id", None)
                        intersecting_handoff = (
                            remaining_w > 0.0
                            and selected_belt.id in momentum_intersection_ids
                            and momentum_source_id in momentum_intersection_ids
                        )
                        if selected_belt.type == "curved" and abs(best_belt[2]) > 1e-9:
                            if getattr(parcel.body, "target_w_belt_id", None) != selected_belt.id:
                                parcel.body.target_w_belt_id = selected_belt.id
                                parcel.body.target_w_remaining = self._arc_span_radians(
                                    selected_belt.startAngle,
                                    selected_belt.endAngle,
                                )
                            parcel.body.target_w_last = best_belt[2]
                            target_w_remaining = getattr(parcel.body, "target_w_remaining", None)
                            if target_w_remaining and target_w_remaining < 1e-9:
                                best_belt = (best_belt[0], best_belt[1], 0.0, selected_belt)
                        elif intersecting_handoff:
                            cw = getattr(parcel.body, "target_w_last", best_belt[2])
                            best_belt = (best_belt[0], best_belt[1], cw, selected_belt)
                        else:
                            parcel.body.target_w_remaining = None
                            parcel.body.momentum_intersection_ids = set()

                        previous_belt_id = getattr(parcel.body, "current_belt_id", None)
                        if previous_belt_id and previous_belt_id != selected_belt.id:
                            # Start a fresh momentum-blend window using the
                            # *new* belt's own configured transition time.
                            # This used to set transition_momentum_time to a
                            # hardcoded 0.9s (or 0.01s for curved belts)
                            # before overwriting transition_momentum_duration
                            # with the belt's real value - since
                            # dynamic_velocity_modifier computes
                            # alpha = momentum_time / momentum_duration, that
                            # mismatch meant a belt's configured
                            # transition_momentum_duration never actually
                            # controlled the transition's timing (e.g. the
                            # 0.6s default produced 0.3s of zero blending
                            # followed by 0.6s of blending - 0.9s total,
                            # ignoring the configured value entirely).
                            current_v = parcel.body.velocity
                            parcel.body.transition_momentum_v = (current_v.x, current_v.y)
                            parcel.body.transition_momentum_duration = selected_belt.transition_momentum_duration
                            parcel.body.transition_momentum_time = parcel.body.transition_momentum_duration
                        parcel.body.current_belt_id = selected_belt.id
                        parcel.body.target_v_enable = True
                        parcel.body.target_v = (best_belt[0], best_belt[1])
                        parcel.body.target_w = best_belt[2]
                        # print(f"Parcel {parcel.id} target_v: {parcel.body.target_v}, target_w: {parcel.body.target_w}")
                else:
                    parcel.body.target_v_enable = False
                    parcel.body.velocity *= 0.9
                    parcel.body.angular_velocity *= 0.9
                    parcel.body.target_w_remaining = None
                    parcel.body.momentum_intersection_ids = set()
                    parcel.body.transition_momentum_v = None
                    parcel.body.transition_momentum_time = 0.0
                    parcel.body.current_belt_id = None
                continue
            else:
                parcel.body.target_v_enable = False

            # --- ORIGINAL MANUAL IMPULSE LOGIC ---
            # Only computed here, not up front - the useSurfaceVelocity
            # branch above `continue`s without ever touching belt_vels, so
            # sampling it unconditionally for every parcel/tick was pure
            # waste whenever that mode is on.
            belt_vels = [ self._get_belt_velocity_at(sp[0], sp[1]) for sp in sample_pts ]
            any_hit = False
            for idx, belt_vel in enumerate(belt_vels):
                sp = sample_pts[idx]
                if belt_vel is not None:
                    any_hit = True
                    pt_vel = parcel.body.velocity_at_world_point(sp)
                    dvx = belt_vel[0] - pt_vel.x
                    dvy = belt_vel[1] - pt_vel.y
                    grip_factor = 0.8
                    if idx == 0:
                        grip_factor = 0.3
                    friction_ratio = 0.2
                    if math.fabs(belt_vel[0]) < 0.01 and math.fabs(dvx) > 0.01:
                        dvx = friction_ratio * dvx
                    if math.fabs(belt_vel[1]) < 0.01 and math.fabs(dvy) > 0.01:
                        dvy = friction_ratio * dvy
                        
                    mass_per_pt = parcel.body.mass / len(sample_pts)
                    ix = dvx * mass_per_pt * grip_factor
                    iy = dvy * mass_per_pt * grip_factor
                    MAX_IMPULSE = 0.5 * parcel.body.mass / len(sample_pts)
                    imp_mag = math.hypot(ix, iy)
                    if imp_mag > MAX_IMPULSE:
                        scale = MAX_IMPULSE / imp_mag
                        ix *= scale
                        iy *= scale
                    parcel.body.apply_impulse_at_world_point((ix, iy), sp)

            if not any_hit:
                parcel.body.velocity *= 0.9
                parcel.body.angular_velocity *= 0.9

    def _get_sensor_shape(self, sensor):
        cache_key = sensor.id
        geom = (sensor.x, sensor.y, sensor.width, sensor.height, sensor.rotation, sensor.sensorType)
        
        cached = self._sensor_shapes_cache.get(cache_key)
        if cached and cached[0] == geom:
            return cached[1]
            
        temp_body = pymunk.Body(body_type=pymunk.Body.STATIC)
        temp_body.position = (sensor.x, sensor.y)
        temp_body.angle = math.radians(sensor.rotation)
        sensor_shape = pymunk.Poly.create_box(temp_body, (sensor.width, sensor.height))
        
        self._sensor_shapes_cache[cache_key] = (geom, sensor_shape)
        return sensor_shape

    def _parcel_vertices(self, parcel) -> List[Tuple[float, float]]:
        body = parcel.body
        vertices = []
        for vertex in parcel.shape.get_vertices():
            world = body.position + vertex.rotated(body.angle)
            vertices.append((world.x, world.y))
        return vertices

    def _project_polygon(self, vertices: List[Tuple[float, float]], axis: Tuple[float, float]) -> Tuple[float, float]:
        projections = [vx * axis[0] + vy * axis[1] for vx, vy in vertices]
        return min(projections), max(projections)

    def _polygons_overlap(self, a: List[Tuple[float, float]], b: List[Tuple[float, float]]) -> bool:
        for vertices in (a, b):
            for idx, current in enumerate(vertices):
                nxt = vertices[(idx + 1) % len(vertices)]
                edge_x = nxt[0] - current[0]
                edge_y = nxt[1] - current[1]
                length = math.hypot(edge_x, edge_y)
                if length < 1e-9:
                    continue

                axis = (-edge_y / length, edge_x / length)
                min_a, max_a = self._project_polygon(a, axis)
                min_b, max_b = self._project_polygon(b, axis)
                if max_a < min_b or max_b < min_a:
                    return False
        return True

    def _orientation(self, a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def _point_on_segment(
        self,
        p: Tuple[float, float],
        a: Tuple[float, float],
        b: Tuple[float, float],
    ) -> bool:
        eps = 1e-9
        if abs(self._orientation(a, b, p)) > eps:
            return False
        return (
            min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps
            and min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps
        )

    def _segments_intersect(
        self,
        a1: Tuple[float, float],
        a2: Tuple[float, float],
        b1: Tuple[float, float],
        b2: Tuple[float, float],
    ) -> bool:
        o1 = self._orientation(a1, a2, b1)
        o2 = self._orientation(a1, a2, b2)
        o3 = self._orientation(b1, b2, a1)
        o4 = self._orientation(b1, b2, a2)
        eps = 1e-9

        if o1 * o2 < -eps and o3 * o4 < -eps:
            return True
        return (
            self._point_on_segment(b1, a1, a2)
            or self._point_on_segment(b2, a1, a2)
            or self._point_on_segment(a1, b1, b2)
            or self._point_on_segment(a2, b1, b2)
        )

    def _segment_intersects_polygon(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float],
        polygon: List[Tuple[float, float]],
    ) -> bool:
        if point_in_polygon(start[0], start[1], polygon) or point_in_polygon(end[0], end[1], polygon):
            return True

        for idx, current in enumerate(polygon):
            nxt = polygon[(idx + 1) % len(polygon)]
            if self._segments_intersect(start, end, current, nxt):
                return True
        return False

    def evaluate_sensors(self, h: float = 0.0):
        """Sensor states from parcel geometry: each beam's raw blocked state
        (an IR sensor's footprint, or each laser banner beam as a segment
        across the sensor), then the sensor's output stage
        (devices.condition: on/off delay, invert, detection counter)."""
        # Every parcel's position once (pymunk reads are not free), for all sensors.
        self._parcel_positions = [(p, *p.body.position) for p in self.parcels]
        for sensor in self.config.sensors:
            raw = self._raw_beams(sensor)
            sensor.sensor_states = devices.condition(sensor, raw, h)
        self._parcel_positions = None

    def _close_parcels(self, sensor, extent: float):
        positions = getattr(self, "_parcel_positions", None)
        if positions is None:
            positions = [(p, *p.body.position) for p in self.parcels]
        out = []
        for p, x, y in positions:
            reach = extent + p.max_extent
            dx, dy = x - sensor.x, y - sensor.y
            if dx * dx + dy * dy <= reach * reach:
                out.append(p)
        return out

    def _raw_beams(self, sensor) -> List[bool]:
        if sensor.sensorType == "ir":
            close = self._close_parcels(sensor, math.hypot(sensor.width / 2.0, sensor.height / 2.0))
            if not close:
                return [False]
            verts = get_rect_vertices(sensor.x, sensor.y, sensor.width, sensor.height, sensor.rotation)
            return [any(self._polygons_overlap(verts, self._parcel_vertices(p)) for p in close)]

        laser_count = int(sensor.laserCount) if sensor.laserCount is not None else 16
        beam_dist = float(sensor.beamDistance) if sensor.beamDistance is not None else 0.05
        close = self._close_parcels(sensor, math.hypot(sensor.width / 2.0, (laser_count - 1) * beam_dist / 2.0))
        if not close:
            return [False] * laser_count
        rad = math.radians(sensor.rotation)
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        polygons = []
        for p in close:
            poly = self._parcel_vertices(p)
            xs, ys = [v[0] for v in poly], [v[1] for v in poly]
            polygons.append((min(xs), min(ys), max(xs), max(ys), poly))
        beams = []
        for i in range(laser_count):
            # Beam i: a segment across the sensor's width at offset `by`.
            by = (i - (laser_count - 1) / 2.0) * beam_dist
            lx = sensor.width / 2.0
            start = (sensor.x - lx * cos_a - by * sin_a, sensor.y - lx * sin_a + by * cos_a)
            end = (sensor.x + lx * cos_a - by * sin_a, sensor.y + lx * sin_a + by * cos_a)
            bx0, bx1 = min(start[0], end[0]), max(start[0], end[0])
            by0, by1 = min(start[1], end[1]), max(start[1], end[1])
            # Only parcels whose bounding box touches the beam's get the exact test.
            beams.append(any(self._segment_intersects_polygon(start, end, poly)
                             for x0, y0, x1, y1, poly in polygons
                             if x0 <= bx1 and bx0 <= x1 and y0 <= by1 and by0 <= y1))
        return beams

    def _update_belt_loads(self):
        """Mass of parcels on each belt with overload protection (drive.maxLoad)."""
        watched = [b for b in self.config.belts if b.drive is not None and b.drive.maxLoad is not None]
        if not watched:
            return
        for belt in watched:
            belt.load = 0.0
        for parcel in self.parcels:
            px, py = parcel.body.position
            for belt in self._belts_under(px, py, watched):
                belt.load += parcel.body.mass

    def step(self, h: float):
        """One fixed physics step: drives, sources, traction, solver, sinks,
        sensors. Sensors run every step so a parcel crossing a narrow sensor
        between two GUI frames is still seen (and counted in risingEdges)."""
        with geometry_frame(self.config.belts):
            # Each belt's bounding box for this step: a point is only tested
            # against the belts whose box holds it (one or two, not all).
            self._belt_boxes = [(belt, *belt_bounds(belt)) for belt in self.config.belts]
            self._step(h)

    def _step(self, h: float):
        self._update_belt_loads()
        for belt in self.config.belts:
            step_drive(belt, h)
        for led in self.config.leds:
            devices.update_led(led, self.sim_time)
        self.handle_sources(h)
        if self.config.tractionModel == "friction":
            # Velocities persist between steps; friction alone slows parcels.
            self.space.damping = 1.0
            self.apply_friction_traction()
        else:
            self.space.damping = 0.1
        self.apply_conveyor_kinematics()
        self.space.step(h)
        self.handle_sinks()
        self.evaluate_sensors(h)
        self.sim_time += h
        self.step_count += 1
        self.config.simTimeNs = int(round(self.sim_time * 1e9))

    def update(self, dt: float):
        """Advance by `dt` seconds of real time in whole `physicsDt` steps,
        carrying the remainder to the next call - physics, drives, and
        sensors always see the same step size no matter how irregular the
        caller's frame rate is.

        The lock is held for the whole advance (CPU-only work, no I/O), so a
        concurrent deserialize() can't interleave mid-step."""
        with self._lock:
            h = self.config.physicsDt
            if h <= 0.0:
                raise ValueError(f"physicsDt must be > 0, got {h}")
            self._accumulator += max(0.0, dt)
            steps = int(self._accumulator / h + 1e-9)
            max_steps = max(1, self.config.maxStepsPerAdvance)
            if steps > max_steps:
                # Can't keep up (or the caller paused): drop the backlog
                # instead of spiraling, and report it.
                self.dropped_time += (steps - max_steps) * h
                steps = max_steps
                self._accumulator = steps * h
            for _ in range(steps):
                self.step(h)
            self._accumulator -= steps * h

    def step_for(self, wall_seconds: float) -> int:
        """As fast as possible: whole physicsDt steps back to back, holding
        the lock, until `wall_seconds` of wall time is used (at least one
        step). Returns the steps run. The caller releases the lock between
        calls (sim_runner's max speed)."""
        with self._lock:
            h = self.config.physicsDt
            end = time.perf_counter() + wall_seconds
            steps = 0
            while True:
                self.step(h)
                steps += 1
                if time.perf_counter() >= end:
                    return steps

    def run_for(self, seconds: float):
        """Deterministic headless stepping (tests, scripted scenarios):
        exactly round(seconds / physicsDt) steps, no step cap."""
        h = self.config.physicsDt
        with self._lock:
            for _ in range(int(round(seconds / h))):
                self.step(h)
