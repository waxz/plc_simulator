from config_parser import Belt
from typing import Dict
from pymunk.vec2d import Vec2d
from contextlib import nullcontext
import math
import random
import pymunk
import threading
from typing import List, Tuple, Optional
from config_parser import SimulatorConfig
from math_utils import point_in_polygon, get_rect_vertices
from drive import step_drive
from surface import belt_bounds, belt_contains, belt_surface_speed, curved_direction, surface_velocity
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
        for parcel in self.parcels:
            px, py = parcel.body.position
            in_a_sink = False
            for sink in self.config.sinks:
                verts = get_rect_vertices(sink.x, sink.y, sink.width, sink.height, 0)
                if point_in_polygon(px, py, verts):
                    in_a_sink = True
                    self.delivered[sink.id] = self.delivered.get(sink.id, 0) + 1
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
                    for belt in self.config.belts:
                        if self._is_point_on_belt(sp[0], sp[1], belt):
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

    @staticmethod
    def _set_ir_state(sensor, state: str) -> None:
        if state == "1" and sensor.sensor_states != "1":
            sensor.risingEdges += 1
        sensor.sensor_states = state

    def evaluate_sensors(self):
        """Compute sensor states from parcel geometry only."""
        import math
        
        for sensor in self.config.sensors:
            if sensor.sensorType == "ir":
                # Sensor diagonal extent
                sensor_max_extent = math.hypot(sensor.width / 2.0, sensor.height / 2.0)
                
                # Pre-filter close parcels
                close_parcels = []
                for p in self.parcels:
                    dx = p.body.position.x - sensor.x
                    dy = p.body.position.y - sensor.y
                    if math.hypot(dx, dy) <= (sensor_max_extent + p.max_extent):
                        close_parcels.append(p)
                
                if not close_parcels:
                    self._set_ir_state(sensor, "0")
                    continue
                
                sensor_vertices = get_rect_vertices(
                    sensor.x,
                    sensor.y,
                    sensor.width,
                    sensor.height,
                    sensor.rotation,
                )
                triggered = "0"
                for parcel in close_parcels:
                    if self._polygons_overlap(sensor_vertices, self._parcel_vertices(parcel)):
                        triggered = "1"
                        break
                self._set_ir_state(sensor, triggered)

            elif sensor.sensorType == "laser_banner":
                laser_count_val = getattr(sensor, 'laserCount', 16)
                laser_count = int(laser_count_val) if laser_count_val is not None else 16
                beam_dist_val = getattr(sensor, 'beamDistance', 0.05)
                beam_dist = float(beam_dist_val) if beam_dist_val is not None else 0.05
                
                angle = sensor.rotation
                rad = math.radians(angle)
                cos_a = math.cos(rad)
                sin_a = math.sin(rad)
                
                # Pre-filter using distance checks
                sensor_max_extent = math.hypot(sensor.width / 2.0, (laser_count - 1) * beam_dist / 2.0)
                close_parcels = []
                for p in self.parcels:
                    dx = p.body.position.x - sensor.x
                    dy = p.body.position.y - sensor.y
                    if math.hypot(dx, dy) <= (sensor_max_extent + p.max_extent):
                        close_parcels.append(p)
                
                if not close_parcels:
                    sensor.sensor_states = "0" * laser_count
                    continue
                
                bits = []
                close_polygons = [self._parcel_vertices(parcel) for parcel in close_parcels]
                for i in range(laser_count):
                    by = (i - (laser_count - 1) / 2.0) * beam_dist
                    
                    # Local points of the horizontal beam segment (spanning sensor width)
                    lx_start, ly_start = -sensor.width / 2.0, by
                    lx_end, ly_end = sensor.width / 2.0, by
                    
                    # Transform to World coordinates
                    x_start = sensor.x + lx_start * cos_a - ly_start * sin_a
                    y_start = sensor.y + lx_start * sin_a + ly_start * cos_a
                    x_end = sensor.x + lx_end * cos_a - ly_end * sin_a
                    y_end = sensor.y + lx_end * sin_a + ly_end * cos_a
                    
                    beam_start = (x_start, y_start)
                    beam_end = (x_end, y_end)
                    blocked = False
                    for polygon in close_polygons:
                        if self._segment_intersects_polygon(beam_start, beam_end, polygon):
                            blocked = True
                            break
                    bits.append("1" if blocked else "0")
                sensor.sensor_states = "".join(bits)

    def step(self, h: float):
        """One fixed physics step: drives, sources, traction, solver, sinks,
        sensors. Sensors run every step so a parcel crossing a narrow sensor
        between two GUI frames is still seen (and counted in risingEdges)."""
        for belt in self.config.belts:
            step_drive(belt, h)
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
        self.evaluate_sensors()
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

    def run_for(self, seconds: float):
        """Deterministic headless stepping (tests, scripted scenarios):
        exactly round(seconds / physicsDt) steps, no step cap."""
        h = self.config.physicsDt
        with self._lock:
            for _ in range(int(round(seconds / h))):
                self.step(h)
