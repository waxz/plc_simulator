from config_parser import Belt
from typing import Dict
from pymunk.vec2d import Vec2d
import math
import random
import pymunk
from typing import List, Tuple, Optional
from config_parser import SimulatorConfig
from math_utils import point_in_polygon, point_in_arc, get_rect_vertices, rotate_point

class Parcel:
    def __init__(self, id: str, body: pymunk.Body, shape: pymunk.Poly, label: str, color: str):
        self.id = id
        self.body = body
        self.shape = shape
        self.label = label
        self.color = color
        self.max_extent = max(math.hypot(v.x, v.y) for v in shape.get_vertices()) if hasattr(shape, 'get_vertices') else 0.5

def dynamic_velocity_modifier(body, gravity, damping, dt):
    pymunk.Body.update_velocity(body, gravity, damping, dt)
    
    target_v_enable = getattr(body, "target_v_enable", False) 
    if not target_v_enable:
        return
    target_v = Vec2d(*getattr(body, "target_v", body.velocity))
    momentum_v = getattr(body, "transition_momentum_v", None)
    momentum_time = getattr(body, "transition_momentum_time", 0.0)
    momentum_duration = getattr(body, "transition_momentum_duration", 0.2)
    # print(f"dynamic_velocity_modifier target_v: {target_v}, momentum_v: {momentum_v}, momentum_time: {momentum_time}, momentum_duration: {momentum_duration}")
    if momentum_v is not None and momentum_time > 0.0 and momentum_duration > 0.01:
        alpha = max(0.0, min(1.0, momentum_time / momentum_duration))
        original_v = Vec2d(*momentum_v)
        body.velocity = original_v * alpha + target_v * (1.0 - alpha)
        body.transition_momentum_time = max(0.0, momentum_time - dt)
    else:
        body.velocity = target_v
        body.transition_momentum_v = None
        body.transition_momentum_time = 0.0

    # body.velocity = target_v

    target_w = getattr(body, "target_w", body.angular_velocity)
    remaining_w = getattr(body, "target_w_remaining", None)
    
    # body.angular_velocity = target_w

    if remaining_w is not None and remaining_w > 0.0 and dt > 0.0 and abs(target_w) > 1e-9:
        clamped_w = min(abs(target_w), remaining_w / dt)
        body.angular_velocity = math.copysign(clamped_w, target_w)
        body.target_w_remaining = max(0.0, remaining_w - clamped_w * dt)
    else:
        body.angular_velocity = target_w

    # print(f"dynamic_velocity_modifier body.velocity: {body.velocity}, body.angular_velocity: {body.angular_velocity}")

# def constant_velocity(body, gravity, damping, dt):
#     body.velocity = (1.95, 0)  # Set your target velocity here
#     pymunk.Body.update_velocity(body, gravity, damping, dt) # Apply space physics
# 1. Define the custom velocity function
# def dynamic_velocity_modifier(body, gravity, damping, dt):
#     """
#     Pymunk calls this every frame automatically.
#     It reads the dynamic target speeds stored directly on the body.
#     """
#     # Fallback to current velocity if custom attributes aren't set yet
#     target_v_enable = getattr(body, "target_v_enable", False) 
#     if not target_v_enable:
#         return

#     target_v = getattr(body, "target_v", body.velocity) 

#     # Assign the exact desired velocity
#     body.velocity =  target_v

class PhysicsEngine:
    # ShapeFilter category bitmasks
    CATEGORY_WALL   = 0b01
    CATEGORY_PARCEL = 0b10
    # collision_type integers used by space.on_collision()
    COLLISION_TYPE_WALL   = 1
    COLLISION_TYPE_PARCEL = 2

    def __init__(self, config: SimulatorConfig):
        self.config = config
        self.space = pymunk.Space()
        self.space.gravity = (0, 0)
        self.space.damping = 0.1 # High damping for top-down
        
        self.parcels: List[Parcel] = []
        self.static_walls: List[pymunk.Shape] = []
        self.parcel_id_counter = 0
        self.sensor_states = {}  # {sensor.id: bool | str}
        
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
        min_x, min_y, max_x, max_y = self._get_belt_bounds(belt)
        if not (min_x <= px <= max_x and min_y <= py <= max_y):
            return False

        if belt.type == "linear":
            if getattr(belt, 'shape', 'rectangle') == 'quadrilateral':
                pts = belt.vertexs
                if not pts:
                    hw = belt.length / 2.0
                    hh = belt.beltWidth / 2.0
                    pts = [{'x': -hw, 'y': -hh}, {'x': hw, 'y': -hh}, {'x': hw, 'y': hh}, {'x': -hw, 'y': hh}]
                verts = []
                for pt in pts:
                    rx, ry = rotate_point(0, 0, belt.rotation, pt['x'], pt['y'])
                    verts.append((belt.x + rx, belt.y + ry))
            else:
                verts = get_rect_vertices(belt.x, belt.y, belt.length, belt.beltWidth, belt.rotation)
            return point_in_polygon(px, py, verts)
        elif belt.type == "curved":
            return point_in_arc(px, py, belt.x, belt.y, belt.radius, belt.beltWidth, belt.startAngle, belt.endAngle)
        return False

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
        if belt.type == "linear":
            if getattr(belt, 'shape', 'rectangle') == 'quadrilateral':
                pts = belt.vertexs
                if not pts:
                    hw = belt.length / 2.0
                    hh = belt.beltWidth / 2.0
                    pts = [{'x': -hw, 'y': -hh}, {'x': hw, 'y': -hh}, {'x': hw, 'y': hh}, {'x': -hw, 'y': hh}]
                verts = []
                for pt in pts:
                    rx, ry = rotate_point(0, 0, belt.rotation, pt['x'], pt['y'])
                    verts.append((belt.x + rx, belt.y + ry))
            else:
                verts = get_rect_vertices(belt.x, belt.y, belt.length, belt.beltWidth, belt.rotation)
            xs = [v[0] for v in verts]
            ys = [v[1] for v in verts]
            return min(xs), min(ys), max(xs), max(ys)

        outer_r = belt.radius + belt.beltWidth / 2.0
        return belt.x - outer_r, belt.y - outer_r, belt.x + outer_r, belt.y + outer_r

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
                if belt.type == "linear":
                    rad = math.radians(belt.rotation + belt.directionAngle)
                    vs.append((belt.speed * math.cos(rad), belt.speed * math.sin(rad), 0.0))

                elif belt.type == "curved":
                    dx = px - belt.x
                    dy = py - belt.y
                    dist = math.hypot(dx, dy)
                    if dist < 1e-6:
                        continue
                    # Tangent velocity proportional to radius (v = w * r)
                    direction = 1 if getattr(belt, 'directionAngle', 0) > 0 else -1
                    w = direction * belt.speed / belt.radius if belt.radius > 0 else 0
                    speed_at_dist = w * dist
                    tx = -dy / dist 
                    ty =  dx / dist
                    vs.append((speed_at_dist * tx, speed_at_dist * ty, w))

        if hit:
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
                    mass = 1.0
                    moment = pymunk.moment_for_box(mass, (w, h))
                    body = pymunk.Body(mass, moment)
                    body.position = (sx, sy)
                    body.angle = math.radians(source.rotation)
                    
                    if getattr(self.config, 'useSurfaceVelocity', False):
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
                    self.parcels.append(parcel)
                    self.source_timers[source.id] = 0.0 # reset timer only if spawned

    def handle_sinks(self):
        to_remove = []
        for parcel in self.parcels:
            px, py = parcel.body.position
            for sink in self.config.sinks:
                verts = get_rect_vertices(sink.x, sink.y, sink.width, sink.height, 0)
                if point_in_polygon(px, py, verts):
                    to_remove.append(parcel)
                    break
                    
        for p in to_remove:
            self.space.remove(p.body, p.shape)
            self.parcels.remove(p)

    def clear_all_parcels(self):
        for p in self.parcels:
            self.space.remove(p.body, p.shape)
        self.parcels.clear()

    def apply_conveyor_kinematics(self):
        """Drive each parcel using the belt velocity sampled at its center + 4 corners."""
        for parcel in self.parcels:
            pos = parcel.body.position
            sample_pts = [(pos.x, pos.y)]
            # for v in parcel.shape.get_vertices():
            #     wv = pos + v.rotated(parcel.body.angle)
            #     sample_pts.append((wv.x, wv.y))

            belt_vels = [ self._get_belt_velocity_at(sp[0], sp[1]) for sp in sample_pts ]

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
                        if belt.type == "linear":
                            rad = math.radians(belt.rotation + belt.directionAngle)
                            cvx = belt.speed * math.cos(rad)
                            cvy = belt.speed * math.sin(rad)
                            cw = 0.0
                        elif belt.type == "curved":
                            dx = pos.x - belt.x
                            dy = pos.y - belt.y
                            dist = math.hypot(dx, dy)
                            direction = 1 if getattr(belt, 'directionAngle', 0) > 0 else -1
                            cw = direction * belt.speed / belt.radius if belt.radius > 0 else 0
                            speed_at_dist = cw * dist
                            if dist > 1e-6:
                                cvx = speed_at_dist * (-dy / dist)
                                cvy = speed_at_dist * (dx / dist)

                                radial_error = belt.radius - dist
                                radial_gain = abs(belt.speed) / max(belt.beltWidth * 0.5, 1e-6)
                                max_radial_speed = abs(belt.speed) * 0.75
                                radial_speed = max(
                                    -max_radial_speed,
                                    min(max_radial_speed, radial_error * radial_gain),
                                )
                                # cvx += radial_speed * dx / dist
                                # cvy += radial_speed * dy / dist
                            else:
                                pass
                                # cvx, cvy = 0.0, 0.0
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
                        # print(f"Parcel {parcel.id} is on belt {previous_belt_id} and switching to belt {selected_belt.id}")
                        if previous_belt_id and previous_belt_id != selected_belt.id:

                            current_v = parcel.body.velocity
                            parcel.body.transition_momentum_v = (current_v.x, current_v.y)
                            parcel.body.transition_momentum_duration = 0.01 if selected_belt.type == "curved" else 0.9
                            parcel.body.transition_momentum_time = parcel.body.transition_momentum_duration
                            parcel.body.transition_momentum_duration = selected_belt.transition_momentum_duration
                            # print(f"Parcel {parcel.id} switched from belt {previous_belt_id} to belt {selected_belt.id}")
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
                    sensor.sensor_states = "0"
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
                sensor.sensor_states = triggered

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

    def update(self, dt: float):
        sub_steps = getattr(self.config, 'simulationSteps', 4)
        if sub_steps <= 0: sub_steps = 1
        
        sub_dt = dt / sub_steps
        
        for _ in range(sub_steps):
            self.handle_sources(sub_dt)
            self.apply_conveyor_kinematics()
            self.space.step(sub_dt)
            self.handle_sinks()
            
        # Sensor evaluation once per frame (not per sub-step)
        self.evaluate_sensors()
