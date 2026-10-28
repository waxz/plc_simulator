"""
test_physics.py - unit tests for physics.py's belt-to-belt velocity
transition system (`_smootherstep`, `dynamic_velocity_modifier`, and the
handoff wiring in `PhysicsEngine.apply_conveyor_kinematics` that feeds it).

Regression coverage in particular for a confirmed bug: a belt handoff used
to set `transition_momentum_time` to a hardcoded 0.9s/0.01s before
overwriting `transition_momentum_duration` with the belt's own configured
value, so `Belt.transition_momentum_duration` never actually controlled the
transition's real timing (see `test_handoff_uses_new_belts_configured_
transition_duration_not_hardcoded`).
"""
import math
import unittest

import pymunk

from config_parser import Belt, SimulatorConfig
from physics import Parcel, PhysicsEngine, _smootherstep, dynamic_velocity_modifier


class SmootherstepTest(unittest.TestCase):
    def test_endpoints(self):
        self.assertEqual(_smootherstep(0.0), 0.0)
        self.assertEqual(_smootherstep(1.0), 1.0)

    def test_clamps_out_of_range_input(self):
        self.assertEqual(_smootherstep(-0.5), 0.0)
        self.assertEqual(_smootherstep(1.5), 1.0)

    def test_midpoint_is_symmetric(self):
        self.assertAlmostEqual(_smootherstep(0.5), 0.5)

    def test_monotonically_increasing(self):
        samples = [i / 100.0 for i in range(101)]
        values = [_smootherstep(t) for t in samples]
        self.assertEqual(values, sorted(values))

    def test_eases_in_and_out_slower_than_a_linear_ramp_near_the_endpoints(self):
        # The defining property of smootherstep vs. a straight line: it
        # changes *slower* than linear right at the edges (zero slope) and
        # faster than linear through the middle - exactly what makes the
        # blend feel like an eased transition instead of a constant-rate
        # ramp with a sharp corner at each end. By point-symmetry around
        # (0.5, 0.5), sitting below the diagonal near t=0.1 means sitting
        # *above* it near t=0.9 (already closer to 1, easing the rest of
        # the way in gently) - both are "slower than linear at the edge".
        self.assertLess(_smootherstep(0.1), 0.1)
        self.assertGreater(_smootherstep(0.9), 0.9)
        self.assertGreater(_smootherstep(0.5) - _smootherstep(0.4), 0.1)


def make_body(**attrs):
    body = pymunk.Body(1.0, pymunk.moment_for_box(1.0, (0.5, 0.5)))
    body.velocity = (0.0, 0.0)
    body.angular_velocity = 0.0
    for key, value in attrs.items():
        setattr(body, key, value)
    return body


class DynamicVelocityModifierTest(unittest.TestCase):
    def test_leaves_default_physics_untouched_when_target_v_disabled(self):
        body = make_body(target_v_enable=False)
        body.velocity = (5.0, 0.0)
        dynamic_velocity_modifier(body, (0, 0), 1.0, 1 / 60.0)
        # No gravity/forces and damping=1.0 -> update_velocity is a no-op,
        # so velocity should be unchanged (not snapped to any target).
        self.assertAlmostEqual(body.velocity.x, 5.0, places=6)

    def test_snaps_directly_to_target_when_no_transition_is_active(self):
        body = make_body(
            target_v_enable=True, target_v=(2.0, 0.0),
            transition_momentum_v=None, transition_momentum_time=0.0,
            transition_momentum_duration=0.6,
        )
        dynamic_velocity_modifier(body, (0, 0), 1.0, 1 / 60.0)
        self.assertAlmostEqual(body.velocity.x, 2.0, places=6)

    def test_snaps_directly_to_target_when_duration_too_small(self):
        # transition_momentum_duration <= 0.01 is treated as "instant".
        body = make_body(
            target_v_enable=True, target_v=(2.0, 0.0),
            transition_momentum_v=(0.0, 0.0), transition_momentum_time=0.01,
            transition_momentum_duration=0.01,
        )
        dynamic_velocity_modifier(body, (0, 0), 1.0, 1 / 60.0)
        self.assertAlmostEqual(body.velocity.x, 2.0, places=6)

    def test_blend_at_start_is_close_to_the_old_velocity(self):
        body = make_body(
            target_v_enable=True, target_v=(2.0, 0.0),
            transition_momentum_v=(0.0, 0.0), transition_momentum_time=0.6,
            transition_momentum_duration=0.6,
        )
        dynamic_velocity_modifier(body, (0, 0), 1.0, 1 / 600.0)  # one tiny step
        # smootherstep(t~1) ~ 1, so velocity should barely have moved from 0.
        self.assertLess(body.velocity.x, 0.1)

    def test_blend_matches_smootherstep_of_remaining_fraction(self):
        body = make_body(
            target_v_enable=True, target_v=(2.0, 0.0),
            transition_momentum_v=(0.0, 0.0), transition_momentum_time=0.3,
            transition_momentum_duration=0.6,
        )
        dynamic_velocity_modifier(body, (0, 0), 1.0, 0.0)  # dt=0: inspect this instant's blend only
        expected_alpha = _smootherstep(0.3 / 0.6)
        expected_vx = 0.0 * expected_alpha + 2.0 * (1.0 - expected_alpha)
        self.assertAlmostEqual(body.velocity.x, expected_vx, places=6)

    def test_transition_completes_exactly_at_the_configured_duration(self):
        # Regression test for the timing bug: stepping for exactly
        # `transition_momentum_duration` seconds (however that's split into
        # sub-steps) must leave momentum_time at 0 and velocity at target_v,
        # for whatever duration a belt is configured with - not a hardcoded
        # 0.9s/0.01s.
        for duration in (0.15, 0.6, 1.2):
            with self.subTest(duration=duration):
                body = make_body(
                    target_v_enable=True, target_v=(2.0, 0.0),
                    transition_momentum_v=(0.0, 0.0),
                    transition_momentum_time=duration,
                    transition_momentum_duration=duration,
                )
                dt = 1 / 60.0
                elapsed = 0.0
                while elapsed < duration - 1e-9:
                    dynamic_velocity_modifier(body, (0, 0), 1.0, dt)
                    elapsed += dt
                # Alpha for a given call is computed from momentum_time
                # *before* that call's own decrement, so the tick that
                # brings momentum_time down to exactly 0 still applies a
                # small leftover alpha from the prior (not-yet-zero) value
                # - matching real continuous simulation (space.step() keeps
                # ticking, it doesn't stop dead at the duration boundary),
                # one more tick is exactly what happens next: momentum_time
                # is now 0.0, so this call takes the direct-snap branch.
                dynamic_velocity_modifier(body, (0, 0), 1.0, dt)
                self.assertAlmostEqual(body.transition_momentum_time, 0.0, places=9)
                self.assertAlmostEqual(body.velocity.x, 2.0, places=6)

    def test_blending_starts_by_the_second_step_not_after_a_dead_zone(self):
        # Direct regression check for the pre-fix behavior, where alpha
        # stayed pinned at 1.0 (zero blending at all) for the first
        # `hardcoded_initial - configured_duration` seconds regardless of
        # how many steps ran. Alpha for a given call is computed from
        # momentum_time *before* that call's decrement, so the very first
        # call is exactly at t=1 (alpha=1, no visible movement yet) by
        # construction, not because of the bug - it's the *second* call
        # (now at t<1) that must show movement, and must do so quickly,
        # not after some multi-hundred-millisecond dead zone.
        body = make_body(
            target_v_enable=True, target_v=(2.0, 0.0),
            transition_momentum_v=(0.0, 0.0), transition_momentum_time=0.6,
            transition_momentum_duration=0.6,
        )
        dynamic_velocity_modifier(body, (0, 0), 1.0, 1 / 60.0)
        self.assertEqual(body.velocity.x, 0.0)
        dynamic_velocity_modifier(body, (0, 0), 1.0, 1 / 60.0)
        self.assertGreater(body.velocity.x, 0.0)

    def test_angular_velocity_is_clamped_to_the_remaining_budget(self):
        # The angular-velocity block only runs when target_v_enable is True
        # (the function returns early otherwise - by design, a parcel not
        # on any belt shouldn't have rotation forced either) - give it a
        # trivial, already-arrived linear target so only the angular
        # clamping behavior is under test here.
        body = make_body(
            target_v_enable=True, target_v=(0.0, 0.0),
            transition_momentum_v=None, transition_momentum_time=0.0,
            transition_momentum_duration=0.0,
            target_w=10.0, target_w_remaining=0.05,
        )
        dynamic_velocity_modifier(body, (0, 0), 1.0, 1.0)  # dt=1s would overshoot a 0.05 rad budget
        self.assertAlmostEqual(body.angular_velocity, 0.05, places=6)
        self.assertAlmostEqual(body.target_w_remaining, 0.0, places=6)

    def test_angular_velocity_uses_target_w_directly_once_budget_exhausted(self):
        body = make_body(
            target_v_enable=True, target_v=(0.0, 0.0),
            transition_momentum_v=None, transition_momentum_time=0.0,
            transition_momentum_duration=0.0,
            target_w=1.5, target_w_remaining=None,
        )
        dynamic_velocity_modifier(body, (0, 0), 1.0, 1 / 60.0)
        self.assertAlmostEqual(body.angular_velocity, 1.5, places=6)


def make_two_belt_config(transition_duration_b: float = 0.3) -> SimulatorConfig:
    belt_a = Belt(
        id="belt_a", type="linear", x=0.0, y=0.0, length=2.0, beltWidth=1.0,
        rotation=0.0, speed=1.0, directionAngle=0.0, color="#3b82f6",
    )
    belt_b = Belt(
        id="belt_b", type="linear", x=3.0, y=0.0, length=2.0, beltWidth=1.0,
        rotation=0.0, speed=3.0, directionAngle=0.0, color="#22c55e",
        transition_momentum_duration=transition_duration_b,
    )
    return SimulatorConfig(belts=[belt_a, belt_b], useSurfaceVelocity=True)


def spawn_parcel(engine: PhysicsEngine, x: float, y: float) -> Parcel:
    body = pymunk.Body(1.0, pymunk.moment_for_box(1.0, (0.3, 0.3)))
    body.position = (x, y)
    body.velocity_func = dynamic_velocity_modifier
    shape = pymunk.Poly.create_box(body, (0.3, 0.3))
    shape.collision_type = PhysicsEngine.COLLISION_TYPE_PARCEL
    shape.filter = pymunk.ShapeFilter(
        categories=PhysicsEngine.CATEGORY_PARCEL,
        mask=PhysicsEngine.CATEGORY_WALL | PhysicsEngine.CATEGORY_PARCEL,
    )
    engine.space.add(body, shape)
    parcel = Parcel(id="p_test", body=body, shape=shape, label="TEST", color="#ffaa00")
    engine.parcels.append(parcel)
    return parcel


class BeltHandoffWiringTest(unittest.TestCase):
    """Integration-level tests for the code in apply_conveyor_kinematics
    that sets up a momentum-blend window on a belt-to-belt handoff."""

    def test_no_transition_state_on_first_contact(self):
        config = make_two_belt_config()
        engine = PhysicsEngine(config)
        parcel = spawn_parcel(engine, 0.0, 0.0)  # starts directly on belt_a

        engine.apply_conveyor_kinematics()

        self.assertEqual(parcel.body.current_belt_id, "belt_a")
        self.assertIsNone(getattr(parcel.body, "transition_momentum_time", None))

    def test_handoff_uses_new_belts_configured_transition_duration_not_hardcoded(self):
        config = make_two_belt_config(transition_duration_b=0.3)
        engine = PhysicsEngine(config)
        parcel = spawn_parcel(engine, 0.0, 0.0)
        engine.apply_conveyor_kinematics()  # establishes current_belt_id = belt_a

        parcel.body.position = (3.0, 0.0)  # teleport onto belt_b
        engine.apply_conveyor_kinematics()  # should detect the handoff

        self.assertEqual(parcel.body.current_belt_id, "belt_b")
        # The bug: transition_momentum_time used to be hardcoded to 0.9s
        # regardless of the belt's own setting. It must now equal the new
        # belt's configured duration exactly, matching
        # transition_momentum_duration (also belt_b's value, not 0.9).
        self.assertEqual(parcel.body.transition_momentum_duration, 0.3)
        self.assertEqual(parcel.body.transition_momentum_time, 0.3)

    def test_handoff_captures_current_velocity_as_momentum(self):
        config = make_two_belt_config()
        engine = PhysicsEngine(config)
        parcel = spawn_parcel(engine, 0.0, 0.0)
        engine.apply_conveyor_kinematics()
        parcel.body.velocity = (1.0, 0.5)  # simulate belt_a having accelerated it

        parcel.body.position = (3.0, 0.0)
        engine.apply_conveyor_kinematics()

        self.assertEqual(tuple(parcel.body.transition_momentum_v), (1.0, 0.5))

    def test_full_handoff_transition_reaches_new_belts_speed(self):
        """End-to-end: after a handoff, running the actual velocity_func
        for the belt's configured duration lands the parcel's velocity on
        belt_b's target speed (3.0 m/s along +X, since both belts have
        directionAngle=0)."""
        config = make_two_belt_config(transition_duration_b=0.2)
        engine = PhysicsEngine(config)
        parcel = spawn_parcel(engine, 0.0, 0.0)
        engine.apply_conveyor_kinematics()

        parcel.body.position = (3.0, 0.0)
        engine.apply_conveyor_kinematics()
        self.assertAlmostEqual(parcel.body.transition_momentum_time, 0.2)

        dt = 1 / 60.0
        elapsed = 0.0
        while elapsed < 0.2 - 1e-9:
            dynamic_velocity_modifier(parcel.body, (0, 0), 1.0, dt)
            elapsed += dt

        self.assertAlmostEqual(parcel.body.transition_momentum_time, 0.0)
        self.assertAlmostEqual(parcel.body.velocity.x, 3.0, places=1)


if __name__ == "__main__":
    unittest.main()


class CrowdedLineSpeedupTest(unittest.TestCase):
    """The per-step shortcuts (geometry looked up once per step, belts
    pre-filtered by bounding box) give the same answers as testing every
    belt the slow way."""

    def test_belts_under_a_point_match_testing_every_belt(self):
        import random
        from config_parser import load_config
        from surface import belt_bounds, belt_contains, geometry_frame
        config = load_config("test_config.json")
        engine = PhysicsEngine(config)
        rng = random.Random(7)
        points = [(rng.uniform(14.0, 22.0), rng.uniform(-21.0, 16.0)) for _ in range(3000)]
        slow = [[b.id for b in config.belts if belt_contains(b, x, y)] for x, y in points]
        with geometry_frame(config.belts):
            engine._belt_boxes = [(b, *belt_bounds(b)) for b in config.belts]
            fast = [[b.id for b in engine._belts_under(x, y)] for x, y in points]
            framed = [[b.id for b in config.belts if belt_contains(b, x, y)] for x, y in points]
        self.assertEqual(fast, slow)
        self.assertEqual(framed, slow)
        self.assertGreater(sum(1 for ids in slow if ids), 300)          # plenty of points on belts

    def test_a_crowded_line_still_delivers(self):
        from config_parser import load_config
        config = load_config("test_config.json")
        for belt in config.belts:
            belt.speed = 2.0
        engine = PhysicsEngine(config)
        for _ in range(400):                                             # 20 s
            engine.update(0.05)
        self.assertGreater(len(engine.parcels), 20)
        self.assertGreater(sum(engine.delivered.values()), 0)
