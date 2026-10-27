"""
traction.py - friction-based belt traction (SimulatorConfig.tractionModel
== "friction").

Each parcel is tied to its own kinematic "carrier" body by a PivotJoint
(linear friction) and a GearJoint (rotational friction), both with
max_bias = 0 so they only ever match velocities, never pull positions.
Their max_force is the friction the belt can transmit, mu * m * g, so:

  - a parcel accelerates to belt speed at most at mu * g,
  - stops on a stopped belt over v^2 / (2 * mu * g),
  - slides when pushed harder than friction allows,

all from pymunk's solver instead of hand-tuned blending. This is pymunk's
standard top-down friction technique (the same one its tank demo uses).

Every step, pre_step() samples the belt surface velocity under 9 points of
the parcel's footprint, fits one rigid motion (v_c, omega) to them, and
drives the carrier with it. That fit is exact for straight and curved belts,
and where a parcel straddles two belts the mismatch shows up as a torque -
the real skew effect a belt transfer has.
"""
import math
from typing import List, Optional, Sequence, Tuple

import pymunk

from config_parser import Belt
from surface import belt_bounds, belt_contains, surface_velocity

GRAVITY = 9.81

# Footprint sample points in the unit square [-1, 1]^2: center, 4 edge
# midpoints, 4 corners. Scaled by the parcel's half extents.
_UNIT_SAMPLES: Tuple[Tuple[float, float], ...] = (
    (0.0, 0.0),
    (1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0),
    (1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0),
)
# Pull corner/edge samples slightly inside the footprint so a parcel sitting
# exactly on a belt edge isn't sampled half on the floor.
_INSET = 0.9


class FrictionTraction:
    def __init__(self, space: pymunk.Space, floor_friction: float):
        self.space = space
        self.floor_friction = floor_friction

    def attach(self, body: pymunk.Body, half_w: float, half_h: float) -> dict:
        """Create the carrier body + friction joints for a newly spawned
        parcel. Returns the state dict physics.Parcel keeps as `traction`."""
        carrier = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        carrier.position = body.position
        carrier.angle = body.angle

        pivot = pymunk.PivotJoint(carrier, body, (0, 0), (0, 0))
        pivot.max_bias = 0.0
        pivot.max_force = 0.0
        gear = pymunk.GearJoint(carrier, body, 0.0, 1.0)
        gear.max_bias = 0.0
        gear.max_force = 0.0

        self.space.add(carrier, pivot, gear)
        samples = [(ux * half_w * _INSET, uy * half_h * _INSET) for ux, uy in _UNIT_SAMPLES]
        return {"carrier": carrier, "pivot": pivot, "gear": gear, "samples": samples,
                "reach": math.hypot(half_w, half_h)}

    def detach(self, state: dict) -> None:
        self.space.remove(state["pivot"], state["gear"], state["carrier"])

    def pre_step(self, body: pymunk.Body, state: dict, belts: Sequence[Belt]) -> Optional[str]:
        """Point the carrier at the belt motion under the parcel and set the
        joints' friction limits. Returns the id of the belt under the parcel
        center (None if on the floor), for display/debugging."""
        carrier: pymunk.Body = state["carrier"]
        carrier.position = body.position
        carrier.angle = body.angle

        center = body.position
        rot = pymunk.Vec2d(1.0, 0.0).rotated(body.angle)

        # Only belts whose bounding box can touch this parcel's footprint.
        reach = state["reach"]
        nearby = []
        for belt in belts:
            min_x, min_y, max_x, max_y = belt_bounds(belt)
            if (min_x - reach <= center.x <= max_x + reach
                    and min_y - reach <= center.y <= max_y + reach):
                nearby.append(belt)

        weights: List[float] = []
        offsets: List[Tuple[float, float]] = []
        velocities: List[Tuple[float, float]] = []
        center_belt_id = None
        for idx, (lx, ly) in enumerate(state["samples"]):
            ox = lx * rot.x - ly * rot.y
            oy = lx * rot.y + ly * rot.x
            px, py = center.x + ox, center.y + oy
            mu = self.floor_friction
            vel = (0.0, 0.0)
            for belt in nearby:
                if belt_contains(belt, px, py):
                    sv = surface_velocity(belt, px, py)
                    if sv is not None:
                        mu = belt.friction
                        vel = (sv[0], sv[1])
                        if idx == 0:
                            center_belt_id = belt.id
                    # Last belt in config order wins where belts overlap -
                    # it's the one drawn on top.
            weights.append(mu)
            offsets.append((ox, oy))
            velocities.append(vel)

        total_w = sum(weights)
        if total_w <= 0.0:
            state["pivot"].max_force = 0.0
            state["gear"].max_force = 0.0
            return center_belt_id

        vcx = sum(w * v[0] for w, v in zip(weights, velocities)) / total_w
        vcy = sum(w * v[1] for w, v in zip(weights, velocities)) / total_w
        num = 0.0
        den = 0.0
        for w, (ox, oy), (vx, vy) in zip(weights, offsets, velocities):
            num += w * (ox * (vy - vcy) - oy * (vx - vcx))
            den += w * (ox * ox + oy * oy)
        omega = num / den if den > 1e-12 else 0.0

        carrier.velocity = (vcx, vcy)
        carrier.angular_velocity = omega

        mean_mu = total_w / len(weights)
        normal = body.mass * GRAVITY
        state["pivot"].max_force = mean_mu * normal
        # Torque friction of a uniformly loaded footprint ~ mu*N*mean lever arm.
        mean_arm = sum(math.hypot(ox, oy) for ox, oy in offsets) / len(offsets)
        state["gear"].max_force = mean_mu * normal * mean_arm
        return center_belt_id
