"""
equipment_geometry.py - how a diverter is built, in its own frame, for every
view that draws it: the designer (layout_editor.DiverterItem), the
simulator window (logistics_draw) and the 3D view (scene3d -> view3d_web/
models.js buildDiverter, which follows the same numbers).

Local frame: metres, x along the belt's travel, y to the left of it, the
zone (`width` along x, `height` across) centred on the origin.

  pusher   a paddle pusher beside the belt: an actuator body (a rodless /
           guided cylinder) on the side it pushes away from, two guide rods
           and a push plate that sweeps across the zone (stroke 0 = home at
           the belt edge .. 1 = across)
  wheels   a pop-up wheel module in the belt: rows of small steerable
           wheels that lift and turn to the divert angle
"""
import math
from dataclasses import dataclass
from typing import List, Tuple

PLATE_T = 0.04          # push plate thickness
PLATE_FRACTION = 0.8    # of the zone length
REACH = 0.85            # stroke, of the zone width
BODY_W = 0.22           # actuator body, along the travel
BODY_GAP = 0.08         # belt edge to the body
BODY_EXTRA = 0.12       # body length beyond the stroke (end caps, valve)
ROD_FRACTION = 0.3      # guide rods at +-this * the plate length
WHEEL_PITCH_X = 0.15    # wheel rows along the travel
WHEEL_PITCH_Y = 0.12    # wheels across
WHEEL_R = 0.035
WHEEL_T = 0.022
WHEEL_INSET = 0.9       # wheels within this share of the zone


Rect = Tuple[float, float, float, float]          # centre x, centre y, size x, size y


@dataclass
class PusherParts:
    side: int            # +1: it stands on the left (pushes right), -1: on the right
    stroke_length: float
    body: Rect
    plate: Rect
    rods: List[Tuple[Tuple[float, float], Tuple[float, float]]]
    push_dir: Tuple[float, float]


def pusher_side(divert_angle: float) -> int:
    """The side the pusher stands on: away from where it pushes."""
    return 1 if math.sin(math.radians(divert_angle)) < 0 else -1


def pusher_parts(width: float, height: float, divert_angle: float, stroke: float = 0.0) -> PusherParts:
    side = pusher_side(divert_angle)
    edge = side * height / 2.0
    travel = height * REACH
    plate_len = width * PLATE_FRACTION
    plate_y = edge + side * PLATE_T / 2.0 - side * travel * max(0.0, min(1.0, stroke))
    body_len = travel + BODY_EXTRA
    body_y = edge + side * (BODY_GAP + body_len / 2.0)
    body_front = edge + side * BODY_GAP
    rods = [((k * plate_len, plate_y + side * PLATE_T / 2.0), (k * plate_len, body_front))
            for k in (-ROD_FRACTION, ROD_FRACTION)]
    a = math.radians(divert_angle)
    return PusherParts(side, travel, (0.0, body_y, BODY_W, body_len), (0.0, plate_y, plate_len, PLATE_T), rods,
                       (math.cos(a), math.sin(a)))


def wheel_centres(width: float, height: float) -> List[Tuple[float, float]]:
    """The wheels of a pop-up wheel module, row by row."""
    nx = max(2, round(width * WHEEL_INSET / WHEEL_PITCH_X))
    ny = max(2, round(height * WHEEL_INSET / WHEEL_PITCH_Y))
    w, h = width * WHEEL_INSET, height * WHEEL_INSET
    return [((i + 0.5) / nx * w - w / 2.0, (j + 0.5) / ny * h - h / 2.0) for i in range(nx) for j in range(ny)]


def wheel_steer(divert_angle: float) -> float:
    """The angle the wheels turn to when active (deg; a wheel steers at
    most 60 degrees off the travel)."""
    return max(-60.0, min(60.0, divert_angle))
