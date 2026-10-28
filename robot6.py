"""
robot6.py - kinematics of a 6-axis industrial robot arm (FANUC M-10iA /
M-20iA style: a vertical articulated arm with a spherical wrist), scaled to
the robot's reach.

Geometry (ratios of the reach R, close to an M-20iA's), base frame at the
foot of the pedestal, z up:

  J1  base swivel about z                 at the pedestal top (mountHeight)
  J2  shoulder, about the horizontal      SHOULDER_UP R above J1, SHOULDER_OUT R out
  J3  elbow                               UPPER_ARM R along the upper arm
  J4  forearm roll                        FOREARM R along the forearm, ELBOW_OFFSET R above its axis
  J5  wrist bend
  J6  flange roll                         FLANGE R past the wrist centre, then the tool

Joint conventions (degrees, FANUC style):
  J1  heading of the arm (0 = world +x, CCW positive)
  J2  upper arm tilt from vertical (+ = forward)
  J3  forearm angle from horizontal, absolute (J2/J3 interaction; + = down)
  J4  forearm roll; J5 wrist bend (+ = down); J6 flange roll

Inverse kinematics: closed form for a tool pointing straight down with a
free yaw (top-down pick and place - the case conveyor robots work in):
elbow up, no wrist flip, J4 = 0. forward() is the general chain for any
joint values; the 3D view (view3d_web/viewer.js) builds the same chain.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

SHOULDER_UP = 0.29
SHOULDER_OUT = 0.083
UPPER_ARM = 0.436
ELBOW_OFFSET = 0.083
FOREARM = 0.461
FLANGE = 0.055

# Joint limits (deg) and maximum speeds (deg/s), M-20iA-like.
LIMITS = ((-170.0, 170.0), (-100.0, 160.0), (-70.0, 200.0), (-190.0, 190.0), (-140.0, 140.0), (-360.0, 360.0))
MAX_SPEED = (195.0, 175.0, 180.0, 360.0, 360.0, 550.0)

Joints = Tuple[float, float, float, float, float, float]
HOME: Joints = (0.0, 0.0, 0.0, 0.0, 90.0, 0.0)          # upright, forearm level, tool down


class Unreachable(ValueError):
    """The target is out of reach (or would need a joint past its limit)."""


@dataclass(frozen=True)
class Geometry:
    reach: float              # m: J1 axis to the wrist centre, arm stretched out
    mount: float = 0.3        # m: pedestal height (J1's foot to the base of the arm)
    tool: float = 0.18        # m: flange to the tool centre point (the gripper's pads)

    @property
    def d1(self) -> float:
        return self.mount + SHOULDER_UP * self.reach

    @property
    def a1(self) -> float:
        return SHOULDER_OUT * self.reach

    @property
    def a2(self) -> float:
        return UPPER_ARM * self.reach

    @property
    def a3(self) -> float:
        return ELBOW_OFFSET * self.reach

    @property
    def d4(self) -> float:
        return FOREARM * self.reach

    @property
    def d6(self) -> float:
        return FLANGE * self.reach

    def as_dict(self) -> dict:
        """The link lengths, for the 3D view's model."""
        return {"d1": self.d1, "a1": self.a1, "a2": self.a2, "a3": self.a3, "d4": self.d4, "d6": self.d6,
                "tool": self.tool, "mount": self.mount, "reach": self.reach}


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def within_limits(q: Sequence[float]) -> bool:
    return all(lo - 1e-6 <= v <= hi + 1e-6 for v, (lo, hi) in zip(q, LIMITS))


def inverse(geo: Geometry, x: float, y: float, z: float, yaw: float = 0.0) -> Joints:
    """Joint angles (deg) putting the tool centre point at (x, y, z) - in
    the robot's base frame - pointing straight down, the gripper's jaws
    across `yaw` (deg). Raises Unreachable."""
    wz = z + geo.d6 + geo.tool                       # wrist centre straight above the TCP
    q1 = math.atan2(y, x)
    r = math.hypot(x, y) - geo.a1
    s = wz - geo.d1
    l3 = math.hypot(geo.a3, geo.d4)
    phi = math.atan2(geo.a3, geo.d4)
    d = (r * r + s * s - geo.a2 ** 2 - l3 ** 2) / (2.0 * geo.a2 * l3)
    if abs(d) > 1.0:
        raise Unreachable(f"({x:.3f}, {y:.3f}, {z:.3f}) is out of reach")
    t3 = math.atan2(-math.sqrt(max(0.0, 1.0 - d * d)), d)        # elbow up
    alpha = math.atan2(s, r) - math.atan2(l3 * math.sin(t3), geo.a2 + l3 * math.cos(t3))
    gamma = alpha + t3                                            # J3 -> wrist centre direction
    theta_f = gamma - phi                                         # forearm axis elevation
    j1 = math.degrees(q1)
    j2 = 90.0 - math.degrees(alpha)
    j3 = -math.degrees(theta_f)
    j5 = 90.0 - j3                      # the tool's elevation is -(J3 + J5): straight down
    j6 = _wrap(j1 - yaw)                # the jaws close across `yaw`
    q = (j1, j2, j3, 0.0, j5, j6)
    if not within_limits(q):
        raise Unreachable(f"({x:.3f}, {y:.3f}, {z:.3f}) needs a joint past its limit: "
                          + ", ".join(f"J{i + 1} {v:.0f}" for i, v in enumerate(q)))
    return q


def _rot(axis: str, deg: float):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == "x":
        return ((1, 0, 0), (0, c, -s), (0, s, c))
    if axis == "y":
        return ((c, 0, s), (0, 1, 0), (-s, 0, c))
    return ((c, -s, 0), (s, c, 0), (0, 0, 1))


def _mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _apply(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def forward(geo: Geometry, q: Sequence[float]) -> dict:
    """The chain for joint angles q (deg): positions (base frame) of the
    shoulder (J2), elbow (J3), wrist centre, flange and TCP, and the tool's
    axis and jaw directions. The same chain the 3D view builds:
      J1 about z, then (a1, 0, d1); J2 about y (upper arm along z);
      (0, 0, a2); J3 about y by (J3 - J2) (forearm along x);
      (d4, 0, a3) to the wrist with J4 about x; J5 about y; (d6) and the
      tool along x with J6 about x."""
    j1, j2, j3, j4, j5, j6 = q
    r = _rot("z", j1)
    shoulder = _apply(r, (geo.a1, 0.0, geo.d1))
    r = _mul(r, _rot("y", j2))
    elbow = _add(shoulder, _apply(r, (0.0, 0.0, geo.a2)))
    r = _mul(r, _rot("y", j3 - j2))
    wrist = _add(elbow, _apply(r, (geo.d4, 0.0, geo.a3)))
    r = _mul(_mul(r, _rot("x", j4)), _rot("y", j5))
    flange = _add(wrist, _apply(r, (geo.d6, 0.0, 0.0)))
    r = _mul(r, _rot("x", j6))
    tcp = _add(flange, _apply(r, (geo.tool, 0.0, 0.0)))
    return {"shoulder": shoulder, "elbow": elbow, "wrist": wrist, "flange": flange, "tcp": tcp,
            "axis": _apply(r, (1.0, 0.0, 0.0)), "jaw": _apply(r, (0.0, 1.0, 0.0))}


def interpolate(a: Sequence[float], b: Sequence[float], t: float) -> Joints:
    return tuple(x + (y - x) * t for x, y in zip(a, b))


def joint_move_time(a: Sequence[float], b: Sequence[float]) -> float:
    """Shortest time (s) for a synchronized joint move at full speed."""
    return max(abs(y - x) / v for x, y, v in zip(a, b, MAX_SPEED))
