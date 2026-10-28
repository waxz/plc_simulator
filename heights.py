"""
heights.py - the third dimension of the (2D) simulation: floor levels,
conveyor height and part heights. Shared by the machines that work in 3D
(robot6.py via machines.py) and the 3D view (scene3d.py), so a robot grips
a parcel exactly where the view draws its top.

  floor of level L     L * level_height (the elevators' levelHeight; 1.0 m
                       when the layout has none)
  belt top             floor + BELT_TOP
  a parcel             PARCEL_H[0] .. PARCEL_H[1] tall, picked per parcel id
  a pallet             PALLET_H tall (a EUR pallet's 144 mm)
"""
import zlib
from typing import Optional

BELT_TOP = 0.75            # m above its floor: a conveyor's working height
PALLET_H = 0.144           # EUR pallet: 22 + 78 + 22 + 22 mm
PARCEL_H = (0.18, 0.42)


def level_height(config) -> float:
    for elevator in getattr(config, "elevators", []):
        if elevator.levelHeight > 0:
            return float(elevator.levelHeight)
    return 1.0


def floor_z(config, level: int, lh: Optional[float] = None) -> float:
    return (lh if lh is not None else level_height(config)) * level


def surface_z(config, level: int, lh: Optional[float] = None) -> float:
    """Top of the belts on floor `level`."""
    return floor_z(config, level, lh) + BELT_TOP


def parcel_height(part_id: str) -> float:
    lo, hi = PARCEL_H
    return lo + (zlib.crc32(part_id.encode("utf-8")) % 1000) / 999.0 * (hi - lo)


def part_height(part) -> float:
    """Height of a physics part (physics.Parcel or a ParcelView)."""
    return PALLET_H if getattr(part, "kind", "parcel") == "pallet" else parcel_height(part.id)
