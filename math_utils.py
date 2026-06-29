import math
from typing import Tuple, List

def rotate_point(cx: float, cy: float, angle_deg: float, px: float, py: float) -> Tuple[float, float]:
    """Rotates a point around a center by an angle in degrees. Note: CAD standard assumes CCW."""
    s = math.sin(math.radians(angle_deg))
    c = math.cos(math.radians(angle_deg))
    
    px -= cx
    py -= cy
    
    xnew = px * c - py * s
    ynew = px * s + py * c
    
    px = xnew + cx
    py = ynew + cy
    return px, py

def get_rect_vertices(cx: float, cy: float, width: float, height: float, rotation: float) -> List[Tuple[float, float]]:
    """Returns the 4 vertices of a rotated rectangle. Center is cx, cy."""
    hw = width / 2.0
    hh = height / 2.0
    
    # Unrotated vertices relative to center
    v1 = (-hw, -hh)
    v2 = (hw, -hh)
    v3 = (hw, hh)
    v4 = (-hw, hh)
    
    # Rotate and translate
    vertices = []
    for vx, vy in [v1, v2, v3, v4]:
        rx, ry = rotate_point(0, 0, rotation, vx, vy)
        vertices.append((cx + rx, cy + ry))
    return vertices

def point_in_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
    """Ray casting algorithm for point in polygon."""
    num_vertices = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(1, num_vertices + 1):
        p2x, p2y = polygon[i % num_vertices]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def point_in_arc(x: float, y: float, cx: float, cy: float, radius: float, thickness: float, start_angle: float, end_angle: float) -> bool:
    """Checks if a point is within the thickness of an arc."""
    dx = x - cx
    dy = y - cy
    dist = math.sqrt(dx*dx + dy*dy)
    
    if dist < (radius - thickness/2.0) or dist > (radius + thickness/2.0):
        return False
        
    angle = math.degrees(math.atan2(dy, dx))
    if angle < 0:
        angle += 360.0
        
    # Check if angle is between start and end (handling wrap around)
    if start_angle <= end_angle:
        return start_angle <= angle <= end_angle
    else:
        # Wrap around 360
        return angle >= start_angle or angle <= end_angle
