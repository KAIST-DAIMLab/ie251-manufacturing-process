from __future__ import annotations
import math
from typing import Any


def wrap_to_pi(radian: float) -> float:
    """Normalise an angle in radians to the range (-pi, pi]."""
    return math.atan2(math.sin(radian), math.cos(radian))


def planar_distance(a: Any, b: Any) -> float:
    """Euclidean distance between any two objects exposing .x and .y."""
    return math.hypot(a.x - b.x, a.y - b.y)


def heading_offset(pose: Any, target: Any) -> float:
    """Signed angle (rad) from pose.theta to the bearing pose -> target, wrapped to (-pi, pi]."""
    return wrap_to_pi(math.atan2(target.y - pose.y, target.x - pose.x) - pose.theta)


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_from_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def segment_pose(
    from_xy: tuple[float, float],
    to_xy: tuple[float, float],
) -> tuple[tuple[float, float], float, float]:
    """Return (midpoint, length, yaw) for a planar segment between two points."""
    from_x, from_y = from_xy
    to_x, to_y = to_xy
    delta_x = to_x - from_x
    delta_y = to_y - from_y
    midpoint = ((from_x + to_x) / 2.0, (from_y + to_y) / 2.0)
    length = math.hypot(delta_x, delta_y)
    yaw = math.atan2(delta_y, delta_x)
    return midpoint, length, yaw
