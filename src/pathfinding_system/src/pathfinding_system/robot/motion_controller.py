from __future__ import annotations
from dataclasses import dataclass
import math

from pathfinding_system.world.node import Node


@dataclass(frozen=True)
class MotionParameters:
    linear_gain: float = 0.5
    angular_gain: float = 1.5
    max_linear_velocity: float = 0.22
    max_angular_velocity: float = 1.5
    arrival_tolerance: float = 0.10
    heading_tolerance: float = 0.2


@dataclass(frozen=True)
class DriveResult:
    arrived: bool
    linear_x: float
    angular_z: float


@dataclass(frozen=True)
class PathStep:
    drive_result: DriveResult
    current_index: int
    completed: bool


class MotionController:
    """Proportional controller that converts a pose and target node into velocity commands."""

    def __init__(self, params: MotionParameters = MotionParameters()) -> None:
        self._params = params

    def drive_towards(self, pose, target: Node) -> DriveResult:
        """Compute velocity commands to steer pose toward target, returning arrived=True when within tolerance."""
        dx = target.x - pose.x
        dy = target.y - pose.y
        distance = math.hypot(dx, dy)

        if distance <= self._params.arrival_tolerance:
            return DriveResult(arrived=True, linear_x=0.0, angular_z=0.0)

        desired_heading = math.atan2(dy, dx)
        heading_error = self._wrap_to_pi(desired_heading - pose.theta)
        angular_z = self._clamp(
            self._params.angular_gain * heading_error,
            self._params.max_angular_velocity,
        )
        linear_x = 0.0
        if abs(heading_error) <= self._params.heading_tolerance:
            linear_x = min(
                self._params.linear_gain * distance,
                self._params.max_linear_velocity,
            )

        return DriveResult(arrived=False, linear_x=linear_x, angular_z=angular_z)

    def _wrap_to_pi(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))
