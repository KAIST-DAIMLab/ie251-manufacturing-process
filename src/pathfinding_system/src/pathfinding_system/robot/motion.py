from __future__ import annotations
from dataclasses import dataclass
import math

from pathfinding_system.world.node import Node


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


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


def compute_drive(pose, target: Node, params: MotionParameters) -> DriveResult:
    dx = target.x - pose.x
    dy = target.y - pose.y
    distance = math.hypot(dx, dy)

    if distance <= params.arrival_tolerance:
        return DriveResult(arrived=True, linear_x=0.0, angular_z=0.0)

    desired_heading = math.atan2(dy, dx)
    heading_error = _wrap_to_pi(desired_heading - pose.theta)
    angular_z = _clamp(
        params.angular_gain * heading_error,
        params.max_angular_velocity,
    )
    linear_x = 0.0
    if abs(heading_error) <= params.heading_tolerance:
        linear_x = min(
            params.linear_gain * distance,
            params.max_linear_velocity,
        )

    return DriveResult(arrived=False, linear_x=linear_x, angular_z=angular_z)


class MotionController:
    def __init__(self, params: MotionParameters = MotionParameters()) -> None:
        self._params = params

    def drive_towards(self, pose, target: Node) -> DriveResult:
        return compute_drive(pose, target, self._params)


class PathFollower:
    def __init__(self, motion_controller: MotionController) -> None:
        self._motion_controller = motion_controller
        self._waypoints: list[Node] = []
        self._current_index = 0

    def start(self, waypoints: list[Node]) -> None:
        self._waypoints = list(waypoints)
        self._current_index = 0

    def cancel(self) -> None:
        self._waypoints = []
        self._current_index = 0

    def step(self, pose) -> PathStep:
        if self._current_index >= len(self._waypoints):
            return PathStep(
                drive_result=DriveResult(arrived=True, linear_x=0.0, angular_z=0.0),
                current_index=0,
                completed=True,
            )

        current_index = self._current_index
        drive_result = self._motion_controller.drive_towards(
            pose,
            self._waypoints[current_index],
        )
        if drive_result.arrived:
            self._current_index += 1

        return PathStep(
            drive_result=drive_result,
            current_index=current_index,
            completed=self._current_index >= len(self._waypoints),
        )
