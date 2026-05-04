from __future__ import annotations

from pathfinding_system.robot.motion_controller import DriveResult, MotionController, PathStep
from pathfinding_system.world.node import Node


class PathFollower:
    """Steps through an ordered list of waypoints, delegating per-tick drive commands to a MotionController."""

    def __init__(self, motion_controller: MotionController) -> None:
        self._motion_controller = motion_controller
        self._waypoints: list[Node] = []
        self._current_index = 0

    def start(self, waypoints: list[Node]) -> None:
        """Begin following a new sequence of waypoints, resetting progress."""
        self._waypoints = list(waypoints)
        self._current_index = 0

    def cancel(self) -> None:
        """Abort the active path and stop issuing drive commands."""
        self._waypoints = []
        self._current_index = 0

    def step(self, pose) -> PathStep:
        """Advance one control tick, returning drive commands and whether the full path is completed."""
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
