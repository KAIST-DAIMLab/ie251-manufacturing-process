from __future__ import annotations

from pathfinder.robot.motion_controller import MotionController
from pathfinder.world.node import Node


class PathFollower:
    """Sequences waypoints by delegating each drive to MotionController."""

    def __init__(self, motion_controller: MotionController) -> None:
        self._motion_controller = motion_controller
        self._current_index = 0

    @property
    def current_index(self) -> int:
        """Index of the waypoint currently being driven toward."""
        return self._current_index

    def follow(self, waypoints: list[Node]) -> bool:
        """Drive through waypoints in order; True when all reached, False if any drive_to fails."""
        for index, waypoint in enumerate(waypoints):
            self._current_index = index
            if not self._motion_controller.drive_to(waypoint):
                return False
        return True

    def cancel(self) -> None:
        """Interrupt an active follow() by stopping the motion controller."""
        self._motion_controller.stop()
