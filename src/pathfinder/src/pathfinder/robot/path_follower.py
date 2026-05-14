from __future__ import annotations
import math

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
            drive = (
                self._motion_controller.drive_through
                if _is_straight_intermediate(waypoints, index)
                else self._motion_controller.drive_to
            )
            if not drive(waypoint):
                return False
        if waypoints:
            final_orientation = waypoints[-1].orientation_rad()
            if final_orientation is not None:
                return self._motion_controller.turn_to(final_orientation)
        return True

    def cancel(self) -> None:
        """Interrupt an active follow() by stopping the motion controller."""
        self._motion_controller.stop()


def _is_straight_intermediate(waypoints: list[Node], index: int) -> bool:
    if index <= 0 or index >= len(waypoints) - 1:
        return False

    previous = waypoints[index - 1]
    current = waypoints[index]
    following = waypoints[index + 1]
    incoming_x = current.x - previous.x
    incoming_y = current.y - previous.y
    outgoing_x = following.x - current.x
    outgoing_y = following.y - current.y
    cross = incoming_x * outgoing_y - incoming_y * outgoing_x
    dot = incoming_x * outgoing_x + incoming_y * outgoing_y

    return math.isclose(cross, 0.0, abs_tol=1e-9) and dot > 0.0
