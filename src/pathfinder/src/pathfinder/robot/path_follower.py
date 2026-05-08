from __future__ import annotations

from typing import Callable

import rospy

from pathfinder.robot.motion_controller import MotionController
from pathfinder.world.node import Node


class PathFollower:
    """Drives a robot through an ordered list of waypoints by ticking MotionController each cycle."""

    def __init__(self, motion_controller: MotionController, rate_hz: float = 5.0) -> None:
        self._motion_controller = motion_controller
        self._rate_hz = rate_hz
        self._cancel = False
        self._current_index = 0
        self._pause_check: Callable[[], bool] | None = None

    @property
    def current_index(self) -> int:
        """Index of the waypoint currently being driven toward."""
        return self._current_index

    def follow(self, waypoints: list[Node]) -> bool:
        """Drive through waypoints in order; True when all reached, False if cancelled or shutdown."""
        self._cancel = False
        rate = rospy.Rate(self._rate_hz)
        for index, waypoint in enumerate(waypoints):
            self._current_index = index
            while True:
                if rospy.is_shutdown() or self._cancel:
                    return False
                if self._pause_check is not None and self._pause_check():
                    self._motion_controller.stop()
                elif self._motion_controller.drive_towards(waypoint):
                    break
                rate.sleep()
        return True

    def cancel(self) -> None:
        """Interrupt the active follow() on the next control tick."""
        self._cancel = True

    def set_pause_check(self, pause_check: Callable[[], bool] | None) -> None:
        """Set the callable consulted before each tick; when it returns True, motion pauses."""
        self._pause_check = pause_check
