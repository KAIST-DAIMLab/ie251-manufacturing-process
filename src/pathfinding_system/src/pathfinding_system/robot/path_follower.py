from __future__ import annotations
import math
import threading

from pathfinding_system.robot.motion_controller import MotionController
from pathfinding_system.world.node import Node

_ANGLE_EPSILON = 1e-3
_DISTANCE_EPSILON = 1e-3


class PathFollower:
    """Steps through an ordered list of waypoints using turn-then-move primitives."""

    def __init__(self, motion_controller: MotionController) -> None:
        self._motion_controller = motion_controller
        self._current_index = 0
        self._lock = threading.Lock()

    @property
    def current_index(self) -> int:
        """Index of the waypoint currently being driven toward."""
        with self._lock:
            return self._current_index

    def follow(self, waypoints: list[Node]) -> bool:
        """Drive through all waypoints in order; True on completion, False if interrupted."""
        with self._lock:
            self._current_index = 0

        for index, waypoint in enumerate(waypoints):
            with self._lock:
                self._current_index = index

            pose = self._motion_controller._pose_provider()
            dx = waypoint.x - pose.x
            dy = waypoint.y - pose.y
            distance = math.hypot(dx, dy)
            angle = self._wrap_to_pi(math.atan2(dy, dx) - pose.theta)

            if abs(angle) >= _ANGLE_EPSILON:
                if angle > 0:
                    if not self._motion_controller.turnLeft(angle):
                        return False
                else:
                    if not self._motion_controller.turnRight(-angle):
                        return False

            if distance >= _DISTANCE_EPSILON:
                if not self._motion_controller.MoveTowards(distance):
                    return False

        return True

    def cancel(self) -> None:
        """Interrupt the current follow by stopping the motion controller."""
        self._motion_controller.stop()

    def _wrap_to_pi(self, radian: float) -> float:
        return math.atan2(math.sin(radian), math.cos(radian))
