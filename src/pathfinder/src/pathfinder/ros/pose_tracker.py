from __future__ import annotations
import threading
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from geometry_msgs.msg import Pose2D


class PoseTracker:
    """Latest pose per robot, with a blocking wait until the first one arrives."""

    def __init__(self, timeout_sec: float = 1.0) -> None:
        """Store timeout and initialise the internal pose dict and condition."""
        self._timeout_sec = timeout_sec
        self._poses: dict[str, Pose2D] = {}
        self._condition = threading.Condition()

    def update(self, robot_id: str, pose: Pose2D) -> None:
        """Record the latest pose for robot_id; notify any waiters on first arrival."""
        with self._condition:
            is_first = robot_id not in self._poses
            self._poses[robot_id] = pose
            if is_first:
                self._condition.notify_all()

    def wait_for(self, robot_id: str) -> Pose2D | None:
        """Block until a pose arrives for robot_id, or return None on timeout."""
        deadline = time.monotonic() + self._timeout_sec
        with self._condition:
            pose = self._poses.get(robot_id)
            while pose is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(timeout=remaining)
                pose = self._poses.get(robot_id)
            return pose
