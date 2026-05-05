from __future__ import annotations
import copy
import threading
from geometry_msgs.msg import Pose2D

from pathfinding_system.robot.robot_mode import RobotMode
from pathfinding_system.robot.robot_state import RobotState


class TurtleBot:
    """Thread-safe state container for a single TurtleBot robot."""

    def __init__(
        self,
        robot_id: str,
        topic_namespace: str | None = None,
        cmd_vel_topic: str | None = None,
        odom_topic: str | None = None,
    ) -> None:
        self.id = robot_id
        self._topic_ns = (topic_namespace or robot_id).strip('/')
        self._cmd_vel_topic = cmd_vel_topic
        self._odom_topic = odom_topic
        self._state = RobotState(id=robot_id)
        self._stop_requested = False
        self._lock = threading.Lock()

    @property
    def cmd_vel_topic(self) -> str:
        """Topic name for publishing velocity commands."""
        if self._cmd_vel_topic is not None:
            return self._cmd_vel_topic
        return f'/{self._topic_ns}/cmd_vel'

    @property
    def odom_topic(self) -> str:
        """Topic name for subscribing to odometry."""
        if self._odom_topic is not None:
            return self._odom_topic
        return f'/{self._topic_ns}/odom'

    @property
    def state_topic(self) -> str:
        """Topic name for publishing robot state."""
        return f'/{self.id}/robot_state'

    def current_pose(self) -> Pose2D:
        """Return a snapshot of the current pose."""
        with self._lock:
            pose = Pose2D()
            pose.x = self._state.pose.x
            pose.y = self._state.pose.y
            pose.theta = self._state.pose.theta
            return pose

    def state_snapshot(self) -> RobotState:
        """Return a deep copy of the current robot state."""
        with self._lock:
            return copy.deepcopy(self._state)

    def update_pose(self, x: float, y: float, theta: float, velocity=None, stamp=None) -> None:
        """Update pose and optionally velocity and timestamp from an odometry message."""
        with self._lock:
            self._state.pose.x = x
            self._state.pose.y = y
            self._state.pose.theta = theta
            if velocity is not None:
                self._state.velocity = velocity
            self._state.stamp = stamp

    def mark_moving(self) -> None:
        """Set robot status to MOVING."""
        with self._lock:
            self._state.status = RobotMode.MOVING

    def mark_idle(self) -> None:
        """Set robot status to IDLE."""
        with self._lock:
            self._state.status = RobotMode.IDLE

    def mark_reached(self) -> None:
        """Set robot status to REACHED."""
        with self._lock:
            self._state.status = RobotMode.REACHED

    def request_stop(self) -> None:
        """Latch a stop request and set status to STOPPED."""
        with self._lock:
            self._stop_requested = True
            self._state.status = RobotMode.STOPPED

    def clear_stop(self) -> None:
        """Clear the latched stop request."""
        with self._lock:
            self._stop_requested = False

    def stop_requested(self) -> bool:
        """Return True if a stop has been latched."""
        with self._lock:
            return self._stop_requested
