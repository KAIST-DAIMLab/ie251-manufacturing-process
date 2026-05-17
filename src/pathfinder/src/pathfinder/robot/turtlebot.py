from __future__ import annotations
import math
import threading

from geometry_msgs.msg import Pose2D

from pathfinder.robot.motion_controller import MotionController
from pathfinder.robot.path_follower import PathFollower
from pathfinder.robot.robot_mode import RobotMode
from pathfinder.robot.robot_state import RobotState
from pathfinder.world.node import Node


class TurtleBot:
    """Robot facade: behavior coordinator wired with injected state and motion collaborators."""

    def __init__(
        self,
        robot_id: str,
        state: RobotState,
        motion_controller: MotionController,
        path_follower: PathFollower,
    ) -> None:
        self.id = robot_id
        self._state = state
        self._motion_controller = motion_controller
        self._path_follower = path_follower
        self._is_online = False
        self._is_following = False
        self._is_obstacle = False
        self._status_lock = threading.Lock()
        self._recompute_status()

    @property
    def motion_controller(self) -> MotionController:
        """The loop-owning motion controller for this robot."""
        return self._motion_controller

    @property
    def path_follower(self) -> PathFollower:
        """The path follower that sequences waypoint traversal."""
        return self._path_follower

    def get_pose(self) -> Pose2D:
        """Return a snapshot of the current pose."""
        return self._state.get_pose()

    def set_following(self, on: bool) -> None:
        """Mark whether a follow_path lifecycle is currently active."""
        with self._status_lock:
            self._is_following = on
            self._recompute_status()

    def set_obstacle(self, on: bool) -> None:
        """Mark whether an obstacle currently blocks forward motion."""
        with self._status_lock:
            self._is_obstacle = on
            self._recompute_status()

    def set_online(self, on: bool) -> None:
        """Mark whether the robot's /odom feed is fresh (robot reachable)."""
        with self._status_lock:
            self._is_online = on
            self._recompute_status()

    def _recompute_status(self) -> None:
        if not self._is_online:
            self._state.status = RobotMode.OFFLINE
        elif not self._is_following:
            self._state.status = RobotMode.IDLE
        elif self._is_obstacle:
            self._state.status = RobotMode.OBSTACLE
        else:
            self._state.status = RobotMode.MOVING

    def follow_path(self, nodes: list[Node]) -> bool:
        """Follow an ordered list of graph nodes; True when all reached, False if cancelled."""
        self.set_following(True)
        try:
            return self._path_follower.follow(nodes)
        finally:
            self.set_following(False)

    def turn_left(self, radian: float) -> bool:
        """Turn left by radian; True when heading reached, False if cancelled or shutdown."""
        return self._motion_controller.turn_to(self.get_pose().theta + radian)

    def turn_right(self, radian: float) -> bool:
        """Turn right by radian; True when heading reached, False if cancelled or shutdown."""
        return self._motion_controller.turn_to(self.get_pose().theta - radian)

    def turn_to(self, heading: float) -> bool:
        """Turn to an absolute world-frame heading; True when heading reached."""
        return self._motion_controller.turn_to(heading)

    def move_forward(self, meter: float) -> bool:
        """Drive forward by meter along current heading; True when reached, False if cancelled."""
        pose = self.get_pose()
        target = Node(id=0, x=pose.x + meter * math.cos(pose.theta), y=pose.y + meter * math.sin(pose.theta))
        return self._motion_controller.drive_to(target)

    def move_backward(self, meter: float) -> bool:
        """Drive backward by meter along current heading; True when reached, False if cancelled."""
        pose = self.get_pose()
        target = Node(id=0, x=pose.x - meter * math.cos(pose.theta), y=pose.y - meter * math.sin(pose.theta))
        return self._motion_controller.drive_to(target)

    def stop(self) -> None:
        """Cancel current movement and halt immediately."""
        self._motion_controller.stop()
        self._path_follower.cancel()
