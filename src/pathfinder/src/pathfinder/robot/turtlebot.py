from __future__ import annotations
import math

from geometry_msgs.msg import Pose2D

from pathfinder.robot.motion_controller import MotionController
from pathfinder.robot.path_follower import PathFollower
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

    def follow_path(self, nodes: list[Node]) -> bool:
        """Follow an ordered list of graph nodes; True when all reached, False if cancelled."""
        return self._path_follower.follow(nodes)

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
