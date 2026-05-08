from __future__ import annotations
import math

import rospy
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
        motion_rate_hz: float = 5.0,
        obstacle_detector_enabled: bool = False,
    ) -> None:
        self.id = robot_id
        self._state = state
        self._motion_controller = motion_controller
        self._path_follower = path_follower
        self._rate_hz = motion_rate_hz
        self._cancel = False
        self._obstacle_blocked = False
        if obstacle_detector_enabled:
            self._path_follower.set_pause_check(lambda: self._obstacle_blocked)

    @property
    def motion_controller(self) -> MotionController:
        """The proportional controller for this robot's velocity commands."""
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
        return self._turn_to(self.get_pose().theta + radian)

    def turn_right(self, radian: float) -> bool:
        """Turn right by radian; True when heading reached, False if cancelled or shutdown."""
        return self._turn_to(self.get_pose().theta - radian)

    def move_forward(self, meter: float) -> bool:
        """Drive forward by meter along current heading; True when reached, False if cancelled."""
        pose = self.get_pose()
        target = Node(id=0, x=pose.x + meter * math.cos(pose.theta), y=pose.y + meter * math.sin(pose.theta))
        return self._move_to(target)

    def move_backward(self, meter: float) -> bool:
        """Drive backward by meter along current heading; True when reached, False if cancelled."""
        pose = self.get_pose()
        target = Node(id=0, x=pose.x - meter * math.cos(pose.theta), y=pose.y - meter * math.sin(pose.theta))
        return self._move_to(target)

    def set_obstacle_blocked(self, blocked: bool) -> None:
        """Update the obstacle-blocked flag from the scan subscriber callback."""
        self._obstacle_blocked = blocked

    def stop(self) -> None:
        """Cancel current movement and halt immediately."""
        self._cancel = True
        self._path_follower.cancel()
        self._motion_controller.stop()

    def _move_to(self, target: Node) -> bool:
        self._cancel = False
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown() and not self._cancel:
            if self._obstacle_blocked:
                self._motion_controller.stop()
            elif self._motion_controller.drive_towards(target):
                return True
            rate.sleep()
        return False

    def _turn_to(self, heading: float) -> bool:
        self._cancel = False
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown() and not self._cancel:
            if self._motion_controller.turn_towards(heading):
                return True
            rate.sleep()
        return False
