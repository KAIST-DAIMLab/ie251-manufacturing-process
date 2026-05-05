from __future__ import annotations
import math

import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

from pathfinding_system.robot.motion_controller import CmdVelPublisher, MotionController, MotionParameters
from pathfinding_system.robot.path_follower import PathFollower
from pathfinding_system.robot.robot_state import RobotState
from pathfinding_system.world.node import Node


class TurtleBot:
    """Robot facade: owns state, motion controller, and path follower."""

    def __init__(
        self,
        robot_id: str,
        cmd_vel_publisher: CmdVelPublisher,
        params: MotionParameters = MotionParameters(),
        motion_rate_hz: float = 5.0,
    ) -> None:
        self.id = robot_id
        self._state = RobotState(id=robot_id)
        self._rate_hz = motion_rate_hz
        self._cancel = False
        self._motion_controller = MotionController(cmd_vel_publisher, self.current_pose, params)
        self._path_follower = PathFollower(self._motion_controller, rate_hz=motion_rate_hz)

    @property
    def motion_controller(self) -> MotionController:
        """The proportional controller for this robot's velocity commands."""
        return self._motion_controller

    @property
    def path_follower(self) -> PathFollower:
        """The path follower that sequences waypoint traversal."""
        return self._path_follower

    def current_pose(self) -> Pose2D:
        """Return a snapshot of the current pose."""
        pose = Pose2D()
        pose.x = self._state.pose.x
        pose.y = self._state.pose.y
        pose.theta = self._state.pose.theta
        return pose

    def update_pose(self, msg: Odometry) -> None:
        """Update pose and velocity from an Odometry message."""
        self._state = RobotState.from_odometry(self.id, msg)

    def follow_path(self, nodes: list[Node]) -> bool:
        """Follow an ordered list of graph nodes; True when all reached, False if cancelled."""
        return self._path_follower.follow(nodes)

    def turn_left(self, radian: float) -> bool:
        """Turn left by radian; True when heading reached, False if cancelled or shutdown."""
        return self._turn_to(self.current_pose().theta + radian)

    def turn_right(self, radian: float) -> bool:
        """Turn right by radian; True when heading reached, False if cancelled or shutdown."""
        return self._turn_to(self.current_pose().theta - radian)

    def move_forward(self, meter: float) -> bool:
        """Drive forward by meter along current heading; True when reached, False if cancelled."""
        pose = self.current_pose()
        target = Node(id=0, x=pose.x + meter * math.cos(pose.theta), y=pose.y + meter * math.sin(pose.theta))
        return self._move_to(target)

    def move_backward(self, meter: float) -> bool:
        """Drive backward by meter along current heading; True when reached, False if cancelled."""
        pose = self.current_pose()
        target = Node(id=0, x=pose.x - meter * math.cos(pose.theta), y=pose.y - meter * math.sin(pose.theta))
        return self._move_to(target)

    def stop(self) -> None:
        """Cancel current movement and halt immediately."""
        self._cancel = True
        self._path_follower.cancel()
        self._motion_controller.stop()

    def _move_to(self, target: Node) -> bool:
        self._cancel = False
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown() and not self._cancel:
            if self._motion_controller.drive_towards(target):
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
