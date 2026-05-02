from __future__ import annotations
from dataclasses import dataclass
import math
import threading
from typing import Optional
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from pathfinding_system.robot.robot_status import RobotStatus
from pathfinding_system.robot.robot_state import RobotState
from pathfinding_system.world.node import Node


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class MotionParameters:
    linear_gain: float = 0.5
    angular_gain: float = 1.5
    max_linear_velocity: float = 0.22
    max_angular_velocity: float = 1.5
    arrival_tolerance: float = 0.10
    heading_tolerance: float = 0.2


@dataclass(frozen=True)
class TurtleBotDriveResult:
    arrived: bool
    velocity_command: Twist


class TurtleBot:
    def __init__(
        self,
        robot_id: str,
        motion_parameters: MotionParameters = MotionParameters(),
    ) -> None:
        self.id = robot_id
        self._ns = robot_id
        self._motion_parameters = motion_parameters
        self._state = RobotState(id=robot_id)
        self._stop_requested = False
        self._lock = threading.Lock()

    @property
    def cmd_vel_topic(self) -> str:
        return f'/{self._ns}/cmd_vel'

    def drive_towards(self, node: Node) -> TurtleBotDriveResult:
        with self._lock:
            x = self._state.pose.x
            y = self._state.pose.y
            theta = self._state.pose.theta
            self._state.status = RobotStatus.MOVING

        dx = node.x - x
        dy = node.y - y
        distance = math.hypot(dx, dy)
        cmd = Twist()
        motion = self._motion_parameters

        if distance <= motion.arrival_tolerance:
            return TurtleBotDriveResult(arrived=True, velocity_command=cmd)

        desired_heading = math.atan2(dy, dx)
        heading_error = _wrap_to_pi(desired_heading - theta)
        cmd.angular.z = _clamp(
            motion.angular_gain * heading_error,
            motion.max_angular_velocity,
        )
        if abs(heading_error) <= motion.heading_tolerance:
            cmd.linear.x = min(
                motion.linear_gain * distance,
                motion.max_linear_velocity,
            )

        return TurtleBotDriveResult(arrived=False, velocity_command=cmd)

    def stop_motion(self) -> Twist:
        return Twist()

    def stop_requested(self) -> bool:
        with self._lock:
            return self._stop_requested

    def current_pose(self) -> Pose2D:
        with self._lock:
            pose = Pose2D()
            pose.x = self._state.pose.x
            pose.y = self._state.pose.y
            pose.theta = self._state.pose.theta
            return pose

    def set_status(self, status: RobotStatus) -> None:
        with self._lock:
            if status == RobotStatus.MOVING:
                self._stop_requested = False
            self._state.status = status

    def state_message(self):
        with self._lock:
            return self._state.to_msg()

    def on_odom(self, msg: Odometry) -> None:
        theta = _yaw_from_quaternion(msg.pose.pose.orientation)
        with self._lock:
            self._state.pose.x = msg.pose.pose.position.x
            self._state.pose.y = msg.pose.pose.position.y
            self._state.pose.theta = theta
            self._state.velocity = msg.twist.twist
            self._state.stamp = msg.header.stamp

    def on_emergency_stop(self, msg: Empty) -> None:
        with self._lock:
            self._stop_requested = True
            self._state.status = RobotStatus.STOPPED
        rospy.logwarn(f"{self.id}: emergency stop received.")
