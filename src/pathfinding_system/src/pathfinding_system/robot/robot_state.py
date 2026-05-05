from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any
from geometry_msgs.msg import Pose2D, Twist
from pathfinding_system.robot.robot_mode import RobotMode
from pathfinding_system.utils.physics import yaw_from_quaternion


@dataclass
class RobotState:
    id: str
    pose: Pose2D = field(default_factory=Pose2D)
    velocity: Twist = field(default_factory=Twist)
    status: RobotMode = RobotMode.IDLE

    def is_moving(self) -> bool:
        return self.status == RobotMode.MOVING

    @classmethod
    def from_odometry(cls, robot_id: str, msg: Any) -> RobotState:
        state = cls(id=robot_id)
        pose = msg.pose.pose
        state.pose.x = pose.position.x
        state.pose.y = pose.position.y
        state.pose.theta = yaw_from_quaternion(pose.orientation)
        state.velocity = msg.twist.twist
        return state
