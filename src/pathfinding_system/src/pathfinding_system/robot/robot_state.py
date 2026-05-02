from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional
import rospy
from geometry_msgs.msg import Pose2D, Twist
from pathfinding_system.msg import RobotState as RobotStateMsg  # type: ignore[import]
from pathfinding_system.robot.robot_status import RobotStatus


@dataclass
class RobotState:
    id: str
    pose: Pose2D = field(default_factory=Pose2D)
    velocity: Twist = field(default_factory=Twist)
    status: RobotStatus = RobotStatus.IDLE
    stamp: Optional[rospy.Time] = None

    def is_moving(self) -> bool:
        return self.status == RobotStatus.MOVING

    def to_msg(self):
        msg = RobotStateMsg()
        msg.robot_id = self.id
        msg.pose = self.pose
        msg.velocity = self.velocity
        msg.status = int(self.status)
        msg.stamp = self.stamp if self.stamp is not None else rospy.Time.now()
        return msg

    @classmethod
    def from_msg(cls, msg) -> RobotState:
        return cls(
            id=msg.robot_id,
            pose=msg.pose,
            velocity=msg.velocity,
            status=RobotStatus(msg.status),
            stamp=msg.stamp,
        )
