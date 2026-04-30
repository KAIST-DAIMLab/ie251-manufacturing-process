from __future__ import annotations
from dataclasses import dataclass
from pathfinding_system.robot.robot_status import RobotStatus


@dataclass
class RobotState:
    id: str
    pose: object        # geometry_msgs/Pose2D
    velocity: object    # geometry_msgs/Twist
    status: RobotStatus
    stamp: object       # rospy.Time

    def is_moving(self) -> bool:
        return self.status == RobotStatus.MOVING

    def to_msg(self):
        from pathfinding_system.msg import RobotState as RobotStateMsg  # type: ignore[import]
        import rospy
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
