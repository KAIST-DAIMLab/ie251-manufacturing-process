from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any
from geometry_msgs.msg import Pose2D, Twist
from pathfinder.robot.robot_mode import RobotMode
from pathfinder.utils.physics import yaw_from_quaternion


@dataclass
class RobotState:
    """Mutable pose/velocity state for one robot, shared between TurtleBot and MotionController."""
    id: str
    pose: Pose2D = field(default_factory=Pose2D)
    velocity: Twist = field(default_factory=Twist)
    status: RobotMode = RobotMode.IDLE
    origin: Pose2D = field(default_factory=Pose2D)

    def is_moving(self) -> bool:
        """True when the robot is actively moving."""
        return self.status == RobotMode.MOVING

    def get_pose(self) -> Pose2D:
        """Return a Pose2D snapshot of the current pose."""
        snapshot = Pose2D()
        snapshot.x = self.pose.x
        snapshot.y = self.pose.y
        snapshot.theta = self.pose.theta
        return snapshot

    @classmethod
    def from_odometry(cls, robot_id: str, msg: Any, origin: Pose2D | None = None) -> RobotState:
        """Construct a new RobotState seeded from an Odometry message."""
        state = cls(id=robot_id, origin=origin or Pose2D())
        odom_pose = msg.pose.pose
        state.pose.x = odom_pose.position.x + state.origin.x
        state.pose.y = odom_pose.position.y + state.origin.y
        state.pose.theta = yaw_from_quaternion(odom_pose.orientation) + state.origin.theta
        state.velocity = msg.twist.twist
        return state
