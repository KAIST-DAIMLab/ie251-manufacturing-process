from __future__ import annotations
from dataclasses import dataclass
from geometry_msgs.msg import Pose2D


@dataclass(frozen=True)
class MotionConfig:
    """Motion controller tuning values for a single robot."""

    linear_speed: float = 0.22
    angular_speed: float = 1.5
    rate_hz: float = 5.0


@dataclass(frozen=True)
class ObstacleConfig:
    """Obstacle detector settings for a single robot."""

    enabled: bool = True
    stop_distance: float = 0.25


@dataclass()
class Robot:
    """One robot's config (from robots.yaml) plus its latest world-frame pose."""

    id: str
    namespace: str
    start_node: int
    yaw: float
    motion: MotionConfig
    obstacle: ObstacleConfig
    pose: Pose2D | None = None

    def set_pose(self, pose: Pose2D) -> None:
        """Record the latest world-frame pose."""
        self.pose = pose

    @classmethod
    def from_dict(cls, data: dict, sim: bool = False) -> Robot:
        """Parse a robots.yaml entry into a Robot instance, applying sim namespace if needed."""
        robot_id = data['id']
        motion_data = data.get('motion', {})
        obstacle_data = data.get('obstacle', {})
        return cls(
            id=robot_id,
            namespace=f"{robot_id}/sim" if sim else robot_id,
            start_node=int(data['start_node']),
            yaw=float(data.get('yaw', 0.0)),
            motion=MotionConfig(
                linear_speed=float(motion_data.get('linear_speed', 0.22)),
                angular_speed=float(motion_data.get('angular_speed', 1.5)),
                rate_hz=float(motion_data.get('rate_hz', 5.0)),
            ),
            obstacle=ObstacleConfig(
                enabled=bool(obstacle_data.get('enabled', True)),
                stop_distance=float(obstacle_data.get('stop_distance', 0.25)),
            ),
        )
