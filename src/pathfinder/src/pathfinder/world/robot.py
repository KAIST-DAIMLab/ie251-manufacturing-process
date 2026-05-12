from __future__ import annotations
import dataclasses
import math
from dataclasses import dataclass
from geometry_msgs.msg import Pose2D


@dataclass(frozen=True)
class MotionConfig:
    """Motion controller tuning values for a single robot."""

    linear_speed: float = 0.22
    angular_speed: float = 1.5
    move_rate_hz: float = 5.0
    arrival_tolerance: float = 0.10


@dataclass(frozen=True)
class ObstacleConfig:
    """Obstacle detector settings for a single robot."""

    enabled: bool = True
    stop_distance: float = 0.25
    detect_degree: int = 20


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
    obstacle_blocked: bool = False
    current_edge: tuple[int, int] | None = None

    def set_pose(self, pose: Pose2D) -> None:
        """Record the latest world-frame pose."""
        self.pose = pose

    def set_obstacle_blocked(self, blocked: bool) -> None:
        """Record whether the executor's forward gate is currently blocking forward motion."""
        self.obstacle_blocked = blocked

    def set_current_edge(self, edge: tuple[int, int] | None) -> None:
        """Record the graph edge the robot is currently traversing (or last traversed if interrupted)."""
        self.current_edge = edge

    def to_dict(self) -> dict:
        """Return config fields serializable as JSON; excludes the mutable pose."""
        return {
            "id": self.id,
            "namespace": self.namespace,
            "start_node": self.start_node,
            "yaw": self.yaw,
            "motion": dataclasses.asdict(self.motion),
            "obstacle": dataclasses.asdict(self.obstacle),
        }

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
            yaw=math.radians(float(data.get('yaw', 0.0))),
            motion=MotionConfig(
                linear_speed=float(motion_data.get('linear_speed', 0.22)),
                angular_speed=float(motion_data.get('angular_speed', 1.5)),
                move_rate_hz=float(motion_data.get('move_rate_hz', 5.0)),
                arrival_tolerance=float(motion_data.get('arrival_tolerance', 0.10)),
            ),
            obstacle=ObstacleConfig(
                enabled=bool(obstacle_data.get('enabled', True)),
                stop_distance=float(obstacle_data.get('stop_distance', 0.25)),
                detect_degree=int(obstacle_data.get('detect_degree', 20)),
            ),
        )
