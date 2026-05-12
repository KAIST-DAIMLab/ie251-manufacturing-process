from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Callable, Protocol

from geometry_msgs.msg import Pose2D, Twist

from pathfinder.world.node import Node
from pathfinder.utils.physics import heading_offset, planar_distance, wrap_to_pi


@dataclass(frozen=True)
class MotionParameters:
    """Tunable parameters shared by motion control methods."""
    linear_speed: float = 0.22
    angular_speed: float = 1.5
    linear_gain: float = 0.5
    angular_gain: float = 1.5
    arrival_tolerance: float = 0.10
    heading_tolerance: float = 0.1


class CmdVelPublisher(Protocol):
    """Structural type for any object that can publish a Twist message."""
    def publish(self, twist: Twist) -> None: ...


class MotionEngine:
    """Single-tick proportional controller for waypoint and heading commands."""

    def __init__(
        self,
        cmd_vel_publisher: CmdVelPublisher,
        pose_provider: Callable[[], Pose2D],
        params: MotionParameters = MotionParameters(),
    ) -> None:
        self._cmd_vel_publisher = cmd_vel_publisher
        self._pose_provider = pose_provider
        self._params = params

    def drive_towards(self, target: Node) -> bool:
        """Publish one proportional cmd_vel toward target; True when within arrival tolerance."""
        if self._get_distance(target) <= self._params.arrival_tolerance:
            self._publish(0.0, 0.0)
            return True

        distance = self._get_distance(target)
        angle = self._get_angle(target)
        speed_angular = _clamp(self._params.angular_gain * angle, self._params.angular_speed)
        speed_linear = min(self._params.linear_gain * distance, self._params.linear_speed) if abs(angle) <= self._params.heading_tolerance else 0.0

        self._publish(speed_linear, speed_angular)
        return False

    def turn_towards(self, heading: float) -> bool:
        """Publish one proportional angular cmd_vel toward absolute heading."""
        angle = wrap_to_pi(heading - self._pose_provider().theta)
        if abs(angle) <= self._params.heading_tolerance:
            self._publish(0.0, 0.0)
            return True

        speed_angular = _clamp(self._params.angular_gain * angle, self._params.angular_speed)
        self._publish(0.0, speed_angular)
        return False

    def stop(self) -> None:
        """Publish a zero Twist to halt the robot."""
        self._publish(0.0, 0.0)

    def _get_distance(self, target: Node) -> float:
        return planar_distance(self._pose_provider(), target)

    def _get_angle(self, target: Node) -> float:
        return heading_offset(self._pose_provider(), target)

    def _publish(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_vel_publisher.publish(twist)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
