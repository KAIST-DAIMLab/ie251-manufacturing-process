from __future__ import annotations
import math
from dataclasses import dataclass

import rospy
from geometry_msgs.msg import Twist


@dataclass(frozen=True)
class Pose:
    """Planar robot pose: 2D position and heading. Duck-typed — any object with x, y, theta floats works."""
    x: float
    y: float
    theta: float


class MotionController:
    """Imperative blocking primitives that drive a robot via an injected cmd_vel publisher using a pose provider."""

    def __init__(
        self,
        cmd_vel_publisher,
        pose_provider,
        linear_speed: float = 0.22,
        angular_speed: float = 1.5,
        rate_hz: float = 5.0,
    ) -> None:
        self._cmd_vel_publisher = cmd_vel_publisher
        self._pose_provider = pose_provider
        self._linear_speed = linear_speed
        self._angular_speed = angular_speed
        self._rate_hz = rate_hz
        self._stop = False

    def turnLeft(self, radian: float) -> bool:
        """Rotate counter-clockwise by `radian` radians; True on completion, False if interrupted."""
        return self._turn(abs(radian), direction=+1.0)

    def turnRight(self, radian: float) -> bool:
        """Rotate clockwise by `radian` radians; True on completion, False if interrupted."""
        return self._turn(abs(radian), direction=-1.0)

    def MoveTowards(self, meter: float) -> bool:
        """Drive forward by `meter` meters; True on completion, False if interrupted."""
        return self._move(abs(meter), direction=+1.0)

    def MoveBackwards(self, meter: float) -> bool:
        """Drive backward by `meter` meters; True on completion, False if interrupted."""
        return self._move(abs(meter), direction=-1.0)

    def SetLinearSpeed(self, speed: float) -> None:
        """Set the linear speed used by Move primitives."""
        self._linear_speed = abs(speed)

    def SetAngularSpeed(self, speed: float) -> None:
        """Set the angular speed used by turn primitives."""
        self._angular_speed = abs(speed)

    def stop(self) -> None:
        """Interrupt any in-flight primitive and publish a zero Twist."""
        self._stop = True
        self._publish(0.0, 0.0)

    def _move(self, distance: float, direction: float) -> bool:
        if distance <= 0.0:
            self.stop()
            return True

        self._stop = False
        start = self._pose_provider()

        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown() and not self._stop:
            current = self._pose_provider()
            travelled = math.hypot(current.x - start.x, current.y - start.y)
            if travelled >= distance:
                self.stop()
                return True
            self._publish(direction * self._linear_speed, 0.0)
            rate.sleep()

        self.stop()
        return False

    def _turn(self, radian: float, direction: float) -> bool:
        if radian <= 0.0:
            self.stop()
            return True

        self._stop = False
        start = self._pose_provider()

        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown() and not self._stop:
            current = self._pose_provider()
            rotated = abs(self._wrap_to_pi(current.theta - start.theta))
            if rotated >= radian:
                self.stop()
                return True
            self._publish(0.0, direction * self._angular_speed)
            rate.sleep()

        self.stop()
        return False

    def _publish(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_vel_publisher.publish(twist)

    def _wrap_to_pi(self, radian: float) -> float:
        return math.atan2(math.sin(radian), math.cos(radian))
