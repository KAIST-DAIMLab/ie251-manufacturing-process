from __future__ import annotations
import math
import rospy
from sensor_msgs.msg import LaserScan


class ObstacleDetector:
    """Returns True when a LaserScan return is inside the front cone and within stop distance."""

    def __init__(
        self,
        stop_distance: float,
        detect_degree: int,
    ) -> None:
        self._stop_distance = stop_distance
        self._half_degree = detect_degree // 2

    def detect(self, scan: LaserScan) -> bool:
        """Return True if forward motion should be paused based on this scan."""
        ranges = scan.ranges

        if len(ranges) == 0 or scan.angle_increment <= 0:
            return True

        n = len(ranges)
        front = round(-scan.angle_min / scan.angle_increment) % n
        detect_ranges = [ranges[(front + i) % n] for i in range(-self._half_degree, self._half_degree + 1)]

        count = 0
        for distance in detect_ranges:
            if math.isnan(distance) or math.isinf(distance):
                continue
            if distance <= 0:
                continue
            if distance < self._stop_distance:
                count += 1
            if count > 4:
                return True

        return False
