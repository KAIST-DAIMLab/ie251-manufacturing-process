from __future__ import annotations
import math
from sensor_msgs.msg import LaserScan

CENTER_DEGREE = 180


class ObstacleDetector:
    """Returns True when a LaserScan return is inside the front cone and within stop distance."""

    def __init__(
        self,
        stop_distance: float,
        detect_degree: int,
    ) -> None:
        half_degree = detect_degree // 2

        self._stop_distance = stop_distance
        self._start_degree = CENTER_DEGREE - half_degree
        self._end_degree = CENTER_DEGREE + half_degree

    def detect(self, scan: LaserScan) -> bool:
        """Return True if forward motion should be paused based on this scan."""
        ranges = scan.ranges

        if len(ranges) == 0 or scan.angle_increment <= 0:
            return True

        detect_ranges = ranges[self._start_degree:self._end_degree + 1]

        if not detect_ranges:
            return True

        for distance in detect_ranges:
            if math.isnan(distance):
                continue
            if math.isinf(distance):
                continue
            if distance < self._stop_distance:
                return True

        return False
