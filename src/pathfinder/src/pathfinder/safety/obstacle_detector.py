from __future__ import annotations
import math
from sensor_msgs.msg import LaserScan

CENTER_DEGREE = 180


class ObstacleDetector:
    """Returns True when a LaserScan return is inside the front cone and within stop distance."""

    def __init__(
        self,
        stop_distance: float,
        detect_degree: float,
    ) -> None:
        self._stop_distance = stop_distance
        self._cone_half_degree = detect_degree / 2.0
        self._cone_slice: slice | None = None

    def detect(self, scan: LaserScan) -> bool:
        """Return True if forward motion should be paused based on this scan."""
        ranges = scan.ranges

        if len(ranges) == 0 or scan.angle_increment <= 0:
            return True

        if self._cone_slice is None:
            half_span = math.floor(self._cone_half_degree / math.degrees(scan.angle_increment))
            self._cone_slice = slice(
                max(0, CENTER_DEGREE - half_span),
                min(len(ranges), CENTER_DEGREE + half_span + 1),
            )

        cone_ranges = ranges[self._cone_slice]

        if not cone_ranges:
            return True

        for distance in cone_ranges:
            if math.isnan(distance):
                return True
            if math.isinf(distance):
                continue
            if distance < self._stop_distance:
                return True

        return False
