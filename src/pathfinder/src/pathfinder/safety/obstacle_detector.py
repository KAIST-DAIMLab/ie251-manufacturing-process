from __future__ import annotations
import math
from sensor_msgs.msg import LaserScan

from pathfinder.utils.physics import wrap_to_pi


class ObstacleDetector:
    """Returns True when a LaserScan return is inside the front cone and within stop distance."""

    def __init__(
        self,
        stop_distance: float,
        detect_radian: float,
    ) -> None:
        self._stop_distance = stop_distance
        self._cone_half_width_radian = detect_radian / 2.0

    def detect(self, scan: LaserScan) -> bool:
        """Return True if forward motion should be paused based on this scan."""
        ranges = scan.ranges

        if len(ranges) == 0 or scan.angle_increment <= 0:
            return True

        cone_beam_found = False

        for index, distance in enumerate(ranges):
            angle = wrap_to_pi(scan.angle_min + index * scan.angle_increment)
            if abs(angle) > self._cone_half_width_radian:
                continue

            cone_beam_found = True

            if math.isnan(distance):
                return True

            if math.isinf(distance):
                continue

            if distance < self._stop_distance:
                return True

        return not cone_beam_found
