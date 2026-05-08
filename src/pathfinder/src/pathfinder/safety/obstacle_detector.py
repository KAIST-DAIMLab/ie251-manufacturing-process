from __future__ import annotations
import math
from sensor_msgs.msg import LaserScan

from pathfinder.utils.physics import wrap_to_pi


class ObstacleDetector:
    """Pauses forward motion when a LaserScan return is inside the front cone."""

    def __init__(
        self,
        stop_distance: float,
        cone_half_width_radian: float,
    ) -> None:
        self._stop_distance = stop_distance
        self._cone_half_width_radian = cone_half_width_radian
        self._blocked = False

    def update(self, scan: LaserScan) -> None:
        """Update whether forward motion should be paused from one scan."""
        ranges = scan.ranges

        if len(ranges) == 0 or scan.angle_increment <= 0:
            self._blocked = True
            return

        cone_beam_found = False

        for index, distance in enumerate(ranges):
            angle = wrap_to_pi(scan.angle_min + index * scan.angle_increment)
            if abs(angle) > self._cone_half_width_radian:
                continue

            cone_beam_found = True

            if math.isnan(distance):
                self._blocked = True
                return

            if math.isinf(distance):
                continue

            if distance < self._stop_distance:
                self._blocked = True
                return

        self._blocked = not cone_beam_found

    def is_blocked(self) -> bool:
        """Return True if forward motion should be paused."""
        return self._blocked
