from __future__ import annotations
import math
from typing import Callable


def _wrap_to_pi(radian: float) -> float:
    return math.atan2(math.sin(radian), math.cos(radian))


class ObstacleGate:
    """Pauses forward motion when a LaserScan return is inside the front cone."""

    def __init__(
        self,
        stop_distance: float,
        error_margin: float,
        stale_timeout_seconds: float,
        cone_half_width_radian: float,
        time_provider: Callable[[], float],
    ) -> None:
        self._stop_distance = stop_distance
        self._error_margin = error_margin
        self._stale_timeout_seconds = stale_timeout_seconds
        self._cone_half_width_radian = cone_half_width_radian
        self._time_provider = time_provider
        self._last_scan = None
        self._last_update_time: float | None = None

    def update(self, scan) -> None:
        """Store the most recent scan and record the current time."""
        self._last_scan = scan
        self._last_update_time = self._time_provider()

    def is_blocked(self) -> bool:
        """Return True if forward motion should be paused."""
        if self._last_scan is None or self._last_update_time is None:
            return True

        age = self._time_provider() - self._last_update_time
        if age > self._stale_timeout_seconds:
            return True

        scan = self._last_scan
        ranges = scan.ranges

        if len(ranges) == 0 or scan.angle_increment <= 0:
            return True

        threshold = self._stop_distance + self._error_margin
        cone_beam_found = False

        for index, distance in enumerate(ranges):
            angle = _wrap_to_pi(scan.angle_min + index * scan.angle_increment)
            if abs(angle) > self._cone_half_width_radian:
                continue

            cone_beam_found = True

            if math.isnan(distance):
                return True

            if math.isinf(distance):
                continue

            if distance < threshold:
                return True

        if not cone_beam_found:
            return True

        return False
