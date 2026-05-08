from __future__ import annotations
import math
import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


from pathfinder.safety.obstacle_gate import ObstacleGate


def _make_scan(ranges, angle_min=0.0, angle_increment=None):
    if angle_increment is None:
        angle_increment = 2 * math.pi / 360
    return types.SimpleNamespace(
        angle_min=angle_min,
        angle_increment=angle_increment,
        ranges=ranges,
    )


def _make_full_scan(front_range, cone_half_width_radian=math.radians(10)):
    num_samples = 360
    angle_increment = 2 * math.pi / num_samples
    ranges = [float('inf')] * num_samples
    for index in range(num_samples):
        angle = math.atan2(
            math.sin(index * angle_increment),
            math.cos(index * angle_increment),
        )
        if abs(angle) <= cone_half_width_radian:
            ranges[index] = front_range
    return _make_scan(ranges)


def _make_gate(current_time=0.0):
    time_container = [current_time]

    def time_provider():
        return time_container[0]

    gate = ObstacleGate(
        stop_distance=0.5,
        error_margin=0.1,
        stale_timeout_seconds=1.0,
        cone_half_width_radian=math.radians(10),
        time_provider=time_provider,
    )
    return gate, time_container


class ObstacleGateTest(unittest.TestCase):
    """Tests for the ObstacleGate LiDAR-based forward motion gating logic."""

    def test_blocked_when_no_scan_received(self):
        """Gate is blocked when no scan has ever been provided."""
        gate, _ = _make_gate()
        self.assertTrue(gate.is_blocked())

    def test_blocked_when_scan_is_stale(self):
        """Gate is blocked when the last scan was received too long ago."""
        gate, time_container = _make_gate(current_time=0.0)
        scan = _make_full_scan(front_range=2.0)
        gate.update(scan)
        time_container[0] = 2.0
        self.assertTrue(gate.is_blocked())

    def test_blocked_when_ranges_empty(self):
        """Gate is blocked when the scan contains no range measurements."""
        gate, _ = _make_gate()
        scan = _make_scan(ranges=[])
        gate.update(scan)
        self.assertTrue(gate.is_blocked())

    def test_blocked_when_front_range_below_threshold(self):
        """Gate is blocked when a cone beam returns a range below stop_distance + error_margin."""
        gate, _ = _make_gate()
        scan = _make_full_scan(front_range=0.4)
        gate.update(scan)
        self.assertTrue(gate.is_blocked())

    def test_clear_when_front_range_above_threshold(self):
        """Gate is clear when all cone beams exceed the threshold distance."""
        gate, _ = _make_gate()
        scan = _make_full_scan(front_range=2.0)
        gate.update(scan)
        self.assertFalse(gate.is_blocked())

    def test_inf_in_cone_treated_as_clear(self):
        """Gate is clear when all in-cone returns are infinite (no obstacle detected)."""
        gate, _ = _make_gate()
        num_samples = 360
        ranges = [float('inf')] * num_samples
        scan = _make_scan(ranges)
        gate.update(scan)
        self.assertFalse(gate.is_blocked())

    def test_nan_in_cone_treated_as_blocked(self):
        """Gate is blocked when a cone beam returns NaN (sensor error or no return)."""
        gate, _ = _make_gate()
        scan = _make_full_scan(front_range=float('nan'))
        gate.update(scan)
        self.assertTrue(gate.is_blocked())

    def test_obstacle_outside_cone_does_not_block(self):
        """A close obstacle at ~90 degrees outside the cone does not trigger blocking."""
        gate, _ = _make_gate()
        num_samples = 360
        angle_increment = 2 * math.pi / num_samples
        ranges = [float('inf')] * num_samples

        target_angle = math.pi / 2.0
        side_index = round(target_angle / angle_increment) % num_samples
        ranges[side_index] = 0.1

        scan = _make_scan(ranges)
        gate.update(scan)
        self.assertFalse(gate.is_blocked())

    def test_threshold_uses_stop_distance_plus_error_margin(self):
        """A range exactly at the threshold does not block; one just below does."""
        gate, _ = _make_gate()

        threshold = 0.5 + 0.1

        scan_at_threshold = _make_full_scan(front_range=threshold)
        gate.update(scan_at_threshold)
        self.assertFalse(gate.is_blocked())

        scan_below_threshold = _make_full_scan(front_range=threshold - 0.001)
        gate.update(scan_below_threshold)
        self.assertTrue(gate.is_blocked())


if __name__ == '__main__':
    unittest.main()
