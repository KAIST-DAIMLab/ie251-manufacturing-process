from __future__ import annotations
import math
import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


from pathfinder.safety.obstacle_detector import ObstacleDetector


def _make_scan(ranges, angle_min=0.0, angle_increment=None):
    if angle_increment is None:
        angle_increment = 2 * math.pi / 360
    return types.SimpleNamespace(
        angle_min=angle_min,
        angle_increment=angle_increment,
        ranges=ranges,
    )


def _make_full_scan(front_range, detect_radian=math.radians(20)):
    num_samples = 360
    angle_increment = 2 * math.pi / num_samples
    ranges = [float('inf')] * num_samples
    for index in range(num_samples):
        angle = math.atan2(
            math.sin(index * angle_increment),
            math.cos(index * angle_increment),
        )
        if abs(angle) <= detect_radian / 2.0:
            ranges[index] = front_range
    return _make_scan(ranges)


def _make_detector():
    return ObstacleDetector(
        stop_distance=0.6,
        detect_radian=math.radians(20),
    )


class ObstacleDetectorTest(unittest.TestCase):
    """Tests for the ObstacleDetector LiDAR-based forward motion gating logic."""

    def test_clear_when_no_scan_received(self):
        """Detector starts clear until a scan reports an obstacle."""
        detector = _make_detector()
        self.assertFalse(detector.is_blocked())

    def test_blocked_when_ranges_empty(self):
        """Detector is blocked when the scan contains no range measurements."""
        detector = _make_detector()
        scan = _make_scan(ranges=[])
        detector.update(scan)
        self.assertTrue(detector.is_blocked())

    def test_blocked_when_front_range_below_threshold(self):
        """Detector is blocked when a cone beam returns a range below stop_distance."""
        detector = _make_detector()
        scan = _make_full_scan(front_range=0.4)
        detector.update(scan)
        self.assertTrue(detector.is_blocked())

    def test_clear_when_front_range_above_threshold(self):
        """Detector is clear when all cone beams exceed the threshold distance."""
        detector = _make_detector()
        scan = _make_full_scan(front_range=2.0)
        detector.update(scan)
        self.assertFalse(detector.is_blocked())

    def test_inf_in_cone_treated_as_clear(self):
        """Detector is clear when all in-cone returns are infinite (no obstacle detected)."""
        detector = _make_detector()
        scan = _make_full_scan(front_range=float('inf'))
        detector.update(scan)
        self.assertFalse(detector.is_blocked())

    def test_nan_in_cone_treated_as_blocked(self):
        """Detector is blocked when a cone beam returns NaN (sensor error or no return)."""
        detector = _make_detector()
        scan = _make_full_scan(front_range=float('nan'))
        detector.update(scan)
        self.assertTrue(detector.is_blocked())

    def test_obstacle_outside_cone_does_not_block(self):
        """A close obstacle at ~90 degrees outside the cone does not trigger blocking."""
        detector = _make_detector()
        num_samples = 360
        angle_increment = 2 * math.pi / num_samples
        ranges = [float('inf')] * num_samples

        target_angle = math.pi / 2.0
        side_index = round(target_angle / angle_increment) % num_samples
        ranges[side_index] = 0.1

        scan = _make_scan(ranges)
        detector.update(scan)
        self.assertFalse(detector.is_blocked())

    def test_threshold_uses_stop_distance(self):
        """A range exactly at the threshold does not block; one just below does."""
        detector = _make_detector()

        threshold = 0.6

        scan_at_threshold = _make_full_scan(front_range=threshold)
        detector.update(scan_at_threshold)
        self.assertFalse(detector.is_blocked())

        scan_below_threshold = _make_full_scan(front_range=threshold - 0.001)
        detector.update(scan_below_threshold)
        self.assertTrue(detector.is_blocked())

    def test_blocked_when_angle_increment_is_zero(self):
        """Detector is blocked when angle_increment is zero (degenerate scan)."""
        detector = _make_detector()
        scan = _make_scan(ranges=[1.0, 2.0, 3.0], angle_increment=0.0)
        detector.update(scan)
        self.assertTrue(detector.is_blocked())


if __name__ == '__main__':
    unittest.main()
