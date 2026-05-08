from __future__ import annotations
import math
import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


from pathfinder.safety.obstacle_detector import ObstacleDetector


def _make_scan(ranges, angle_min=-math.pi, angle_increment=None):
    if angle_increment is None:
        angle_increment = 2 * math.pi / 360
    return types.SimpleNamespace(
        angle_min=angle_min,
        angle_increment=angle_increment,
        ranges=ranges,
    )


def _make_full_scan(front_range, detect_degree=20):
    num_samples = 360
    angle_increment = 2 * math.pi / num_samples
    ranges = [float('inf')] * num_samples
    for index in range(num_samples):
        angle = -math.pi + index * angle_increment
        if abs(angle) <= math.radians(detect_degree) / 2.0:
            ranges[index] = front_range
    return _make_scan(ranges)


def _make_detector():
    return ObstacleDetector(
        stop_distance=0.6,
        detect_degree=20,
    )


class ObstacleDetectorTest(unittest.TestCase):
    """Tests for the ObstacleDetector LiDAR-based forward motion gating logic."""

    def test_blocked_when_ranges_empty(self):
        """Detector returns True when the scan contains no range measurements."""
        self.assertTrue(_make_detector().detect(_make_scan(ranges=[])))

    def test_blocked_when_front_range_below_threshold(self):
        """Detector returns True when a cone beam returns a range below stop_distance."""
        self.assertTrue(_make_detector().detect(_make_full_scan(front_range=0.4)))

    def test_clear_when_front_range_above_threshold(self):
        """Detector returns False when all cone beams exceed the threshold distance."""
        self.assertFalse(_make_detector().detect(_make_full_scan(front_range=2.0)))

    def test_inf_in_cone_treated_as_clear(self):
        """Detector returns False when all in-cone returns are infinite (no obstacle detected)."""
        self.assertFalse(_make_detector().detect(_make_full_scan(front_range=float('inf'))))

    def test_nan_in_cone_treated_as_clear(self):
        """Detector returns False when a cone beam returns NaN (treated same as no return)."""
        self.assertFalse(_make_detector().detect(_make_full_scan(front_range=float('nan'))))

    def test_obstacle_outside_cone_does_not_block(self):
        """A close obstacle at ~90 degrees outside the cone does not return True."""
        num_samples = 360
        angle_increment = 2 * math.pi / num_samples
        ranges = [float('inf')] * num_samples

        target_angle = math.pi / 2.0
        side_index = round((target_angle + math.pi) / angle_increment) % num_samples
        ranges[side_index] = 0.1

        self.assertFalse(_make_detector().detect(_make_scan(ranges)))

    def test_threshold_uses_stop_distance(self):
        """A range exactly at the threshold returns False; one just below returns True."""
        detector = _make_detector()
        threshold = 0.6
        self.assertFalse(detector.detect(_make_full_scan(front_range=threshold)))
        self.assertTrue(detector.detect(_make_full_scan(front_range=threshold - 0.001)))

    def test_blocked_when_angle_increment_is_zero(self):
        """Detector returns True when angle_increment is zero (degenerate scan)."""
        self.assertTrue(_make_detector().detect(_make_scan(ranges=[1.0, 2.0, 3.0], angle_increment=0.0)))


if __name__ == '__main__':
    unittest.main()
