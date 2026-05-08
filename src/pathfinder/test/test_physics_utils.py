import math
import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


class PhysicsUtilsTest(unittest.TestCase):
    def test_yaw_from_quaternion_returns_planar_heading(self):
        yaw = 0.75
        half = yaw / 2.0
        q = types.SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(half),
            w=math.cos(half),
        )

        from pathfinder.utils.physics import yaw_from_quaternion

        self.assertAlmostEqual(yaw_from_quaternion(q), yaw)


class SegmentPoseTest(unittest.TestCase):
    """Pure-math tests for the edge segment pose helper."""

    def test_horizontal_segment(self):
        from pathfinder.utils.physics import segment_pose

        midpoint, length, yaw = segment_pose((0.0, 0.0), (2.0, 0.0))

        self.assertAlmostEqual(midpoint[0], 1.0)
        self.assertAlmostEqual(midpoint[1], 0.0)
        self.assertAlmostEqual(length, 2.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_vertical_segment(self):
        from pathfinder.utils.physics import segment_pose

        midpoint, length, yaw = segment_pose((1.0, 1.0), (1.0, 4.0))

        self.assertAlmostEqual(midpoint[0], 1.0)
        self.assertAlmostEqual(midpoint[1], 2.5)
        self.assertAlmostEqual(length, 3.0)
        self.assertAlmostEqual(yaw, math.pi / 2.0)

    def test_diagonal_segment(self):
        from pathfinder.utils.physics import segment_pose

        midpoint, length, yaw = segment_pose((0.0, 0.0), (1.0, 1.0))

        self.assertAlmostEqual(midpoint[0], 0.5)
        self.assertAlmostEqual(midpoint[1], 0.5)
        self.assertAlmostEqual(length, math.sqrt(2.0))
        self.assertAlmostEqual(yaw, math.pi / 4.0)

    def test_reversed_endpoints_flip_yaw_by_pi(self):
        from pathfinder.utils.physics import segment_pose

        _, _, forward_yaw = segment_pose((0.0, 0.0), (1.0, 1.0))
        _, _, reverse_yaw = segment_pose((1.0, 1.0), (0.0, 0.0))

        self.assertAlmostEqual(abs(forward_yaw - reverse_yaw), math.pi)


if __name__ == '__main__':
    unittest.main()
