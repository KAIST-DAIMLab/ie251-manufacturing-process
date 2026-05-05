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

        from pathfinding_system.utils.physics import yaw_from_quaternion

        self.assertAlmostEqual(yaw_from_quaternion(q), yaw)


if __name__ == '__main__':
    unittest.main()
