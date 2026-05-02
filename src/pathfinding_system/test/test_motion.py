import math
import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


from pathfinding_system.robot.motion import (
    MotionController,
    MotionParameters,
    PathFollower,
    compute_drive,
)
from pathfinding_system.world.node import Node


class MotionTest(unittest.TestCase):
    def test_arrival_within_tolerance_returns_zero_velocities(self):
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        params = MotionParameters(arrival_tolerance=0.10)

        result = compute_drive(pose, Node(id=1, x=0.05, y=0.0), params)

        self.assertTrue(result.arrived)
        self.assertEqual(result.linear_x, 0.0)
        self.assertEqual(result.angular_z, 0.0)

    def test_heading_error_above_tolerance_blocks_forward_velocity(self):
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=math.pi / 2.0)
        params = MotionParameters(heading_tolerance=0.2)

        result = compute_drive(pose, Node(id=1, x=1.0, y=0.0), params)

        self.assertFalse(result.arrived)
        self.assertEqual(result.linear_x, 0.0)
        self.assertLess(result.angular_z, 0.0)

    def test_large_distance_clamps_linear_velocity(self):
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        params = MotionParameters(linear_gain=2.0, max_linear_velocity=0.3)

        result = compute_drive(pose, Node(id=1, x=10.0, y=0.0), params)

        self.assertFalse(result.arrived)
        self.assertEqual(result.linear_x, 0.3)

    def test_large_heading_error_clamps_angular_velocity(self):
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        params = MotionParameters(angular_gain=10.0, max_angular_velocity=1.5)

        result = compute_drive(pose, Node(id=1, x=0.0, y=1.0), params)

        self.assertFalse(result.arrived)
        self.assertEqual(result.angular_z, 1.5)

    def test_wrap_around_heading_uses_shortest_angular_direction(self):
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=math.radians(179.0))
        params = MotionParameters(angular_gain=1.0, max_angular_velocity=1.5)

        result = compute_drive(pose, Node(id=1, x=-1.0, y=-0.01), params)

        self.assertFalse(result.arrived)
        self.assertGreater(result.angular_z, 0.0)
        self.assertLess(result.angular_z, math.radians(2.0))

    def test_motion_controller_delegates_drive_calculation(self):
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        controller = MotionController(
            MotionParameters(linear_gain=2.0, max_linear_velocity=0.3)
        )

        result = controller.drive_towards(pose, Node(id=1, x=1.0, y=0.0))

        self.assertFalse(result.arrived)
        self.assertEqual(result.linear_x, 0.3)

    def test_path_follower_advances_when_waypoint_is_reached(self):
        follower = PathFollower(MotionController(MotionParameters()))
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)

        follower.start([
            Node(id=1, x=0.0, y=0.0),
            Node(id=2, x=1.0, y=0.0),
        ])

        first = follower.step(pose)
        second = follower.step(pose)

        self.assertFalse(first.completed)
        self.assertEqual(first.current_index, 0)
        self.assertTrue(first.drive_result.arrived)
        self.assertFalse(second.completed)
        self.assertEqual(second.current_index, 1)
        self.assertFalse(second.drive_result.arrived)
        self.assertGreater(second.drive_result.linear_x, 0.0)

    def test_path_follower_completes_after_final_waypoint_arrival(self):
        follower = PathFollower(MotionController(MotionParameters()))
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)

        follower.start([Node(id=1, x=0.0, y=0.0)])
        step = follower.step(pose)

        self.assertTrue(step.completed)
        self.assertEqual(step.current_index, 0)
        self.assertTrue(step.drive_result.arrived)

    def test_path_follower_cancel_clears_active_path(self):
        follower = PathFollower(MotionController(MotionParameters()))
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)

        follower.start([Node(id=1, x=1.0, y=0.0)])
        follower.cancel()
        step = follower.step(pose)

        self.assertTrue(step.completed)
        self.assertTrue(step.drive_result.arrived)


if __name__ == '__main__':
    unittest.main()
