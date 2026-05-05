import math
import os
import sys
import threading
import time
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.sleep_callbacks = []
    rospy.sleep_count = 0
    rospy.max_sleep_count = 20
    rospy.shutdown = False

    class Rate:
        def __init__(self, hz):
            self.hz = hz

        def sleep(self):
            import rospy as _r
            _r.sleep_count += 1
            if _r.sleep_count > _r.max_sleep_count:
                raise AssertionError('movement primitive did not finish')
            for callback in list(_r.sleep_callbacks):
                callback()
            time.sleep(0.001)

    rospy.Rate = Rate
    rospy.is_shutdown = lambda: sys.modules['rospy'].shutdown
    sys.modules['rospy'] = rospy

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')

    class Pose2D:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    geometry_msgs_msg.Pose2D = Pose2D
    geometry_msgs_msg.Twist = Twist
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg


# Install stubs before importing ROS-coupled modules so their module-level
# 'import rospy' binds to the stub rather than the real rospy.
_install_ros_stubs()

from pathfinding_system.robot.motion_controller import MotionController
from pathfinding_system.robot.path_follower import PathFollower
from pathfinding_system.world.node import Node


def _odom_pose(x=0.0, y=0.0, yaw=0.0):
    return types.SimpleNamespace(x=x, y=y, theta=yaw)


class FakePublisher:
    """Fake cmd_vel publisher that records published messages."""

    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeMotionController:
    """Fake MotionController that records primitive calls and returns configurable results."""

    def __init__(self, return_value=True):
        self.calls = []
        self.return_value = return_value
        self._stop = False

    def turnLeft(self, radian):
        """Record a turnLeft call."""
        self.calls.append(('turnLeft', radian))
        return self.return_value

    def turnRight(self, radian):
        """Record a turnRight call."""
        self.calls.append(('turnRight', radian))
        return self.return_value

    def MoveTowards(self, meter):
        """Record a MoveTowards call."""
        self.calls.append(('MoveTowards', meter))
        return self.return_value

    def stop(self):
        """Record a stop request."""
        self._stop = True

    def _pose_provider(self):
        return types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)


def _reset_rospy_state():
    rospy = sys.modules['rospy']
    rospy.sleep_count = 0
    rospy.max_sleep_count = 20
    rospy.shutdown = False
    rospy.sleep_callbacks[:] = []


class MotionControllerTest(unittest.TestCase):
    def setUp(self):
        _reset_rospy_state()

    def _make_controller(self, pose=None):
        publisher = FakePublisher()
        pose_holder = [pose or _odom_pose()]
        controller = MotionController(
            cmd_vel_publisher=publisher,
            pose_provider=lambda: pose_holder[0],
        )
        return controller, publisher, pose_holder

    def test_set_speed_methods_control_published_primitive_velocities(self):
        rospy = sys.modules['rospy']
        controller, publisher, pose_holder = self._make_controller(_odom_pose(x=0.0))
        rospy.sleep_callbacks.append(lambda: pose_holder.__setitem__(0, _odom_pose(x=0.5)))

        controller.SetLinearSpeed(0.11)
        self.assertTrue(controller.MoveTowards(0.2))

        self.assertEqual(publisher.published[0].linear.x, 0.11)

        publisher.published[:] = []
        rospy.sleep_callbacks[:] = []
        rospy.sleep_count = 0
        pose_holder[0] = _odom_pose(yaw=0.0)
        rospy.sleep_callbacks.append(lambda: pose_holder.__setitem__(0, _odom_pose(yaw=0.5)))

        controller.SetAngularSpeed(0.33)
        self.assertTrue(controller.turnLeft(0.2))

        self.assertEqual(publisher.published[0].angular.z, 0.33)

    def test_move_towards_publishes_positive_linear_velocity_until_distance_reached(self):
        rospy = sys.modules['rospy']
        controller, publisher, pose_holder = self._make_controller(_odom_pose(x=0.0))
        rospy.sleep_callbacks.append(lambda: pose_holder.__setitem__(0, _odom_pose(x=0.3)))

        result = controller.MoveTowards(0.2)

        self.assertTrue(result)
        self.assertGreater(publisher.published[0].linear.x, 0.0)
        self.assertEqual(publisher.published[-1].linear.x, 0.0)
        self.assertEqual(publisher.published[-1].angular.z, 0.0)

    def test_move_backwards_publishes_negative_linear_velocity_until_distance_reached(self):
        rospy = sys.modules['rospy']
        controller, publisher, pose_holder = self._make_controller(_odom_pose(x=0.0))
        rospy.sleep_callbacks.append(lambda: pose_holder.__setitem__(0, _odom_pose(x=-0.3)))

        result = controller.MoveBackwards(0.2)

        self.assertTrue(result)
        self.assertLess(publisher.published[0].linear.x, 0.0)
        self.assertEqual(publisher.published[-1].linear.x, 0.0)

    def test_turn_left_publishes_positive_angular_velocity_until_angle_reached(self):
        rospy = sys.modules['rospy']
        controller, publisher, pose_holder = self._make_controller(_odom_pose(yaw=0.0))
        rospy.sleep_callbacks.append(lambda: pose_holder.__setitem__(0, _odom_pose(yaw=0.4)))

        result = controller.turnLeft(0.2)

        self.assertTrue(result)
        self.assertGreater(publisher.published[0].angular.z, 0.0)
        self.assertEqual(publisher.published[-1].angular.z, 0.0)

    def test_turn_right_publishes_negative_angular_velocity_until_angle_reached(self):
        rospy = sys.modules['rospy']
        controller, publisher, pose_holder = self._make_controller(_odom_pose(yaw=0.0))
        rospy.sleep_callbacks.append(lambda: pose_holder.__setitem__(0, _odom_pose(yaw=-0.4)))

        result = controller.turnRight(0.2)

        self.assertTrue(result)
        self.assertLess(publisher.published[0].angular.z, 0.0)
        self.assertEqual(publisher.published[-1].angular.z, 0.0)

    def test_stop_interrupts_blocking_movement_and_publishes_zero_velocity(self):
        sys.modules['rospy'].max_sleep_count = 1000
        controller, publisher, pose_holder = self._make_controller(_odom_pose(x=0.0))
        result = []

        thread = threading.Thread(target=lambda: result.append(controller.MoveTowards(10.0)))
        thread.start()
        while not publisher.published:
            time.sleep(0.001)

        controller.stop()
        thread.join(timeout=1.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(result, [False])
        self.assertEqual(publisher.published[-1].linear.x, 0.0)
        self.assertEqual(publisher.published[-1].angular.z, 0.0)


class PathFollowerTest(unittest.TestCase):
    def test_follow_returns_true_after_all_waypoints(self):
        controller = FakeMotionController(return_value=True)
        follower = PathFollower(controller)

        result = follower.follow([
            Node(id=1, x=1.0, y=0.0),
            Node(id=2, x=2.0, y=0.0),
        ])

        self.assertTrue(result)
        move_calls = [name for name, _ in controller.calls if name == 'MoveTowards']
        self.assertEqual(len(move_calls), 2)

    def test_follow_returns_false_when_primitive_interrupted(self):
        controller = FakeMotionController(return_value=False)
        follower = PathFollower(controller)

        result = follower.follow([Node(id=1, x=1.0, y=0.0)])

        self.assertFalse(result)

    def test_follow_skips_turn_when_robot_already_aligned(self):
        controller = FakeMotionController(return_value=True)
        follower = PathFollower(controller)

        # Robot at origin facing right (theta=0), waypoint directly ahead — no turn needed
        result = follower.follow([Node(id=1, x=1.0, y=0.0)])

        self.assertTrue(result)
        turn_calls = [name for name, _ in controller.calls if 'turn' in name.lower()]
        self.assertEqual(len(turn_calls), 0)

    def test_follow_turns_left_for_positive_heading_error(self):
        controller = FakeMotionController(return_value=True)
        follower = PathFollower(controller)

        # Robot at origin facing right (theta=0), waypoint to the upper-left
        result = follower.follow([Node(id=1, x=0.0, y=1.0)])

        self.assertTrue(result)
        self.assertEqual(controller.calls[0][0], 'turnLeft')

    def test_follow_turns_right_for_negative_heading_error(self):
        controller = FakeMotionController(return_value=True)
        # Robot at origin facing up (theta=pi/2), waypoint to the right (x positive)
        # Expected heading = atan2(0, 1) = 0; error = 0 - pi/2 = -pi/2 → turnRight
        controller._pose_provider = lambda: types.SimpleNamespace(x=0.0, y=0.0, theta=math.pi / 2)
        follower = PathFollower(controller)

        result = follower.follow([Node(id=1, x=1.0, y=0.0)])

        self.assertTrue(result)
        self.assertEqual(controller.calls[0][0], 'turnRight')

    def test_follow_current_index_tracks_waypoint_progress(self):
        controller = FakeMotionController(return_value=True)
        follower = PathFollower(controller)
        observed_indices = []

        original_move = controller.MoveTowards

        def recording_move(meter):
            observed_indices.append(follower.current_index)
            return original_move(meter)

        controller.MoveTowards = recording_move

        follower.follow([Node(id=1, x=1.0, y=0.0), Node(id=2, x=2.0, y=0.0)])

        self.assertEqual(observed_indices, [0, 1])

    def test_cancel_calls_motion_controller_stop(self):
        controller = FakeMotionController(return_value=True)
        follower = PathFollower(controller)

        follower.cancel()

        self.assertTrue(controller._stop)


if __name__ == '__main__':
    unittest.main()
