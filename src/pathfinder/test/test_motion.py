import math
import os
import sys
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

from pathfinder.robot.motion_controller import MotionController, MotionParameters
from pathfinder.robot.path_follower import PathFollower
from pathfinder.world.node import Node


def _odom_pose(x=0.0, y=0.0, yaw=0.0):
    return types.SimpleNamespace(x=x, y=y, theta=yaw)


class FakePublisher:
    """Fake cmd_vel publisher that records published messages."""

    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeMotionController:
    """Fake MotionController that records drive_towards calls and returns configurable results."""

    def __init__(self, return_value=True):
        self.calls = []
        self.return_value = return_value
        self.stopped = False

    def drive_towards(self, target):
        """Record a drive_towards call and return the configured result."""
        self.calls.append(target)
        return self.return_value

    def stop(self):
        """Record a stop request."""
        self.stopped = True


def _reset_rospy_state():
    rospy = sys.modules['rospy']
    rospy.sleep_count = 0
    rospy.max_sleep_count = 20
    rospy.shutdown = False
    rospy.sleep_callbacks[:] = []


class DriveTowardsTest(unittest.TestCase):
    """Tests for the single-tick closed-loop proportional method."""

    def setUp(self):
        _reset_rospy_state()

    def _controller_at(self, x=0.0, y=0.0, theta=0.0, **params_kwargs):
        publisher = FakePublisher()
        pose = _odom_pose(x=x, y=y, yaw=theta)
        controller = MotionController(
            cmd_vel_publisher=publisher,
            pose_provider=lambda: pose,
            params=MotionParameters(**params_kwargs),
        )
        return (controller, publisher)

    def test_arrival_within_tolerance_returns_true_and_publishes_zero(self):
        controller, publisher = self._controller_at(x=0.0, y=0.0, arrival_tolerance=0.10)

        arrived = controller.drive_towards(Node(id=1, x=0.05, y=0.0))

        self.assertTrue(arrived)
        self.assertEqual(publisher.published[-1].linear.x, 0.0)
        self.assertEqual(publisher.published[-1].angular.z, 0.0)

    def test_heading_error_above_tolerance_blocks_forward_velocity(self):
        controller, publisher = self._controller_at(x=0.0, y=0.0, theta=math.pi / 2.0, heading_tolerance=0.2)

        arrived = controller.drive_towards(Node(id=1, x=1.0, y=0.0))

        self.assertFalse(arrived)
        self.assertEqual(publisher.published[-1].linear.x, 0.0)
        self.assertLess(publisher.published[-1].angular.z, 0.0)

    def test_large_distance_clamps_linear_velocity(self):
        controller, publisher = self._controller_at(linear_gain=2.0, linear_speed=0.3)

        arrived = controller.drive_towards(Node(id=1, x=10.0, y=0.0))

        self.assertFalse(arrived)
        self.assertEqual(publisher.published[-1].linear.x, 0.3)

    def test_large_heading_error_clamps_angular_velocity(self):
        controller, publisher = self._controller_at(angular_gain=10.0, angular_speed=1.5)

        arrived = controller.drive_towards(Node(id=1, x=0.0, y=1.0))

        self.assertFalse(arrived)
        self.assertEqual(publisher.published[-1].angular.z, 1.5)

    def test_wrap_around_heading_uses_shortest_angular_direction(self):
        controller, publisher = self._controller_at(
            theta=math.radians(179.0),
            angular_gain=1.0,
            angular_speed=1.5,
            heading_tolerance=math.radians(1.0),
        )

        arrived = controller.drive_towards(Node(id=1, x=-1.0, y=-0.01))

        self.assertFalse(arrived)
        self.assertGreater(publisher.published[-1].angular.z, 0.0)
        self.assertLess(publisher.published[-1].angular.z, math.radians(2.0))


class TurnTowardsTest(unittest.TestCase):
    """Tests for the single-tick closed-loop heading method."""

    def setUp(self):
        _reset_rospy_state()

    def _controller_at(self, theta=0.0, **params_kwargs):
        publisher = FakePublisher()
        pose = _odom_pose(yaw=theta)
        controller = MotionController(
            cmd_vel_publisher=publisher,
            pose_provider=lambda: pose,
            params=MotionParameters(**params_kwargs),
        )
        controller._publisher = publisher
        return controller

    def test_heading_within_tolerance_returns_true_and_publishes_zero(self):
        controller = self._controller_at(theta=0.18, heading_tolerance=0.2)

        arrived = controller.turn_towards(0.0)

        self.assertTrue(arrived)
        self.assertEqual(controller._publisher.published[-1].linear.x, 0.0)
        self.assertEqual(controller._publisher.published[-1].angular.z, 0.0)

    def test_counterclockwise_heading_error_publishes_positive_angular_velocity(self):
        controller = self._controller_at(theta=0.0, angular_gain=1.0, angular_speed=1.5)

        arrived = controller.turn_towards(1.0)

        self.assertFalse(arrived)
        self.assertEqual(controller._publisher.published[-1].linear.x, 0.0)
        self.assertGreater(controller._publisher.published[-1].angular.z, 0.0)

    def test_clockwise_heading_error_publishes_negative_angular_velocity(self):
        controller = self._controller_at(theta=1.0, angular_gain=1.0, angular_speed=1.5)

        arrived = controller.turn_towards(0.0)

        self.assertFalse(arrived)
        self.assertEqual(controller._publisher.published[-1].linear.x, 0.0)
        self.assertLess(controller._publisher.published[-1].angular.z, 0.0)

    def test_large_heading_error_clamps_angular_velocity(self):
        controller = self._controller_at(theta=0.0, angular_gain=10.0, angular_speed=1.5)

        arrived = controller.turn_towards(math.pi / 2.0)

        self.assertFalse(arrived)
        self.assertEqual(controller._publisher.published[-1].angular.z, 1.5)

    def test_wrap_around_heading_uses_shortest_angular_direction(self):
        controller = self._controller_at(
            theta=math.radians(179.0),
            angular_gain=1.0,
            angular_speed=1.5,
            heading_tolerance=math.radians(1.0),
        )

        arrived = controller.turn_towards(math.radians(-179.0))

        self.assertFalse(arrived)
        self.assertGreater(controller._publisher.published[-1].angular.z, 0.0)
        self.assertLess(controller._publisher.published[-1].angular.z, math.radians(3.0))


class PathFollowerTest(unittest.TestCase):
    """Tests for PathFollower's closed-loop follow/cancel behavior."""

    def setUp(self):
        _reset_rospy_state()

    def test_follow_returns_true_after_all_waypoints_reached(self):
        controller = FakeMotionController(return_value=True)
        follower = PathFollower(controller)

        result = follower.follow([Node(id=1, x=1.0, y=0.0), Node(id=2, x=2.0, y=0.0)])

        self.assertTrue(result)
        self.assertEqual(len(controller.calls), 2)

    def test_follow_ticks_until_waypoint_is_reached(self):
        sys.modules['rospy'].max_sleep_count = 1000
        tick_count = [0]

        class CountingController:
            def drive_towards(self, target):
                tick_count[0] += 1
                return tick_count[0] >= 3

        follower = PathFollower(CountingController())
        result = follower.follow([Node(id=1, x=1.0, y=0.0)])

        self.assertTrue(result)
        self.assertEqual(tick_count[0], 3)

    def test_cancel_stops_follow_and_returns_false(self):
        follower = PathFollower(FakeMotionController(return_value=False))
        sys.modules['rospy'].sleep_callbacks.append(lambda: follower.cancel())

        result = follower.follow([Node(id=1, x=1.0, y=0.0)])

        self.assertFalse(result)

    def test_current_index_reflects_active_waypoint(self):
        observed = []
        tick_count = [0]

        class IndexRecorder:
            def drive_towards(self, target):
                observed.append(follower.current_index)
                tick_count[0] += 1
                return tick_count[0] in (1, 2)

        follower = PathFollower(IndexRecorder())
        follower.follow([Node(id=1, x=1.0, y=0.0), Node(id=2, x=2.0, y=0.0)])

        self.assertEqual(observed[0], 0)
        self.assertEqual(observed[-1], 1)


    def test_follow_pauses_when_pause_check_returns_true(self):
        """PathFollower calls stop and does not call drive_towards when pause_check is blocked."""
        controller = FakeMotionController(return_value=False)
        follower = PathFollower(controller)
        blocked = [True]
        follower.set_pause_check(lambda: blocked[0])

        tick = [0]

        def unblock():
            tick[0] += 1
            if tick[0] >= 2:
                blocked[0] = False
                controller.return_value = True

        sys.modules['rospy'].sleep_callbacks.append(unblock)

        result = follower.follow([Node(id=1, x=1.0, y=0.0)])

        self.assertTrue(result)
        self.assertTrue(controller.stopped)

    def test_follow_proceeds_normally_when_pause_check_is_none(self):
        """PathFollower follows waypoints normally when no pause_check is set."""
        controller = FakeMotionController(return_value=True)
        follower = PathFollower(controller)

        result = follower.follow([Node(id=1, x=1.0, y=0.0)])

        self.assertTrue(result)
        self.assertEqual(len(controller.calls), 1)


if __name__ == '__main__':
    unittest.main()
