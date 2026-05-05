import math
import os
import sys
import types
import unittest
from typing import get_type_hints


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.Time = types.SimpleNamespace(now=lambda: 0)
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
    rospy.is_shutdown = lambda: False
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

    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs_msg = types.ModuleType('nav_msgs.msg')

    class Odometry:
        pass

    nav_msgs_msg.Odometry = Odometry
    sys.modules['nav_msgs'] = nav_msgs
    sys.modules['nav_msgs.msg'] = nav_msgs_msg

    class RobotStateMsg:
        pass

    pathfinding_system_msg = types.ModuleType('pathfinding_system.msg')
    pathfinding_system_msg.RobotState = RobotStateMsg
    sys.modules['pathfinding_system.msg'] = pathfinding_system_msg


_install_ros_stubs()

from pathfinding_system.robot.motion_controller import MotionController
from pathfinding_system.robot.path_follower import PathFollower
from pathfinding_system.robot.robot_mode import RobotMode
from pathfinding_system.robot.robot_state import RobotState
from pathfinding_system.robot.turtlebot import TurtleBot


class FakePublisher:
    """Fake cmd_vel publisher that records published messages."""

    def __init__(self):
        self.published = []

    def publish(self, message):
        """Record published message."""
        self.published.append(message)


def _odom_msg(x=1.0, y=2.0, yaw=0.5, linear_x=0.1, stamp='stamp'):
    half = yaw / 2.0
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=stamp),
        pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=x, y=y),
                orientation=types.SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(half),
                    w=math.cos(half),
                ),
            )
        ),
        twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=linear_x))
        ),
    )


class TurtleBotTest(unittest.TestCase):
    def setUp(self):
        import rospy

        def unexpected_ros_lifecycle(*args, **kwargs):
            raise AssertionError('TurtleBot must not create ROS publishers, subscribers, or timers')

        rospy.Publisher = unexpected_ros_lifecycle
        rospy.Subscriber = unexpected_ros_lifecycle
        rospy.Timer = unexpected_ros_lifecycle

    def test_constructing_turtlebot_creates_no_ros_lifecycle_objects(self):
        robot = TurtleBot('tb3_0', cmd_vel_publisher=FakePublisher())

        self.assertEqual(robot.id, 'tb3_0')

    def test_robot_state_defaults_require_only_robot_id(self):
        state = RobotState(id='tb3_0')

        self.assertEqual(state.id, 'tb3_0')
        self.assertEqual(state.pose.x, 0.0)
        self.assertEqual(state.velocity.linear.x, 0.0)
        self.assertEqual(state.status, RobotMode.IDLE)
        self.assertFalse(hasattr(state, 'stamp'))

    def test_robot_state_can_be_created_from_odometry(self):
        state = RobotState.from_odometry('tb3_0', _odom_msg(x=1.5, y=2.5, yaw=0.75, linear_x=0.4))

        self.assertEqual(state.id, 'tb3_0')
        self.assertEqual(state.pose.x, 1.5)
        self.assertEqual(state.pose.y, 2.5)
        self.assertAlmostEqual(state.pose.theta, 0.75)
        self.assertEqual(state.velocity.linear.x, 0.4)

    def test_update_pose_is_typed_for_odometry_messages(self):
        from nav_msgs.msg import Odometry

        hints = get_type_hints(TurtleBot.update_pose)

        self.assertIs(hints['msg'], Odometry)
        self.assertIs(hints['return'], type(None))

    def test_turtlebot_exposes_motion_controller(self):
        robot = TurtleBot('tb3_0', cmd_vel_publisher=FakePublisher())

        self.assertIsInstance(robot.motion_controller, MotionController)

    def test_turtlebot_exposes_path_follower(self):
        robot = TurtleBot('tb3_0', cmd_vel_publisher=FakePublisher())

        self.assertIsInstance(robot.path_follower, PathFollower)

    def test_update_state_camel_case_api_is_removed(self):
        robot = TurtleBot('tb3_0', cmd_vel_publisher=FakePublisher())

        self.assertFalse(hasattr(robot, 'updateState'))

    def test_turtlebot_does_not_create_lock(self):
        robot = TurtleBot('tb3_0', cmd_vel_publisher=FakePublisher())

        self.assertFalse(hasattr(robot, '_lock'))

    def test_turtlebot_does_not_define_robot_state_topic(self):
        robot = TurtleBot('tb3_0', cmd_vel_publisher=FakePublisher())

        self.assertFalse(hasattr(robot, 'state_topic'))

    def test_turtlebot_does_not_define_ros_io_topics(self):
        robot = TurtleBot('tb3_0', cmd_vel_publisher=FakePublisher())

        self.assertFalse(hasattr(robot, 'cmd_vel_topic'))
        self.assertFalse(hasattr(robot, 'odom_topic'))

    def test_update_pose_updates_state_from_odometry_message(self):
        robot = TurtleBot('tb3_0', cmd_vel_publisher=FakePublisher())

        robot.update_pose(_odom_msg(x=1.5, y=2.5, yaw=0.75, linear_x=0.4, stamp='odom-stamp'))

        self.assertEqual(robot._state.pose.x, 1.5)
        self.assertEqual(robot._state.pose.y, 2.5)
        self.assertAlmostEqual(robot._state.pose.theta, 0.75)
        self.assertEqual(robot._state.velocity.linear.x, 0.4)

if __name__ == '__main__':
    unittest.main()
