import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.Duration = lambda seconds: seconds
    rospy.Time = types.SimpleNamespace(now=lambda: 0)
    rospy.Subscriber = lambda *args, **kwargs: ('subscriber', args, kwargs)
    sys.modules['rospy'] = rospy

    actionlib = types.ModuleType('actionlib')
    class UnexpectedSimpleActionClient:
        def __init__(self, *args, **kwargs):
            raise AssertionError('TurtleBot must not depend on move_base at startup')

    actionlib.SimpleActionClient = UnexpectedSimpleActionClient
    sys.modules['actionlib'] = actionlib

    actionlib_msgs = types.ModuleType('actionlib_msgs')
    actionlib_msgs_msg = types.ModuleType('actionlib_msgs.msg')
    actionlib_msgs_msg.GoalStatus = types.SimpleNamespace(
        PENDING=0, ACTIVE=1, PREEMPTED=2, SUCCEEDED=3, ABORTED=4
    )
    sys.modules['actionlib_msgs'] = actionlib_msgs
    sys.modules['actionlib_msgs.msg'] = actionlib_msgs_msg

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
    nav_msgs_msg.Odometry = object
    sys.modules['nav_msgs'] = nav_msgs
    sys.modules['nav_msgs.msg'] = nav_msgs_msg

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.Empty = object
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = std_msgs_msg

    pathfinding_system_msg = types.ModuleType('pathfinding_system.msg')
    pathfinding_system_msg.RobotState = object
    sys.modules['pathfinding_system.msg'] = pathfinding_system_msg


_install_ros_stubs()

from pathfinding_system.robot.turtlebot import MotionParameters, TurtleBot
import pathfinding_system.robot.turtlebot as turtlebot_module
from pathfinding_system.robot.robot_state import RobotState
from pathfinding_system.robot.robot_status import RobotStatus
from pathfinding_system.world.node import Node


class TurtleBotTest(unittest.TestCase):
    def setUp(self):
        self.published_by_topic = {}
        self.subscriptions = []
        self.timers = []

        def publisher(topic, msg_type, queue_size=10):
            published = []
            self.published_by_topic[topic] = published
            return types.SimpleNamespace(publish=published.append)

        def subscriber(topic, msg_type, callback):
            self.subscriptions.append((topic, msg_type, callback))
            return types.SimpleNamespace()

        def timer(duration, callback):
            self.timers.append((duration, callback))
            return types.SimpleNamespace()

        import rospy
        rospy.Publisher = publisher
        rospy.Subscriber = subscriber
        rospy.Timer = timer
        turtlebot_module.rospy.Publisher = publisher
        turtlebot_module.rospy.Timer = timer
        turtlebot_module.rospy.Subscriber = subscriber

    def test_uses_namespaced_cmd_vel_topic(self):
        robot = TurtleBot('tb3_0')

        self.assertEqual(robot.cmd_vel_topic, '/tb3_0/cmd_vel')

    def test_does_not_create_ros_topic_wiring(self):
        TurtleBot('tb3_0')

        self.assertEqual(self.subscriptions, [])
        self.assertEqual(self.published_by_topic, {})
        self.assertEqual(self.timers, [])

    def test_robot_state_defaults_require_only_robot_id(self):
        state = RobotState(id='tb3_0')

        self.assertEqual(state.id, 'tb3_0')
        self.assertEqual(state.pose.x, 0.0)
        self.assertEqual(state.velocity.linear.x, 0.0)
        self.assertEqual(state.status, RobotStatus.IDLE)
        self.assertIsNone(state.stamp)

    def test_drive_towards_waypoint_returns_forward_velocity(self):
        robot = TurtleBot('tb3_0')

        result = robot.drive_towards(Node(id=2, x=1.0, y=0.0))

        self.assertFalse(result.arrived)
        self.assertGreater(result.velocity_command.linear.x, 0.0)
        self.assertEqual(result.velocity_command.angular.z, 0.0)

    def test_drive_towards_uses_readable_motion_parameters(self):
        robot = TurtleBot(
            'tb3_0',
            motion_parameters=MotionParameters(
                linear_gain=2.0,
                angular_gain=1.5,
                max_linear_velocity=0.3,
                max_angular_velocity=1.5,
                arrival_tolerance=0.1,
                heading_tolerance=0.2,
            ),
        )

        result = robot.drive_towards(Node(id=2, x=1.0, y=0.0))

        self.assertFalse(result.arrived)
        self.assertEqual(result.velocity_command.linear.x, 0.3)


if __name__ == '__main__':
    unittest.main()
