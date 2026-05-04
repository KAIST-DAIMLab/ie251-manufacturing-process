import math
import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.publishers = []
    rospy.subscribers = []
    rospy.timers = []
    rospy.warnings = []

    class Duration:
        @staticmethod
        def from_sec(seconds):
            return seconds

    class Publisher:
        def __init__(self, topic, msg_type, queue_size=10):
            self.topic = topic
            self.msg_type = msg_type
            self.queue_size = queue_size
            self.published = []
            rospy.publishers.append(self)

        def publish(self, msg):
            self.published.append(msg)

    def subscriber(topic, msg_type, callback):
        sub = types.SimpleNamespace(topic=topic, msg_type=msg_type, callback=callback)
        rospy.subscribers.append(sub)
        return sub

    def timer(duration, callback):
        timer_obj = types.SimpleNamespace(duration=duration, callback=callback)
        rospy.timers.append(timer_obj)
        return timer_obj

    rospy.Duration = Duration
    rospy.Publisher = Publisher
    rospy.Subscriber = subscriber
    rospy.Timer = timer
    rospy.Time = types.SimpleNamespace(now=lambda: 'now')
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
    rospy.is_shutdown = lambda: False
    rospy.loginfo = lambda msg: None
    rospy.logwarn = rospy.warnings.append
    sys.modules['rospy'] = rospy

    actionlib = types.ModuleType('actionlib')
    actionlib.action_servers = []

    class SimpleActionServer:
        def __init__(self, name, action_type, execute_cb, auto_start=False):
            self.name = name
            self.action_type = action_type
            self.execute_cb = execute_cb
            self.auto_start = auto_start
            self.started = False
            self.preempt_requested = False
            self.feedback = []
            self.preempted = False
            self.aborted = None
            self.succeeded = None
            actionlib.action_servers.append(self)

        def start(self):
            self.started = True

        def is_preempt_requested(self):
            return self.preempt_requested

        def publish_feedback(self, feedback):
            self.feedback.append(feedback)

        def set_preempted(self):
            self.preempted = True

        def set_aborted(self, result):
            self.aborted = result

        def set_succeeded(self, result):
            self.succeeded = result

    actionlib.SimpleActionServer = SimpleActionServer
    sys.modules['actionlib'] = actionlib

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

    class RobotStateMsg:
        pass

    class FollowPathAction:
        pass

    class FollowPathResult:
        def __init__(self, success=False, message=''):
            self.success = success
            self.message = message

    class FollowPathFeedback:
        def __init__(self):
            self.current_index = None
            self.current_pose = None

    pathfinding_system_msg.RobotState = RobotStateMsg
    pathfinding_system_msg.FollowPathAction = FollowPathAction
    pathfinding_system_msg.FollowPathResult = FollowPathResult
    pathfinding_system_msg.FollowPathFeedback = FollowPathFeedback
    sys.modules['pathfinding_system.msg'] = pathfinding_system_msg


_install_ros_stubs()

from pathfinding_system.robot.motion_controller import DriveResult
from pathfinding_system.robot.robot_status import RobotStatus
from pathfinding_system.robot.turtlebot import TurtleBot
from pathfinding_system.robot.turtlebot_node import TurtleBotNode
from pathfinding_system.world.node import Node


class FakeGraph:
    def __init__(self):
        self.nodes = {
            1: Node(id=1, x=0.0, y=0.0),
            2: Node(id=2, x=1.0, y=0.0),
        }

    def get_node(self, node_id):
        return self.nodes[node_id]


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


class TurtleBotNodeTest(unittest.TestCase):
    def setUp(self):
        import rospy

        rospy.publishers[:] = []
        rospy.subscribers[:] = []
        rospy.timers[:] = []
        rospy.warnings[:] = []
        import actionlib
        actionlib.action_servers[:] = []

    def test_constructor_creates_expected_publishers_subscribers_and_timer(self):
        robot = TurtleBot('tb3_0')

        TurtleBotNode(robot, state_publish_rate_hz=10.0)

        import rospy
        self.assertEqual([pub.topic for pub in rospy.publishers], [
            '/tb3_0/cmd_vel',
            '/tb3_0/robot_state',
        ])
        self.assertEqual([sub.topic for sub in rospy.subscribers], [
            '/tb3_0/odom',
            '/tb3_0/emergency_stop',
        ])
        self.assertEqual(len(rospy.timers), 1)
        self.assertEqual(rospy.timers[0].duration, 0.1)

    def test_constructor_can_use_separate_robot_io_namespace(self):
        robot = TurtleBot('tb3_0', topic_namespace='tb3_0/sim')

        TurtleBotNode(robot, state_publish_rate_hz=10.0)

        import rospy
        self.assertEqual([pub.topic for pub in rospy.publishers], [
            '/tb3_0/sim/cmd_vel',
            '/tb3_0/robot_state',
        ])
        self.assertEqual([sub.topic for sub in rospy.subscribers], [
            '/tb3_0/sim/odom',
            '/tb3_0/emergency_stop',
        ])

    def test_invalid_publish_rate_raises_value_error(self):
        with self.assertRaises(ValueError):
            TurtleBotNode(TurtleBot('tb3_0'), state_publish_rate_hz=0.0)

    def test_timer_callback_publishes_robot_state(self):
        robot = TurtleBot('tb3_0')
        node = TurtleBotNode(robot, state_publish_rate_hz=10.0)

        import rospy
        rospy.timers[0].callback(None)

        self.assertIs(node.state_publisher.published[0].robot_id, 'tb3_0')

    def test_odom_callback_updates_robot_pose_state(self):
        robot = TurtleBot('tb3_0')
        TurtleBotNode(robot)

        import rospy
        rospy.subscribers[0].callback(_odom_msg(yaw=0.5))

        state = robot.state_snapshot()
        self.assertEqual(state.pose.x, 1.0)
        self.assertEqual(state.pose.y, 2.0)
        self.assertAlmostEqual(state.pose.theta, 0.5)
        self.assertEqual(state.velocity.linear.x, 0.1)
        self.assertEqual(state.stamp, 'stamp')

    def test_emergency_stop_requests_stop_and_publishes_zero_twist(self):
        robot = TurtleBot('tb3_0')
        node = TurtleBotNode(robot)

        import rospy
        rospy.subscribers[1].callback(object())

        self.assertTrue(robot.stop_requested())
        self.assertEqual(robot.state_snapshot().status, RobotStatus.STOPPED)
        cmd = node.cmd_vel_publisher.published[0]
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(cmd.angular.z, 0.0)

    def test_publish_drive_result_converts_scalars_to_twist(self):
        node = TurtleBotNode(TurtleBot('tb3_0'))

        node.publish_drive_result(DriveResult(arrived=False, linear_x=0.2, angular_z=-0.3))

        cmd = node.cmd_vel_publisher.published[0]
        self.assertEqual(cmd.linear.x, 0.2)
        self.assertEqual(cmd.angular.z, -0.3)

    def test_start_creates_follow_path_action_server(self):
        node = TurtleBotNode(TurtleBot('tb3_0'), graph=FakeGraph())

        node.start()

        import actionlib
        self.assertEqual(len(actionlib.action_servers), 1)
        server = actionlib.action_servers[0]
        self.assertEqual(server.name, '/tb3_0/follow_path')
        self.assertTrue(server.started)

    def test_follow_path_action_steps_robot_path_and_succeeds(self):
        robot = TurtleBot('tb3_0')
        node = TurtleBotNode(robot, graph=FakeGraph())
        node.start()

        import actionlib
        server = actionlib.action_servers[0]
        server.execute_cb(types.SimpleNamespace(node_ids=[1]))

        self.assertEqual(robot.state_snapshot().status, RobotStatus.REACHED)
        self.assertTrue(server.succeeded.success)
        self.assertEqual(server.succeeded.message, 'reached goal')
        self.assertEqual(len(node.cmd_vel_publisher.published), 2)
        self.assertEqual(node.cmd_vel_publisher.published[-1].linear.x, 0.0)

    def test_follow_path_action_preempt_cancels_robot_path(self):
        robot = TurtleBot('tb3_0')
        node = TurtleBotNode(robot, graph=FakeGraph())
        node.start()

        import actionlib
        server = actionlib.action_servers[0]
        server.preempt_requested = True
        server.execute_cb(types.SimpleNamespace(node_ids=[2]))

        self.assertTrue(server.preempted)
        self.assertEqual(robot.state_snapshot().status, RobotStatus.IDLE)
        self.assertEqual(node.cmd_vel_publisher.published[-1].linear.x, 0.0)


if __name__ == '__main__':
    unittest.main()
