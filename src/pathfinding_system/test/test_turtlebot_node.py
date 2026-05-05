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

    class RobotCommandAction:
        pass

    class RobotCommandResult:
        def __init__(self, success=False, message=''):
            self.success = success
            self.message = message

    pathfinding_system_msg.RobotState = RobotStateMsg
    pathfinding_system_msg.FollowPathAction = FollowPathAction
    pathfinding_system_msg.FollowPathResult = FollowPathResult
    pathfinding_system_msg.FollowPathFeedback = FollowPathFeedback
    pathfinding_system_msg.RobotCommandAction = RobotCommandAction
    pathfinding_system_msg.RobotCommandResult = RobotCommandResult
    sys.modules['pathfinding_system.msg'] = pathfinding_system_msg


_install_ros_stubs()

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


class FakePathFollower:
    """Controllable fake PathFollower for action server tests."""

    def __init__(self, return_value=True):
        self.return_value = return_value
        self.followed_waypoints = None
        self.current_index = 0
        self.cancelled = False

    def follow(self, waypoints):
        self.followed_waypoints = waypoints
        return self.return_value

    def cancel(self):
        self.cancelled = True


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
        node = TurtleBotNode('tb3_0')

        import rospy
        self.assertEqual(node.topic_stop, '/tb3_0/stop')
        self.assertEqual([pub.topic for pub in rospy.publishers], ['/tb3_0/cmd_vel'])
        self.assertEqual([sub.topic for sub in rospy.subscribers], [
            '/tb3_0/odom',
            '/tb3_0/stop',
        ])
        self.assertEqual(rospy.timers, [])

    def test_constructor_wires_odom_subscriber_to_robot_update_pose(self):
        node = TurtleBotNode('tb3_0')

        import rospy
        robot = node._robot
        self.assertIs(rospy.subscribers[0].callback.__self__, robot)
        self.assertEqual(rospy.subscribers[0].callback.__func__.__name__, 'update_pose')

    def test_constructor_can_use_separate_robot_io_namespace(self):
        node = TurtleBotNode('tb3_0', namespace='tb3_0/sim')

        import rospy
        self.assertEqual(node.topic_odom, '/tb3_0/sim/odom')
        self.assertEqual([sub.topic for sub in rospy.subscribers], [
            '/tb3_0/sim/odom',
            '/tb3_0/stop',
        ])

    def test_constructor_uses_namespace_for_robot_io_topics(self):
        node = TurtleBotNode('tb3_0', namespace='tb3_0/custom')

        import rospy
        self.assertEqual(node.topic_odom, '/tb3_0/custom/odom')
        self.assertEqual([sub.topic for sub in rospy.subscribers], [
            '/tb3_0/custom/odom',
            '/tb3_0/stop',
        ])

    def test_constructor_does_not_publish_robot_state(self):
        node = TurtleBotNode('tb3_0')

        import rospy
        self.assertFalse(hasattr(node, 'state_publisher'))
        self.assertNotIn('/tb3_0/robot_state', [pub.topic for pub in rospy.publishers])
        self.assertEqual(rospy.timers, [])

    def test_odom_callback_updates_robot_pose_state(self):
        node = TurtleBotNode('tb3_0')

        import rospy
        rospy.subscribers[0].callback(_odom_msg(yaw=0.5))

        robot = node._robot
        self.assertEqual(robot._state.pose.x, 1.0)
        self.assertEqual(robot._state.pose.y, 2.0)
        self.assertAlmostEqual(robot._state.pose.theta, 0.5)
        self.assertEqual(robot._state.velocity.linear.x, 0.1)

    def test_stop_topic_requests_stop_and_publishes_zero_twist(self):
        node = TurtleBotNode('tb3_0')

        import rospy
        rospy.subscribers[1].callback(object())

        cmd = rospy.publishers[0].published[0]
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(cmd.angular.z, 0.0)

    def test_start_creates_follow_path_action_server(self):
        node = TurtleBotNode('tb3_0', graph=FakeGraph())

        node.start()

        import actionlib
        self.assertEqual(
            [server.name for server in actionlib.action_servers],
            ['/tb3_0/user_command', '/tb3_0/follow_path'],
        )
        self.assertTrue(all(server.started for server in actionlib.action_servers))

    def test_follow_path_action_completes_and_succeeds(self):
        node = TurtleBotNode('tb3_0', graph=FakeGraph())
        fake_follower = FakePathFollower(return_value=True)
        node._robot._path_follower = fake_follower
        node.start()

        import actionlib
        server = actionlib.action_servers[1]
        server.execute_cb(types.SimpleNamespace(node_ids=[1]))

        self.assertTrue(server.succeeded.success)
        self.assertEqual(server.succeeded.message, 'reached goal')

    def test_follow_path_action_preempt_marks_robot_idle(self):
        node = TurtleBotNode('tb3_0', graph=FakeGraph())
        fake_follower = FakePathFollower(return_value=True)
        node._robot._path_follower = fake_follower
        node.start()

        import actionlib
        server = actionlib.action_servers[1]
        server.preempt_requested = True
        server.execute_cb(types.SimpleNamespace(node_ids=[2]))

        self.assertTrue(server.preempted)

    def test_user_command_action_calls_robot_command(self):
        node = TurtleBotNode('tb3_0', graph=FakeGraph())
        calls = []
        node._robot.turn_left = lambda value: calls.append(('turn_left', value)) or True
        node.start()

        import actionlib
        server = actionlib.action_servers[0]
        server.execute_cb(types.SimpleNamespace(command='turn_left', value=1.5))

        self.assertEqual(calls, [('turn_left', 1.5)])
        self.assertTrue(server.succeeded.success)
        self.assertEqual(server.succeeded.message, 'turn_left completed')

    def test_user_command_action_rejects_unknown_command(self):
        node = TurtleBotNode('tb3_0', graph=FakeGraph())
        node.start()

        import actionlib
        server = actionlib.action_servers[0]
        server.execute_cb(types.SimpleNamespace(command='spin', value=1.5))

        self.assertFalse(server.aborted.success)
        self.assertEqual(server.aborted.message, 'unknown command: spin')


if __name__ == '__main__':
    unittest.main()
