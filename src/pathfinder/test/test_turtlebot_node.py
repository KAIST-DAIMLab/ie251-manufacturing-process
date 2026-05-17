import math
import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


PUBLISHERS = {}
SUBSCRIBERS = []


def _install_ros_stubs():
    rospy = sys.modules.get('rospy') or types.ModuleType('rospy')
    rospy.Time = types.SimpleNamespace(now=lambda: 0)
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
    rospy.is_shutdown = lambda: False
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.logwarn = lambda *args, **kwargs: None

    class Publisher:
        def __init__(self, topic, message_type, queue_size=1, latch=False):
            self.topic = topic
            self.message_type = message_type
            self.queue_size = queue_size
            self.latch = latch
            self.published = []
            PUBLISHERS[topic] = self

        def publish(self, message):
            self.published.append(message)

    def Subscriber(topic, message_type, callback):
        SUBSCRIBERS.append((topic, message_type, callback))
        return types.SimpleNamespace(unregister=lambda: None)

    rospy.Publisher = Publisher
    rospy.Subscriber = Subscriber
    sys.modules['rospy'] = rospy

    tf = sys.modules.get('tf') or types.ModuleType('tf')
    tf.LookupException = Exception
    tf.ConnectivityException = Exception
    tf.ExtrapolationException = Exception
    tf.TransformListener = lambda: types.SimpleNamespace()
    sys.modules['tf'] = tf

    actionlib = sys.modules.get('actionlib') or types.ModuleType('actionlib')
    actionlib.SimpleActionServer = object
    sys.modules['actionlib'] = actionlib

    geometry_msgs = sys.modules.get('geometry_msgs') or types.ModuleType('geometry_msgs')
    geometry_msgs_msg = sys.modules.get('geometry_msgs.msg') or types.ModuleType('geometry_msgs.msg')

    class Pose2D:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class PoseWithCovarianceStamped:
        pass

    geometry_msgs_msg.Pose2D = Pose2D
    geometry_msgs_msg.Twist = Twist
    geometry_msgs_msg.PoseWithCovarianceStamped = PoseWithCovarianceStamped
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

    nav_msgs = sys.modules.get('nav_msgs') or types.ModuleType('nav_msgs')
    nav_msgs_msg = sys.modules.get('nav_msgs.msg') or types.ModuleType('nav_msgs.msg')
    nav_msgs_msg.Odometry = object
    sys.modules['nav_msgs'] = nav_msgs
    sys.modules['nav_msgs.msg'] = nav_msgs_msg

    sensor_msgs = sys.modules.get('sensor_msgs') or types.ModuleType('sensor_msgs')
    sensor_msgs_msg = sys.modules.get('sensor_msgs.msg') or types.ModuleType('sensor_msgs.msg')
    sensor_msgs_msg.LaserScan = object
    sys.modules['sensor_msgs'] = sensor_msgs
    sys.modules['sensor_msgs.msg'] = sensor_msgs_msg

    std_msgs = sys.modules.get('std_msgs') or types.ModuleType('std_msgs')
    std_msgs_msg = sys.modules.get('std_msgs.msg') or types.ModuleType('std_msgs.msg')

    class Bool:
        def __init__(self, data=False):
            self.data = data

    class String:
        pass

    std_msgs_msg.Bool = Bool
    std_msgs_msg.String = String
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = std_msgs_msg

    pathfinder_msg = sys.modules.get('pathfinder.msg') or types.ModuleType('pathfinder.msg')
    pathfinder_msg.FollowPathAction = object
    pathfinder_msg.FollowPathFeedback = object
    pathfinder_msg.FollowPathResult = lambda success=False, message='': types.SimpleNamespace(success=success, message=message)
    pathfinder_msg.RobotCommandAction = object
    pathfinder_msg.RobotCommandResult = lambda success=False, message='': types.SimpleNamespace(success=success, message=message)
    sys.modules['pathfinder.msg'] = pathfinder_msg


_install_ros_stubs()

from pathfinder.ros.turtlebot_node import TurtleBotNode


def _amcl_msg(x=1.0, y=2.0, yaw=0.5):
    half = yaw / 2.0
    return types.SimpleNamespace(
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
        )
    )


def _odom_msg(x=1.0, y=2.0, yaw=0.5, linear_x=0.1):
    half = yaw / 2.0
    return types.SimpleNamespace(
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


class TurtleBotNodePoseSourceTest(unittest.TestCase):
    def setUp(self):
        _install_ros_stubs()
        PUBLISHERS.clear()
        SUBSCRIBERS.clear()

    def test_amcl_pose_updates_xy_but_not_theta(self):
        node = TurtleBotNode('tb3_01', obstacle_enabled=False)
        node._state.pose.theta = 0.25

        node._on_amcl_pose(_amcl_msg(x=1.2, y=-0.4, yaw=0.75))

        self.assertAlmostEqual(node._state.pose.x, 1.2)
        self.assertAlmostEqual(node._state.pose.y, -0.4)
        self.assertAlmostEqual(node._state.pose.theta, 0.25)
        published = PUBLISHERS['/tb3_01/pose'].published
        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(published[-1].x, 1.2)
        self.assertAlmostEqual(published[-1].y, -0.4)
        self.assertAlmostEqual(published[-1].theta, 0.25)

    def test_pose_publisher_is_latched_for_late_web_ui_subscribers(self):
        TurtleBotNode('tb3_01', obstacle_enabled=False)

        self.assertTrue(PUBLISHERS['/tb3_01/pose'].latch)

    def test_odom_updates_theta_and_velocity_in_real_robot_mode(self):
        node = TurtleBotNode('tb3_01', obstacle_enabled=False)
        node._state.pose.x = 9.0
        node._state.pose.y = 8.0
        node._state.pose.theta = 0.25

        node._on_odom(_odom_msg(x=1.2, y=-0.4, yaw=0.75, linear_x=0.33))

        self.assertAlmostEqual(node._state.velocity.linear.x, 0.33)
        self.assertAlmostEqual(node._state.pose.x, 9.0)
        self.assertAlmostEqual(node._state.pose.y, 8.0)
        self.assertAlmostEqual(node._state.pose.theta, 0.75)
        published = PUBLISHERS['/tb3_01/pose'].published
        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(published[-1].theta, 0.75)

    def test_odom_pose_source_updates_state_and_publishes_pose_with_origin(self):
        origin = types.SimpleNamespace(x=1.0, y=2.0, theta=0.25)
        node = TurtleBotNode('tb3_01', namespace='tb3_01/sim', origin=origin, odom_pose_enabled=True, obstacle_enabled=False)

        node._on_odom(_odom_msg(x=1.2, y=-0.4, yaw=0.75, linear_x=0.33))

        self.assertAlmostEqual(node._state.velocity.linear.x, 0.33)
        self.assertAlmostEqual(node._state.pose.x, 2.2)
        self.assertAlmostEqual(node._state.pose.y, 1.6)
        self.assertAlmostEqual(node._state.pose.theta, 1.0)
        published = PUBLISHERS['/tb3_01/sim/pose'].published
        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(published[-1].x, 2.2)
        self.assertAlmostEqual(published[-1].y, 1.6)
        self.assertAlmostEqual(published[-1].theta, 1.0)


if __name__ == '__main__':
    unittest.main()
