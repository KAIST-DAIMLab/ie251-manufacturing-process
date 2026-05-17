import math
import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


PUBLISHERS = {}
SUBSCRIBERS = []
SERVICES = {}


class FakeTime:
    """Controllable wall-clock stub; tests advance _current to simulate elapsed time."""
    _current: float = 100.0

    def __init__(self, secs: float) -> None:
        self.secs = secs

    @classmethod
    def now(cls) -> 'FakeTime':
        return cls(cls._current)

    def to_sec(self) -> float:
        return self.secs

    def __sub__(self, other: 'FakeTime') -> 'FakeTime':
        return FakeTime(self.secs - other.secs)


def _install_ros_stubs():
    rospy = sys.modules.get('rospy') or types.ModuleType('rospy')
    rospy.Time = FakeTime
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

    def Service(name, srv_type, callback):
        SERVICES[name] = (srv_type, callback)
        return types.SimpleNamespace(shutdown=lambda: None)

    rospy.Publisher = Publisher
    rospy.Subscriber = Subscriber
    rospy.Service = Service
    rospy.Duration = lambda secs: secs
    rospy.Timer = lambda duration, callback: types.SimpleNamespace(shutdown=lambda: None)
    sys.modules['rospy'] = rospy

    tf = sys.modules.get('tf') or types.ModuleType('tf')
    tf.LookupException = Exception
    tf.ConnectivityException = Exception
    tf.ExtrapolationException = Exception
    def _fake_lookup(*_args):
        raise Exception('tf not available in tests')
    tf.TransformListener = lambda: types.SimpleNamespace(lookupTransform=_fake_lookup)
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
        def __init__(self):
            self.header = types.SimpleNamespace(frame_id='', stamp=None)
            self.pose = types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                covariance=[0.0] * 36,
            )

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

    class Int8:
        def __init__(self, data=0):
            self.data = data

    std_msgs_msg.Bool = Bool
    std_msgs_msg.String = String
    std_msgs_msg.Int8 = Int8
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = std_msgs_msg

    pathfinder_msg = sys.modules.get('pathfinder.msg') or types.ModuleType('pathfinder.msg')
    pathfinder_msg.FollowPathAction = object
    pathfinder_msg.FollowPathFeedback = object
    pathfinder_msg.FollowPathResult = lambda success=False, message='': types.SimpleNamespace(success=success, message=message)
    pathfinder_msg.RobotCommandAction = object
    pathfinder_msg.RobotCommandResult = lambda success=False, message='': types.SimpleNamespace(success=success, message=message)
    sys.modules['pathfinder.msg'] = pathfinder_msg

    pathfinder_srv = sys.modules.get('pathfinder.srv') or types.ModuleType('pathfinder.srv')
    pathfinder_srv.Relocalize = object
    pathfinder_srv.RelocalizeRequest = lambda x=0.0, y=0.0, theta=0.0: types.SimpleNamespace(x=x, y=y, theta=theta)
    pathfinder_srv.RelocalizeResponse = lambda success=False, message='': types.SimpleNamespace(success=success, message=message)
    sys.modules['pathfinder.srv'] = pathfinder_srv


_install_ros_stubs()

from pathfinder.ros.turtlebot_node import TurtleBotNode


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
        SERVICES.clear()
        FakeTime._current = 100.0

    def test_pose_publisher_is_latched_for_late_web_ui_subscribers(self):
        TurtleBotNode('tb3_01', obstacle_enabled=False)

        self.assertTrue(PUBLISHERS['/tb3_01/pose'].latch)

    def test_odom_updates_velocity_and_publishes_pose_in_real_robot_mode(self):
        node = TurtleBotNode('tb3_01', obstacle_enabled=False)

        node._on_odom(_odom_msg(x=1.2, y=-0.4, yaw=0.75, linear_x=0.33))

        self.assertAlmostEqual(node._state.velocity.linear.x, 0.33)
        published = PUBLISHERS['/tb3_01/pose'].published
        self.assertEqual(len(published), 1)
        # tf not available in tests → falls back to odom + origin (zero origin)
        self.assertAlmostEqual(published[-1].x, 1.2)
        self.assertAlmostEqual(published[-1].y, -0.4)
        self.assertAlmostEqual(published[-1].theta, 0.75)

    def test_sim_mode_publishes_odom_plus_origin(self):
        origin = types.SimpleNamespace(x=1.0, y=2.0, theta=0.25)
        node = TurtleBotNode('tb3_01', namespace='tb3_01/sim', origin=origin, odom_pose_enabled=True, obstacle_enabled=False)

        node._on_odom(_odom_msg(x=1.2, y=-0.4, yaw=0.75, linear_x=0.33))

        self.assertAlmostEqual(node._state.velocity.linear.x, 0.33)
        published = PUBLISHERS['/tb3_01/sim/pose'].published
        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(published[-1].x, 2.2)
        self.assertAlmostEqual(published[-1].y, 1.6)
        self.assertAlmostEqual(published[-1].theta, 1.0)


class TurtleBotNodeStateTopicTest(unittest.TestCase):
    def setUp(self):
        _install_ros_stubs()
        PUBLISHERS.clear()
        SUBSCRIBERS.clear()
        SERVICES.clear()
        FakeTime._current = 100.0

    def test_state_topic_publisher_is_created_and_latched(self):
        TurtleBotNode('tb3_01', obstacle_enabled=False)

        publisher = PUBLISHERS['/tb3_01/state']
        self.assertTrue(publisher.latch)
        self.assertEqual(publisher.queue_size, 1)

    def test_publish_state_writes_current_robotmode_value(self):
        from pathfinder.robot.robot_mode import RobotMode
        node = TurtleBotNode('tb3_01', obstacle_enabled=False)
        node._last_odom_received_at = FakeTime(FakeTime._current)
        node._robot.set_following(True)

        node._publish_state()

        published = PUBLISHERS['/tb3_01/state'].published
        self.assertGreaterEqual(len(published), 1)
        self.assertEqual(published[-1].data, int(RobotMode.MOVING))

    def test_on_scan_flips_state_to_obstacle_only_when_following(self):
        from pathfinder.robot.robot_mode import RobotMode
        node = TurtleBotNode('tb3_01', obstacle_enabled=True)
        node._obstacle_detector = types.SimpleNamespace(detect=lambda message: True)
        node._last_odom_received_at = FakeTime(FakeTime._current)

        # Idle: scan detecting an obstacle should NOT promote status.
        node._on_scan(types.SimpleNamespace())
        self.assertEqual(node._state.status, RobotMode.IDLE)

        # Following: scan detecting an obstacle SHOULD flip to OBSTACLE.
        node._robot.set_following(True)
        node._on_scan(types.SimpleNamespace())
        self.assertEqual(node._state.status, RobotMode.OBSTACLE)

        # Scan clearing the obstacle returns to MOVING.
        node._obstacle_detector.detect = lambda message: False
        node._on_scan(types.SimpleNamespace())
        self.assertEqual(node._state.status, RobotMode.MOVING)


class TurtleBotNodeOfflineDetectionTest(unittest.TestCase):
    def setUp(self):
        _install_ros_stubs()
        PUBLISHERS.clear()
        SUBSCRIBERS.clear()
        SERVICES.clear()
        FakeTime._current = 100.0

    def test_initial_publish_state_writes_offline_before_any_odom(self):
        from pathfinder.robot.robot_mode import RobotMode
        node = TurtleBotNode('tb3_01', obstacle_enabled=False)

        node._publish_state()

        published = PUBLISHERS['/tb3_01/state'].published
        self.assertEqual(published[-1].data, int(RobotMode.OFFLINE))

    def test_on_odom_stamps_last_received_and_publish_state_writes_idle(self):
        from pathfinder.robot.robot_mode import RobotMode
        node = TurtleBotNode('tb3_01', obstacle_enabled=False, odom_pose_enabled=True)

        node._on_odom(_odom_msg())
        node._publish_state()

        self.assertIsNotNone(node._last_odom_received_at)
        published = PUBLISHERS['/tb3_01/state'].published
        self.assertEqual(published[-1].data, int(RobotMode.IDLE))

    def test_stale_odom_causes_publish_state_to_write_offline(self):
        from pathfinder.robot.robot_mode import RobotMode
        from pathfinder.ros.turtlebot_node import ODOM_FRESHNESS_SEC
        node = TurtleBotNode('tb3_01', obstacle_enabled=False)
        node._last_odom_received_at = FakeTime(FakeTime._current - ODOM_FRESHNESS_SEC - 1.0)

        node._publish_state()

        published = PUBLISHERS['/tb3_01/state'].published
        self.assertEqual(published[-1].data, int(RobotMode.OFFLINE))


class TurtleBotNodeRelocalizeTest(unittest.TestCase):
    def setUp(self):
        _install_ros_stubs()
        PUBLISHERS.clear()
        SUBSCRIBERS.clear()
        SERVICES.clear()
        FakeTime._current = 100.0

    def test_initialpose_publisher_is_created_on_construction(self):
        TurtleBotNode('tb3_01', obstacle_enabled=False)

        self.assertIn('/tb3_01/initialpose', PUBLISHERS)

    def test_on_relocalize_publishes_one_message_with_map_frame_and_pose(self):
        node = TurtleBotNode('tb3_01', obstacle_enabled=False)
        request = types.SimpleNamespace(x=1.0, y=2.0, theta=math.pi / 2)

        response = node._on_relocalize(request)

        published = PUBLISHERS['/tb3_01/initialpose'].published
        self.assertEqual(len(published), 1)
        msg = published[0]
        self.assertEqual(msg.header.frame_id, 'map')
        self.assertAlmostEqual(msg.pose.pose.position.x, 1.0)
        self.assertAlmostEqual(msg.pose.pose.position.y, 2.0)
        self.assertAlmostEqual(msg.pose.pose.orientation.z, math.sin(math.pi / 4))
        self.assertAlmostEqual(msg.pose.pose.orientation.w, math.cos(math.pi / 4))
        self.assertTrue(response.success)

    def test_on_relocalize_sets_covariance_diagonal(self):
        from pathfinder.ros.turtlebot_node import (
            RELOCALIZE_XY_VARIANCE, RELOCALIZE_THETA_VARIANCE,
        )
        node = TurtleBotNode('tb3_01', obstacle_enabled=False)

        node._on_relocalize(types.SimpleNamespace(x=0.0, y=0.0, theta=0.0))

        msg = PUBLISHERS['/tb3_01/initialpose'].published[0]
        self.assertAlmostEqual(msg.pose.covariance[0], RELOCALIZE_XY_VARIANCE)
        self.assertAlmostEqual(msg.pose.covariance[7], RELOCALIZE_XY_VARIANCE)
        self.assertAlmostEqual(msg.pose.covariance[35], RELOCALIZE_THETA_VARIANCE)
        # Off-diagonal entries must remain zero so AMCL doesn't infer false cross-correlations.
        for i, value in enumerate(msg.pose.covariance):
            if i not in (0, 7, 35):
                self.assertEqual(value, 0.0)

    def test_start_registers_relocalize_service(self):
        node = TurtleBotNode('tb3_01', obstacle_enabled=False)
        node._motion_control_server = types.SimpleNamespace(start=lambda: None)
        node._path_follow_server = None

        node.start()

        self.assertIn('/tb3_01/relocalize', SERVICES)


if __name__ == '__main__':
    unittest.main()
