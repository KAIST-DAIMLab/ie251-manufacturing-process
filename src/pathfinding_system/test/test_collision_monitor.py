import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.subscribers = []
    rospy.publishers = []
    rospy.timers = []

    def subscriber(topic, msg_type, callback):
        sub = types.SimpleNamespace(topic=topic, msg_type=msg_type, callback=callback)
        rospy.subscribers.append(sub)
        return sub

    class Publisher:
        def __init__(self, topic, msg_type, queue_size=10):
            self.topic = topic
            self.msg_type = msg_type
            self.queue_size = queue_size
            self.published = []
            rospy.publishers.append(self)

        def publish(self, msg):
            self.published.append(msg)

    rospy.Duration = lambda seconds: seconds
    rospy.Subscriber = subscriber
    rospy.Publisher = Publisher
    rospy.Timer = lambda duration, callback: rospy.timers.append(
        types.SimpleNamespace(duration=duration, callback=callback)
    )
    rospy.loginfo = lambda msg: None
    rospy.logwarn = lambda msg: None
    sys.modules['rospy'] = rospy

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

from pathfinding_system.safety.collision_monitor import CollisionMonitor


class FakePredictor:
    def will_collide(self, s1, s2, horizon):
        return False


class CollisionMonitorTest(unittest.TestCase):
    def setUp(self):
        import rospy
        rospy.subscribers[:] = []
        rospy.publishers[:] = []
        rospy.timers[:] = []

    def test_start_subscribes_to_odom_topics_not_robot_state_topics(self):
        monitor = CollisionMonitor(
            FakePredictor(),
            ['tb3_0', 'tb3_1'],
            horizon=2.0,
            check_rate_hz=10.0,
            robot_odom_topics={
                'tb3_0': '/tb3_0/sim/odom',
                'tb3_1': '/tb3_1/sim/odom',
            },
        )

        monitor.start()

        import rospy
        self.assertEqual([sub.topic for sub in rospy.subscribers], [
            '/tb3_0/sim/odom',
            '/tb3_1/sim/odom',
        ])
        self.assertNotIn('/tb3_0/robot_state', [sub.topic for sub in rospy.subscribers])
        self.assertEqual([pub.topic for pub in rospy.publishers], [
            '/tb3_0/stop',
            '/tb3_1/stop',
        ])


if __name__ == '__main__':
    unittest.main()
