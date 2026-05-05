import math
import os
import sys
import threading
import time
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


from pathfinding_system.robot.motion_controller import (
    MotionController,
    MotionParameters,
)
from pathfinding_system.robot.path_follower import PathFollower
from pathfinding_system.world.node import Node


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.publishers = []
    rospy.subscribers = []
    rospy.sleep_callbacks = []
    rospy.sleep_count = 0
    rospy.max_sleep_count = 20
    rospy.shutdown = False

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

    class Rate:
        def __init__(self, hz):
            self.hz = hz

        def sleep(self):
            rospy.sleep_count += 1
            if rospy.sleep_count > rospy.max_sleep_count:
                raise AssertionError('movement primitive did not finish')
            for callback in list(rospy.sleep_callbacks):
                callback()
            time.sleep(0.001)

    rospy.Publisher = Publisher
    rospy.Subscriber = subscriber
    rospy.Rate = Rate
    rospy.is_shutdown = lambda: rospy.shutdown
    sys.modules['rospy'] = rospy

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    geometry_msgs_msg.Twist = Twist
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs_msg = types.ModuleType('nav_msgs.msg')
    nav_msgs_msg.Odometry = object
    sys.modules['nav_msgs'] = nav_msgs
    sys.modules['nav_msgs.msg'] = nav_msgs_msg


def _odom_msg(x=0.0, y=0.0, yaw=0.0):
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


class MotionTest(unittest.TestCase):
    def setUp(self):
        self._ros_modules = {
            name: sys.modules.get(name)
            for name in (
                'rospy',
                'geometry_msgs',
                'geometry_msgs.msg',
                'nav_msgs',
                'nav_msgs.msg',
            )
        }
        _install_ros_stubs()

    def tearDown(self):
        for name, module in self._ros_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    def test_arrival_within_tolerance_returns_zero_velocities(self):
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        controller = MotionController(MotionParameters(arrival_tolerance=0.10))

        result = controller.drive_towards(pose, Node(id=1, x=0.05, y=0.0))

        self.assertTrue(result.arrived)
        self.assertEqual(result.linear_x, 0.0)
        self.assertEqual(result.angular_z, 0.0)

    def test_heading_error_above_tolerance_blocks_forward_velocity(self):
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=math.pi / 2.0)
        controller = MotionController(MotionParameters(heading_tolerance=0.2))

        result = controller.drive_towards(pose, Node(id=1, x=1.0, y=0.0))

        self.assertFalse(result.arrived)
        self.assertEqual(result.linear_x, 0.0)
        self.assertLess(result.angular_z, 0.0)

    def test_large_distance_clamps_linear_velocity(self):
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        controller = MotionController(MotionParameters(linear_gain=2.0, max_linear_velocity=0.3))

        result = controller.drive_towards(pose, Node(id=1, x=10.0, y=0.0))

        self.assertFalse(result.arrived)
        self.assertEqual(result.linear_x, 0.3)

    def test_large_heading_error_clamps_angular_velocity(self):
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        controller = MotionController(MotionParameters(angular_gain=10.0, max_angular_velocity=1.5))

        result = controller.drive_towards(pose, Node(id=1, x=0.0, y=1.0))

        self.assertFalse(result.arrived)
        self.assertEqual(result.angular_z, 1.5)

    def test_wrap_around_heading_uses_shortest_angular_direction(self):
        pose = types.SimpleNamespace(x=0.0, y=0.0, theta=math.radians(179.0))
        controller = MotionController(MotionParameters(angular_gain=1.0, max_angular_velocity=1.5))

        result = controller.drive_towards(pose, Node(id=1, x=-1.0, y=-0.01))

        self.assertFalse(result.arrived)
        self.assertGreater(result.angular_z, 0.0)
        self.assertLess(result.angular_z, math.radians(2.0))

    def test_motion_controller_clamps_linear_velocity(self):
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

    def test_ros_constructor_creates_cmd_vel_publisher_and_odom_subscriber(self):
        MotionController(cmd_vel_topic='/tb3_0/cmd_vel', odom_topic='/tb3_0/odom')

        import rospy
        self.assertEqual([pub.topic for pub in rospy.publishers], ['/tb3_0/cmd_vel'])
        self.assertEqual([sub.topic for sub in rospy.subscribers], ['/tb3_0/odom'])

    def test_set_speed_methods_control_published_primitive_velocities(self):
        controller = MotionController(cmd_vel_topic='/cmd_vel', odom_topic='/odom')
        import rospy
        rospy.subscribers[0].callback(_odom_msg(x=0.0))
        rospy.sleep_callbacks.append(lambda: rospy.subscribers[0].callback(_odom_msg(x=0.5)))

        controller.SetLinearSpeed(0.11)
        self.assertTrue(controller.MoveTowards(0.2))

        first_cmd = rospy.publishers[0].published[0]
        self.assertEqual(first_cmd.linear.x, 0.11)

        rospy.publishers[0].published[:] = []
        rospy.sleep_callbacks[:] = []
        rospy.sleep_count = 0
        rospy.subscribers[0].callback(_odom_msg(yaw=0.0))
        rospy.sleep_callbacks.append(lambda: rospy.subscribers[0].callback(_odom_msg(yaw=0.5)))

        controller.SetAngularSpeed(0.33)
        self.assertTrue(controller.turnLeft(0.2))

        first_cmd = rospy.publishers[0].published[0]
        self.assertEqual(first_cmd.angular.z, 0.33)

    def test_move_towards_publishes_positive_linear_velocity_until_distance_reached(self):
        controller = MotionController(cmd_vel_topic='/cmd_vel', odom_topic='/odom')
        import rospy
        rospy.subscribers[0].callback(_odom_msg(x=0.0))
        rospy.sleep_callbacks.append(lambda: rospy.subscribers[0].callback(_odom_msg(x=0.3)))

        result = controller.MoveTowards(0.2)

        self.assertTrue(result)
        self.assertGreater(rospy.publishers[0].published[0].linear.x, 0.0)
        self.assertEqual(rospy.publishers[0].published[-1].linear.x, 0.0)
        self.assertEqual(rospy.publishers[0].published[-1].angular.z, 0.0)

    def test_move_backwards_publishes_negative_linear_velocity_until_distance_reached(self):
        controller = MotionController(cmd_vel_topic='/cmd_vel', odom_topic='/odom')
        import rospy
        rospy.subscribers[0].callback(_odom_msg(x=0.0))
        rospy.sleep_callbacks.append(lambda: rospy.subscribers[0].callback(_odom_msg(x=-0.3)))

        result = controller.MoveBackwards(0.2)

        self.assertTrue(result)
        self.assertLess(rospy.publishers[0].published[0].linear.x, 0.0)
        self.assertEqual(rospy.publishers[0].published[-1].linear.x, 0.0)

    def test_turn_left_publishes_positive_angular_velocity_until_angle_reached(self):
        controller = MotionController(cmd_vel_topic='/cmd_vel', odom_topic='/odom')
        import rospy
        rospy.subscribers[0].callback(_odom_msg(yaw=0.0))
        rospy.sleep_callbacks.append(lambda: rospy.subscribers[0].callback(_odom_msg(yaw=0.4)))

        result = controller.turnLeft(0.2)

        self.assertTrue(result)
        self.assertGreater(rospy.publishers[0].published[0].angular.z, 0.0)
        self.assertEqual(rospy.publishers[0].published[-1].angular.z, 0.0)

    def test_turn_right_publishes_negative_angular_velocity_until_angle_reached(self):
        controller = MotionController(cmd_vel_topic='/cmd_vel', odom_topic='/odom')
        import rospy
        rospy.subscribers[0].callback(_odom_msg(yaw=0.0))
        rospy.sleep_callbacks.append(lambda: rospy.subscribers[0].callback(_odom_msg(yaw=-0.4)))

        result = controller.turnRight(0.2)

        self.assertTrue(result)
        self.assertLess(rospy.publishers[0].published[0].angular.z, 0.0)
        self.assertEqual(rospy.publishers[0].published[-1].angular.z, 0.0)

    def test_stop_interrupts_blocking_movement_and_publishes_zero_velocity(self):
        controller = MotionController(cmd_vel_topic='/cmd_vel', odom_topic='/odom')
        import rospy
        rospy.max_sleep_count = 1000
        rospy.subscribers[0].callback(_odom_msg(x=0.0))
        result = []

        thread = threading.Thread(target=lambda: result.append(controller.MoveTowards(10.0)))
        thread.start()
        while not rospy.publishers[0].published:
            time.sleep(0.001)

        controller.stop()
        thread.join(timeout=1.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(result, [False])
        self.assertEqual(rospy.publishers[0].published[-1].linear.x, 0.0)
        self.assertEqual(rospy.publishers[0].published[-1].angular.z, 0.0)


if __name__ == '__main__':
    unittest.main()
