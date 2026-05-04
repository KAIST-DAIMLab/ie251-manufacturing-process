import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.Time = types.SimpleNamespace(now=lambda: 0)
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

    class RobotStateMsg:
        pass

    pathfinding_system_msg = types.ModuleType('pathfinding_system.msg')
    pathfinding_system_msg.RobotState = RobotStateMsg
    sys.modules['pathfinding_system.msg'] = pathfinding_system_msg


_install_ros_stubs()

from pathfinding_system.robot.motion_controller import MotionParameters
from pathfinding_system.robot.turtlebot import TurtleBot
from pathfinding_system.robot.robot_state import RobotState
from pathfinding_system.robot.robot_status import RobotStatus
from pathfinding_system.world.node import Node


class TurtleBotTest(unittest.TestCase):
    def setUp(self):
        import rospy

        def unexpected_ros_lifecycle(*args, **kwargs):
            raise AssertionError('TurtleBot must not create ROS publishers, subscribers, or timers')

        rospy.Publisher = unexpected_ros_lifecycle
        rospy.Subscriber = unexpected_ros_lifecycle
        rospy.Timer = unexpected_ros_lifecycle

    def test_constructing_turtlebot_creates_no_ros_lifecycle_objects(self):
        robot = TurtleBot('tb3_0')

        self.assertEqual(robot.id, 'tb3_0')

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
        self.assertGreater(result.linear_x, 0.0)
        self.assertEqual(result.angular_z, 0.0)
        self.assertEqual(robot.state_snapshot().status, RobotStatus.MOVING)

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
        self.assertEqual(result.linear_x, 0.3)

    def test_start_path_clears_stop_and_marks_moving(self):
        robot = TurtleBot('tb3_0')
        robot.request_stop()

        robot.start_path([Node(id=1, x=1.0, y=0.0)])

        self.assertFalse(robot.stop_requested())
        self.assertEqual(robot.state_snapshot().status, RobotStatus.MOVING)

    def test_step_path_uses_owned_path_follower_and_marks_reached(self):
        robot = TurtleBot('tb3_0')
        robot.start_path([Node(id=1, x=0.0, y=0.0)])

        step = robot.step_path()

        self.assertTrue(step.completed)
        self.assertTrue(step.drive_result.arrived)
        self.assertEqual(robot.state_snapshot().status, RobotStatus.REACHED)

    def test_cancel_path_marks_robot_idle(self):
        robot = TurtleBot('tb3_0')
        robot.start_path([Node(id=1, x=1.0, y=0.0)])

        robot.cancel_path()

        self.assertEqual(robot.state_snapshot().status, RobotStatus.IDLE)

    def test_update_pose_updates_state_without_ros_message_types(self):
        robot = TurtleBot('tb3_0')
        velocity = types.SimpleNamespace(linear=types.SimpleNamespace(x=0.2))

        robot.update_pose(x=1.0, y=2.0, theta=3.0, velocity=velocity, stamp='stamp')

        state = robot.state_snapshot()
        self.assertEqual(state.pose.x, 1.0)
        self.assertEqual(state.pose.y, 2.0)
        self.assertEqual(state.pose.theta, 3.0)
        self.assertEqual(state.velocity.linear.x, 0.2)
        self.assertEqual(state.stamp, 'stamp')

    def test_request_stop_sets_stopped_status_and_stop_flag(self):
        robot = TurtleBot('tb3_0')

        robot.request_stop()

        self.assertTrue(robot.stop_requested())
        self.assertEqual(robot.state_snapshot().status, RobotStatus.STOPPED)

    def test_explicit_status_methods_do_not_clear_stop_flag(self):
        robot = TurtleBot('tb3_0')
        robot.request_stop()

        robot.mark_moving()
        self.assertTrue(robot.stop_requested())
        self.assertEqual(robot.state_snapshot().status, RobotStatus.MOVING)

        robot.mark_idle()
        self.assertTrue(robot.stop_requested())
        self.assertEqual(robot.state_snapshot().status, RobotStatus.IDLE)

        robot.mark_reached()
        self.assertTrue(robot.stop_requested())
        self.assertEqual(robot.state_snapshot().status, RobotStatus.REACHED)

    def test_clear_stop_clears_stop_flag_without_status_change(self):
        robot = TurtleBot('tb3_0')
        robot.request_stop()

        robot.clear_stop()

        self.assertFalse(robot.stop_requested())
        self.assertEqual(robot.state_snapshot().status, RobotStatus.STOPPED)


if __name__ == '__main__':
    unittest.main()
