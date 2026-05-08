import os
import sys
import threading
import time
import types
import unittest
import math


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.Duration = lambda seconds: seconds
    rospy.Time = types.SimpleNamespace(now=lambda: 0)
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
    nav_msgs_msg.Odometry = object
    sys.modules['nav_msgs'] = nav_msgs
    sys.modules['nav_msgs.msg'] = nav_msgs_msg

    actionlib = types.ModuleType('actionlib')
    actionlib.ActionServer = object
    actionlib.SimpleActionClient = object
    sys.modules['actionlib'] = actionlib

    actionlib_msgs = types.ModuleType('actionlib_msgs')
    actionlib_msgs_msg = types.ModuleType('actionlib_msgs.msg')
    actionlib_msgs_msg.GoalStatus = types.SimpleNamespace(
        PENDING=0,
        ACTIVE=1,
        PREEMPTING=6,
        RECALLING=7,
    )
    sys.modules['actionlib_msgs'] = actionlib_msgs
    sys.modules['actionlib_msgs.msg'] = actionlib_msgs_msg

    pathfinder_msg = types.ModuleType('pathfinder.msg')

    class MoveToNodeResult:
        def __init__(self):
            self.success = False
            self.message = ''

    pathfinder_msg.RobotState = object
    pathfinder_msg.MoveToNodeResult = MoveToNodeResult
    sys.modules['pathfinder.msg'] = pathfinder_msg


_install_ros_stubs()

from pathfinder.planning.path_server import PathServer
from pathfinder.world.node import Node


class FakeGraph:
    def __init__(self):
        self._nodes = [
            Node(id=0, x=0.0, y=0.0),
            Node(id=1, x=10.0, y=0.0),
        ]

    def all_nodes(self):
        return self._nodes


class FakeGoalHandle:
    def __init__(self):
        self.aborted_message = None

    def set_aborted(self, result):
        self.aborted_message = result.message


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
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=linear_x),
                angular=types.SimpleNamespace(z=0.0),
            )
        ),
    )


class PathServerTest(unittest.TestCase):
    def test_waits_briefly_for_first_odom_before_resolving_start_node(self):
        server = PathServer(
            FakeGraph(),
            planner=None,
            monitor=None,
            robot_namespaces=['tb3_0'],
        )
        goal_handle = FakeGoalHandle()

        def publish_odom():
            time.sleep(0.05)
            server._on_odom('tb3_0', _odom_msg(x=9.5, y=0.0))

        thread = threading.Thread(target=publish_odom)
        thread.start()

        node = server._resolve_start_node('tb3_0', goal_handle)
        thread.join()

        self.assertIsNotNone(node)
        self.assertEqual(node.id, 1)
        self.assertIsNone(goal_handle.aborted_message)


if __name__ == '__main__':
    unittest.main()
