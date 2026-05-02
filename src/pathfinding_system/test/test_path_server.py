import os
import sys
import threading
import time
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.Duration = lambda seconds: seconds
    rospy.Time = types.SimpleNamespace(now=lambda: 0)
    rospy.is_shutdown = lambda: False
    sys.modules['rospy'] = rospy

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

    pathfinding_system_msg = types.ModuleType('pathfinding_system.msg')

    class MoveToNodeResult:
        def __init__(self):
            self.success = False
            self.message = ''

    pathfinding_system_msg.RobotState = object
    pathfinding_system_msg.MoveToNodeResult = MoveToNodeResult
    sys.modules['pathfinding_system.msg'] = pathfinding_system_msg


_install_ros_stubs()

from pathfinding_system.planning.path_server import PathServer
from pathfinding_system.world.node import Node


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


class PathServerTest(unittest.TestCase):
    def test_waits_briefly_for_first_robot_state_before_resolving_start_node(self):
        server = PathServer(
            FakeGraph(),
            planner=None,
            monitor=None,
            robot_namespaces=['tb3_0'],
        )
        goal_handle = FakeGoalHandle()

        def publish_state():
            time.sleep(0.05)
            state = types.SimpleNamespace(
                pose=types.SimpleNamespace(x=9.5, y=0.0)
            )
            server._on_robot_state('tb3_0', state)

        thread = threading.Thread(target=publish_state)
        thread.start()

        node = server._resolve_start_node('tb3_0', goal_handle)
        thread.join()

        self.assertIsNotNone(node)
        self.assertEqual(node.id, 1)
        self.assertIsNone(goal_handle.aborted_message)


if __name__ == '__main__':
    unittest.main()
