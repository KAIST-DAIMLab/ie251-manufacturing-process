from __future__ import annotations
import math
import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.service_proxies = []
    rospy.loginfo = lambda msg: None
    rospy.wait_for_service = lambda name: None

    class ServiceProxy:
        def __init__(self, name, service_type):
            self.name = name
            self.calls = []
            rospy.service_proxies.append(self)

        def __call__(self, *args):
            self.calls.append(args)
            return types.SimpleNamespace(success=True, message='ok')

    rospy.ServiceProxy = ServiceProxy
    sys.modules['rospy'] = rospy

    actionlib = types.ModuleType('actionlib')
    actionlib.clients = []

    class SimpleActionClient:
        def __init__(self, name, action_type):
            self.name = name
            self.goal = None
            self.result = types.SimpleNamespace(success=True, message='ok')
            actionlib.clients.append(self)

        def wait_for_server(self):
            return True

        def send_goal(self, goal, feedback_cb=None):
            self.goal = goal

        def wait_for_result(self):
            return True

        def get_result(self):
            return self.result

    actionlib.SimpleActionClient = SimpleActionClient
    sys.modules['actionlib'] = actionlib

    pathfinder_msg = types.ModuleType('pathfinder.msg')

    class RobotCommandAction:
        pass

    class RobotCommandGoal:
        def __init__(self):
            self.command = ''
            self.value = 0.0

    pathfinder_msg.RobotCommandAction = RobotCommandAction
    pathfinder_msg.RobotCommandGoal = RobotCommandGoal
    sys.modules['pathfinder.msg'] = pathfinder_msg

    pathfinder_srv = types.ModuleType('pathfinder.srv')

    class MoveToNode:
        pass

    class CancelPath:
        pass

    pathfinder_srv.MoveToNode = MoveToNode
    pathfinder_srv.CancelPath = CancelPath
    sys.modules['pathfinder.srv'] = pathfinder_srv


_install_ros_stubs()

import rospy
import actionlib
from pathfinder.client.client import Client


class ClientTest(unittest.TestCase):
    def setUp(self):
        rospy.service_proxies[:] = []
        actionlib.clients[:] = []

    def test_send_goal_calls_move_service_with_correct_args(self):
        client = Client()

        ok = client.send_goal('tb3_0', 5)

        move_proxy = next(p for p in rospy.service_proxies if p.name == '/path_server/move_to_node')
        self.assertTrue(ok)
        self.assertEqual(len(move_proxy.calls), 1)
        self.assertEqual(move_proxy.calls[0], ('tb3_0', 5))

    def test_cancel_calls_cancel_service_with_robot_id(self):
        client = Client()

        client.cancel('tb3_0')

        cancel_proxy = next(p for p in rospy.service_proxies if p.name == '/path_server/cancel_path')
        self.assertEqual(len(cancel_proxy.calls), 1)
        self.assertEqual(cancel_proxy.calls[0], ('tb3_0',))

    def test_turn_commands_convert_degrees_to_radians(self):
        client = Client()

        ok = client.send_command('tb3_0', 'turn_left', 90.0)

        self.assertTrue(ok)
        self.assertEqual(actionlib.clients[-1].name, '/tb3_0/user_command')
        self.assertEqual(actionlib.clients[-1].goal.command, 'turn_left')
        self.assertAlmostEqual(actionlib.clients[-1].goal.value, math.pi / 2.0)

    def test_move_commands_keep_meter_value(self):
        client = Client()

        ok = client.send_command('tb3_0', 'move_backward', 0.5)

        self.assertTrue(ok)
        self.assertEqual(actionlib.clients[-1].name, '/tb3_0/user_command')
        self.assertEqual(actionlib.clients[-1].goal.command, 'move_backward')
        self.assertEqual(actionlib.clients[-1].goal.value, 0.5)


if __name__ == '__main__':
    unittest.main()
