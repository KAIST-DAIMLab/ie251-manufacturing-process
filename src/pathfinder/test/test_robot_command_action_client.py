from __future__ import annotations
import os
import sys
import types
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.Duration = lambda seconds: seconds
    sys.modules['rospy'] = rospy

    actionlib = types.ModuleType('actionlib')

    class _FakeSimpleActionClient:
        def __init__(self, *args, **kwargs):
            pass

    actionlib.SimpleActionClient = _FakeSimpleActionClient
    sys.modules['actionlib'] = actionlib

    actionlib_msgs = types.ModuleType('actionlib_msgs')
    actionlib_msgs_msg = types.ModuleType('actionlib_msgs.msg')
    actionlib_msgs_msg.GoalStatus = types.SimpleNamespace(
        PENDING=0,
        ACTIVE=1,
        SUCCEEDED=3,
        PREEMPTING=6,
        RECALLING=7,
        LOST=9,
    )
    sys.modules['actionlib_msgs'] = actionlib_msgs
    sys.modules['actionlib_msgs.msg'] = actionlib_msgs_msg

    pathfinder_msg = types.ModuleType('pathfinder.msg')

    class RobotCommandGoal:
        def __init__(self):
            self.command = ''
            self.value = 0.0

    class RobotCommandAction:
        pass

    pathfinder_msg.RobotCommandGoal = RobotCommandGoal
    pathfinder_msg.RobotCommandAction = RobotCommandAction
    sys.modules['pathfinder.msg'] = pathfinder_msg


_install_ros_stubs()

from pathfinder.ros.robot_command_action_client import RobotCommandActionClient


class FakeActionClient:
    """Controllable stub for actionlib.SimpleActionClient."""

    def __init__(self, server_available=True):
        self._server_available = server_available
        self.sent_goal = None
        self.canceled = False
        self.waited_for_result = False
        self.state = 9  # LOST — no goal sent yet

    def wait_for_server(self, timeout):
        return self._server_available

    def send_goal(self, goal):
        self.sent_goal = goal

    def cancel_goal(self):
        self.canceled = True

    def wait_for_result(self, timeout):
        self.waited_for_result = True
        return True

    def get_state(self):
        return self.state


def _make_client_with_fake(fake_action_client):
    client = RobotCommandActionClient.__new__(RobotCommandActionClient)
    client._client = fake_action_client
    return client


class TestRobotCommandActionClient(unittest.TestCase):
    def test_send_dispatches_goal_with_command_and_value(self):
        fake = FakeActionClient(server_available=True)
        client = _make_client_with_fake(fake)

        result = client.send('turn_to', 1.5)

        self.assertTrue(result)
        self.assertEqual(fake.sent_goal.command, 'turn_to')
        self.assertEqual(fake.sent_goal.value, 1.5)

    def test_send_returns_false_when_server_unavailable(self):
        fake = FakeActionClient(server_available=False)
        client = _make_client_with_fake(fake)

        result = client.send('turn_to', 0.0)

        self.assertFalse(result)
        self.assertIsNone(fake.sent_goal)

    def test_cancel_is_noop_when_goal_already_terminal(self):
        fake = FakeActionClient()
        fake.state = 3  # SUCCEEDED
        client = _make_client_with_fake(fake)

        client.cancel()

        self.assertFalse(fake.canceled)
        self.assertFalse(fake.waited_for_result)

    def test_cancel_is_noop_when_no_goal_ever_sent(self):
        fake = FakeActionClient()
        fake.state = 9  # LOST
        client = _make_client_with_fake(fake)

        client.cancel()

        self.assertFalse(fake.canceled)

    def test_cancel_dispatches_and_waits_when_goal_active(self):
        fake = FakeActionClient()
        fake.state = 1  # ACTIVE
        client = _make_client_with_fake(fake)

        client.cancel()

        self.assertTrue(fake.canceled)
        self.assertTrue(fake.waited_for_result)

    def test_is_active_reflects_active_action_state(self):
        fake = FakeActionClient()
        fake.state = 1
        client = _make_client_with_fake(fake)

        self.assertTrue(client.is_active())


if __name__ == '__main__':
    unittest.main()
