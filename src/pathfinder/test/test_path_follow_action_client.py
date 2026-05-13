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
        PREEMPTING=6,
        RECALLING=7,
    )
    sys.modules['actionlib_msgs'] = actionlib_msgs
    sys.modules['actionlib_msgs.msg'] = actionlib_msgs_msg

    pathfinder_msg = types.ModuleType('pathfinder.msg')

    class FollowPathGoal:
        def __init__(self):
            self.node_ids = []

    class FollowPathAction:
        pass

    pathfinder_msg.FollowPathGoal = FollowPathGoal
    pathfinder_msg.FollowPathAction = FollowPathAction
    sys.modules['pathfinder.msg'] = pathfinder_msg


_install_ros_stubs()

from pathfinder.ros.path_follow_action_client import PathFollowActionClient


class FakeActionClient:
    """Controllable stub for actionlib.SimpleActionClient."""

    def __init__(self, server_available=True):
        self._server_available = server_available
        self.sent_goal = None
        self.canceled = False
        self.state = 3

    def wait_for_server(self, timeout):
        return self._server_available

    def send_goal(self, goal):
        self.sent_goal = goal

    def cancel_goal(self):
        self.canceled = True

    def get_state(self):
        return self.state


def _make_client_with_fake(fake_action_client):
    """Return a PathFollowActionClient whose internal actionlib client is replaced by the fake."""
    client = PathFollowActionClient.__new__(PathFollowActionClient)
    client._client = fake_action_client
    return client


class TestPathFollowActionClient(unittest.TestCase):
    def test_send_returns_true_and_dispatches_goal_when_server_available(self):
        fake = FakeActionClient(server_available=True)
        client = _make_client_with_fake(fake)

        result = client.send([1, 2, 3])

        self.assertTrue(result)
        self.assertIsNotNone(fake.sent_goal)
        self.assertEqual(fake.sent_goal.node_ids, [1, 2, 3])

    def test_send_returns_false_when_server_unavailable(self):
        fake = FakeActionClient(server_available=False)
        client = _make_client_with_fake(fake)

        result = client.send([1, 2])

        self.assertFalse(result)
        self.assertIsNone(fake.sent_goal)

    def test_cancel_delegates_to_underlying_client(self):
        fake = FakeActionClient()
        client = _make_client_with_fake(fake)

        client.cancel()

        self.assertTrue(fake.canceled)

    def test_is_active_reflects_active_action_state(self):
        fake = FakeActionClient()
        fake.state = 1
        client = _make_client_with_fake(fake)

        self.assertTrue(client.is_active())


if __name__ == '__main__':
    unittest.main()
