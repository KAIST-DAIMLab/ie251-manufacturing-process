import os
import sys
import types
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.Duration = lambda seconds: seconds
    rospy.is_shutdown = lambda: False
    sys.modules['rospy'] = rospy

    actionlib = types.ModuleType('actionlib')

    class _FakeSimpleActionClient:
        def __init__(self, *args, **kwargs):
            pass

    actionlib.SimpleActionClient = _FakeSimpleActionClient
    sys.modules['actionlib'] = actionlib

    actionlib_msgs = types.ModuleType('actionlib_msgs')
    actionlib_msgs_msg = types.ModuleType('actionlib_msgs.msg')
    actionlib_msgs_msg.GoalStatus = types.SimpleNamespace(PREEMPTING=6, RECALLING=7)
    sys.modules['actionlib_msgs'] = actionlib_msgs
    sys.modules['actionlib_msgs.msg'] = actionlib_msgs_msg

    pathfinder_msg = types.ModuleType('pathfinder.msg')

    class FollowPathGoal:
        def __init__(self):
            self.node_ids = []

    class FollowPathFeedback:
        def __init__(self, current_index=0):
            self.current_index = current_index

    class FollowPathResult:
        def __init__(self, success=True, message=''):
            self.success = success
            self.message = message

    class FollowPathAction:
        pass

    pathfinder_msg.FollowPathGoal = FollowPathGoal
    pathfinder_msg.FollowPathFeedback = FollowPathFeedback
    pathfinder_msg.FollowPathResult = FollowPathResult
    pathfinder_msg.FollowPathAction = FollowPathAction
    sys.modules['pathfinder.msg'] = pathfinder_msg


_install_ros_stubs()

from pathfinder.ros.follow_path_client import FollowPathClient  # noqa: E402


class FakeActionClient:
    """Controllable stub for actionlib.SimpleActionClient."""

    def __init__(self):
        self.sent_goal = None
        self.canceled = False
        self._server_available = True
        self._poll_results = []
        self._feedback_cb = None
        self._result = None

    def wait_for_server(self, timeout):
        return self._server_available

    def send_goal(self, goal, feedback_cb=None):
        self.sent_goal = goal
        self._feedback_cb = feedback_cb

    def wait_for_result(self, timeout):
        if self._poll_results:
            return self._poll_results.pop(0)
        return True

    def cancel_goal(self):
        self.canceled = True

    def get_result(self):
        return self._result

    def deliver_feedback(self, feedback):
        if self._feedback_cb is not None:
            self._feedback_cb(feedback)


def _make_client_with_fake(fake_action_client):
    """Return a FollowPathClient whose internal actionlib client is replaced by the fake."""
    client = FollowPathClient.__new__(FollowPathClient)
    client._client = fake_action_client
    return client


class TestFollowPathClientDispatch(unittest.TestCase):
    def test_returns_result_when_server_available_and_goal_succeeds(self):
        fake = FakeActionClient()
        from pathfinder.msg import FollowPathResult
        fake._result = FollowPathResult(success=True, message='done')
        fake._poll_results = [False, True]

        client = _make_client_with_fake(fake)
        result = client.dispatch([1, 2, 3], on_feedback=lambda fb: None, is_canceled=lambda: False)

        self.assertIsNotNone(result)
        self.assertTrue(result.success)
        self.assertEqual(result.message, 'done')

    def test_returns_none_when_server_unavailable(self):
        fake = FakeActionClient()
        fake._server_available = False

        client = _make_client_with_fake(fake)
        result = client.dispatch([1, 2], on_feedback=lambda fb: None, is_canceled=lambda: False)

        self.assertIsNone(result)
        self.assertIsNone(fake.sent_goal)

    def test_calls_on_feedback_when_feedback_arrives(self):
        fake = FakeActionClient()
        from pathfinder.msg import FollowPathResult, FollowPathFeedback
        fake._result = FollowPathResult(success=True, message='ok')

        received_feedback = []

        client = _make_client_with_fake(fake)

        poll_count = [0]

        def controlled_wait(timeout):
            poll_count[0] += 1
            if poll_count[0] == 1:
                fb = FollowPathFeedback(current_index=1)
                fake.deliver_feedback(fb)
                return False
            return True

        fake.wait_for_result = controlled_wait

        result = client.dispatch([1, 2], on_feedback=received_feedback.append, is_canceled=lambda: False)

        self.assertIsNotNone(result)
        self.assertTrue(len(received_feedback) >= 1)
        self.assertEqual(received_feedback[0].current_index, 1)

    def test_cancels_goal_and_returns_none_when_is_canceled_is_true(self):
        fake = FakeActionClient()

        poll_count = [0]

        def controlled_wait(timeout):
            poll_count[0] += 1
            return False

        fake.wait_for_result = controlled_wait

        cancel_after = [1]

        def is_canceled():
            return poll_count[0] >= cancel_after[0]

        client = _make_client_with_fake(fake)
        result = client.dispatch([1, 2], on_feedback=lambda fb: None, is_canceled=is_canceled)

        self.assertIsNone(result)
        self.assertTrue(fake.canceled)


if __name__ == '__main__':
    unittest.main()
