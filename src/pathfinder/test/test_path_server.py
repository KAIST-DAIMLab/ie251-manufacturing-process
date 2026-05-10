from __future__ import annotations
import os
import sys
import types
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.loginfo = lambda message: None
    rospy.logwarn = lambda message: None
    rospy.Service = object
    sys.modules['rospy'] = rospy

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')

    class Pose2D:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0

    geometry_msgs_msg.Pose2D = Pose2D
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs_msg = types.ModuleType('nav_msgs.msg')
    nav_msgs_msg.Odometry = object
    sys.modules['nav_msgs'] = nav_msgs
    sys.modules['nav_msgs.msg'] = nav_msgs_msg

    actionlib = types.ModuleType('actionlib')
    actionlib.SimpleActionClient = object
    sys.modules['actionlib'] = actionlib

    pathfinder_msg = types.ModuleType('pathfinder.msg')

    class FollowPathAction:
        pass

    class FollowPathGoal:
        def __init__(self):
            self.node_ids = []

    pathfinder_msg.FollowPathAction = FollowPathAction
    pathfinder_msg.FollowPathGoal = FollowPathGoal
    sys.modules['pathfinder.msg'] = pathfinder_msg

    pathfinder_srv = types.ModuleType('pathfinder.srv')

    class MoveToNode:
        pass

    class MoveToNodeRequest:
        def __init__(self, robot_id='', target_node_id=0):
            self.robot_id = robot_id
            self.target_node_id = target_node_id

    class MoveToNodeResponse:
        def __init__(self, success=False, message=''):
            self.success = success
            self.message = message

    class CancelPath:
        pass

    class CancelPathRequest:
        def __init__(self, robot_id=''):
            self.robot_id = robot_id

    class CancelPathResponse:
        def __init__(self, success=False, message=''):
            self.success = success
            self.message = message

    pathfinder_srv.MoveToNode = MoveToNode
    pathfinder_srv.MoveToNodeRequest = MoveToNodeRequest
    pathfinder_srv.MoveToNodeResponse = MoveToNodeResponse
    pathfinder_srv.CancelPath = CancelPath
    pathfinder_srv.CancelPathRequest = CancelPathRequest
    pathfinder_srv.CancelPathResponse = CancelPathResponse
    sys.modules['pathfinder.srv'] = pathfinder_srv


_install_ros_stubs()

from pathfinder.ros.path_request_service import PathRequestService
from pathfinder.planning.path_orchestrator import PathOrchestrator, UnknownRobotError, NoPathError, NodeNotFoundError
from pathfinder.ros.pose_tracker import PoseTracker
from pathfinder.srv import MoveToNodeRequest, MoveToNodeResponse, CancelPathRequest, CancelPathResponse


class FakeOrchestrator:
    """Returns a fixed node_ids list or raises a given exception."""

    def __init__(self, node_ids=None, raises=None):
        self._node_ids = node_ids or [1, 2, 3]
        self._raises = raises
        self.calls = []

    def plan(self, robot_id, pose, target_node_id):
        self.calls.append((robot_id, pose, target_node_id))
        if self._raises is not None:
            raise self._raises
        return self._node_ids


class FakePathFollowActionClient:
    """Records send and cancel calls."""

    def __init__(self, send_returns=True):
        self._send_returns = send_returns
        self.sent_node_ids = None
        self.cancel_called = False

    def send(self, node_ids):
        self.sent_node_ids = node_ids
        return self._send_returns

    def cancel(self):
        self.cancel_called = True


class PathRequestServiceTest(unittest.TestCase):
    def _make_service(self, robot_id='tb3_0', send_returns=True):
        tracker = PoseTracker(timeout_sec=1.0)
        tracker.update(robot_id, types.SimpleNamespace(x=0.0, y=0.0, theta=0.0))
        orchestrator = FakeOrchestrator(node_ids=[1, 2, 3])
        client = FakePathFollowActionClient(send_returns=send_returns)
        service = PathRequestService(
            orchestrator=orchestrator,
            tracker=tracker,
            clients={robot_id: client},
        )
        return service, orchestrator, client

    def test_handle_move_calls_orchestrator_and_client(self):
        service, orchestrator, client = self._make_service()
        request = MoveToNodeRequest(robot_id='tb3_0', target_node_id=3)

        response = service._handle_move(request)

        self.assertTrue(response.success)
        self.assertEqual(orchestrator.calls[0][0], 'tb3_0')
        self.assertEqual(orchestrator.calls[0][2], 3)
        self.assertEqual(client.sent_node_ids, [1, 2, 3])

    def test_handle_move_auto_preempts_before_send(self):
        service, _, client = self._make_service()
        call_order = []
        original_cancel = client.cancel
        original_send = client.send

        def tracking_cancel():
            call_order.append('cancel')
            original_cancel()

        def tracking_send(node_ids):
            call_order.append('send')
            return original_send(node_ids)

        client.cancel = tracking_cancel
        client.send = tracking_send

        service._handle_move(MoveToNodeRequest(robot_id='tb3_0', target_node_id=3))

        self.assertEqual(call_order, ['cancel', 'send'])

    def test_handle_move_aborts_when_tracker_returns_none(self):
        robot_id = 'tb3_0'
        tracker = PoseTracker(timeout_sec=0.01)
        orchestrator = FakeOrchestrator()
        client = FakePathFollowActionClient()
        service = PathRequestService(
            orchestrator=orchestrator,
            tracker=tracker,
            clients={robot_id: client},
        )

        response = service._handle_move(MoveToNodeRequest(robot_id=robot_id, target_node_id=3))

        self.assertFalse(response.success)
        self.assertIn('no pose', response.message)
        self.assertIsNone(client.sent_node_ids)

    def test_handle_move_aborts_on_unknown_robot_error(self):
        service, orchestrator, client = self._make_service()
        orchestrator._raises = UnknownRobotError("unknown robot: tb3_0")

        response = service._handle_move(MoveToNodeRequest(robot_id='tb3_0', target_node_id=3))

        self.assertFalse(response.success)
        self.assertIn('unknown robot', response.message)
        self.assertIsNone(client.sent_node_ids)

    def test_handle_move_aborts_on_no_path_error(self):
        service, orchestrator, client = self._make_service()
        orchestrator._raises = NoPathError("no path exists")

        response = service._handle_move(MoveToNodeRequest(robot_id='tb3_0', target_node_id=3))

        self.assertFalse(response.success)
        self.assertIn('no path exists', response.message)
        self.assertIsNone(client.sent_node_ids)

    def test_handle_move_aborts_on_node_not_found_error(self):
        service, orchestrator, client = self._make_service()
        orchestrator._raises = NodeNotFoundError("node 99 not found")

        response = service._handle_move(MoveToNodeRequest(robot_id='tb3_0', target_node_id=99))

        self.assertFalse(response.success)
        self.assertIsNone(client.sent_node_ids)

    def test_handle_move_rejects_unknown_robot_id(self):
        service, _, _ = self._make_service()

        response = service._handle_move(MoveToNodeRequest(robot_id='unknown_bot', target_node_id=3))

        self.assertFalse(response.success)
        self.assertIn('unknown robot', response.message)

    def test_handle_move_fails_when_send_returns_false(self):
        service, _, client = self._make_service(send_returns=False)

        response = service._handle_move(MoveToNodeRequest(robot_id='tb3_0', target_node_id=3))

        self.assertFalse(response.success)
        self.assertIn('unreachable', response.message)

    def test_handle_cancel_calls_client_cancel(self):
        service, _, client = self._make_service()

        response = service._handle_cancel(CancelPathRequest(robot_id='tb3_0'))

        self.assertTrue(response.success)
        self.assertTrue(client.cancel_called)

    def test_handle_cancel_rejects_unknown_robot(self):
        service, _, _ = self._make_service()

        response = service._handle_cancel(CancelPathRequest(robot_id='ghost_bot'))

        self.assertFalse(response.success)
        self.assertIn('unknown robot', response.message)


if __name__ == '__main__':
    unittest.main()
