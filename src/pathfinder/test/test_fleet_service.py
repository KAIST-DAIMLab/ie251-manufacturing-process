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

    class GetGraph:
        pass

    class GetGraphRequest:
        pass

    class GetGraphResponse:
        def __init__(self, graph_json=''):
            self.graph_json = graph_json

    class GetRobots:
        pass

    class GetRobotsRequest:
        pass

    class GetRobotsResponse:
        def __init__(self, robots_json=''):
            self.robots_json = robots_json

    pathfinder_srv.MoveToNode = MoveToNode
    pathfinder_srv.MoveToNodeRequest = MoveToNodeRequest
    pathfinder_srv.MoveToNodeResponse = MoveToNodeResponse
    pathfinder_srv.CancelPath = CancelPath
    pathfinder_srv.CancelPathRequest = CancelPathRequest
    pathfinder_srv.CancelPathResponse = CancelPathResponse
    pathfinder_srv.GetGraph = GetGraph
    pathfinder_srv.GetGraphRequest = GetGraphRequest
    pathfinder_srv.GetGraphResponse = GetGraphResponse
    pathfinder_srv.GetRobots = GetRobots
    pathfinder_srv.GetRobotsRequest = GetRobotsRequest
    pathfinder_srv.GetRobotsResponse = GetRobotsResponse
    sys.modules['pathfinder.srv'] = pathfinder_srv


_install_ros_stubs()

from pathfinder.ros.fleet_service import FleetService
from pathfinder.planning.path_orchestrator import PathOrchestrator, NoPathError, NodeNotFoundError
from pathfinder.srv import MoveToNodeRequest, MoveToNodeResponse, CancelPathRequest, CancelPathResponse
from pathfinder.world.graph import Graph
from pathfinder.world.node import Node
from pathfinder.world.edge import Edge


class FakeOrchestrator:
    """Returns a fixed node_ids list or raises a given exception."""

    def __init__(self, node_ids=None, raises=None):
        self._node_ids = node_ids or [1, 2, 3]
        self._raises = raises
        self.calls = []

    def plan(self, pose, target_node_id):
        self.calls.append((pose, target_node_id))
        if self._raises is not None:
            raise self._raises
        return self._node_ids


class FakePathFollowActionClient:
    """Records send and cancel calls."""

    def __init__(self, robot_id='tb3_0', send_returns=True):
        self.robot_id = robot_id
        self._send_returns = send_returns
        self.sent_node_ids = None
        self.cancel_called = False

    def send(self, node_ids):
        self.sent_node_ids = node_ids
        return self._send_returns

    def cancel(self):
        self.cancel_called = True


_DEFAULT_POSE = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)

_FAKE_GRAPH = Graph(
    nodes=[Node(id=1, x=0.0, y=0.0), Node(id=2, x=1.0, y=0.0)],
    edges=[Edge(Node(id=1, x=0.0, y=0.0), Node(id=2, x=1.0, y=0.0))],
)


class FleetServiceTest(unittest.TestCase):
    def _make_service(self, robot_id='tb3_0', send_returns=True, pose=_DEFAULT_POSE):
        robot = types.SimpleNamespace(id=robot_id, namespace=robot_id, pose=pose)
        orchestrator = FakeOrchestrator(node_ids=[1, 2, 3])
        client = FakePathFollowActionClient(robot_id=robot_id, send_returns=send_returns)
        service = FleetService(
            graph=_FAKE_GRAPH,
            orchestrator=orchestrator,
            robots=[robot],
            clients=[client],
        )
        return service, orchestrator, client

    def test_handle_move_calls_orchestrator_and_client(self):
        service, orchestrator, client = self._make_service()
        request = MoveToNodeRequest(robot_id='tb3_0', target_node_id=3)

        response = service._handle_move(request)

        self.assertTrue(response.success)
        self.assertEqual(orchestrator.calls[0][1], 3)
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

    def test_handle_move_aborts_when_robot_has_no_pose(self):
        service, _, client = self._make_service(pose=None)

        response = service._handle_move(MoveToNodeRequest(robot_id='tb3_0', target_node_id=3))

        self.assertFalse(response.success)
        self.assertIn('no pose', response.message)
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

    def test_handle_get_graph_returns_valid_json_with_nodes_and_edges(self):
        import json
        service, _, _ = self._make_service()

        response = service._handle_get_graph(None)

        payload = json.loads(response.graph_json)
        self.assertIn('nodes', payload)
        self.assertIn('edges', payload)
        self.assertEqual(len(payload['nodes']), 2)
        self.assertEqual(len(payload['edges']), 1)
        node_ids = {node['id'] for node in payload['nodes']}
        self.assertEqual(node_ids, {1, 2})
        edge = payload['edges'][0]
        self.assertIn('from', edge)
        self.assertIn('to', edge)

    def test_handle_get_robots_returns_valid_json_with_robot_id_and_namespace(self):
        import json
        service, _, _ = self._make_service(robot_id='tb3_0')

        response = service._handle_get_robots(None)

        payload = json.loads(response.robots_json)
        self.assertIn('robots', payload)
        self.assertEqual(len(payload['robots']), 1)
        robot = payload['robots'][0]
        self.assertEqual(robot['id'], 'tb3_0')
        self.assertEqual(robot['namespace'], 'tb3_0')


if __name__ == '__main__':
    unittest.main()
