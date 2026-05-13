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

    class FollowPathAction:
        pass

    class FollowPathGoal:
        def __init__(self):
            self.node_ids = []

    class RobotCommandAction:
        pass

    class RobotCommandGoal:
        def __init__(self):
            self.command = ''
            self.value = 0.0

    pathfinder_msg.FollowPathAction = FollowPathAction
    pathfinder_msg.FollowPathGoal = FollowPathGoal
    pathfinder_msg.RobotCommandAction = RobotCommandAction
    pathfinder_msg.RobotCommandGoal = RobotCommandGoal
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

    class RotateRobot:
        pass

    class RotateRobotRequest:
        def __init__(self, robot_id='', target_theta=0.0):
            self.robot_id = robot_id
            self.target_theta = target_theta

    class RotateRobotResponse:
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
    pathfinder_srv.RotateRobot = RotateRobot
    pathfinder_srv.RotateRobotRequest = RotateRobotRequest
    pathfinder_srv.RotateRobotResponse = RotateRobotResponse
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
from pathfinder.srv import MoveToNodeRequest, MoveToNodeResponse, CancelPathRequest, CancelPathResponse, RotateRobotRequest
from pathfinder.world.graph import Graph
from pathfinder.world.node import Node
from pathfinder.world.edge import Edge


class FakeOrchestrator:
    """Returns a fixed node_ids list or raises a given exception."""

    def __init__(self, node_ids=None, raises=None):
        self._node_ids = node_ids or [1, 2, 3]
        self._raises = raises
        self.calls = []

    def plan(self, pose, target_node_id, current_edge=None, obstacle_blocked=False):
        self.calls.append((pose, target_node_id, current_edge, obstacle_blocked))
        if self._raises is not None:
            raise self._raises
        return self._node_ids


class FakePathFollowActionClient:
    """Records send and cancel calls."""

    def __init__(self, robot_id='tb3_0', send_returns=True, active=False):
        self.robot_id = robot_id
        self._send_returns = send_returns
        self.active = active
        self.sent_node_ids = None
        self.cancel_called = False

    def send(self, node_ids):
        self.sent_node_ids = node_ids
        return self._send_returns

    def cancel(self):
        self.cancel_called = True

    def is_active(self):
        return self.active


class FakeRobotCommandActionClient:
    """Records primitive motion command sends and cancels."""

    def __init__(self, robot_id='tb3_0', send_returns=True):
        self.robot_id = robot_id
        self._send_returns = send_returns
        self.sent_command = None
        self.sent_value = None
        self.cancel_called = False

    def send(self, command, value):
        self.sent_command = command
        self.sent_value = value
        return self._send_returns

    def cancel(self):
        self.cancel_called = True


_DEFAULT_POSE = types.SimpleNamespace(x=0.0, y=0.0, theta=0.0)

_FAKE_GRAPH = Graph(
    nodes=[Node(id=1, x=0.0, y=0.0, orientation=90.0, station=1), Node(id=2, x=1.0, y=0.0)],
    edges=[Edge(Node(id=1, x=0.0, y=0.0), Node(id=2, x=1.0, y=0.0))],
)


class FleetServiceTest(unittest.TestCase):
    def _make_service(self, robot_id='tb3_0', send_returns=True, pose=_DEFAULT_POSE, path_active=False, command_returns=True):
        motion = types.SimpleNamespace(linear_speed=0.22, angular_speed=1.5, move_rate_hz=5.0, arrival_tolerance=0.05)
        obstacle = types.SimpleNamespace(enabled=True, stop_distance=0.4, detect_degree=20)
        robot = types.SimpleNamespace(
            id=robot_id, namespace=robot_id, pose=pose, start_node=1,
            motion=motion, obstacle=obstacle, obstacle_blocked=False, current_edge=None,
            to_dict=lambda: {
                "id": robot_id, "namespace": robot_id, "start_node": 1,
                "motion": {"linear_speed": 0.22, "angular_speed": 1.5, "move_rate_hz": 5.0, "arrival_tolerance": 0.05},
                "obstacle": {"enabled": True, "stop_distance": 0.4, "detect_degree": 20},
            },
        )
        orchestrator = FakeOrchestrator(node_ids=[1, 2, 3])
        client = FakePathFollowActionClient(robot_id=robot_id, send_returns=send_returns, active=path_active)
        command_client = FakeRobotCommandActionClient(robot_id=robot_id, send_returns=command_returns)
        service = FleetService(
            graph=_FAKE_GRAPH,
            orchestrator=orchestrator,
            robots=[robot],
            clients=[client],
            command_clients=[command_client],
        )
        return service, orchestrator, client, command_client

    def test_handle_move_calls_orchestrator_and_client(self):
        service, orchestrator, client, _ = self._make_service()
        request = MoveToNodeRequest(robot_id='tb3_0', target_node_id=3)

        response = service._handle_move(request)

        self.assertTrue(response.success)
        self.assertEqual(orchestrator.calls[0][1], 3)
        self.assertEqual(client.sent_node_ids, [1, 2, 3])

    def test_handle_move_auto_preempts_before_send(self):
        service, _, client, _ = self._make_service()
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
        service, _, client, _ = self._make_service(pose=None)

        response = service._handle_move(MoveToNodeRequest(robot_id='tb3_0', target_node_id=3))

        self.assertFalse(response.success)
        self.assertIn('no pose', response.message)
        self.assertIsNone(client.sent_node_ids)

    def test_handle_move_aborts_on_no_path_error(self):
        service, orchestrator, client, _ = self._make_service()
        orchestrator._raises = NoPathError("no path exists")

        response = service._handle_move(MoveToNodeRequest(robot_id='tb3_0', target_node_id=3))

        self.assertFalse(response.success)
        self.assertIn('no path exists', response.message)
        self.assertIsNone(client.sent_node_ids)

    def test_handle_move_aborts_on_node_not_found_error(self):
        service, orchestrator, client, _ = self._make_service()
        orchestrator._raises = NodeNotFoundError("node 99 not found")

        response = service._handle_move(MoveToNodeRequest(robot_id='tb3_0', target_node_id=99))

        self.assertFalse(response.success)
        self.assertIsNone(client.sent_node_ids)

    def test_handle_move_rejects_unknown_robot_id(self):
        service, _, _, _ = self._make_service()

        response = service._handle_move(MoveToNodeRequest(robot_id='unknown_bot', target_node_id=3))

        self.assertFalse(response.success)
        self.assertIn('unknown robot', response.message)

    def test_handle_move_fails_when_send_returns_false(self):
        service, _, client, _ = self._make_service(send_returns=False)

        response = service._handle_move(MoveToNodeRequest(robot_id='tb3_0', target_node_id=3))

        self.assertFalse(response.success)
        self.assertIn('unreachable', response.message)

    def test_handle_cancel_calls_client_cancel(self):
        service, _, client, command_client = self._make_service()

        response = service._handle_cancel(CancelPathRequest(robot_id='tb3_0'))

        self.assertTrue(response.success)
        self.assertTrue(client.cancel_called)
        self.assertTrue(command_client.cancel_called)

    def test_handle_cancel_rejects_unknown_robot(self):
        service, _, _, _ = self._make_service()

        response = service._handle_cancel(CancelPathRequest(robot_id='ghost_bot'))

        self.assertFalse(response.success)
        self.assertIn('unknown robot', response.message)

    def test_handle_get_graph_returns_valid_json_with_nodes_and_edges(self):
        import json
        service, _, _, _ = self._make_service()

        response = service._handle_get_graph(None)

        payload = json.loads(response.graph_json)
        self.assertIn('nodes', payload)
        self.assertIn('edges', payload)
        self.assertEqual(len(payload['nodes']), 2)
        self.assertEqual(len(payload['edges']), 1)
        node_ids = {node['id'] for node in payload['nodes']}
        self.assertEqual(node_ids, {1, 2})
        node_by_id = {node['id']: node for node in payload['nodes']}
        self.assertEqual(node_by_id[1]['orientation'], 90.0)
        self.assertIsNone(node_by_id[2]['orientation'])
        self.assertEqual(node_by_id[1]['station'], 1)
        self.assertIsNone(node_by_id[2]['station'])
        edge = payload['edges'][0]
        self.assertIn('from', edge)
        self.assertIn('to', edge)

    def test_handle_get_robots_returns_valid_json_with_robot_id_and_namespace(self):
        import json
        service, _, _, _ = self._make_service(robot_id='tb3_0')

        response = service._handle_get_robots(None)

        payload = json.loads(response.robots_json)
        self.assertIn('robots', payload)
        self.assertEqual(len(payload['robots']), 1)
        robot = payload['robots'][0]
        self.assertEqual(robot['id'], 'tb3_0')
        self.assertEqual(robot['namespace'], 'tb3_0')
        self.assertEqual(robot['start_node'], 1)
        self.assertNotIn('yaw', robot)
        self.assertIn('motion', robot)
        self.assertEqual(robot['motion']['linear_speed'], 0.22)
        self.assertEqual(robot['motion']['arrival_tolerance'], 0.05)
        self.assertIn('obstacle', robot)
        self.assertTrue(robot['obstacle']['enabled'])
        self.assertEqual(robot['obstacle']['stop_distance'], 0.4)
        self.assertEqual(robot['obstacle']['detect_degree'], 20)

    def test_handle_rotate_sends_absolute_turn_command(self):
        service, _, _, command_client = self._make_service()

        response = service._handle_rotate(RotateRobotRequest(robot_id='tb3_0', target_theta=1.25))

        self.assertTrue(response.success)
        self.assertEqual(command_client.sent_command, 'turn_to')
        self.assertEqual(command_client.sent_value, 1.25)

    def test_handle_rotate_rejects_unknown_robot_id(self):
        service, _, _, command_client = self._make_service()

        response = service._handle_rotate(RotateRobotRequest(robot_id='unknown_bot', target_theta=1.25))

        self.assertFalse(response.success)
        self.assertIn('unknown robot', response.message)
        self.assertIsNone(command_client.sent_command)

    def test_handle_rotate_rejects_active_path(self):
        service, _, _, command_client = self._make_service(path_active=True)

        response = service._handle_rotate(RotateRobotRequest(robot_id='tb3_0', target_theta=1.25))

        self.assertFalse(response.success)
        self.assertIn('moving', response.message)
        self.assertIsNone(command_client.sent_command)

    def test_handle_rotate_fails_when_command_server_unreachable(self):
        service, _, _, command_client = self._make_service(command_returns=False)

        response = service._handle_rotate(RotateRobotRequest(robot_id='tb3_0', target_theta=1.25))

        self.assertFalse(response.success)
        self.assertIn('unreachable', response.message)
        self.assertEqual(command_client.sent_command, 'turn_to')


if __name__ == '__main__':
    unittest.main()
