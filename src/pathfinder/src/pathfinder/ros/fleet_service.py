from __future__ import annotations

import json
import rospy

from pathfinder.planning.path_orchestrator import PathOrchestrator, NodeNotFoundError, NoPathError
from pathfinder.ros.path_follow_action_client import PathFollowActionClient
from pathfinder.ros.robot_command_action_client import RobotCommandActionClient
from pathfinder.srv import MoveToNode, MoveToNodeRequest, MoveToNodeResponse, CancelPath, CancelPathRequest, CancelPathResponse, RotateRobot, RotateRobotRequest, RotateRobotResponse, GetGraph, GetGraphRequest, GetGraphResponse, GetRobots, GetRobotsRequest, GetRobotsResponse  # type: ignore[import]
from pathfinder.world.graph import Graph
from pathfinder.world.robot import Robot


class FleetService:
    """Service handler for fleet operations: graph queries, robot queries, path dispatch, and cancellation."""

    MOVE_SERVICE_NAME = '/fleet/move_to_node'
    CANCEL_SERVICE_NAME = '/fleet/cancel_path'
    ROTATE_SERVICE_NAME = '/fleet/rotate_robot'
    GRAPH_SERVICE_NAME = '/fleet/get_graph'
    ROBOTS_SERVICE_NAME = '/fleet/get_robots'

    def __init__(
        self,
        graph: Graph,
        orchestrator: PathOrchestrator,
        robots: list[Robot],
        clients: list[PathFollowActionClient],
        command_clients: list[RobotCommandActionClient],
    ) -> None:
        """Store injected graph, planning, and dispatch components."""
        self._graph = graph
        self._orchestrator = orchestrator
        self._robots = {robot.id: robot for robot in robots}
        self._clients = {client.robot_id: client for client in clients}
        self._command_clients = {client.robot_id: client for client in command_clients}

    def start(self) -> None:
        """Register all four fleet services."""
        rospy.Service(self.MOVE_SERVICE_NAME, MoveToNode, self._handle_move)
        rospy.Service(self.CANCEL_SERVICE_NAME, CancelPath, self._handle_cancel)
        rospy.Service(self.ROTATE_SERVICE_NAME, RotateRobot, self._handle_rotate)
        rospy.Service(self.GRAPH_SERVICE_NAME, GetGraph, self._handle_get_graph)
        rospy.Service(self.ROBOTS_SERVICE_NAME, GetRobots, self._handle_get_robots)
        rospy.loginfo("FleetService started.")

    def _handle_move(self, request: MoveToNodeRequest) -> MoveToNodeResponse:
        """Plan a path and dispatch it to the robot's FollowPath action server."""
        robot_id = request.robot_id
        if robot_id not in self._clients:
            return MoveToNodeResponse(success=False, message=f"unknown robot: {robot_id}")

        robot = self._robots[robot_id]
        if robot.pose is None:
            return MoveToNodeResponse(success=False, message=f"no pose for {robot_id}")

        try:
            node_ids = self._orchestrator.plan(
                robot.pose,
                request.target_node_id,
                current_edge=robot.current_edge,
                obstacle_blocked=robot.obstacle_blocked,
            )
        except (NodeNotFoundError, NoPathError) as error:
            return MoveToNodeResponse(success=False, message=str(error))

        self._command_clients[robot_id].cancel()
        self._clients[robot_id].cancel()
        ok = self._clients[robot_id].send(node_ids)

        if not ok:
            return MoveToNodeResponse(success=False, message="follow server unreachable")

        return MoveToNodeResponse(success=True, message=f"dispatched {len(node_ids)} waypoints")

    def _handle_cancel(self, request: CancelPathRequest) -> CancelPathResponse:
        """Cancel any in-flight FollowPath goal for the given robot."""
        robot_id = request.robot_id
        if robot_id not in self._clients:
            return CancelPathResponse(success=False, message=f"unknown robot: {robot_id}")

        self._command_clients[robot_id].cancel()
        self._clients[robot_id].cancel()

        return CancelPathResponse(success=True, message="canceled")

    def _handle_rotate(self, request: RotateRobotRequest) -> RotateRobotResponse:
        """Dispatch an absolute in-place rotation command for an idle robot."""
        robot_id = request.robot_id
        if robot_id not in self._command_clients or robot_id not in self._clients:
            return RotateRobotResponse(success=False, message=f"unknown robot: {robot_id}")

        if self._clients[robot_id].is_active():
            return RotateRobotResponse(success=False, message=f"{robot_id} is moving")

        self._command_clients[robot_id].cancel()
        ok = self._command_clients[robot_id].send('turn_to', request.target_theta)
        if not ok:
            return RotateRobotResponse(success=False, message="command server unreachable")

        return RotateRobotResponse(success=True, message="rotation dispatched")

    def _handle_get_graph(self, request: GetGraphRequest) -> GetGraphResponse:
        """Return graph nodes and edges as a JSON string."""
        payload = {
            "nodes": [
                {"id": node.id, "x": node.x, "y": node.y, "orientation": node.orientation}
                for node in self._graph.all_nodes()
            ],
            "edges": [{"from": edge.from_node.id, "to": edge.to_node.id} for edge in self._graph.all_edges()],
        }
        return GetGraphResponse(graph_json=json.dumps(payload))

    def _handle_get_robots(self, request: GetRobotsRequest) -> GetRobotsResponse:
        """Return robot ids, namespaces, and per-robot config as a JSON string."""
        payload = {"robots": [robot.to_dict() for robot in self._robots.values()]}
        return GetRobotsResponse(robots_json=json.dumps(payload))
