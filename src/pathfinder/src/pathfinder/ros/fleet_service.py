from __future__ import annotations

import rospy

from pathfinder.planning.path_orchestrator import PathOrchestrator, NodeNotFoundError, NoPathError
from pathfinder.ros.path_follow_action_client import PathFollowActionClient
from pathfinder.srv import MoveToNode, MoveToNodeRequest, MoveToNodeResponse, CancelPath, CancelPathRequest, CancelPathResponse  # type: ignore[import]
from pathfinder.world.robot import Robot


class FleetService:
    """Service handler for MoveToNode and CancelPath: plans routes and dispatches to PathFollowActionServer."""

    MOVE_SERVICE_NAME = '/fleet/move_to_node'
    CANCEL_SERVICE_NAME = '/fleet/cancel_path'

    def __init__(
        self,
        orchestrator: PathOrchestrator,
        robots: list[Robot],
        clients: list[PathFollowActionClient],
    ) -> None:
        """Store injected planning and dispatch components."""
        self._orchestrator = orchestrator
        self._robots = {robot.id: robot for robot in robots}
        self._clients = {client.robot_id: client for client in clients}

    def start(self) -> None:
        """Register the MoveToNode and CancelPath services."""
        rospy.Service(self.MOVE_SERVICE_NAME, MoveToNode, self._handle_move)
        rospy.Service(self.CANCEL_SERVICE_NAME, CancelPath, self._handle_cancel)
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
            node_ids = self._orchestrator.plan(robot.pose, request.target_node_id)
        except (NodeNotFoundError, NoPathError) as error:
            return MoveToNodeResponse(success=False, message=str(error))

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

        self._clients[robot_id].cancel()

        return CancelPathResponse(success=True, message="canceled")
