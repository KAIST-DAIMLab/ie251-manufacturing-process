from __future__ import annotations
import threading

import rospy

from pathfinder.planning.path_orchestrator import PathOrchestrator, UnknownRobotError, NodeNotFoundError, NoPathError
from pathfinder.ros.path_follow_action_client import PathFollowActionClient
from pathfinder.ros.pose_tracker import PoseTracker
from pathfinder.srv import MoveToNode, MoveToNodeRequest, MoveToNodeResponse, CancelPath, CancelPathRequest, CancelPathResponse  # type: ignore[import]


class PathRequestService:
    """Service handler for MoveToNode and CancelPath: plans routes and dispatches to PathFollowActionServer."""

    def __init__(
        self,
        orchestrator: PathOrchestrator,
        tracker: PoseTracker,
        clients: dict[str, PathFollowActionClient],
        robot_locks: dict[str, threading.Lock],
        move_service_name: str,
        cancel_service_name: str,
    ) -> None:
        """Store injected planning and dispatch components."""
        self._orchestrator = orchestrator
        self._tracker = tracker
        self._clients = clients
        self._robot_locks = robot_locks
        self._move_service_name = move_service_name
        self._cancel_service_name = cancel_service_name

    def start(self) -> None:
        """Register the MoveToNode and CancelPath services."""
        rospy.Service(self._move_service_name, MoveToNode, self._handle_move)
        rospy.Service(self._cancel_service_name, CancelPath, self._handle_cancel)
        rospy.loginfo("PathRequestService started.")

    def _handle_move(self, request: MoveToNodeRequest) -> MoveToNodeResponse:
        """Plan a path and dispatch it to the robot's FollowPath action server."""
        robot_id = request.robot_id
        if robot_id not in self._clients:
            return MoveToNodeResponse(success=False, message=f"unknown robot: {robot_id}")

        pose = self._tracker.wait_for(robot_id)
        if pose is None:
            return MoveToNodeResponse(success=False, message=f"no pose for {robot_id}")

        try:
            node_ids = self._orchestrator.plan(robot_id, pose, request.target_node_id)
        except (UnknownRobotError, NodeNotFoundError, NoPathError) as error:
            return MoveToNodeResponse(success=False, message=str(error))

        with self._robot_locks[robot_id]:
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

        with self._robot_locks[robot_id]:
            self._clients[robot_id].cancel()

        return CancelPathResponse(success=True, message="canceled")
