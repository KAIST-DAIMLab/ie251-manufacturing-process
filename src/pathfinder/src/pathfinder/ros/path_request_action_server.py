from __future__ import annotations
import threading

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from pathfinder.planning.path_orchestrator import PathOrchestrator, UnknownRobotError, NodeNotFoundError, NoPathError
from pathfinder.ros.follow_path_client import FollowPathClient
from pathfinder.ros.pose_tracker import PoseTracker


class PathRequestActionServer:
    """ActionServer for MoveToNode: receives path goals from UserClient, plans routes, and relays to PathFollowActionServer."""

    def __init__(
        self,
        orchestrator: PathOrchestrator,
        tracker: PoseTracker,
        clients: dict[str, FollowPathClient],
        robot_locks: dict[str, threading.Lock],
        topic: str,
    ) -> None:
        """Store injected planning and dispatch components."""
        self._orchestrator = orchestrator
        self._tracker = tracker
        self._clients = clients
        self._robot_locks = robot_locks
        self._topic = topic
        self._server = None

    def start(self) -> None:
        """Start the MoveToNode action server."""
        from pathfinder.msg import MoveToNodeAction  # type: ignore[import]

        self._server = actionlib.ActionServer(
            self._topic,
            MoveToNodeAction,
            goal_cb=self._on_goal_received,
            cancel_cb=self._on_cancel,
            auto_start=False,
        )
        self._server.start()
        rospy.loginfo("PathRequestActionServer started.")

    def _on_cancel(self, goal_handle) -> None:
        rospy.loginfo("PathRequestActionServer: cancel requested.")

    def _on_goal_received(self, goal_handle) -> None:
        goal_handle.set_accepted()
        threading.Thread(target=self._execute, args=(goal_handle,), daemon=True).start()

    def _execute(self, goal_handle) -> None:
        goal = goal_handle.get_goal()
        namespace = goal.robot_id

        pose = self._tracker.wait_for(namespace)
        if pose is None:
            self._abort(goal_handle, f"no state received from {namespace}")
            return

        try:
            node_ids = self._orchestrator.plan(namespace, pose, goal.target_node_id)
        except (UnknownRobotError, NodeNotFoundError, NoPathError) as error:
            self._abort(goal_handle, str(error))
            return

        last_published_index = -1

        def on_feedback(feedback):
            nonlocal last_published_index
            if feedback.current_index != last_published_index:
                self._publish_feedback(goal_handle, node_ids, feedback)
                last_published_index = feedback.current_index

        def is_canceled():
            return goal_handle.get_goal_status().status in (
                GoalStatus.PREEMPTING,
                GoalStatus.RECALLING,
            )

        with self._robot_locks[namespace]:
            result = self._clients[namespace].dispatch(
                node_ids,
                on_feedback=on_feedback,
                is_canceled=is_canceled,
            )

        if is_canceled():
            goal_handle.set_canceled()
            return

        self._finish(goal_handle, result)

    def _publish_feedback(self, goal_handle, node_ids: list[int], feedback) -> None:
        from pathfinder.msg import MoveToNodeFeedback  # type: ignore[import]
        move_feedback = MoveToNodeFeedback()
        index = feedback.current_index
        move_feedback.current_node_id = node_ids[index - 1] if index > 0 else -1
        move_feedback.nodes_remaining = len(node_ids) - index
        goal_handle.publish_feedback(move_feedback)

    def _finish(self, goal_handle, follow_result) -> None:
        if follow_result and follow_result.success:
            self._succeed(goal_handle, follow_result.message)
        else:
            self._abort(goal_handle, follow_result.message if follow_result else "no result")

    def _succeed(self, goal_handle, message: str) -> None:
        from pathfinder.msg import MoveToNodeResult  # type: ignore[import]
        result = MoveToNodeResult()
        result.success = True
        result.message = message
        goal_handle.set_succeeded(result)

    def _abort(self, goal_handle, message: str) -> None:
        from pathfinder.msg import MoveToNodeResult  # type: ignore[import]
        result = MoveToNodeResult()
        result.success = False
        result.message = message
        goal_handle.set_aborted(result)
