from __future__ import annotations
import threading
import time
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from pathfinder.robot.robot_state import RobotState
from pathfinder.world.graph import Graph
from pathfinder.world.node import Node
from pathfinder.planning.path_planner import PathPlanner


class PathServer:
    """ROS action server: resolves start node, plans path, and dispatches to robot executor."""

    def __init__(
        self,
        graph: Graph,
        planner: PathPlanner,
        robot_namespaces: list[str],
        robot_odom_topics: dict[str, str] | None = None,
        robot_action_namespaces: dict[str, str] | None = None,
        first_state_timeout_sec: float = 1.0,
    ) -> None:
        self._graph = graph
        self._planner = planner
        self._namespaces = robot_namespaces
        self._robot_odom_topics = robot_odom_topics or {
            ns: f'/{ns}/odom' for ns in robot_namespaces
        }
        self._robot_action_namespaces = robot_action_namespaces or {
            ns: ns for ns in robot_namespaces
        }
        self._first_state_timeout_sec = first_state_timeout_sec
        self._robot_states: dict[str, object] = {}
        self._state_available = threading.Condition()
        self._follow_clients: dict[str, object] = {}
        # Per-robot lock so two simultaneous goals to the same robot are serialized.
        self._robot_locks: dict[str, threading.Lock] = {ns: threading.Lock() for ns in robot_namespaces}
        self._server = None

    def start(self) -> None:
        """Register subscribers, action clients, and start the action server."""
        from pathfinder.msg import (  # type: ignore[import]
            MoveToNodeAction,
            FollowPathAction,
        )
        for ns in self._namespaces:
            rospy.Subscriber(
                self._robot_odom_topics[ns],
                Odometry,
                lambda message, n=ns: self._on_odom(n, message),
            )
            self._follow_clients[ns] = actionlib.SimpleActionClient(
                f'/{self._robot_action_namespaces[ns]}/follow_path', FollowPathAction
            )

        # ActionServer (not SimpleActionServer) supports concurrent goals.
        self._server = actionlib.ActionServer(
            '/path_server/move_to_node',
            MoveToNodeAction,
            goal_cb=self._on_goal_received,
            cancel_cb=self._on_cancel,
            auto_start=False,
        )
        self._server.start()
        rospy.loginfo("PathServer started.")

    def _on_odom(self, ns: str, message: Odometry) -> None:
        with self._state_available:
            is_first = ns not in self._robot_states
            self._robot_states[ns] = RobotState.from_odometry(ns, message)
            if is_first:
                self._state_available.notify_all()

    def _on_cancel(self, goal_handle) -> None:
        rospy.loginfo("PathServer: cancel requested.")

    def _on_goal_received(self, goal_handle) -> None:
        goal_handle.set_accepted()
        threading.Thread(target=self._execute, args=(goal_handle,), daemon=True).start()

    def _execute(self, goal_handle) -> None:
        goal = goal_handle.get_goal()
        ns = goal.robot_id

        start_node = self._resolve_start_node(ns, goal_handle)
        if start_node is None:
            return

        node_ids = self._plan_node_ids(start_node, goal.target_node_id, goal_handle)
        if node_ids is None:
            return

        self._dispatch_to_executor(ns, node_ids, goal_handle)

    def _resolve_start_node(self, ns: str, goal_handle) -> Node | None:
        if ns not in self._namespaces:
            self._abort(goal_handle, f"unknown robot: {ns}")
            return None
        state_msg = self._wait_for_robot_state(ns)
        if state_msg is None:
            self._abort(goal_handle, f"no state received from {ns}")
            return None
        pose = state_msg.get_pose()
        return min(
            self._graph.all_nodes(),
            key=lambda n: (n.x - pose.x) ** 2 + (n.y - pose.y) ** 2,
        )

    def _wait_for_robot_state(self, ns: str):
        deadline = time.monotonic() + self._first_state_timeout_sec
        with self._state_available:
            state_msg = self._robot_states.get(ns)
            while state_msg is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._state_available.wait(timeout=remaining)
                state_msg = self._robot_states.get(ns)
            return state_msg

    def _plan_node_ids(self, start: Node, target_id: int, goal_handle) -> list[int] | None:
        try:
            target = self._graph.get_node(target_id)
            waypoints = self._planner.plan(start, target)
        except (KeyError, ValueError) as e:
            self._abort(goal_handle, str(e))
            return None
        return [n.id for n in waypoints]

    def _dispatch_to_executor(self, ns: str, node_ids: list[int], goal_handle) -> None:
        from pathfinder.msg import FollowPathGoal  # type: ignore[import]

        client = self._follow_clients[ns]
        with self._robot_locks[ns]:
            if not client.wait_for_server(timeout=rospy.Duration(5.0)):
                self._abort(goal_handle, f"executor {ns} not available")
                return

            latest_fb = None
            last_published_index = -1

            def _on_feedback(feedback):
                nonlocal latest_fb
                latest_fb = feedback

            fp_goal = FollowPathGoal()
            fp_goal.node_ids = node_ids
            client.send_goal(fp_goal, feedback_cb=_on_feedback)

            while not client.wait_for_result(timeout=rospy.Duration(0.1)):
                status = goal_handle.get_goal_status().status
                if status in (GoalStatus.PREEMPTING, GoalStatus.RECALLING):
                    client.cancel_goal()
                    goal_handle.set_canceled()
                    return
                if latest_fb is not None and latest_fb.current_index != last_published_index:
                    self._publish_feedback(goal_handle, node_ids, latest_fb)
                    last_published_index = latest_fb.current_index

            follow_result = client.get_result()

        self._finish(goal_handle, follow_result)

    def _publish_feedback(self, goal_handle, node_ids: list[int], fb) -> None:
        from pathfinder.msg import MoveToNodeFeedback  # type: ignore[import]
        feedback = MoveToNodeFeedback()
        idx = fb.current_index
        feedback.current_node_id = node_ids[idx - 1] if idx > 0 else -1
        feedback.nodes_remaining = len(node_ids) - idx
        goal_handle.publish_feedback(feedback)

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
