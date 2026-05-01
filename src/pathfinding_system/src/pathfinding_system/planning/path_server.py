from __future__ import annotations
import threading
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from pathfinding_system.world.graph import Graph
from pathfinding_system.world.node import Node
from pathfinding_system.planning.path_planner import PathPlanner


class PathServer:
    def __init__(
        self,
        graph: Graph,
        planner: PathPlanner,
        monitor,
        robot_namespaces: list[str],
    ) -> None:
        self._graph = graph
        self._planner = planner
        self._monitor = monitor
        self._namespaces = robot_namespaces
        self._robot_states: dict[str, object] = {}
        self._states_lock = threading.Lock()
        self._follow_clients: dict[str, object] = {}
        # Per-robot lock so two simultaneous goals to the same robot are serialized.
        self._robot_locks: dict[str, threading.Lock] = {ns: threading.Lock() for ns in robot_namespaces}
        self._server = None

    def start(self) -> None:
        from pathfinding_system.msg import (  # type: ignore[import]
            RobotState as RobotStateMsg,
            MoveToNodeAction,
            FollowPathAction,
        )
        for ns in self._namespaces:
            rospy.Subscriber(
                f'/{ns}/robot_state',
                RobotStateMsg,
                lambda msg, n=ns: self._on_robot_state(n, msg),
            )
            self._follow_clients[ns] = actionlib.SimpleActionClient(
                f'/{ns}/follow_path', FollowPathAction
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

    def _on_robot_state(self, ns: str, msg) -> None:
        with self._states_lock:
            self._robot_states[ns] = msg

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
        with self._states_lock:
            state_msg = self._robot_states.get(ns)
        if state_msg is None:
            self._abort(goal_handle, f"no state received from {ns}")
            return None
        pose = state_msg.pose
        return min(
            self._graph.all_nodes(),
            key=lambda n: (n.x - pose.x) ** 2 + (n.y - pose.y) ** 2,
        )

    def _plan_node_ids(self, start: Node, target_id: int, goal_handle) -> list[int] | None:
        try:
            target = self._graph.get_node(target_id)
            path = self._planner.plan(start, target)
        except (KeyError, ValueError) as e:
            self._abort(goal_handle, str(e))
            return None
        return [n.id for n in path.remaining()]

    def _dispatch_to_executor(self, ns: str, node_ids: list[int], goal_handle) -> None:
        from pathfinding_system.msg import FollowPathGoal  # type: ignore[import]

        client = self._follow_clients[ns]
        with self._robot_locks[ns]:
            if not client.wait_for_server(timeout=rospy.Duration(5.0)):
                self._abort(goal_handle, f"executor {ns} not available")
                return

            latest_fb: list = [None]
            fp_goal = FollowPathGoal()
            fp_goal.node_ids = node_ids
            client.send_goal(fp_goal, feedback_cb=lambda fb: latest_fb.__setitem__(0, fb))

            while not client.wait_for_result(timeout=rospy.Duration(0.1)):
                status = goal_handle.get_goal_status().status
                if status in (GoalStatus.PREEMPTING, GoalStatus.RECALLING):
                    client.cancel_goal()
                    goal_handle.set_canceled()
                    return
                if latest_fb[0] is not None:
                    self._publish_feedback(goal_handle, node_ids, latest_fb[0])

            follow_result = client.get_result()

        self._finish(goal_handle, follow_result)

    def _publish_feedback(self, goal_handle, node_ids: list[int], fb) -> None:
        from pathfinding_system.msg import MoveToNodeFeedback  # type: ignore[import]
        mtn_fb = MoveToNodeFeedback()
        idx = fb.current_index
        mtn_fb.current_node_id = node_ids[idx - 1] if idx > 0 else -1
        mtn_fb.nodes_remaining = len(node_ids) - idx
        goal_handle.publish_feedback(mtn_fb)

    def _finish(self, goal_handle, follow_result) -> None:
        from pathfinding_system.msg import MoveToNodeResult  # type: ignore[import]
        result = MoveToNodeResult()
        if follow_result and follow_result.success:
            result.success = True
            result.message = follow_result.message
            goal_handle.set_succeeded(result)
        else:
            result.success = False
            result.message = follow_result.message if follow_result else "no result"
            goal_handle.set_aborted(result)

    def _abort(self, goal_handle, message: str) -> None:
        from pathfinding_system.msg import MoveToNodeResult  # type: ignore[import]
        result = MoveToNodeResult()
        result.success = False
        result.message = message
        goal_handle.set_aborted(result)
