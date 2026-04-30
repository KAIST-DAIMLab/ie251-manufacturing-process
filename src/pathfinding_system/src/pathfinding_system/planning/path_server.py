from __future__ import annotations
import threading
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from pathfinding_system.world.graph import Graph
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
            client = actionlib.SimpleActionClient(f'/{ns}/follow_path', FollowPathAction)
            self._follow_clients[ns] = client

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
        t = threading.Thread(target=self._execute, args=(goal_handle,), daemon=True)
        t.start()

    def _execute(self, goal_handle) -> None:
        from pathfinding_system.msg import (  # type: ignore[import]
            MoveToNodeResult,
            MoveToNodeFeedback,
            FollowPathGoal,
        )
        goal = goal_handle.get_goal()
        result = MoveToNodeResult()
        ns = goal.robot_id

        if ns not in self._namespaces:
            result.success = False
            result.message = f"unknown robot: {ns}"
            goal_handle.set_aborted(result)
            return

        with self._states_lock:
            state_msg = self._robot_states.get(ns)
        if state_msg is None:
            result.success = False
            result.message = f"no state received from {ns}"
            goal_handle.set_aborted(result)
            return

        pose = state_msg.pose
        all_nodes = list(self._graph._nodes.values())
        start_node = min(all_nodes, key=lambda n: (n.x - pose.x) ** 2 + (n.y - pose.y) ** 2)

        try:
            target_node = self._graph.get_node(goal.target_node_id)
            path = self._planner.plan(start_node, target_node)
        except (KeyError, ValueError) as e:
            result.success = False
            result.message = str(e)
            goal_handle.set_aborted(result)
            return

        node_ids = [n.id for n in path._waypoints]
        client = self._follow_clients[ns]

        with self._robot_locks[ns]:
            if not client.wait_for_server(timeout=rospy.Duration(5.0)):
                result.success = False
                result.message = f"executor {ns} not available"
                goal_handle.set_aborted(result)
                return

            latest_fb: list = [None]

            def follow_fb_cb(fb) -> None:
                latest_fb[0] = fb

            fp_goal = FollowPathGoal()
            fp_goal.node_ids = node_ids
            client.send_goal(fp_goal, feedback_cb=follow_fb_cb)

            while not client.wait_for_result(timeout=rospy.Duration(0.1)):
                status = goal_handle.get_goal_status().status
                if status in (GoalStatus.PREEMPTING, GoalStatus.RECALLING):
                    client.cancel_goal()
                    goal_handle.set_canceled()
                    return
                fb = latest_fb[0]
                if fb is not None:
                    mtn_fb = MoveToNodeFeedback()
                    idx = fb.current_index
                    mtn_fb.current_node_id = node_ids[idx - 1] if idx > 0 else -1
                    mtn_fb.nodes_remaining = len(node_ids) - idx
                    goal_handle.publish_feedback(mtn_fb)

            follow_result = client.get_result()
            result.success = follow_result.success if follow_result else False
            result.message = follow_result.message if follow_result else "no result"

        if result.success:
            goal_handle.set_succeeded(result)
        else:
            goal_handle.set_aborted(result)
