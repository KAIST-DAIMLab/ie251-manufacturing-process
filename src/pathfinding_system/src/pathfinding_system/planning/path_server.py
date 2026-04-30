from __future__ import annotations
import rospy
import actionlib
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
        self._follow_clients: dict[str, object] = {}
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

        self._server = actionlib.SimpleActionServer(
            '/path_server/move_to_node',
            MoveToNodeAction,
            execute_cb=self._on_move_to_node,
            auto_start=False,
        )
        self._server.start()
        rospy.loginfo("PathServer started.")

    def _on_robot_state(self, ns: str, msg) -> None:
        self._robot_states[ns] = msg

    def _on_move_to_node(self, goal) -> None:
        from pathfinding_system.msg import (  # type: ignore[import]
            MoveToNodeResult,
            MoveToNodeFeedback,
            FollowPathGoal,
        )
        result = MoveToNodeResult()
        ns = goal.robot_id

        if ns not in self._namespaces:
            result.success = False
            result.message = f"unknown robot: {ns}"
            self._server.set_aborted(result)
            return

        state_msg = self._robot_states.get(ns)
        if state_msg is None:
            result.success = False
            result.message = f"no state received from {ns}"
            self._server.set_aborted(result)
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
            self._server.set_aborted(result)
            return

        node_ids = [n.id for n in path._waypoints]
        client = self._follow_clients[ns]

        if not client.wait_for_server(timeout=rospy.Duration(5.0)):
            result.success = False
            result.message = f"executor {ns} not available"
            self._server.set_aborted(result)
            return

        latest_fb = [None]

        def follow_fb_cb(fb):
            latest_fb[0] = fb

        fp_goal = FollowPathGoal()
        fp_goal.node_ids = node_ids
        client.send_goal(fp_goal, feedback_cb=follow_fb_cb)

        while not client.wait_for_result(timeout=rospy.Duration(0.1)):
            if self._server.is_preempt_requested():
                client.cancel_goal()
                self._server.set_preempted()
                return
            fb = latest_fb[0]
            if fb is not None:
                mtn_fb = MoveToNodeFeedback()
                idx = fb.current_index
                mtn_fb.current_node_id = node_ids[idx - 1] if idx > 0 else -1
                mtn_fb.nodes_remaining = len(node_ids) - idx
                self._server.publish_feedback(mtn_fb)

        follow_result = client.get_result()
        result.success = follow_result.success if follow_result else False
        result.message = follow_result.message if follow_result else "no result"
        if result.success:
            self._server.set_succeeded(result)
        else:
            self._server.set_aborted(result)
