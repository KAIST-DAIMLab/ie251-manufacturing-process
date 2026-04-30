from __future__ import annotations
import rospy
import actionlib


class UserClient:
    def __init__(self) -> None:
        from pathfinding_system.msg import MoveToNodeAction  # type: ignore[import]
        self._client = actionlib.SimpleActionClient(
            '/path_server/move_to_node', MoveToNodeAction
        )
        rospy.loginfo("Waiting for path_server/move_to_node...")
        self._client.wait_for_server()
        rospy.loginfo("Connected to path_server.")

    def send_goal(self, robot_id: str, target_node_id: int) -> bool:
        from pathfinding_system.msg import MoveToNodeGoal  # type: ignore[import]
        goal = MoveToNodeGoal()
        goal.robot_id = robot_id
        goal.target_node_id = target_node_id
        self._client.send_goal(goal, feedback_cb=self._on_feedback)
        self._client.wait_for_result()
        result = self._client.get_result()
        if result:
            rospy.loginfo(f"Result: success={result.success}, message={result.message}")
            return result.success
        return False

    def _on_feedback(self, fb) -> None:
        rospy.loginfo(
            f"  -> node {fb.current_node_id}, {fb.nodes_remaining} remaining"
        )
