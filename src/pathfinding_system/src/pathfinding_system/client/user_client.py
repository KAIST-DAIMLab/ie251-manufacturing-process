from __future__ import annotations
import rospy
import actionlib
from std_msgs.msg import Empty  # type: ignore[import]
from pathfinding_system.msg import MoveToNodeAction  # type: ignore[import]
from pathfinding_system.msg import MoveToNodeGoal  # type: ignore[import]

class UserClient:
    def __init__(self) -> None:
        self._client = actionlib.SimpleActionClient(
            '/path_server/move_to_node', MoveToNodeAction
        )

    def cancel(self, robot_id: str) -> None:
        
        pub = rospy.Publisher(f'/{robot_id}/emergency_stop', Empty, queue_size=1)
        rospy.sleep(0.1)  # allow publisher to register with subscribers
        pub.publish(Empty())
        rospy.loginfo(f"Emergency stop sent to {robot_id}.")

    def send_goal(self, robot_id: str, target_node_id: int) -> bool:
        
        rospy.loginfo("Waiting for path_server/move_to_node...")
        self._client.wait_for_server()
        rospy.loginfo("Connected to path_server.")
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
