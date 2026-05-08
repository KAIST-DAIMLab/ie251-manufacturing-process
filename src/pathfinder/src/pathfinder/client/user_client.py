from __future__ import annotations
import math
import rospy
import actionlib
from std_msgs.msg import Empty  # type: ignore[import]
from pathfinder.msg import MoveToNodeAction  # type: ignore[import]
from pathfinder.msg import MoveToNodeGoal  # type: ignore[import]
from pathfinder.msg import RobotCommandAction  # type: ignore[import]
from pathfinder.msg import RobotCommandGoal  # type: ignore[import]


USER_COMMANDS = {
    'turn_left',
    'turn_right',
    'move_forward',
    'move_backward',
}

_TURN_COMMANDS = {'turn_left', 'turn_right'}


class UserClient:
    def __init__(self) -> None:
        self._move_client = actionlib.SimpleActionClient(
            '/path_server/move_to_node', MoveToNodeAction
        )

    def cancel(self, robot_id: str) -> None:
        pub = rospy.Publisher(f'/{robot_id}/stop', Empty, queue_size=1)
        rospy.sleep(0.1)  # allow publisher to register with subscribers
        pub.publish(Empty())
        rospy.loginfo(f"Stop sent to {robot_id}.")

    def send_goal(self, robot_id: str, target_node_id: int) -> bool:
        rospy.loginfo("Waiting for path_server/move_to_node...")
        self._move_client.wait_for_server()
        rospy.loginfo("Connected to path_server.")
        goal = MoveToNodeGoal()
        goal.robot_id = robot_id
        goal.target_node_id = target_node_id
        self._move_client.send_goal(goal, feedback_cb=self._on_feedback)
        self._move_client.wait_for_result()
        result = self._move_client.get_result()
        if result:
            rospy.loginfo(f"Result: success={result.success}, message={result.message}")
            return result.success
        return False

    def send_command(self, robot_id: str, command: str, value: float) -> bool:
        if command not in USER_COMMANDS:
            raise ValueError(f"unknown command: {command}")

        client = actionlib.SimpleActionClient(
            f'/{robot_id}/user_command', RobotCommandAction
        )
        rospy.loginfo(f"Waiting for {robot_id}/user_command...")
        client.wait_for_server()
        rospy.loginfo(f"Connected to {robot_id}/user_command.")
        goal = RobotCommandGoal()
        goal.command = command
        goal.value = math.radians(value) if command in _TURN_COMMANDS else value
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        if result:
            rospy.loginfo(f"Result: success={result.success}, message={result.message}")
            return result.success
        return False

    def _on_feedback(self, fb) -> None:
        rospy.loginfo(
            f"  -> node {fb.current_node_id}, {fb.nodes_remaining} remaining"
        )
