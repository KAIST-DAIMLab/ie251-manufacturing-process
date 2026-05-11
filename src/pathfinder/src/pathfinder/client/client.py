from __future__ import annotations
import math
import rospy
import actionlib
from pathfinder.msg import RobotCommandAction  # type: ignore[import]
from pathfinder.msg import RobotCommandGoal  # type: ignore[import]
from pathfinder.srv import MoveToNode, CancelPath  # type: ignore[import]


USER_COMMANDS = {
    'turn_left',
    'turn_right',
    'move_forward',
    'move_backward',
}

_TURN_COMMANDS = {'turn_left', 'turn_right'}


class Client:
    """User-facing client: sends MoveToNode service calls and user commands to robots."""

    def __init__(self) -> None:
        self._move_proxy = rospy.ServiceProxy('/fleet/move_to_node', MoveToNode)
        self._cancel_proxy = rospy.ServiceProxy('/fleet/cancel_path', CancelPath)

    def cancel(self, robot_id: str) -> None:
        result = self._cancel_proxy(robot_id)
        rospy.loginfo(f"Cancel sent to {robot_id}: {result.message}")

    def send_goal(self, robot_id: str, target_node_id: int) -> bool:
        rospy.loginfo("Waiting for fleet/move_to_node...")
        rospy.wait_for_service('/fleet/move_to_node')
        rospy.loginfo("Connected to fleet.")
        result = self._move_proxy(robot_id, target_node_id)
        rospy.loginfo(f"Result: success={result.success}, message={result.message}")
        return result.success

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
