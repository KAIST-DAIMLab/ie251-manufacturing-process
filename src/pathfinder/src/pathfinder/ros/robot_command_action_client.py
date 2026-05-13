from __future__ import annotations

import rospy
import actionlib

from pathfinder.msg import RobotCommandAction, RobotCommandGoal  # type: ignore[import]


class RobotCommandActionClient:
    """Wraps a per-robot primitive motion command action client."""

    def __init__(self, robot_id: str, action_namespace: str) -> None:
        self.robot_id = robot_id
        self._client = actionlib.SimpleActionClient(
            f'/{action_namespace}/user_command', RobotCommandAction
        )

    def send(self, command: str, value: float) -> bool:
        """Send a primitive motion command asynchronously."""
        if not self._client.wait_for_server(timeout=rospy.Duration(5.0)):
            return False
        goal = RobotCommandGoal()
        goal.command = command
        goal.value = value
        self._client.send_goal(goal)
        return True

    def cancel(self) -> None:
        """Cancel any in-flight primitive motion command."""
        self._client.cancel_goal()
