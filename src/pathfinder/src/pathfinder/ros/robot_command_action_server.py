from __future__ import annotations
from typing import Any

import actionlib

from pathfinder.robot.turtlebot import TurtleBot


_COMMANDS = {
    'turn_left',
    'turn_right',
    'move_forward',
    'move_backward',
}


class RobotCommandActionServer:
    """Handles primitive user motion commands for a single robot."""

    def __init__(self, robot: TurtleBot, namespace: str) -> None:
        self._robot = robot
        self._namespace = namespace
        self._server = None

    def start(self) -> None:
        """Start the robot command action server."""
        from pathfinder.msg import RobotCommandAction  # type: ignore[import]

        self._server = actionlib.SimpleActionServer(
            f'/{self._namespace}/user_command',
            RobotCommandAction,
            execute_cb=self._on_user_command,
            auto_start=False,
        )
        self._server.start()

    def _on_user_command(self, goal: Any) -> None:
        from pathfinder.msg import RobotCommandResult  # type: ignore[import]

        command = goal.command
        if command not in _COMMANDS:
            self._server.set_aborted(
                RobotCommandResult(success=False, message=f"unknown command: {command}")
            )
            return

        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return

        success = getattr(self._robot, command)(goal.value)
        if success:
            self._server.set_succeeded(
                RobotCommandResult(success=True, message=f"{command} completed")
            )
        else:
            self._server.set_aborted(
                RobotCommandResult(success=False, message=f"{command} interrupted")
            )
