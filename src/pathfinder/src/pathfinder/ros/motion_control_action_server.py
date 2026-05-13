from __future__ import annotations
import threading
from typing import Any

import actionlib
import rospy

from pathfinder.msg import RobotCommandAction, RobotCommandResult  # type: ignore[import]
from pathfinder.robot.turtlebot import TurtleBot


_COMMANDS = {
    'turn_left',
    'turn_right',
    'turn_to',
    'move_forward',
    'move_backward',
}


class MotionControlActionServer:
    """ActionServer for RobotCommand: receives primitive motion goals from UserClient and executes them."""

    def __init__(self, robot: TurtleBot, topic: str) -> None:
        """Store the robot facade and action topic."""
        self._robot = robot
        self._topic = topic
        self._server = None

    def start(self) -> None:
        """Start the RobotCommand action server."""
        self._server = actionlib.SimpleActionServer(
            self._topic,
            RobotCommandAction,
            execute_cb=self._on_user_command,
            auto_start=False,
        )
        self._server.start()

    def _on_user_command(self, goal: Any) -> None:
        command = goal.command
        if command not in _COMMANDS:
            self._server.set_aborted(
                RobotCommandResult(success=False, message=f"unknown command: {command}")
            )
            return

        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return

        result_container: list[bool] = []
        command_thread = threading.Thread(
            target=lambda: result_container.append(getattr(self._robot, command)(goal.value)),
            daemon=True,
        )
        command_thread.start()

        rate = rospy.Rate(20)
        while command_thread.is_alive():
            if self._server.is_preempt_requested():
                self._robot.stop()
                command_thread.join()
                self._server.set_preempted()
                return
            rate.sleep()

        command_thread.join()
        success = bool(result_container and result_container[0])
        if success:
            self._server.set_succeeded(
                RobotCommandResult(success=True, message=f"{command} completed")
            )
        else:
            self._server.set_aborted(
                RobotCommandResult(success=False, message=f"{command} interrupted")
            )
