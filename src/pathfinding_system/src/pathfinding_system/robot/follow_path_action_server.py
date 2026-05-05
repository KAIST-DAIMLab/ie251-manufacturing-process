from __future__ import annotations
import threading
from typing import Any

import rospy
import actionlib

from pathfinding_system.robot.turtlebot import TurtleBot
from pathfinding_system.world.graph import Graph


class FollowPathActionServer:
    """Handles the FollowPath action for a single robot."""

    def __init__(self, robot: TurtleBot, graph: Graph) -> None:
        self._robot = robot
        self._graph = graph
        self._server = None

    def start(self) -> None:
        """Start the FollowPath action server."""
        from pathfinding_system.msg import FollowPathAction  # type: ignore[import]

        self._server = actionlib.SimpleActionServer(
            f'/{self._robot.id}/follow_path',
            FollowPathAction,
            execute_cb=self._on_follow_path,
            auto_start=False,
        )
        self._server.start()

    def _on_follow_path(self, goal: Any) -> None:
        from pathfinding_system.msg import (  # type: ignore[import]
            FollowPathFeedback,
            FollowPathResult,
        )

        waypoints = [self._graph.get_node(nid) for nid in goal.node_ids]

        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return

        result_container: list[bool] = []
        follow_thread = threading.Thread(
            target=lambda: result_container.append(self._robot.follow_path(waypoints)),
            daemon=True,
        )
        follow_thread.start()

        rate = rospy.Rate(20)
        while follow_thread.is_alive():
            if self._server.is_preempt_requested():
                self._robot.stop()
                follow_thread.join()
                self._server.set_preempted()
                return

            fb = FollowPathFeedback()
            fb.current_index = self._robot.path_follower.current_index
            fb.current_pose = self._robot.current_pose()
            self._server.publish_feedback(fb)
            rate.sleep()

        follow_thread.join()
        if result_container and result_container[0]:
            self._server.set_succeeded(
                FollowPathResult(success=True, message="reached goal")
            )
        else:
            self._server.set_aborted(
                FollowPathResult(success=False, message="interrupted")
            )
