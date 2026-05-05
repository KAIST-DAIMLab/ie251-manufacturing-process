from __future__ import annotations
import threading
from typing import Any

import rospy
import actionlib
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from pathfinding_system.robot.turtlebot import TurtleBot
from pathfinding_system.world.graph import Graph


class TurtleBotNode:
    """ROS adapter: wires topics and action server for one TurtleBot."""

    def __init__(
        self,
        robot: TurtleBot,
        graph: Graph | None = None,
        topic_namespace: str | None = None,
    ) -> None:
        self._robot = robot
        self._graph = graph
        self._action_server = None
        self._topic_namespace = (topic_namespace or robot.id).strip('/')

        self._odom_subscriber = rospy.Subscriber(
            self.topic_odom,
            Odometry,
            robot.update_pose,
        )
        self._emergency_stop_subscriber = rospy.Subscriber(
            f'/{robot.id}/emergency_stop',
            Empty,
            self._on_emergency_stop,
        )

    @property
    def topic_odom(self) -> str:
        """Topic name for the odometry subscriber."""
        return f'/{self._topic_namespace}/odom'

    def start(self) -> None:
        """Start the FollowPath action server (no-op if no graph was provided)."""
        if self._graph is None:
            return

        from pathfinding_system.msg import FollowPathAction  # type: ignore[import]

        self._action_server = actionlib.SimpleActionServer(
            f'/{self._robot.id}/follow_path',
            FollowPathAction,
            execute_cb=self._on_follow_path,
            auto_start=False,
        )
        self._action_server.start()
        rospy.loginfo(f"TurtleBotNode for {self._robot.id} started.")

    def _on_emergency_stop(self, msg: Empty) -> None:
        self._robot.stop()
        rospy.logwarn(f"{self._robot.id}: emergency stop received.")

    def _on_follow_path(self, goal: Any) -> None:
        from pathfinding_system.msg import (  # type: ignore[import]
            FollowPathFeedback,
            FollowPathResult,
        )

        waypoints = [self._graph.get_node(nid) for nid in goal.node_ids]

        if self._action_server.is_preempt_requested():
            self._action_server.set_preempted()
            return

        result_container: list[bool] = []
        follow_thread = threading.Thread(
            target=lambda: result_container.append(self._robot.follow_path(waypoints)),
            daemon=True,
        )
        follow_thread.start()

        rate = rospy.Rate(20)
        while follow_thread.is_alive():
            if self._action_server.is_preempt_requested():
                self._robot.stop()
                follow_thread.join()
                self._action_server.set_preempted()
                return

            fb = FollowPathFeedback()
            fb.current_index = self._robot.path_follower.current_index
            fb.current_pose = self._robot.current_pose()
            self._action_server.publish_feedback(fb)
            rate.sleep()

        follow_thread.join()
        if result_container and result_container[0]:
            self._action_server.set_succeeded(
                FollowPathResult(success=True, message="reached goal")
            )
        else:
            self._action_server.set_aborted(
                FollowPathResult(success=False, message="interrupted")
            )
