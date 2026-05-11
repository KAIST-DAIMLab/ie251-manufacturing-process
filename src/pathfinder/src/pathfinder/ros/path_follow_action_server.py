from __future__ import annotations
import json
import threading
from typing import Any

import rospy
import actionlib
from std_msgs.msg import String

from pathfinder.msg import FollowPathAction, FollowPathFeedback, FollowPathResult  # type: ignore[import]
from pathfinder.robot.turtlebot import TurtleBot
from pathfinder.world.graph import Graph

_IDLE_STATUS = json.dumps({"node_ids": [], "current_index": -1})


class PathFollowActionServer:
    """ActionServer for FollowPath: receives path goals from PathRequestActionServer and drives the robot."""

    def __init__(self, robot: TurtleBot, graph: Graph, topic: str) -> None:
        """Store the robot facade, graph, and action topic."""
        self._robot = robot
        self._graph = graph
        self._topic = topic
        self._server = None
        self._status_publisher = None

    def start(self) -> None:
        """Start the FollowPath action server and path status publisher."""
        ns = self._topic.rsplit('/', 1)[0]
        self._status_publisher = rospy.Publisher(f"{ns}/path_status", String, queue_size=1, latch=True)
        self._status_publisher.publish(_IDLE_STATUS)
        self._server = actionlib.SimpleActionServer(
            self._topic,
            FollowPathAction,
            execute_cb=self._on_follow_path,
            auto_start=False,
        )
        self._server.start()

    def _publish_status(self, node_ids: list, current_index: int) -> None:
        self._status_publisher.publish(json.dumps({"node_ids": list(node_ids), "current_index": current_index}))

    def _on_follow_path(self, goal: Any) -> None:
        waypoints = [self._graph.get_node(nid) for nid in goal.node_ids]

        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return

        self._publish_status(goal.node_ids, 0)

        result_container: list[bool] = []
        follow_thread = threading.Thread(
            target=lambda: result_container.append(self._robot.follow_path(waypoints)),
            daemon=True,
        )
        follow_thread.start()

        last_index = -1
        rate = rospy.Rate(20)
        while follow_thread.is_alive():
            if self._server.is_preempt_requested():
                self._robot.stop()
                follow_thread.join()
                self._status_publisher.publish(_IDLE_STATUS)
                self._server.set_preempted()
                return

            current_index = self._robot.path_follower.current_index
            if current_index != last_index:
                last_index = current_index
                self._publish_status(goal.node_ids, current_index)

            fb = FollowPathFeedback()
            fb.current_index = current_index
            fb.current_pose = self._robot.get_pose()
            self._server.publish_feedback(fb)
            rate.sleep()

        follow_thread.join()
        self._status_publisher.publish(_IDLE_STATUS)
        if result_container and result_container[0]:
            self._server.set_succeeded(
                FollowPathResult(success=True, message="reached goal")
            )
        else:
            self._server.set_aborted(
                FollowPathResult(success=False, message="interrupted")
            )
