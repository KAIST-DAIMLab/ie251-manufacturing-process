from __future__ import annotations

import rospy
from nav_msgs.msg import Odometry

from pathfinder.planning.a_star_planner import AStarPlanner
from pathfinder.planning.path_orchestrator import PathOrchestrator
from pathfinder.robot.robot_state import RobotState
from pathfinder.ros.path_follow_action_client import PathFollowActionClient
from pathfinder.ros.path_request_service import PathRequestService
from pathfinder.ros.pose_tracker import PoseTracker
from pathfinder.world.graph import Graph
from pathfinder.world.robot_config import Robot


class PathServerNode:
    """ROS node: builds path planning, action clients, and services from config values."""

    def __init__(
        self,
        graph: Graph,
        robots: list[Robot],
        sim: bool,
    ) -> None:
        """Assemble all path-server components from the given configuration values."""
        robot_ids = [robot.id for robot in robots]
        robot_odom_topics = {
            robot_id: f"/{robot_id}/sim/odom" if sim else f"/{robot_id}/odom"
            for robot_id in robot_ids
        }
        robot_action_namespaces = {
            robot_id: f"{robot_id}/sim" if sim else robot_id
            for robot_id in robot_ids
        }

        planner = AStarPlanner(graph)
        orchestrator = PathOrchestrator(graph, planner, known_robots=robot_ids)
        tracker = PoseTracker()
        clients = {
            robot_id: PathFollowActionClient(robot_action_namespaces[robot_id])
            for robot_id in robot_ids
        }

        self._robot_odom_topics = robot_odom_topics
        self._tracker = tracker
        self._request_service = PathRequestService(orchestrator, tracker, clients)

    def start(self) -> None:
        """Register odom subscribers and start the path services."""
        for robot_id, topic in self._robot_odom_topics.items():
            rospy.Subscriber(
                topic,
                Odometry,
                lambda message, namespace=robot_id: self._on_odom(namespace, message),
            )
        self._request_service.start()
        rospy.loginfo("PathServerNode started.")

    def _on_odom(self, namespace: str, message: Odometry) -> None:
        self._tracker.update(namespace, RobotState.from_odometry(namespace, message).get_pose())
