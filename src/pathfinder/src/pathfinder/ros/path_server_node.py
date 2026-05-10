from __future__ import annotations

import rospy
from nav_msgs.msg import Odometry

from pathfinder.planning.a_star_planner import AStarPlanner
from pathfinder.planning.path_orchestrator import PathOrchestrator
from pathfinder.robot.robot_state import RobotState
from pathfinder.ros.path_follow_action_client import PathFollowActionClient
from pathfinder.ros.path_request_service import PathRequestService
from pathfinder.ros.pose_tracker import PoseTracker
from pathfinder.safety.collision_monitor import CollisionMonitor
from pathfinder.safety.linear_predictor import LinearPredictor
from pathfinder.world.graph import Graph


class PathServerNode:
    """ROS node: builds path planning, action clients, safety monitor, and services from config values."""

    def __init__(
        self,
        graph_file: str,
        robots: list[dict],
        sim: bool,
        horizon: float,
        check_rate_hz: float,
        safety_radius: float,
        time_step: float,
    ) -> None:
        """Assemble all path-server components from the given configuration values."""
        robot_ids = [r['id'] for r in robots]
        robot_odom_topics = {
            robot_id: f"/{robot_id}/sim/odom" if sim else f"/{robot_id}/odom"
            for robot_id in robot_ids
        }
        robot_stop_topics = {
            robot_id: f"/{robot_id}/sim/stop" if sim else f"/{robot_id}/stop"
            for robot_id in robot_ids
        }
        robot_action_namespaces = {
            robot_id: f"{robot_id}/sim" if sim else robot_id
            for robot_id in robot_ids
        }

        graph = Graph.load_from_yaml(graph_file)
        planner = AStarPlanner(graph)
        orchestrator = PathOrchestrator(graph, planner, known_robots=robot_ids)
        tracker = PoseTracker()
        clients = {
            robot_id: PathFollowActionClient(robot_action_namespaces[robot_id])
            for robot_id in robot_ids
        }
        predictor = LinearPredictor(safety_radius, time_step)

        self._robot_odom_topics = robot_odom_topics
        self._tracker = tracker
        self._monitor = CollisionMonitor(
            predictor,
            robot_ids,
            horizon,
            check_rate_hz,
            robot_odom_topics=robot_odom_topics,
            robot_stop_topics=robot_stop_topics,
        )
        self._request_service = PathRequestService(orchestrator, tracker, clients)

    def start(self) -> None:
        """Register odom subscribers, start the safety monitor and path services."""
        for robot_id, topic in self._robot_odom_topics.items():
            rospy.Subscriber(
                topic,
                Odometry,
                lambda message, namespace=robot_id: self._on_odom(namespace, message),
            )
        self._monitor.start()
        self._request_service.start()
        rospy.loginfo("PathServerNode started.")

    def _on_odom(self, namespace: str, message: Odometry) -> None:
        self._tracker.update(namespace, RobotState.from_odometry(namespace, message).get_pose())
