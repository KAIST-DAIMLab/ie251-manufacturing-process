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
from pathfinder.world.robot import Robot


class PathServerNode:
    """ROS node: builds path planning, action clients, and services from config values."""

    def __init__(
        self,
        graph: Graph,
        robots: list[Robot],
        sim: bool,
    ) -> None:
        """Assemble all path-server components from the given configuration values."""
        planner = AStarPlanner(graph)
        orchestrator = PathOrchestrator(graph, planner)
        tracker = PoseTracker()
        clients = [PathFollowActionClient(robot.id, robot.namespace) for robot in robots]

        self._robots = robots
        self._tracker = tracker
        self._request_service = PathRequestService(orchestrator, tracker, clients)

    def start(self) -> None:
        """Register odom subscribers and start the path services."""
        for robot in self._robots:
            rospy.Subscriber(
                f"/{robot.namespace}/odom",
                Odometry,
                lambda message, robot_id=robot.id: self._on_odom(robot_id, message),
            )
        self._request_service.start()
        rospy.loginfo("PathServerNode started.")

    def _on_odom(self, robot_id: str, message: Odometry) -> None:
        self._tracker.update(robot_id, RobotState.from_odometry(robot_id, message).get_pose())
