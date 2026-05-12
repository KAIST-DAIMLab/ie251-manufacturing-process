from __future__ import annotations

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, String

from pathfinder.planning.a_star_planner import AStarPlanner
from pathfinder.planning.path_orchestrator import PathOrchestrator
from pathfinder.ros.path_follow_action_client import PathFollowActionClient
from pathfinder.ros.fleet_service import FleetService
from pathfinder.world.edge import Edge
from pathfinder.world.graph import Graph
from pathfinder.world.robot import Robot


class PathServerNode:
    """ROS node: builds path planning, action clients, and services from config values."""

    def __init__(
        self,
        graph: Graph,
        robots: list[Robot],
    ) -> None:
        """Assemble all path-server components from the given configuration values."""
        planner = AStarPlanner(graph)
        orchestrator = PathOrchestrator(graph, planner)
        clients = [PathFollowActionClient(robot.id, robot.namespace) for robot in robots]

        self._graph = graph
        self._robots = robots
        self._request_service = FleetService(graph, orchestrator, robots, clients)

    def start(self) -> None:
        """Register pose, obstacle_blocked, and current_edge subscribers and start the path services."""
        for robot in self._robots:
            rospy.Subscriber(f"/{robot.namespace}/pose", Pose2D, robot.set_pose)
            rospy.Subscriber(
                f"/{robot.namespace}/obstacle_blocked",
                Bool,
                lambda msg, r=robot: r.set_obstacle_blocked(msg.data),
            )
            rospy.Subscriber(
                f"/{robot.namespace}/current_edge",
                String,
                lambda msg, r=robot: r.set_current_edge(Edge.decode_wire(msg.data, self._graph)),
            )
        self._request_service.start()
        rospy.loginfo("PathServerNode started.")
