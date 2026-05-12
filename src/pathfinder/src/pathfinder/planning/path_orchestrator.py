from __future__ import annotations
import math
from typing import TYPE_CHECKING
from pathfinder.utils.physics import wrap_to_pi
from pathfinder.world.graph import Graph
from pathfinder.world.node import Node
from pathfinder.planning.path_planner import PathPlanner

if TYPE_CHECKING:
    from geometry_msgs.msg import Pose2D


class NodeNotFoundError(Exception):
    """Raised when target_node_id does not exist in the graph."""


class NoPathError(Exception):
    """Raised when A* cannot find a path between start and goal nodes."""


class PathOrchestrator:
    """Resolves start node from a pose, plans a route, returns waypoint ids."""

    def __init__(
        self,
        graph: Graph,
        planner: PathPlanner,
    ) -> None:
        """Store graph and planner."""
        self._graph = graph
        self._planner = planner

    def plan(self, current_pose: Pose2D, target_node_id: int, obstacle_blocked: bool = False) -> list[int]:
        """Plan an A* route from the robot's current pose to the target node.

        Default (not blocked): resolved start is treated as a temporary node and
        dropped from the returned list — the robot is at a node, the first
        waypoint is the next real node to drive to.

        When obstacle_blocked is True: the robot is mid-edge between two graph
        nodes, paused by an obstacle in front. Pick the behind endpoint of the
        current edge as A* start and keep it in the returned path so the robot
        drives back along the edge it is currently on instead of cutting across.
        """
        if obstacle_blocked:
            start = self._behind_endpoint(current_pose) or self._nearest_node(current_pose)
        else:
            start = self._nearest_node(current_pose)

        try:
            target = self._graph.get_node(target_node_id)
            waypoints = self._planner.plan(start, target)
        except KeyError as error:
            raise NodeNotFoundError(f"node {target_node_id} not found") from error
        except ValueError as error:
            raise NoPathError(str(error)) from error

        if not obstacle_blocked and len(waypoints) > 1:
            waypoints = waypoints[1:]
        return [node.id for node in waypoints]

    def _nearest_node(self, pose: Pose2D) -> Node:
        return min(
            self._graph.all_nodes(),
            key=lambda node: (node.x - pose.x) ** 2 + (node.y - pose.y) ** 2,
        )

    def _behind_endpoint(self, pose: Pose2D) -> Node | None:
        """If the robot is mid-edge between its two nearest connected nodes, return the one behind its heading."""
        nodes = sorted(
            self._graph.all_nodes(),
            key=lambda n: (n.x - pose.x) ** 2 + (n.y - pose.y) ** 2,
        )
        if len(nodes) < 2 or not self._graph.has_edge(nodes[0], nodes[1]):
            return None
        a, b = nodes[0], nodes[1]
        a_offset = abs(wrap_to_pi(math.atan2(a.y - pose.y, a.x - pose.x) - pose.theta))
        b_offset = abs(wrap_to_pi(math.atan2(b.y - pose.y, b.x - pose.x) - pose.theta))
        return a if a_offset > b_offset else b
