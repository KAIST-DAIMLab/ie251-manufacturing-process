from __future__ import annotations
from typing import TYPE_CHECKING
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

    def plan(self, current_pose: Pose2D, target_node_id: int) -> list[int]:
        """Resolve nearest start node, plan A* route, return ordered node id list."""
        start = self._nearest_node(current_pose)

        try:
            target = self._graph.get_node(target_node_id)
            waypoints = self._planner.plan(start, target)
        except KeyError as error:
            raise NodeNotFoundError(f"node {target_node_id} not found") from error
        except ValueError as error:
            raise NoPathError(str(error)) from error
            
        return [node.id for node in waypoints]

    def _nearest_node(self, pose: Pose2D) -> Node:
        return min(
            self._graph.all_nodes(),
            key=lambda node: (node.x - pose.x) ** 2 + (node.y - pose.y) ** 2,
        )
