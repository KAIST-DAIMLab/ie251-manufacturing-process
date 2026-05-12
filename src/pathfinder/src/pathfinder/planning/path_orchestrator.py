from __future__ import annotations
from typing import TYPE_CHECKING
from pathfinder.utils.physics import heading_offset, planar_distance
from pathfinder.world.edge import Edge
from pathfinder.world.graph import Graph
from pathfinder.world.node import Node
from pathfinder.planning.path_planner import PathPlanner

if TYPE_CHECKING:
    from geometry_msgs.msg import Pose2D


class NodeNotFoundError(Exception):
    """Raised when target_node_id does not exist in the graph."""


class NoPathError(Exception):
    """Raised when A* cannot find a path between start and goal nodes."""


_AT_NODE_TOL = 0.15


class PathOrchestrator:
    """Resolves start node from the robot's pose + tracked edge, plans a route, returns waypoint ids."""

    def __init__(
        self,
        graph: Graph,
        planner: PathPlanner,
    ) -> None:
        self._graph = graph
        self._planner = planner

    def plan(
        self,
        current_pose: Pose2D,
        target_node_id: int,
        current_edge: Edge | None = None,
        obstacle_blocked: bool = False,
    ) -> list[int]:
        """Plan an A* route from the robot's current pose to the target node.

        The returned list always begins with the resolved start node so the
        executor sees a real graph edge for every leg it traverses. Start node
        resolution uses the robot's tracked `current_edge`:
        - At an endpoint: that endpoint.
        - Mid-edge, obstacle_blocked: the BEHIND endpoint (forward is impassable).
        - Mid-edge, free: try BOTH endpoints, pick whichever gives the shorter
          total trajectory (robot->endpoint + planner path).
        - No edge: fall back to the nearest node.
        """
        try:
            target = self._graph.get_node(target_node_id)
        except KeyError as error:
            raise NodeNotFoundError(f"node {target_node_id} not found") from error

        candidates = self._candidate_starts(current_pose, current_edge, obstacle_blocked)

        best: tuple[float, list[Node]] | None = None
        for start in candidates:
            try:
                waypoints = self._planner.plan(start, target)
            except ValueError:
                continue
            cost = planar_distance(current_pose, start) + _path_length(waypoints)
            if best is None or cost < best[0]:
                best = (cost, waypoints)

        if best is None:
            raise NoPathError(f"no path from any candidate start to {target_node_id}")

        return [node.id for node in best[1]]

    def _candidate_starts(
        self,
        pose: Pose2D,
        current_edge: Edge | None,
        obstacle_blocked: bool,
    ) -> list[Node]:
        if current_edge is None:
            return [self._nearest_node(pose)]

        a = current_edge.from_node
        b = current_edge.to_node

        if planar_distance(pose, a) <= _AT_NODE_TOL:
            return [a]
        if planar_distance(pose, b) <= _AT_NODE_TOL:
            return [b]

        if obstacle_blocked:
            a_offset = abs(heading_offset(pose, a))
            b_offset = abs(heading_offset(pose, b))
            return [a] if a_offset > b_offset else [b]

        return [a, b]

    def _nearest_node(self, pose: Pose2D) -> Node:
        return min(self._graph.all_nodes(), key=lambda node: planar_distance(pose, node))


def _path_length(waypoints: list[Node]) -> float:
    return sum(planar_distance(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1))
