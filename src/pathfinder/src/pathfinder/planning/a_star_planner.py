from __future__ import annotations
import heapq
from pathfinder.world.graph import Graph
from pathfinder.world.node import Node


class AStarPlanner:
    """A* path planner that returns an ordered list of waypoint Nodes."""

    def __init__(self, graph: Graph) -> None:
        """Initialise the planner with the graph to search."""
        self._graph = graph

    def plan(self, start: Node, goal: Node) -> list[Node]:
        """Return the optimal node sequence from start to goal, inclusive."""
        g_costs: dict[int, float] = {start.id: 0.0}
        came_from: dict[int, int] = {}
        visited: set[int] = set()
        counter = 0
        open_heap: list = [(start.distance_to(goal), counter, start)]

        while open_heap:
            _f, _c, current = heapq.heappop(open_heap)
            if current.id in visited:
                continue
            visited.add(current.id)

            if current.id == goal.id:
                return self._reconstruct(came_from, goal)

            for neighbor in self._graph.get_neighbors(current):
                if neighbor.id in visited:
                    continue
                new_g = g_costs[current.id] + self._graph.edge_cost(current, neighbor)
                if new_g < g_costs.get(neighbor.id, float('inf')):
                    g_costs[neighbor.id] = new_g
                    came_from[neighbor.id] = current.id
                    counter += 1
                    f = new_g + neighbor.distance_to(goal)
                    heapq.heappush(open_heap, (f, counter, neighbor))

        raise ValueError(f"no path from {start.id} to {goal.id}")

    def _reconstruct(self, came_from: dict[int, int], goal: Node) -> list[Node]:
        ids = [goal.id]
        while ids[-1] in came_from:
            ids.append(came_from[ids[-1]])
        ids.reverse()
        return [self._graph.get_node(i) for i in ids]
