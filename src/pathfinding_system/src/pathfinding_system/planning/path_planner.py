from __future__ import annotations
import heapq
from pathfinding_system.world.graph import Graph
from pathfinding_system.world.node import Node
from pathfinding_system.planning.path import Path


class PathPlanner:
    def __init__(self, graph: Graph) -> None:
        self._graph = graph

    def plan(self, start: Node, goal: Node) -> Path:
        # A* with Euclidean heuristic.
        # Tiebreak: when f-costs are equal, prefer path with lexicographically smaller
        # node-ID sequence (so lower-ID neighbors are preferred on ties).
        g_costs: dict[int, float] = {start.id: 0.0}
        counter = 0
        open_heap: list = []
        heapq.heappush(
            open_heap,
            (start.distance_to(goal), (start.id,), counter, start, [start]),
        )
        visited: set[int] = set()

        while open_heap:
            _f, _pids, _c, current, path_so_far = heapq.heappop(open_heap)
            if current.id in visited:
                continue
            visited.add(current.id)

            if current.id == goal.id:
                return Path(path_so_far)

            g = g_costs[current.id]
            for neighbor in sorted(self._graph.get_neighbors(current), key=lambda n: n.id):
                if neighbor.id in visited:
                    continue
                new_g = g + self._graph.edge_cost(current, neighbor)
                best_g = g_costs.get(neighbor.id, float('inf'))
                if new_g <= best_g:
                    if new_g < best_g:
                        g_costs[neighbor.id] = new_g
                    h = neighbor.distance_to(goal)
                    counter += 1
                    heapq.heappush(
                        open_heap,
                        (new_g + h, _pids + (neighbor.id,), counter, neighbor, path_so_far + [neighbor]),
                    )

        raise ValueError(f"no path from {start.id} to {goal.id}")
