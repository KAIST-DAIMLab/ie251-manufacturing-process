import heapq
from typing import Dict, List, Optional

from tb3_graph_nav.graph import Graph


def dijkstra(graph: Graph, start_id: str, goal_id: str) -> Optional[List[str]]:
    if start_id not in graph.nodes or goal_id not in graph.nodes:
        raise ValueError(f"Node {start_id!r} or {goal_id!r} not in graph")
    if start_id == goal_id:
        return [start_id]

    dist: Dict[str, float] = {nid: float('inf') for nid in graph.nodes}
    prev: Dict[str, Optional[str]] = {nid: None for nid in graph.nodes}
    dist[start_id] = 0.0

    heap = [(0.0, start_id)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        if u == goal_id:
            break
        for v in graph.edges[u]:
            w = graph.nodes[u].distance_to(graph.nodes[v])
            new_dist = dist[u] + w
            if new_dist < dist[v]:
                dist[v] = new_dist
                prev[v] = u
                heapq.heappush(heap, (new_dist, v))

    if dist[goal_id] == float('inf'):
        return None

    path: List[str] = []
    cur: Optional[str] = goal_id
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path
