import math
import yaml
from typing import Dict, List, Optional, Tuple


class Node:
    def __init__(self, node_id: str, x: float, y: float):
        self.node_id = node_id
        self.x = x
        self.y = y

    def distance_to(self, other: 'Node') -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def distance_to_point(self, x: float, y: float) -> float:
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2)


class Graph:
    def __init__(self):
        self.nodes: Dict[str, Node] = {}
        self.edges: Dict[str, List[str]] = {}

    def add_node(self, node_id: str, x: float, y: float) -> None:
        self.nodes[node_id] = Node(node_id, x, y)
        self.edges[node_id] = []

    def add_edge(self, node_a: str, node_b: str) -> None:
        if node_a not in self.nodes or node_b not in self.nodes:
            raise ValueError(f"Node {node_a!r} or {node_b!r} not in graph")
        self.edges[node_a].append(node_b)
        self.edges[node_b].append(node_a)

    def nearest_node(self, x: float, y: float) -> Tuple[str, float]:
        best_id: Optional[str] = None
        best_dist = float('inf')
        for node_id, node in self.nodes.items():
            d = node.distance_to_point(x, y)
            if d < best_dist:
                best_dist = d
                best_id = node_id
        return best_id, best_dist

    @classmethod
    def from_yaml(cls, path: str) -> 'Graph':
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        g = cls()
        for node_id, coords in data['nodes'].items():
            g.add_node(node_id, float(coords['x']), float(coords['y']))
        for edge in data['edges']:
            g.add_edge(str(edge[0]), str(edge[1]))
        return g
