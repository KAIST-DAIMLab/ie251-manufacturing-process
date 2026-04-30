from __future__ import annotations
import yaml
from pathfinding_system.world.node import Node
from pathfinding_system.world.edge import Edge


class Graph:
    def __init__(self, nodes: list[Node], edges: list[Edge]) -> None:
        self._nodes: dict[int, Node] = {n.id: n for n in nodes}
        self._adj: dict[int, list[Edge]] = {n.id: [] for n in nodes}
        for edge in edges:
            self._adj[edge.from_node.id].append(edge)
            self._adj[edge.to_node.id].append(Edge(edge.to_node, edge.from_node))

    def get_node(self, node_id: int) -> Node:
        return self._nodes[node_id]

    def get_neighbors(self, node: Node) -> list[Node]:
        return [e.to_node for e in self._adj[node.id]]

    def edge_cost(self, a: Node, b: Node) -> float:
        for e in self._adj[a.id]:
            if e.to_node.id == b.id:
                return e.cost
        raise KeyError(f"no edge from {a.id} to {b.id}")

    def has_edge(self, a: Node, b: Node) -> bool:
        return any(e.to_node.id == b.id for e in self._adj[a.id])

    @classmethod
    def load_from_yaml(cls, path: str) -> Graph:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        nodes = [Node(id=n['id'], x=float(n['x']), y=float(n['y'])) for n in data['nodes']]
        node_map = {n.id: n for n in nodes}
        edges = [Edge(node_map[e['from']], node_map[e['to']]) for e in data['edges']]
        return cls(nodes, edges)
