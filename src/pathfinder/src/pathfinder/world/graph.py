from __future__ import annotations
import yaml
from pathfinder.world.node import Node
from pathfinder.world.edge import Edge
from pathfinder.world.station import Station


class Graph:
    def __init__(self, nodes: list[Node], edges: list[Edge], stations: list[Station] | None = None) -> None:
        self._nodes: dict[int, Node] = {n.id: n for n in nodes}
        self._station_list: list[Station] = list(stations) if stations is not None else [
            Station(id=n.station, node=n, orientation=n.orientation)
            for n in nodes
            if n.station is not None and n.orientation is not None
        ]
        self._stations: dict[int, Node] = {station.id: station.node for station in self._station_list}
        self._edges: list[Edge] = list(edges)
        self._adj: dict[int, list[Edge]] = {n.id: [] for n in nodes}
        for edge in edges:
            self._adj[edge.from_node.id].append(edge)
            self._adj[edge.to_node.id].append(Edge(edge.to_node, edge.from_node))

    def get_node(self, node_id: int) -> Node:
        return self._nodes[node_id]

    def get_station_node(self, station: int) -> Node:
        try:
            return self._stations[station]
        except KeyError:
            raise KeyError(f"station {station} not in graph") from None

    def all_nodes(self) -> list[Node]:
        return list(self._nodes.values())

    def all_edges(self) -> list[Edge]:
        """Return the undirected edge list as originally loaded."""
        return list(self._edges)

    def all_stations(self) -> list[Station]:
        return list(self._station_list)

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
        nodes = [
            Node(
                id=n['id'],
                x=float(n['x']),
                y=float(n['y']),
            )
            for n in data['nodes']
        ]
        node_map = {n.id: n for n in nodes}
        stations = []
        for index, station_data in enumerate(data.get('stations', []), start=1):
            if 'orientation' not in station_data:
                raise KeyError(f"missing 'orientation' for station {index}")
            node_id = int(station_data['node'])
            if node_id not in node_map:
                raise KeyError(f"station {index} references unknown node {node_id}")
            node = node_map[node_id]
            orientation = float(station_data['orientation'])
            node = Node(id=node.id, x=node.x, y=node.y, orientation=orientation, station=index)
            node_map[node_id] = node
            stations.append(Station(id=index, node=node, orientation=orientation))
        nodes = [node_map[n.id] for n in nodes]
        edges = [Edge(node_map[e['from']], node_map[e['to']]) for e in data['edges']]
        return cls(nodes, edges, stations)
