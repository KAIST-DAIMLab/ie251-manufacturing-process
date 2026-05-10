from __future__ import annotations
import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)

from pathfinder.world.node import Node
from pathfinder.world.edge import Edge
from pathfinder.world.graph import Graph
from pathfinder.planning.a_star_planner import AStarPlanner


def _build_graph(node_tuples: list[tuple[int, float, float]], edge_pairs: list[tuple[int, int]]) -> Graph:
    nodes = [Node(id=node_id, x=x, y=y) for node_id, x, y in node_tuples]
    node_map = {n.id: n for n in nodes}
    edges = [Edge(node_map[from_id], node_map[to_id]) for from_id, to_id in edge_pairs]
    return Graph(nodes, edges)


class AStarPlannerTest(unittest.TestCase):
    def test_basic_path_found(self):
        graph = _build_graph(
            [(0, 0.0, 0.0), (1, 1.0, 0.0), (2, 2.0, 0.0)],
            [(0, 1), (1, 2)],
        )
        planner = AStarPlanner(graph)
        path = planner.plan(graph.get_node(0), graph.get_node(2))
        self.assertEqual([node.id for node in path], [0, 1, 2])

    def test_no_path_raises_value_error(self):
        graph = _build_graph(
            [(0, 0.0, 0.0), (1, 10.0, 0.0)],
            [],
        )
        planner = AStarPlanner(graph)
        with self.assertRaises(ValueError):
            planner.plan(graph.get_node(0), graph.get_node(1))

    def test_single_node_graph_start_equals_goal(self):
        graph = _build_graph(
            [(0, 0.0, 0.0)],
            [],
        )
        planner = AStarPlanner(graph)
        path = planner.plan(graph.get_node(0), graph.get_node(0))
        self.assertEqual([node.id for node in path], [0])

    def test_chooses_shorter_path_when_multiple_routes_exist(self):
        graph = _build_graph(
            [(0, 0.0, 0.0), (1, 1.0, 0.0), (2, 2.0, 0.0), (3, 100.0, 0.0)],
            [(0, 1), (1, 2), (0, 3), (3, 2)],
        )
        planner = AStarPlanner(graph)
        path = planner.plan(graph.get_node(0), graph.get_node(2))
        self.assertEqual([node.id for node in path], [0, 1, 2])

    def test_disconnected_graph_raises_value_error(self):
        graph = _build_graph(
            [(0, 0.0, 0.0), (1, 1.0, 0.0), (2, 2.0, 0.0), (3, 5.0, 0.0), (4, 6.0, 0.0)],
            [(0, 1), (1, 2), (3, 4)],
        )
        planner = AStarPlanner(graph)
        with self.assertRaises(ValueError):
            planner.plan(graph.get_node(0), graph.get_node(4))

    def test_g_cost_relaxation_finds_minimum_cost_path(self):
        graph = _build_graph(
            [(0, 0.0, 0.0), (1, 10.0, 0.0), (2, 20.0, 0.0), (3, 10.0, 1.0)],
            [(0, 1), (1, 2), (0, 3), (3, 2)],
        )
        planner = AStarPlanner(graph)
        path = planner.plan(graph.get_node(0), graph.get_node(2))
        self.assertEqual([node.id for node in path], [0, 1, 2])


if __name__ == '__main__':
    unittest.main()
