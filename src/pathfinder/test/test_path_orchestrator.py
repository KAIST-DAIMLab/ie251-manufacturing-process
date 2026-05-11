from __future__ import annotations
import os
import sys
import types
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)

from pathfinder.world.node import Node
from pathfinder.world.edge import Edge
from pathfinder.world.graph import Graph
from pathfinder.planning.path_orchestrator import (
    PathOrchestrator,
    NodeNotFoundError,
    NoPathError,
)


def _make_graph() -> Graph:
    node_a = Node(id=1, x=0.0, y=0.0)
    node_b = Node(id=2, x=3.0, y=0.0)
    node_c = Node(id=3, x=6.0, y=0.0)
    return Graph(
        nodes=[node_a, node_b, node_c],
        edges=[Edge(node_a, node_b), Edge(node_b, node_c)],
    )


class FixedPathPlanner:
    """Returns a fixed sequence of nodes regardless of start/goal."""

    def __init__(self, path: list[Node]) -> None:
        self._path = path

    def plan(self, start: Node, goal: Node) -> list[Node]:
        return self._path


class FailingPlanner:
    """Always raises ValueError to simulate A* failure."""

    def plan(self, start: Node, goal: Node) -> list[Node]:
        raise ValueError("no path exists")


class TestPathOrchestratorHappyPath(unittest.TestCase):
    def test_drops_resolved_start_from_returned_path(self):
        graph = _make_graph()
        planner_path = [graph.get_node(1), graph.get_node(2), graph.get_node(3)]
        orchestrator = PathOrchestrator(graph, FixedPathPlanner(planner_path))

        pose = types.SimpleNamespace(x=0.5, y=0.0)
        result = orchestrator.plan(pose, target_node_id=3)

        self.assertEqual(result, [2, 3])

    def test_resolves_nearest_node_as_start(self):
        graph = _make_graph()
        received_starts = []

        class CapturingPlanner:
            def plan(self, start: Node, goal: Node) -> list[Node]:
                received_starts.append(start)
                return [start, goal]

        orchestrator = PathOrchestrator(graph, CapturingPlanner())
        pose = types.SimpleNamespace(x=5.8, y=0.0)
        orchestrator.plan(pose, target_node_id=1)

        self.assertEqual(received_starts[0].id, 3)

    def test_target_equal_to_start_returns_single_target(self):
        graph = _make_graph()
        orchestrator = PathOrchestrator(graph, FixedPathPlanner([graph.get_node(1)]))

        pose = types.SimpleNamespace(x=0.0, y=0.0)
        result = orchestrator.plan(pose, target_node_id=1)

        self.assertEqual(result, [1])


class TestPathOrchestratorErrors(unittest.TestCase):
    def test_missing_target_node_raises_node_not_found_error(self):
        graph = _make_graph()
        orchestrator = PathOrchestrator(graph, FixedPathPlanner([]))

        pose = types.SimpleNamespace(x=0.0, y=0.0)
        with self.assertRaises(NodeNotFoundError):
            orchestrator.plan(pose, target_node_id=999)

    def test_no_path_raises_no_path_error(self):
        graph = _make_graph()
        orchestrator = PathOrchestrator(graph, FailingPlanner())

        pose = types.SimpleNamespace(x=0.0, y=0.0)
        with self.assertRaises(NoPathError):
            orchestrator.plan(pose, target_node_id=3)


if __name__ == '__main__':
    unittest.main()
