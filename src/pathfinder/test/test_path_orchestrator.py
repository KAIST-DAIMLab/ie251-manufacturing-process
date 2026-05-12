from __future__ import annotations
import math
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


def _pose(x, y, theta=0.0):
    return types.SimpleNamespace(x=x, y=y, theta=theta)


class FixedPathPlanner:
    """Returns a fixed sequence of nodes regardless of start/goal."""

    def __init__(self, path: list[Node]) -> None:
        self._path = path

    def plan(self, start: Node, goal: Node) -> list[Node]:
        return self._path


class CapturingPlanner:
    """Records the start passed to plan and returns [start, goal]."""

    def __init__(self) -> None:
        self.starts: list[Node] = []

    def plan(self, start: Node, goal: Node) -> list[Node]:
        self.starts.append(start)
        return [start, goal]


class FailingPlanner:
    """Always raises ValueError to simulate A* failure."""

    def plan(self, start: Node, goal: Node) -> list[Node]:
        raise ValueError("no path exists")


class TestNoEdge(unittest.TestCase):
    """When current_edge is None, fall back to nearest node and drop start."""

    def test_drops_resolved_start_from_returned_path(self):
        graph = _make_graph()
        planner_path = [graph.get_node(1), graph.get_node(2), graph.get_node(3)]
        orchestrator = PathOrchestrator(graph, FixedPathPlanner(planner_path))

        result = orchestrator.plan(_pose(0.5, 0.0), target_node_id=3)

        self.assertEqual(result, [2, 3])

    def test_resolves_nearest_node_as_start(self):
        graph = _make_graph()
        planner = CapturingPlanner()
        orchestrator = PathOrchestrator(graph, planner)

        orchestrator.plan(_pose(5.8, 0.0), target_node_id=1)

        self.assertEqual(planner.starts[0].id, 3)

    def test_target_equal_to_start_returns_single_target(self):
        graph = _make_graph()
        orchestrator = PathOrchestrator(graph, FixedPathPlanner([graph.get_node(1)]))

        result = orchestrator.plan(_pose(0.0, 0.0), target_node_id=1)

        self.assertEqual(result, [1])


class TestAtEdgeEndpoint(unittest.TestCase):
    """When pose is within tolerance of an endpoint of current_edge, that endpoint is the start and is dropped."""

    def test_at_endpoint_a_uses_a_as_start_and_drops_it(self):
        graph = _make_graph()
        planner = CapturingPlanner()
        orchestrator = PathOrchestrator(graph, planner)

        result = orchestrator.plan(
            _pose(0.05, 0.0),
            target_node_id=3,
            current_edge=(1, 2),
        )

        self.assertEqual(planner.starts[0].id, 1)
        self.assertEqual(result, [3])

    def test_at_endpoint_b_uses_b_as_start_and_drops_it(self):
        graph = _make_graph()
        planner = CapturingPlanner()
        orchestrator = PathOrchestrator(graph, planner)

        result = orchestrator.plan(
            _pose(3.0, 0.0),
            target_node_id=3,
            current_edge=(1, 2),
        )

        self.assertEqual(planner.starts[0].id, 2)
        self.assertEqual(result, [3])


class TestMidEdge(unittest.TestCase):
    """When pose is between the two endpoints of current_edge, pick by heading + blocked flag and keep start."""

    def test_mid_edge_blocked_picks_behind_endpoint_and_keeps_it(self):
        graph = _make_graph()
        planner = CapturingPlanner()
        orchestrator = PathOrchestrator(graph, planner)

        result = orchestrator.plan(
            _pose(1.5, 0.0, theta=0.0),
            target_node_id=3,
            current_edge=(1, 2),
            obstacle_blocked=True,
        )

        self.assertEqual(planner.starts[0].id, 1)
        self.assertEqual(result[0], 1)

    def test_mid_edge_not_blocked_picks_forward_endpoint_and_keeps_it(self):
        graph = _make_graph()
        planner = CapturingPlanner()
        orchestrator = PathOrchestrator(graph, planner)

        result = orchestrator.plan(
            _pose(1.5, 0.0, theta=0.0),
            target_node_id=3,
            current_edge=(1, 2),
            obstacle_blocked=False,
        )

        self.assertEqual(planner.starts[0].id, 2)
        self.assertEqual(result[0], 2)

    def test_behind_endpoint_flips_with_heading(self):
        graph = _make_graph()
        planner = CapturingPlanner()
        orchestrator = PathOrchestrator(graph, planner)

        orchestrator.plan(
            _pose(1.5, 0.0, theta=math.pi),
            target_node_id=3,
            current_edge=(1, 2),
            obstacle_blocked=True,
        )

        self.assertEqual(planner.starts[0].id, 2)


class TestPathOrchestratorErrors(unittest.TestCase):
    def test_missing_target_node_raises_node_not_found_error(self):
        graph = _make_graph()
        orchestrator = PathOrchestrator(graph, FixedPathPlanner([]))

        with self.assertRaises(NodeNotFoundError):
            orchestrator.plan(_pose(0.0, 0.0), target_node_id=999)

    def test_no_path_raises_no_path_error(self):
        graph = _make_graph()
        orchestrator = PathOrchestrator(graph, FailingPlanner())

        with self.assertRaises(NoPathError):
            orchestrator.plan(_pose(0.0, 0.0), target_node_id=3)


if __name__ == '__main__':
    unittest.main()
