import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import pytest
from tb3_graph_nav.graph import Graph
from tb3_graph_nav.planner import dijkstra


def _is_valid_path(graph, path):
    """Every consecutive pair of node IDs must be a real edge in the graph."""
    return all(path[i+1] in graph.edges[path[i]] for i in range(len(path)-1))


@pytest.fixture
def linear_graph():
    """A — B — C — D  (1 m apart on x-axis)"""
    g = Graph()
    for i, nid in enumerate(['A', 'B', 'C', 'D']):
        g.add_node(nid, float(i), 0.0)
    g.add_edge('A', 'B')
    g.add_edge('B', 'C')
    g.add_edge('C', 'D')
    return g


@pytest.fixture
def grid_graph():
    """2x2 grid: A-B / C-D with no diagonals"""
    g = Graph()
    g.add_node('A', 0.0, 0.0)
    g.add_node('B', 1.0, 0.0)
    g.add_node('C', 0.0, 1.0)
    g.add_node('D', 1.0, 1.0)
    g.add_edge('A', 'B')
    g.add_edge('A', 'C')
    g.add_edge('B', 'D')
    g.add_edge('C', 'D')
    return g


def test_same_start_goal_returns_single_node(linear_graph):
    assert dijkstra(linear_graph, 'A', 'A') == ['A']


def test_adjacent_nodes(linear_graph):
    assert dijkstra(linear_graph, 'A', 'B') == ['A', 'B']


def test_three_hop_path(linear_graph):
    path = dijkstra(linear_graph, 'A', 'D')
    assert path == ['A', 'B', 'C', 'D']
    assert _is_valid_path(linear_graph, path)


def test_reverse_path(linear_graph):
    assert dijkstra(linear_graph, 'D', 'A') == ['D', 'C', 'B', 'A']


def test_grid_path_length(grid_graph):
    path = dijkstra(grid_graph, 'A', 'D')
    assert path[0] == 'A'
    assert path[-1] == 'D'
    assert len(path) == 3  # 2 hops
    assert _is_valid_path(grid_graph, path)


def test_disconnected_graph_returns_none():
    g = Graph()
    g.add_node('A', 0.0, 0.0)
    g.add_node('B', 1.0, 0.0)
    # no edge added
    assert dijkstra(g, 'A', 'B') is None


def test_unknown_goal_raises(linear_graph):
    with pytest.raises(ValueError, match="not in graph"):
        dijkstra(linear_graph, 'A', 'Z')


def test_unknown_start_raises(linear_graph):
    with pytest.raises(ValueError, match="not in graph"):
        dijkstra(linear_graph, 'Z', 'A')
