import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import pytest
import tempfile
import yaml
from tb3_graph_nav.graph import Graph, Node


def test_node_euclidean_distance():
    a = Node('A', 0.0, 0.0)
    b = Node('B', 3.0, 4.0)
    assert abs(a.distance_to(b) - 5.0) < 1e-9


def test_node_distance_to_point():
    n = Node('A', 1.0, 1.0)
    assert abs(n.distance_to_point(4.0, 5.0) - 5.0) < 1e-9


def test_add_node_stores_coords():
    g = Graph()
    g.add_node('A', 1.5, 2.5)
    assert 'A' in g.nodes
    assert g.nodes['A'].x == 1.5
    assert g.nodes['A'].y == 2.5


def test_add_edge_is_bidirectional():
    g = Graph()
    g.add_node('A', 0.0, 0.0)
    g.add_node('B', 1.0, 0.0)
    g.add_edge('A', 'B')
    assert 'B' in g.edges['A']
    assert 'A' in g.edges['B']


def test_add_edge_unknown_node_raises():
    g = Graph()
    g.add_node('A', 0.0, 0.0)
    with pytest.raises(ValueError, match="not in graph"):
        g.add_edge('A', 'Z')


def test_nearest_node_picks_closest():
    g = Graph()
    g.add_node('A', 0.0, 0.0)
    g.add_node('B', 5.0, 0.0)
    node_id, dist = g.nearest_node(1.0, 0.0)
    assert node_id == 'A'
    assert abs(dist - 1.0) < 1e-9


def test_nearest_node_exact_match():
    g = Graph()
    g.add_node('A', 2.0, 3.0)
    node_id, dist = g.nearest_node(2.0, 3.0)
    assert node_id == 'A'
    assert dist == 0.0


def test_from_yaml_loads_nodes_and_edges():
    data = {
        'nodes': {
            'N1': {'x': 0.0, 'y': 0.0},
            'N2': {'x': 1.0, 'y': 0.0},
        },
        'edges': [['N1', 'N2']]
    }
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(data, f)
        path = f.name
    try:
        g = Graph.from_yaml(path)
        assert 'N1' in g.nodes
        assert 'N2' in g.nodes
        assert 'N2' in g.edges['N1']
        assert 'N1' in g.edges['N2']
    finally:
        os.unlink(path)
