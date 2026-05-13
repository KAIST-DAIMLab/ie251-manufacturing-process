from __future__ import annotations
import os
import sys
import tempfile
import textwrap
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)

from pathfinder.world.graph import Graph


class GraphYamlTest(unittest.TestCase):
    def test_load_from_yaml_reads_optional_node_orientation(self):
        graph = _load_graph_yaml("""
            nodes:
              - {id: 1, x: 0.0, y: 0.0, orientation: 90}
              - {id: 2, x: 1.0, y: 0.0}
            edges:
              - {from: 1, to: 2}
        """)

        self.assertEqual(graph.get_node(1).orientation, 90.0)
        self.assertIsNone(graph.get_node(2).orientation)

    def test_load_from_yaml_without_orientation_remains_supported(self):
        graph = _load_graph_yaml("""
            nodes:
              - {id: 1, x: 0.0, y: 0.0}
            edges: []
        """)

        self.assertIsNone(graph.get_node(1).orientation)

    def test_load_from_yaml_treats_null_orientation_as_absent(self):
        graph = _load_graph_yaml("""
            nodes:
              - {id: 1, x: 0.0, y: 0.0, orientation: null}
            edges: []
        """)

        self.assertIsNone(graph.get_node(1).orientation)

    def test_load_from_yaml_reads_optional_node_station(self):
        graph = _load_graph_yaml("""
            nodes:
              - {id: 1, x: 0.0, y: 0.0, station: 1}
              - {id: 2, x: 1.0, y: 0.0}
            edges:
              - {from: 1, to: 2}
        """)

        self.assertEqual(graph.get_node(1).station, 1)
        self.assertIsNone(graph.get_node(2).station)

    def test_load_from_yaml_treats_null_station_as_absent(self):
        graph = _load_graph_yaml("""
            nodes:
              - {id: 1, x: 0.0, y: 0.0, station: null}
            edges: []
        """)

        self.assertIsNone(graph.get_node(1).station)

    def test_get_station_node_returns_node_for_station(self):
        graph = _load_graph_yaml("""
            nodes:
              - {id: 1, x: 0.0, y: 0.0, station: 3}
              - {id: 2, x: 1.0, y: 0.0}
            edges:
              - {from: 1, to: 2}
        """)

        self.assertEqual(graph.get_station_node(3).id, 1)

    def test_get_station_node_rejects_non_station_nodes(self):
        graph = _load_graph_yaml("""
            nodes:
              - {id: 1, x: 0.0, y: 0.0}
            edges: []
        """)

        with self.assertRaisesRegex(KeyError, "station 1"):
            graph.get_station_node(1)


def _load_graph_yaml(contents: str) -> Graph:
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as config_file:
        config_file.write(textwrap.dedent(contents))
        config_path = config_file.name
    try:
        return Graph.load_from_yaml(config_path)
    finally:
        os.unlink(config_path)


if __name__ == '__main__':
    unittest.main()
