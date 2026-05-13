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
    def test_load_from_yaml_reads_top_level_stations_in_order(self):
        graph = _load_graph_yaml("""
            nodes:
              - {id: 1, x: 0.0, y: 0.0}
              - {id: 2, x: 1.0, y: 0.0}
            edges:
              - {from: 1, to: 2}
            stations:
              - {node: 2, orientation: 180}
              - {node: 1, orientation: 0}
        """)

        stations = graph.all_stations()
        self.assertEqual([(station.id, station.node.id, station.orientation) for station in stations], [
            (1, 2, 180.0),
            (2, 1, 0.0),
        ])
        self.assertEqual(graph.get_station_node(1).id, 2)
        self.assertEqual(graph.get_station_node(2).id, 1)
        self.assertEqual(graph.get_node(2).station, 1)
        self.assertEqual(graph.get_node(2).orientation, 180.0)
        self.assertEqual(graph.get_node(1).station, 2)
        self.assertEqual(graph.get_node(1).orientation, 0.0)

    def test_load_from_yaml_without_stations_has_no_station_nodes(self):
        graph = _load_graph_yaml("""
            nodes:
              - {id: 1, x: 0.0, y: 0.0}
            edges: []
        """)

        self.assertEqual(graph.all_stations(), [])
        self.assertIsNone(graph.get_node(1).station)
        self.assertIsNone(graph.get_node(1).orientation)

    def test_load_from_yaml_rejects_station_with_unknown_node(self):
        with self.assertRaisesRegex(KeyError, "station 1 references unknown node 99"):
            _load_graph_yaml("""
                nodes:
                  - {id: 1, x: 0.0, y: 0.0}
                edges: []
                stations:
                  - {node: 99, orientation: 0}
            """)

    def test_load_from_yaml_requires_station_orientation(self):
        with self.assertRaisesRegex(KeyError, "missing 'orientation' for station 1"):
            _load_graph_yaml("""
                nodes:
                  - {id: 1, x: 0.0, y: 0.0}
                edges: []
                stations:
                  - {node: 1}
            """)

    def test_load_from_yaml_ignores_station_and_orientation_fields_on_nodes(self):
        graph = _load_graph_yaml("""
            nodes:
              - {id: 1, x: 0.0, y: 0.0, station: 9, orientation: 45}
            edges: []
        """)

        self.assertEqual(graph.all_stations(), [])
        self.assertIsNone(graph.get_node(1).station)
        self.assertIsNone(graph.get_node(1).orientation)

    def test_get_station_node_returns_node_for_station(self):
        graph = _load_graph_yaml("""
            nodes:
              - {id: 1, x: 0.0, y: 0.0}
              - {id: 2, x: 1.0, y: 0.0}
            edges:
              - {from: 1, to: 2}
            stations:
              - {node: 2, orientation: 180}
              - {node: 1, orientation: 0}
        """)

        self.assertEqual(graph.get_station_node(2).id, 1)

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
