from __future__ import annotations
import importlib.machinery
import importlib.util
import os
import sys
import types
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
ROOT = os.path.join(PACKAGE_ROOT, 'src')
SCRIPT_PATH = os.path.join(PACKAGE_ROOT, 'scripts', 'simulation_rviz_markers')
sys.path.insert(0, ROOT)


def _load_module():
    stubbed_modules = [
        'rospy',
        'geometry_msgs',
        'geometry_msgs.msg',
        'visualization_msgs',
        'visualization_msgs.msg',
    ]
    originals = {name: sys.modules.get(name) for name in stubbed_modules}

    sys.modules['rospy'] = types.ModuleType('rospy')

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')

    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class Pose:
        def __init__(self):
            self.position = Point()
            self.orientation = Quaternion()

    class Pose2D:
        def __init__(self, x=0.0, y=0.0, theta=0.0):
            self.x = x
            self.y = y
            self.theta = theta

    geometry_msgs_msg.Point = Point
    geometry_msgs_msg.Pose = Pose
    geometry_msgs_msg.Pose2D = Pose2D
    geometry_msgs_msg.Quaternion = Quaternion
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

    visualization_msgs = types.ModuleType('visualization_msgs')
    visualization_msgs_msg = types.ModuleType('visualization_msgs.msg')

    class Marker:
        ADD = 0
        ARROW = 0
        SPHERE_LIST = 7
        LINE_LIST = 5
        TEXT_VIEW_FACING = 9

        def __init__(self):
            self.header = types.SimpleNamespace(frame_id='')
            self.ns = ''
            self.id = 0
            self.type = 0
            self.action = 0
            self.pose = Pose()
            self.scale = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.color = types.SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0)
            self.points = []
            self.text = ''

    class MarkerArray:
        def __init__(self, markers=None):
            self.markers = markers or []

    visualization_msgs_msg.Marker = Marker
    visualization_msgs_msg.MarkerArray = MarkerArray
    sys.modules['visualization_msgs'] = visualization_msgs
    sys.modules['visualization_msgs.msg'] = visualization_msgs_msg

    loader = importlib.machinery.SourceFileLoader('simulation_rviz_markers', SCRIPT_PATH)
    spec = importlib.util.spec_from_loader('simulation_rviz_markers', loader)
    module = importlib.util.module_from_spec(spec)
    sys.modules['simulation_rviz_markers'] = module
    try:
        spec.loader.exec_module(module)
    finally:
        for name, original in originals.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original
    return module


class SimulationRvizMarkersTest(unittest.TestCase):
    def setUp(self):
        self.module = _load_module()

    def test_build_graph_markers_creates_node_and_edge_markers_in_map_frame(self):
        graph_data = {
            'nodes': [
                {'id': 1, 'x': 0.0, 'y': 1.0},
                {'id': 2, 'x': 2.0, 'y': 1.0},
            ],
            'edges': [{'from': 1, 'to': 2}],
        }

        marker_array = self.module.build_graph_markers(graph_data, frame_id='map')

        markers_by_ns = {marker.ns: marker for marker in marker_array.markers}
        self.assertEqual(set(markers_by_ns), {'graph_edges', 'graph_nodes'})
        self.assertEqual(markers_by_ns['graph_edges'].header.frame_id, 'map')
        self.assertEqual(markers_by_ns['graph_edges'].type, self.module.Marker.LINE_LIST)
        self.assertEqual(len(markers_by_ns['graph_edges'].points), 2)
        self.assertEqual(markers_by_ns['graph_nodes'].header.frame_id, 'map')
        self.assertEqual(markers_by_ns['graph_nodes'].type, self.module.Marker.SPHERE_LIST)
        self.assertEqual(len(markers_by_ns['graph_nodes'].points), 2)

    def test_build_robot_markers_uses_display_name_and_pose_heading(self):
        pose = self.module.Pose2D(x=1.0, y=2.0, theta=1.5708)

        marker_array = self.module.build_robot_markers(
            [('tb3_01', 'Robot 01', pose)],
            frame_id='map',
        )

        robot_marker, label_marker = marker_array.markers
        self.assertEqual(robot_marker.ns, 'robots')
        self.assertEqual(robot_marker.header.frame_id, 'map')
        self.assertEqual(robot_marker.pose.position.x, 1.0)
        self.assertEqual(robot_marker.pose.position.y, 2.0)
        self.assertEqual(label_marker.ns, 'robot_labels')
        self.assertEqual(label_marker.text, 'Robot 01')
        self.assertEqual(label_marker.pose.position.z, 0.35)


if __name__ == '__main__':
    unittest.main()
