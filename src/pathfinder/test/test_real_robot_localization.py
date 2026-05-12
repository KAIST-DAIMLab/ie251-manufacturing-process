from __future__ import annotations
import importlib.util
import importlib.machinery
import os
import sys
import tempfile
import types
import unittest

import yaml


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SCRIPT_PATH = os.path.join(PACKAGE_ROOT, 'scripts', 'real_robot_localization')


def _load_module():
    stubbed_modules = ['rospy', 'roslaunch', 'roslaunch.scriptapi', 'roslaunch.core']
    originals = {name: sys.modules.get(name) for name in stubbed_modules}
    rospy = types.ModuleType('rospy')
    rospy.set_param = lambda name, value: None
    rospy.get_param = lambda name, default=None: default
    rospy.init_node = lambda name: None
    rospy.spin = lambda: None
    rospy.loginfo = lambda *args, **kwargs: None
    sys.modules['rospy'] = rospy

    roslaunch = types.ModuleType('roslaunch')
    roslaunch_scriptapi = types.ModuleType('roslaunch.scriptapi')
    roslaunch_core = types.ModuleType('roslaunch.core')

    class Node:
        def __init__(self, package, node_type, name=None, namespace=None, args=None, output=None, **kwargs):
            self.package = package
            self.node_type = node_type
            self.name = name
            self.namespace = namespace
            self.args = args
            self.output = output
            self.remap_args = kwargs.get('remap_args')

    roslaunch_core.Node = Node
    roslaunch.scriptapi = roslaunch_scriptapi
    roslaunch.core = roslaunch_core
    sys.modules['roslaunch'] = roslaunch
    sys.modules['roslaunch.scriptapi'] = roslaunch_scriptapi
    sys.modules['roslaunch.core'] = roslaunch_core

    loader = importlib.machinery.SourceFileLoader('real_robot_localization', SCRIPT_PATH)
    spec = importlib.util.spec_from_loader('real_robot_localization', loader)
    module = importlib.util.module_from_spec(spec)
    sys.modules['real_robot_localization'] = module
    try:
        spec.loader.exec_module(module)
    finally:
        for name, original in originals.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original
    return module


def _write_yaml(content: dict) -> str:
    handle = tempfile.NamedTemporaryFile('w', delete=False)
    with handle:
        yaml.safe_dump(content, handle)
    return handle.name


class RealRobotLocalizationTest(unittest.TestCase):
    def setUp(self):
        self.module = _load_module()

    def test_builds_localization_nodes_for_all_robots_when_active_ids_empty(self):
        robots_path = _write_yaml({
            'robots': [
                {'id': 'tb3_01', 'start_node': 1},
                {'id': 'tb3_05', 'start_node': 4},
            ],
        })
        graph_path = _write_yaml({
            'nodes': [{'id': 1, 'x': 0.0, 'y': 0.0}, {'id': 4, 'x': 0.0, 'y': 1.3}],
            'edges': [],
        })

        plan = self.module.build_launch_plan(robots_path, graph_path, '')

        self.assertEqual([robot.robot_id for robot in plan], ['tb3_01', 'tb3_05'])
        self.assertEqual([node.name for robot in plan for node in robot.nodes], [
            'scan_relay_tb3_01',
            'tb3_01_base_footprint_to_base_link',
            'tb3_01_base_link_to_scan',
            'amcl',
            'scan_relay_tb3_05',
            'tb3_05_base_footprint_to_base_link',
            'tb3_05_base_link_to_scan',
            'amcl',
        ])

    def test_filters_to_active_robot_ids_and_uses_start_node_for_amcl_pose(self):
        robots_path = _write_yaml({
            'robots': [
                {'id': 'tb3_01', 'start_node': 1},
                {'id': 'tb3_05', 'start_node': 4},
            ],
        })
        graph_path = _write_yaml({
            'nodes': [{'id': 1, 'x': 0.0, 'y': 0.0}, {'id': 4, 'x': 0.0, 'y': 1.3}],
            'edges': [],
        })

        plan = self.module.build_launch_plan(robots_path, graph_path, 'tb3_05')

        self.assertEqual([robot.robot_id for robot in plan], ['tb3_05'])
        self.assertEqual(plan[0].params['/tb3_05/amcl/initial_pose_x'], 0.0)
        self.assertEqual(plan[0].params['/tb3_05/amcl/initial_pose_y'], 1.3)
        self.assertEqual(plan[0].params['/tb3_05/amcl/initial_pose_a'], 0.0)
        self.assertEqual(plan[0].params['/scan_relay_tb3_05/input'], '/tb3_05/scan')
        self.assertEqual(plan[0].params['/scan_relay_tb3_05/output'], '/tb3_05/scan_relayed')
        self.assertEqual(plan[0].params['/scan_relay_tb3_05/frame_id'], 'tb3_05/base_scan')

    def test_unknown_start_node_raises_clear_error(self):
        robots_path = _write_yaml({'robots': [{'id': 'tb3_01', 'start_node': 99}]})
        graph_path = _write_yaml({'nodes': [{'id': 1, 'x': 0.0, 'y': 0.0}], 'edges': []})

        with self.assertRaisesRegex(KeyError, "start_node 99"):
            self.module.build_launch_plan(robots_path, graph_path, '')


if __name__ == '__main__':
    unittest.main()
