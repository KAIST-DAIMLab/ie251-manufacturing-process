from __future__ import annotations
import importlib.machinery
import importlib.util
import os
import sys
import tempfile
import types
import unittest

import yaml


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SCRIPT_PATH = os.path.join(PACKAGE_ROOT, 'scripts', 'simulation_robots')


def _load_module():
    stubbed_modules = [
        'rospy',
        'rospkg',
        'xacro',
        'roslaunch',
        'roslaunch.scriptapi',
        'roslaunch.core',
        'roslaunch.rlutil',
        'gazebo_msgs',
        'gazebo_msgs.srv',
        'geometry_msgs',
        'geometry_msgs.msg',
    ]
    originals = {name: sys.modules.get(name) for name in stubbed_modules}

    rospy = types.ModuleType('rospy')
    rospy.logerr = lambda *args, **kwargs: None
    sys.modules['rospy'] = rospy

    sys.modules['rospkg'] = types.ModuleType('rospkg')
    sys.modules['xacro'] = types.ModuleType('xacro')

    roslaunch = types.ModuleType('roslaunch')
    roslaunch.scriptapi = types.ModuleType('roslaunch.scriptapi')
    roslaunch.core = types.ModuleType('roslaunch.core')
    roslaunch.rlutil = types.ModuleType('roslaunch.rlutil')
    sys.modules['roslaunch'] = roslaunch
    sys.modules['roslaunch.scriptapi'] = roslaunch.scriptapi
    sys.modules['roslaunch.core'] = roslaunch.core
    sys.modules['roslaunch.rlutil'] = roslaunch.rlutil

    gazebo_msgs = types.ModuleType('gazebo_msgs')
    gazebo_msgs_srv = types.ModuleType('gazebo_msgs.srv')
    gazebo_msgs_srv.SpawnModel = object
    gazebo_msgs_srv.SpawnModelRequest = object
    sys.modules['gazebo_msgs'] = gazebo_msgs
    sys.modules['gazebo_msgs.srv'] = gazebo_msgs_srv

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
    geometry_msgs_msg.Pose = object
    geometry_msgs_msg.Point = object
    geometry_msgs_msg.Quaternion = object
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

    loader = importlib.machinery.SourceFileLoader('simulation_robots', SCRIPT_PATH)
    spec = importlib.util.spec_from_loader('simulation_robots', loader)
    module = importlib.util.module_from_spec(spec)
    sys.modules['simulation_robots'] = module
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


class SimulationRobotsTest(unittest.TestCase):
    def setUp(self):
        self.module = _load_module()

    def test_build_station_map_uses_only_station_nodes(self):
        graph_path = _write_yaml({
            'nodes': [
                {'id': 1, 'x': 0.0, 'y': 0.0, 'station': 3},
                {'id': 2, 'x': 1.0, 'y': 0.0},
            ],
        })

        self.assertEqual(self.module._build_station_map(graph_path), {3: (1, 0.0, 0.0)})

    def test_validate_robot_accepts_start_station(self):
        self.module._validate_robot({'id': 'tb3_01', 'start_station': 3}, {3: (1, 0.0, 0.0)})

    def test_validate_robot_rejects_non_station_start(self):
        with self.assertRaisesRegex(KeyError, "start_station 2"):
            self.module._validate_robot({'id': 'tb3_01', 'start_station': 2}, {3: (1, 0.0, 0.0)})


if __name__ == '__main__':
    unittest.main()
