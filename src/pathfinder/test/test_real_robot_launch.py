from __future__ import annotations
import importlib.util
import importlib.machinery
import contextlib
import io
import os
import subprocess
import sys
import tempfile
import types
import unittest

import yaml


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SCRIPT_PATH = os.path.join(PACKAGE_ROOT, 'scripts', 'real_robot_launch')


def _load_module():
    original_rospkg = sys.modules.get('rospkg')
    rospkg = types.ModuleType('rospkg')

    class RosPack:
        def get_path(self, package):
            return PACKAGE_ROOT

    rospkg.RosPack = RosPack
    sys.modules['rospkg'] = rospkg

    loader = importlib.machinery.SourceFileLoader('real_robot_launch', SCRIPT_PATH)
    spec = importlib.util.spec_from_loader('real_robot_launch', loader)
    module = importlib.util.module_from_spec(spec)
    sys.modules['real_robot_launch'] = module
    try:
        spec.loader.exec_module(module)
    finally:
        if original_rospkg is None:
            sys.modules.pop('rospkg', None)
        else:
            sys.modules['rospkg'] = original_rospkg
    return module


def _write_robots_config(robot_ids: list[str]) -> str:
    handle = tempfile.NamedTemporaryFile('w', delete=False)
    with handle:
        yaml.safe_dump({'robots': [{'id': robot_id, 'start_station': 1} for robot_id in robot_ids]}, handle)
    return handle.name


class RealRobotLaunchTest(unittest.TestCase):
    def setUp(self):
        self.module = _load_module()

    def test_reads_robot_ids_from_yaml_and_probes_scan_topics(self):
        robots_config = _write_robots_config(['tb3_01', 'tb3_07'])
        probed = []

        def fake_has_publishers(topic):
            probed.append(topic)
            return topic == '/tb3_07/scan'

        with contextlib.redirect_stdout(io.StringIO()):
            cmd = self.module.build_roslaunch_command(
                ['robots_config:=' + robots_config, 'open_rviz:=false'],
                has_publishers=fake_has_publishers,
            )

        self.assertEqual(probed, ['/tb3_01/scan', '/tb3_07/scan'])
        self.assertEqual(cmd, [
            'roslaunch',
            'pathfinder',
            'robots.launch',
            'active_robot_ids:=tb3_07',
            'robots_config:=' + robots_config,
            'open_rviz:=false',
        ])

    def test_exits_when_no_configured_robots_are_detected(self):
        robots_config = _write_robots_config(['tb3_01'])

        with self.assertRaises(SystemExit) as raised:
            with contextlib.redirect_stdout(io.StringIO()):
                self.module.build_roslaunch_command(
                    ['robots_config:=' + robots_config],
                    has_publishers=lambda topic: False,
                )

        self.assertEqual(raised.exception.code, 1)


if __name__ == '__main__':
    unittest.main()
