from __future__ import annotations
import importlib.machinery
import importlib.util
import os
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SCRIPT_PATH = os.path.join(PACKAGE_ROOT, 'scripts', 'real_robot_bringup')


def _load_module():
    loader = importlib.machinery.SourceFileLoader('real_robot_bringup', SCRIPT_PATH)
    spec = importlib.util.spec_from_loader('real_robot_bringup', loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class RealRobotBringupTest(unittest.TestCase):
    def test_builds_namespaced_bringup_command_with_raw_tf_remapped(self):
        module = _load_module()

        command = module.build_command('tb3_01', ['foo:=bar'])

        self.assertEqual(command, [
            'roslaunch',
            'turtlebot3_bringup',
            'turtlebot3_robot.launch',
            '/tf:=/tb3_01/raw_tf',
            '/tf_static:=/tb3_01/raw_tf_static',
            'foo:=bar',
        ])

    def test_builds_environment_with_robot_namespace(self):
        module = _load_module()

        env = module.build_environment('tb3_05', {'ROS_MASTER_URI': 'http://master:11311'})

        self.assertEqual(env['ROS_MASTER_URI'], 'http://master:11311')
        self.assertEqual(env['ROS_NAMESPACE'], 'tb3_05')


if __name__ == '__main__':
    unittest.main()
