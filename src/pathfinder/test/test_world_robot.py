from __future__ import annotations
import os
import sys
import types
import unittest
import yaml

PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
ROOT = os.path.join(PACKAGE_ROOT, 'src')
sys.path.insert(0, ROOT)

geometry_msgs = types.ModuleType('geometry_msgs')
geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')


class Pose2D:
    pass


geometry_msgs_msg.Pose2D = Pose2D
sys.modules['geometry_msgs'] = geometry_msgs
sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

from pathfinder.world.robot import Robot


class RobotConfigTest(unittest.TestCase):
    def test_robot_config_does_not_include_starting_yaw(self):
        robot = Robot.from_dict({
            "id": "tb3_01",
            "start_node": 9,
            "motion": {},
            "obstacle": {},
        })

        self.assertFalse(hasattr(robot, "yaw"))
        self.assertNotIn("yaw", robot.to_dict())

    def test_default_robots_yaml_does_not_include_starting_yaw(self):
        with open(os.path.join(PACKAGE_ROOT, 'config', 'robots.yaml')) as config_file:
            config = yaml.safe_load(config_file)

        self.assertEqual([robot["id"] for robot in config["robots"]], ["tb3_01", "tb3_05"])
        for robot in config["robots"]:
            self.assertNotIn("yaw", robot)


if __name__ == '__main__':
    unittest.main()
