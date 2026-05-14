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
            "start_station": 4,
            "motion": {},
            "obstacle": {},
        })

        self.assertFalse(hasattr(robot, "yaw"))
        self.assertNotIn("yaw", robot.to_dict())

    def test_robot_config_uses_start_station_not_start_node(self):
        robot = Robot.from_dict({
            "id": "tb3_01",
            "start_station": 4,
            "motion": {},
            "obstacle": {},
        })

        self.assertEqual(robot.start_station, 4)
        self.assertFalse(hasattr(robot, "start_node"))
        self.assertEqual(robot.to_dict()["start_station"], 4)
        self.assertNotIn("start_node", robot.to_dict())

    def test_robot_config_includes_display_name(self):
        robot = Robot.from_dict({
            "id": "tb3_01",
            "name": "Robot 01",
            "start_station": 4,
            "motion": {},
            "obstacle": {},
        })

        self.assertEqual(robot.name, "Robot 01")
        self.assertEqual(robot.to_dict()["name"], "Robot 01")

    def test_robot_config_defaults_blank_display_name_to_id(self):
        robot = Robot.from_dict({
            "id": "tb3_01",
            "name": "  ",
            "start_station": 4,
            "motion": {},
            "obstacle": {},
        })

        self.assertEqual(robot.name, "tb3_01")
        self.assertEqual(robot.to_dict()["name"], "tb3_01")

    def test_default_robots_yaml_does_not_include_starting_yaw(self):
        with open(os.path.join(PACKAGE_ROOT, 'config', 'robots.yaml')) as config_file:
            config = yaml.safe_load(config_file)

        self.assertEqual([robot["id"] for robot in config["robots"]], ["tb3_01", "tb3_05"])
        self.assertEqual([robot["name"] for robot in config["robots"]], ["Robot 1", "Robot 2"])
        for robot in config["robots"]:
            self.assertNotIn("yaw", robot)
            self.assertIn("start_station", robot)
            self.assertNotIn("start_node", robot)


if __name__ == '__main__':
    unittest.main()
