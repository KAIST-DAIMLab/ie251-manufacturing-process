import os
import unittest
import xml.etree.ElementTree as ET


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
LAUNCH_DIR = os.path.join(ROOT, 'launch')


def _launch_tree(filename):
    return ET.parse(os.path.join(LAUNCH_DIR, filename)).getroot()


class LaunchSplitTest(unittest.TestCase):
    def test_simulation_launch_can_optionally_include_system_stack_in_sim_mode(self):
        root = _launch_tree('simulation.launch')

        system_includes = [
            include for include in root.findall('include')
            if include.get('file') == '$(find pathfinder)/launch/system.launch'
        ]
        start_system_arg = root.find("./arg[@name='start_system']")

        self.assertEqual(len(system_includes), 1)
        self.assertEqual(start_system_arg.get('default'), 'false')
        robots_config_arg = system_includes[0].find("./arg[@name='robots_config']")
        sim_arg = system_includes[0].find("./arg[@name='sim']")
        self.assertEqual(robots_config_arg.get('value'), '$(arg robots_config)')
        self.assertEqual(sim_arg.get('value'), 'true')

    def test_system_launch_has_config_and_sim_defaults(self):
        root = _launch_tree('system.launch')
        executor_nodes = [
            node for node in root.findall('node')
            if node.get('type') == 'robot_executor_node'
        ]
        executor_manager = root.find("./node[@type='robot_executors_node']")

        self.assertEqual(executor_nodes, [])
        self.assertIsNotNone(executor_manager)
        config_arg = root.find("./arg[@name='robots_config']")
        self.assertEqual(config_arg.get('default'), '$(find pathfinder)/config/robot.yaml')
        sim_arg = root.find("./arg[@name='sim']")
        self.assertEqual(sim_arg.get('default'), 'false')

    def test_system_launch_does_not_start_cmd_vel_router(self):
        root = _launch_tree('system.launch')
        router_nodes = [
            node for node in root.findall('node')
            if node.get('type') == 'cmd_vel_router_node'
        ]

        self.assertEqual(router_nodes, [])

    def test_system_launch_loads_robot_config_for_runtime_nodes(self):
        root = _launch_tree('system.launch')

        for node_type in ('path_server_node', 'robot_executors_node'):
            node = root.find(f"./node[@type='{node_type}']")
            robots_config = node.find("./rosparam[@file='$(arg robots_config)']")
            sim_param = node.find("./param[@name='sim']")

            self.assertIsNotNone(robots_config)
            self.assertEqual(sim_param.get('value'), '$(arg sim)')

    def test_robot_config_is_auditable(self):
        with open(os.path.join(ROOT, 'config', 'robot.yaml')) as f:
            config = f.read()

        self.assertIn('robot_ids:\n', config)
        self.assertIn('  - tb3_01\n', config)
        self.assertIn('  - tb3_05\n', config)


if __name__ == '__main__':
    unittest.main()
