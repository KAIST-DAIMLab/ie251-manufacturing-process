import os
import unittest
import xml.etree.ElementTree as ET


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
LAUNCH_DIR = os.path.join(ROOT, 'launch')


def _launch_tree(filename):
    return ET.parse(os.path.join(LAUNCH_DIR, filename)).getroot()


class LaunchSplitTest(unittest.TestCase):
    def test_simulation_launch_includes_system_stack_in_sim_mode(self):
        root = _launch_tree('simulation.launch')

        system_includes = [
            include for include in root.findall('include')
            if include.get('file') == '$(find pathfinding_system)/launch/system.launch'
        ]

        self.assertEqual(len(system_includes), 1)
        mode_arg = system_includes[0].find("./arg[@name='mode']")
        self.assertEqual(mode_arg.get('value'), 'sim')

    def test_system_launch_has_real_default_and_single_executor_manager(self):
        root = _launch_tree('system.launch')
        executor_nodes = [
            node for node in root.findall('node')
            if node.get('type') == 'robot_executor_node'
        ]
        executor_manager = root.find("./node[@type='robot_executors_node']")

        self.assertEqual(executor_nodes, [])
        self.assertIsNotNone(executor_manager)
        mode_arg = root.find("./arg[@name='mode']")
        self.assertEqual(mode_arg.get('default'), 'real')

    def test_system_launch_does_not_start_cmd_vel_router(self):
        root = _launch_tree('system.launch')
        router_nodes = [
            node for node in root.findall('node')
            if node.get('type') == 'cmd_vel_router_node'
        ]

        self.assertEqual(router_nodes, [])

    def test_system_launch_loads_robot_config_by_mode_for_runtime_nodes(self):
        root = _launch_tree('system.launch')

        for node_type in ('path_server_node', 'robot_executors_node'):
            node = root.find(f"./node[@type='{node_type}']")
            real_config = node.find("./rosparam[@file='$(find pathfinding_system)/config/robots.real.yaml']")
            sim_config = node.find("./rosparam[@file='$(find pathfinding_system)/config/robots.sim.yaml']")

            self.assertEqual(real_config.get('if'), "$(eval arg('mode') == 'real')")
            self.assertEqual(sim_config.get('if'), "$(eval arg('mode') == 'sim')")

    def test_robot_mode_configs_are_auditable(self):
        with open(os.path.join(ROOT, 'config', 'robots.real.yaml')) as f:
            real_config = f.read()
        with open(os.path.join(ROOT, 'config', 'robots.sim.yaml')) as f:
            sim_config = f.read()

        self.assertIn('robots:\n', real_config)
        self.assertIn('  tb3_01:\n    topic_namespace: tb3_01\n', real_config)
        self.assertIn('  tb3_05:\n    topic_namespace: tb3_05\n', real_config)
        self.assertIn('  tb3_01:\n    topic_namespace: tb3_01/sim\n', sim_config)
        self.assertIn('  tb3_05:\n    topic_namespace: tb3_05/sim\n', sim_config)


if __name__ == '__main__':
    unittest.main()
