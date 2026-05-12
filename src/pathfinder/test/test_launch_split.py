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
            if include.get('file') == '$(find pathfinder)/launch/robots.launch'
        ]
        start_system_arg = root.find("./arg[@name='start_system']")

        self.assertEqual(len(system_includes), 1)
        self.assertEqual(start_system_arg.get('default'), 'true')
        robots_config_arg = system_includes[0].find("./arg[@name='robots_config']")
        graph_config_arg = system_includes[0].find("./arg[@name='graph_config']")
        sim_arg = system_includes[0].find("./arg[@name='sim']")
        self.assertEqual(robots_config_arg.get('value'), '$(arg robots_config)')
        self.assertEqual(graph_config_arg.get('value'), '$(arg graph_config)')
        self.assertEqual(sim_arg.get('value'), 'true')

    def test_robots_launch_has_config_and_sim_defaults(self):
        root = _launch_tree('robots.launch')
        executor_nodes = [
            node for node in root.findall('node')
            if node.get('type') == 'robot_executor_node'
        ]
        executor_manager = root.find("./node[@type='robot']")

        self.assertEqual(executor_nodes, [])
        self.assertIsNotNone(executor_manager)
        config_arg = root.find("./arg[@name='robots_config']")
        self.assertEqual(config_arg.get('default'), '$(find pathfinder)/config/robots.yaml')
        graph_arg = root.find("./arg[@name='graph_config']")
        self.assertEqual(graph_arg.get('default'), '$(find pathfinder)/config/graph.yaml')
        sim_arg = root.find("./arg[@name='sim']")
        self.assertEqual(sim_arg.get('default'), 'false')

    def test_robots_launch_does_not_start_cmd_vel_router(self):
        root = _launch_tree('robots.launch')
        router_nodes = [
            node for node in root.findall('node')
            if node.get('type') == 'cmd_vel_router_node'
        ]

        self.assertEqual(router_nodes, [])

    def test_robots_launch_loads_robot_config_for_runtime_nodes(self):
        root = _launch_tree('robots.launch')

        for node_type in ('path_server', 'robot'):
            node = root.find(f"./node[@type='{node_type}']")
            robots_config = node.find("./rosparam[@file='$(arg robots_config)']")
            sim_param = node.find("./param[@name='sim']")
            graph_param = node.find("./param[@name='graph_file']")

            self.assertIsNotNone(robots_config)
            self.assertEqual(sim_param.get('value'), '$(arg sim)')
            self.assertEqual(graph_param.get('value'), '$(arg graph_config)')

    def test_robot_config_is_auditable(self):
        with open(os.path.join(ROOT, 'config', 'robots.yaml')) as f:
            config = f.read()

        self.assertIn('robots:\n', config)
        self.assertIn('id: tb3_01\n', config)
        self.assertIn('id: tb3_05\n', config)

    def test_robots_launch_integrates_real_robot_localization(self):
        root = _launch_tree('robots.launch')

        self.assertIsNone(root.find("./arg[@name='amcl_tb3_01']"))
        self.assertIsNone(root.find("./arg[@name='amcl_tb3_05']"))
        self.assertIsNotNone(root.find("./arg[@name='active_robot_ids']"))
        self.assertIsNotNone(root.find("./arg[@name='graph_config']"))
        self.assertEqual(root.findall(".//group[@ns='tb3_01']"), [])
        self.assertEqual(root.findall(".//group[@ns='tb3_05']"), [])

        real_group = root.find("./group[@unless='$(arg sim)']")
        self.assertIsNotNone(real_group)

        localization_node = real_group.find("./node[@type='real_robot_localization']")
        self.assertIsNotNone(localization_node)
        self.assertIsNotNone(localization_node.find("./param[@name='robots_config'][@value='$(arg robots_config)']"))
        self.assertIsNotNone(localization_node.find("./param[@name='graph_config'][@value='$(arg graph_config)']"))
        self.assertIsNotNone(localization_node.find("./param[@name='active_robot_ids'][@value='$(arg active_robot_ids)']"))

        map_server = real_group.find("./node[@type='map_server']")
        self.assertIsNotNone(map_server)
        self.assertEqual(map_server.get('args'), '$(arg map_file)')

        rviz = real_group.find("./node[@type='rviz']")
        self.assertIsNotNone(rviz)
        self.assertEqual(rviz.get('if'), '$(arg open_rviz)')

    def test_real_robot_launch_is_removed_in_favor_of_robots_launch(self):
        self.assertFalse(os.path.exists(os.path.join(LAUNCH_DIR, 'real_robot.launch')))


if __name__ == '__main__':
    unittest.main()
