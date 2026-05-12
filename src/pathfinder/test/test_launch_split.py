import os
import unittest
import xml.etree.ElementTree as ET


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
LAUNCH_DIR = os.path.join(ROOT, 'launch')


def _launch_tree(filename):
    return ET.parse(os.path.join(LAUNCH_DIR, filename)).getroot()


class LaunchSplitTest(unittest.TestCase):
    def test_simulation_launch_can_optionally_start_system_stack_in_sim_mode(self):
        root = _launch_tree('simulation.launch')

        start_system_arg = root.find("./arg[@name='start_system']")

        self.assertEqual(start_system_arg.get('default'), 'true')
        self.assertEqual(root.findall("./include[@file='$(find pathfinder)/launch/robots.launch']"), [])

        for node_type in ('path_server', 'robot'):
            node = root.find(f"./node[@type='{node_type}']")
            self.assertIsNotNone(node)
            self.assertEqual(node.get('if'), '$(arg start_system)')
            self.assertIsNotNone(node.find("./rosparam[@file='$(arg robots_config)']"))
            self.assertEqual(node.find("./param[@name='graph_file']").get('value'), '$(arg graph_config)')
            self.assertEqual(node.find("./param[@name='sim']").get('value'), 'true')

    def test_simulation_launch_starts_rosbridge_for_web_ui(self):
        root = _launch_tree('simulation.launch')

        rosbridge_arg = root.find("./arg[@name='rosbridge']")
        rosbridge_port_arg = root.find("./arg[@name='rosbridge_port']")
        self.assertEqual(rosbridge_arg.get('default'), 'true')
        self.assertEqual(rosbridge_port_arg.get('default'), '9090')

        rosbridge_include = root.find("./include[@file='$(find rosbridge_server)/launch/rosbridge_websocket.launch']")
        self.assertIsNotNone(rosbridge_include)
        self.assertEqual(rosbridge_include.get('if'), '$(arg rosbridge)')
        self.assertEqual(rosbridge_include.find("./arg[@name='port']").get('value'), '$(arg rosbridge_port)')

    def test_robots_launch_has_real_robot_config_defaults(self):
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
        self.assertIsNone(root.find("./arg[@name='sim']"))

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
            graph_param = node.find("./param[@name='graph_file']")

            self.assertIsNotNone(robots_config)
            self.assertIsNone(node.find("./param[@name='sim']"))
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

        real_group = root.find("./group")
        self.assertIsNotNone(real_group)
        self.assertIsNone(real_group.get('unless'))

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
