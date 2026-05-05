import os
import unittest
import xml.etree.ElementTree as ET


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
LAUNCH_DIR = os.path.join(ROOT, 'launch')


def _launch_tree(filename):
    return ET.parse(os.path.join(LAUNCH_DIR, filename)).getroot()


class LaunchSplitTest(unittest.TestCase):
    def test_simulation_launch_contains_only_simulation_nodes(self):
        root = _launch_tree('simulation.launch')

        node_types = [node.get('type') for node in root.findall('node')]

        self.assertNotIn('path_server_node', node_types)
        self.assertNotIn('robot_executor_node', node_types)

    def test_system_launch_points_executors_at_sim_topics(self):
        root = _launch_tree('system.launch')
        executor_nodes = [
            node for node in root.findall('node')
            if node.get('type') == 'robot_executor_node'
        ]

        self.assertEqual(len(executor_nodes), 2)
        topic_namespaces = {
            node.get('ns'): node.find("./param[@name='topic_namespace']").get('value')
            for node in executor_nodes
        }
        self.assertEqual(topic_namespaces, {
            'tb3_0': 'tb3_0/sim',
            'tb3_1': 'tb3_1/sim',
        })

        explicit_topic_params = [
            node.find("./param[@name='cmd_vel_topic']")
            for node in executor_nodes
        ] + [
            node.find("./param[@name='odom_topic']")
            for node in executor_nodes
        ]
        self.assertEqual(explicit_topic_params, [None, None, None, None])

        derived_odom_topics = {
            ns: f'/{topic_namespace}/odom'
            for ns, topic_namespace in topic_namespaces.items()
        }
        self.assertEqual(derived_odom_topics, {
            'tb3_0': '/tb3_0/sim/odom',
            'tb3_1': '/tb3_1/sim/odom',
        })
    def test_system_launch_does_not_start_cmd_vel_router(self):
        root = _launch_tree('system.launch')
        router_nodes = [
            node for node in root.findall('node')
            if node.get('type') == 'cmd_vel_router_node'
        ]

        self.assertEqual(router_nodes, [])

    def test_system_launch_configures_path_server_odom_topics(self):
        root = _launch_tree('system.launch')
        path_server = root.find("./node[@type='path_server_node']")

        odom_topics = path_server.find("./rosparam[@param='robot_odom_topics']").text

        self.assertIn('tb3_0: /tb3_0/sim/odom', odom_topics)
        self.assertIn('tb3_1: /tb3_1/sim/odom', odom_topics)


if __name__ == '__main__':
    unittest.main()
