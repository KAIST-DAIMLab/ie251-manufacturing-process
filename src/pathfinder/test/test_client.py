import math
import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.publishers = []
    rospy.sleep = lambda seconds: None
    rospy.loginfo = lambda msg: None

    class Publisher:
        def __init__(self, topic, msg_type, queue_size=10):
            self.topic = topic
            self.msg_type = msg_type
            self.queue_size = queue_size
            self.published = []
            rospy.publishers.append(self)

        def publish(self, msg):
            self.published.append(msg)

    rospy.Publisher = Publisher
    sys.modules['rospy'] = rospy

    actionlib = types.ModuleType('actionlib')
    actionlib.clients = []

    class SimpleActionClient:
        def __init__(self, name, action_type):
            self.name = name
            self.action_type = action_type
            self.goal = None
            self.result = types.SimpleNamespace(success=True, message='ok')
            actionlib.clients.append(self)

        def wait_for_server(self):
            return True

        def send_goal(self, goal, feedback_cb=None):
            self.goal = goal

        def wait_for_result(self):
            return True

        def get_result(self):
            return self.result

    actionlib.SimpleActionClient = SimpleActionClient
    sys.modules['actionlib'] = actionlib

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.Empty = object
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = std_msgs_msg

    pathfinder_msg = types.ModuleType('pathfinder.msg')

    class MoveToNodeAction:
        pass

    class MoveToNodeGoal:
        def __init__(self):
            self.robot_id = ''
            self.target_node_id = 0

    class RobotCommandAction:
        pass

    class RobotCommandGoal:
        def __init__(self):
            self.command = ''
            self.value = 0.0

    pathfinder_msg.MoveToNodeAction = MoveToNodeAction
    pathfinder_msg.MoveToNodeGoal = MoveToNodeGoal
    pathfinder_msg.RobotCommandAction = RobotCommandAction
    pathfinder_msg.RobotCommandGoal = RobotCommandGoal
    sys.modules['pathfinder.msg'] = pathfinder_msg


_install_ros_stubs()

from pathfinder.client.client import Client


class ClientTest(unittest.TestCase):
    def setUp(self):
        import actionlib
        import rospy
        actionlib.clients[:] = []
        rospy.publishers[:] = []

    def test_send_goal_still_targets_path_server(self):
        client = Client()

        ok = client.send_goal('tb3_0', 5)

        import actionlib
        self.assertTrue(ok)
        self.assertEqual(actionlib.clients[0].name, '/path_server/move_to_node')
        self.assertEqual(actionlib.clients[0].goal.robot_id, 'tb3_0')
        self.assertEqual(actionlib.clients[0].goal.target_node_id, 5)

    def test_cancel_still_publishes_stop_topic(self):
        client = Client()

        client.cancel('tb3_0')

        import rospy
        self.assertEqual(rospy.publishers[0].topic, '/tb3_0/stop')
        self.assertEqual(len(rospy.publishers[0].published), 1)

    def test_turn_commands_convert_degrees_to_radians(self):
        client = Client()

        ok = client.send_command('tb3_0', 'turn_left', 90.0)

        import actionlib
        self.assertTrue(ok)
        self.assertEqual(actionlib.clients[-1].name, '/tb3_0/user_command')
        self.assertEqual(actionlib.clients[-1].goal.command, 'turn_left')
        self.assertAlmostEqual(actionlib.clients[-1].goal.value, math.pi / 2.0)

    def test_move_commands_keep_meter_value(self):
        client = Client()

        ok = client.send_command('tb3_0', 'move_backward', 0.5)

        import actionlib
        self.assertTrue(ok)
        self.assertEqual(actionlib.clients[-1].name, '/tb3_0/user_command')
        self.assertEqual(actionlib.clients[-1].goal.command, 'move_backward')
        self.assertEqual(actionlib.clients[-1].goal.value, 0.5)


if __name__ == '__main__':
    unittest.main()
