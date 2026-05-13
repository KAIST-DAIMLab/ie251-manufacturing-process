from __future__ import annotations
import os
import sys
import time
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.shutdown = False

    class Rate:
        def __init__(self, hz):
            self.hz = hz

        def sleep(self):
            time.sleep(0.001)

    rospy.Rate = Rate
    rospy.is_shutdown = lambda: rospy.shutdown
    sys.modules['rospy'] = rospy

    actionlib = types.ModuleType('actionlib')
    actionlib.SimpleActionServer = object
    sys.modules['actionlib'] = actionlib

    pathfinder_msg = types.ModuleType('pathfinder.msg')
    pathfinder_msg.RobotCommandAction = object
    pathfinder_msg.RobotCommandResult = lambda success=False, message='': types.SimpleNamespace(success=success, message=message)
    sys.modules['pathfinder.msg'] = pathfinder_msg


_install_ros_stubs()

from pathfinder.ros.motion_control_action_server import MotionControlActionServer


class FakeActionServer:
    def __init__(self, preempt_after_checks=999):
        self._checks = 0
        self._preempt_after_checks = preempt_after_checks
        self.succeeded = None
        self.aborted = None
        self.preempted = False

    def is_preempt_requested(self):
        self._checks += 1
        return self._checks >= self._preempt_after_checks

    def set_succeeded(self, result):
        self.succeeded = result

    def set_aborted(self, result):
        self.aborted = result

    def set_preempted(self):
        self.preempted = True


class FakeRobot:
    def __init__(self):
        self.calls = []
        self.stop_called = False

    def turn_to(self, value):
        self.calls.append(('turn_to', value))
        return True

    def turn_left(self, value):
        self.calls.append(('turn_left', value))
        time.sleep(0.01)
        return False

    def stop(self):
        self.stop_called = True


class MotionControlActionServerTest(unittest.TestCase):
    def test_turn_to_command_rotates_to_absolute_heading(self):
        robot = FakeRobot()
        server = MotionControlActionServer(robot, '/tb3_0/user_command')
        server._server = FakeActionServer()

        server._on_user_command(types.SimpleNamespace(command='turn_to', value=1.25))

        self.assertEqual(robot.calls, [('turn_to', 1.25)])
        self.assertTrue(server._server.succeeded.success)

    def test_preempt_stops_active_motion_command(self):
        robot = FakeRobot()
        server = MotionControlActionServer(robot, '/tb3_0/user_command')
        server._server = FakeActionServer(preempt_after_checks=2)

        server._on_user_command(types.SimpleNamespace(command='turn_left', value=1.0))

        self.assertTrue(robot.stop_called)
        self.assertTrue(server._server.preempted)
        self.assertIsNone(server._server.aborted)


if __name__ == '__main__':
    unittest.main()
