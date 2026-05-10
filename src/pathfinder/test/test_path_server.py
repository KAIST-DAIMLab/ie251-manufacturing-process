from __future__ import annotations
import os
import sys
import threading
import types
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.Duration = lambda seconds: seconds
    rospy.Time = types.SimpleNamespace(now=lambda: 0)
    rospy.is_shutdown = lambda: False
    rospy.loginfo = lambda message: None
    sys.modules['rospy'] = rospy

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')

    class Pose2D:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    geometry_msgs_msg.Pose2D = Pose2D
    geometry_msgs_msg.Twist = Twist
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs_msg = types.ModuleType('nav_msgs.msg')
    nav_msgs_msg.Odometry = object
    sys.modules['nav_msgs'] = nav_msgs
    sys.modules['nav_msgs.msg'] = nav_msgs_msg

    actionlib = types.ModuleType('actionlib')
    actionlib.ActionServer = object
    actionlib.SimpleActionClient = object
    sys.modules['actionlib'] = actionlib

    actionlib_msgs = types.ModuleType('actionlib_msgs')
    actionlib_msgs_msg = types.ModuleType('actionlib_msgs.msg')
    actionlib_msgs_msg.GoalStatus = types.SimpleNamespace(
        PENDING=0,
        ACTIVE=1,
        PREEMPTING=6,
        RECALLING=7,
    )
    sys.modules['actionlib_msgs'] = actionlib_msgs
    sys.modules['actionlib_msgs.msg'] = actionlib_msgs_msg

    pathfinder_msg = types.ModuleType('pathfinder.msg')

    class MoveToNodeResult:
        def __init__(self):
            self.success = False
            self.message = ''

    class MoveToNodeFeedback:
        def __init__(self):
            self.current_node_id = -1
            self.nodes_remaining = 0

    pathfinder_msg.RobotState = object
    pathfinder_msg.MoveToNodeResult = MoveToNodeResult
    pathfinder_msg.MoveToNodeFeedback = MoveToNodeFeedback
    sys.modules['pathfinder.msg'] = pathfinder_msg


_install_ros_stubs()

from pathfinder.ros.path_request_action_server import PathRequestActionServer
from pathfinder.planning.path_orchestrator import PathOrchestrator, UnknownRobotError, NoPathError, NodeNotFoundError
from pathfinder.ros.pose_tracker import PoseTracker


class FakeGoalHandle:
    """Minimal stand-in for a ROS goal handle."""

    def __init__(self, robot_id: str, target_node_id: int, status: int = 1):
        self._robot_id = robot_id
        self._target_node_id = target_node_id
        self._status = status
        self.aborted_message = None
        self.succeeded_message = None
        self.canceled = False

    def get_goal(self):
        return types.SimpleNamespace(
            robot_id=self._robot_id,
            target_node_id=self._target_node_id,
        )

    def get_goal_status(self):
        return types.SimpleNamespace(status=self._status)

    def set_aborted(self, result):
        self.aborted_message = result.message

    def set_succeeded(self, result):
        self.succeeded_message = result.message

    def set_canceled(self):
        self.canceled = True

    def publish_feedback(self, feedback):
        pass


class FakeOrchestrator:
    """Returns a fixed node_ids list or raises a given exception."""

    def __init__(self, node_ids=None, raises=None):
        self._node_ids = node_ids or [1, 2, 3]
        self._raises = raises
        self.calls = []

    def plan(self, robot_id, pose, target_node_id):
        self.calls.append((robot_id, pose, target_node_id))
        if self._raises is not None:
            raise self._raises
        return self._node_ids


class FakeFollowPathClient:
    """Returns a fixed result or None."""

    def __init__(self, result=None):
        self._result = result
        self.dispatched_node_ids = None

    def dispatch(self, node_ids, on_feedback, is_canceled):
        self.dispatched_node_ids = node_ids
        return self._result


class PathRequestActionServerTest(unittest.TestCase):
    def _make_server(self, robot_id='tb3_0'):
        tracker = PoseTracker(timeout_sec=1.0)
        tracker.update(robot_id, types.SimpleNamespace(x=0.0, y=0.0, theta=0.0))

        orchestrator = FakeOrchestrator(node_ids=[1, 2, 3])
        follow_result = types.SimpleNamespace(success=True, message='done')
        client = FakeFollowPathClient(result=follow_result)
        robot_locks = {robot_id: threading.Lock()}

        server = PathRequestActionServer(
            orchestrator=orchestrator,
            tracker=tracker,
            clients={robot_id: client},
            robot_locks=robot_locks,
            topic='/path_server/move_to_node',
        )
        return server, orchestrator, client

    def test_execute_calls_tracker_orchestrator_and_client(self):
        server, orchestrator, client = self._make_server()
        goal_handle = FakeGoalHandle(robot_id='tb3_0', target_node_id=3)

        server._execute(goal_handle)

        self.assertEqual(len(orchestrator.calls), 1)
        self.assertEqual(orchestrator.calls[0][0], 'tb3_0')
        self.assertEqual(orchestrator.calls[0][2], 3)
        self.assertEqual(client.dispatched_node_ids, [1, 2, 3])
        self.assertEqual(goal_handle.succeeded_message, 'done')

    def test_execute_aborts_when_tracker_returns_none(self):
        robot_id = 'tb3_0'
        tracker = PoseTracker(timeout_sec=0.01)
        orchestrator = FakeOrchestrator()
        client = FakeFollowPathClient()
        robot_locks = {robot_id: threading.Lock()}

        server = PathRequestActionServer(
            orchestrator=orchestrator,
            tracker=tracker,
            clients={robot_id: client},
            robot_locks=robot_locks,
            topic='/path_server/move_to_node',
        )
        goal_handle = FakeGoalHandle(robot_id=robot_id, target_node_id=3)

        server._execute(goal_handle)

        self.assertIn('no state received from tb3_0', goal_handle.aborted_message)
        self.assertIsNone(client.dispatched_node_ids)

    def test_execute_aborts_on_unknown_robot_error(self):
        robot_id = 'tb3_0'
        tracker = PoseTracker(timeout_sec=1.0)
        tracker.update(robot_id, types.SimpleNamespace(x=0.0, y=0.0, theta=0.0))
        orchestrator = FakeOrchestrator(raises=UnknownRobotError("unknown robot: tb3_0"))
        client = FakeFollowPathClient()
        robot_locks = {robot_id: threading.Lock()}

        server = PathRequestActionServer(
            orchestrator=orchestrator,
            tracker=tracker,
            clients={robot_id: client},
            robot_locks=robot_locks,
            topic='/path_server/move_to_node',
        )
        goal_handle = FakeGoalHandle(robot_id=robot_id, target_node_id=3)

        server._execute(goal_handle)

        self.assertIn('unknown robot', goal_handle.aborted_message)
        self.assertIsNone(client.dispatched_node_ids)

    def test_execute_aborts_on_no_path_error(self):
        robot_id = 'tb3_0'
        tracker = PoseTracker(timeout_sec=1.0)
        tracker.update(robot_id, types.SimpleNamespace(x=0.0, y=0.0, theta=0.0))
        orchestrator = FakeOrchestrator(raises=NoPathError("no path exists"))
        client = FakeFollowPathClient()
        robot_locks = {robot_id: threading.Lock()}

        server = PathRequestActionServer(
            orchestrator=orchestrator,
            tracker=tracker,
            clients={robot_id: client},
            robot_locks=robot_locks,
            topic='/path_server/move_to_node',
        )
        goal_handle = FakeGoalHandle(robot_id=robot_id, target_node_id=3)

        server._execute(goal_handle)

        self.assertIn('no path exists', goal_handle.aborted_message)
        self.assertIsNone(client.dispatched_node_ids)

    def test_execute_aborts_on_node_not_found_error(self):
        robot_id = 'tb3_0'
        tracker = PoseTracker(timeout_sec=1.0)
        tracker.update(robot_id, types.SimpleNamespace(x=0.0, y=0.0, theta=0.0))
        orchestrator = FakeOrchestrator(raises=NodeNotFoundError("node 99 not found"))
        client = FakeFollowPathClient()
        robot_locks = {robot_id: threading.Lock()}

        server = PathRequestActionServer(
            orchestrator=orchestrator,
            tracker=tracker,
            clients={robot_id: client},
            robot_locks=robot_locks,
            topic='/path_server/move_to_node',
        )
        goal_handle = FakeGoalHandle(robot_id=robot_id, target_node_id=99)

        server._execute(goal_handle)

        self.assertIsNotNone(goal_handle.aborted_message)
        self.assertIsNone(client.dispatched_node_ids)

    def test_execute_aborts_when_client_returns_none(self):
        robot_id = 'tb3_0'
        tracker = PoseTracker(timeout_sec=1.0)
        tracker.update(robot_id, types.SimpleNamespace(x=0.0, y=0.0, theta=0.0))
        orchestrator = FakeOrchestrator(node_ids=[1, 2])
        client = FakeFollowPathClient(result=None)
        robot_locks = {robot_id: threading.Lock()}

        server = PathRequestActionServer(
            orchestrator=orchestrator,
            tracker=tracker,
            clients={robot_id: client},
            robot_locks=robot_locks,
            topic='/path_server/move_to_node',
        )
        goal_handle = FakeGoalHandle(robot_id=robot_id, target_node_id=2)

        server._execute(goal_handle)

        self.assertIsNotNone(goal_handle.aborted_message)


if __name__ == '__main__':
    unittest.main()
