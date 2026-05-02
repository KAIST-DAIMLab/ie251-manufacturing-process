from __future__ import annotations
import rospy
import actionlib
from pathfinding_system.world.graph import Graph
from pathfinding_system.robot.robot_status import RobotStatus
from pathfinding_system.robot.turtlebot import TurtleBot


class TurtleBotExecuter:
    def __init__(self, robot: TurtleBot, graph: Graph) -> None:
        self._robot = robot
        self._graph = graph
        self._action_server = None

    def start(self) -> None:
        from pathfinding_system.msg import FollowPathAction  # type: ignore[import]

        self._robot.start()
        self._action_server = actionlib.SimpleActionServer(
            f'/{self._robot.id}/follow_path',
            FollowPathAction,
            execute_cb=self._on_follow_path,
            auto_start=False,
        )
        self._action_server.start()
        rospy.loginfo(f"TurtleBotExecuter for {self._robot.id} started.")

    def _on_follow_path(self, goal) -> None:
        from pathfinding_system.msg import (  # type: ignore[import]
            FollowPathResult,
            FollowPathFeedback,
        )

        waypoints = [self._graph.get_node(nid) for nid in goal.node_ids]
        rate = rospy.Rate(20)
        self._robot.set_status(RobotStatus.MOVING)

        for idx, waypoint in enumerate(waypoints):
            while not rospy.is_shutdown():
                if self._action_server.is_preempt_requested():
                    self._robot.stop_motion()
                    self._robot.set_status(RobotStatus.IDLE)
                    self._action_server.set_preempted()
                    return
                if self._robot.stop_requested():
                    self._robot.stop_motion()
                    self._action_server.set_aborted(
                        FollowPathResult(success=False, message="emergency stop")
                    )
                    return
                if self._robot.drive_towards(waypoint):
                    break

                fb = FollowPathFeedback()
                fb.current_index = idx
                fb.current_pose = self._robot.current_pose()
                self._action_server.publish_feedback(fb)
                rate.sleep()
            else:
                return

        self._robot.stop_motion()
        self._robot.set_status(RobotStatus.REACHED)
        self._action_server.set_succeeded(
            FollowPathResult(success=True, message="reached goal")
        )
