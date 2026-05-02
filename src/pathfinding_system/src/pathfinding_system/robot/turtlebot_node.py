from __future__ import annotations
import math

import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from pathfinding_system.msg import RobotState as RobotStateMsg  # type: ignore[import]
from pathfinding_system.robot.motion import DriveResult
from pathfinding_system.robot.turtlebot import TurtleBot
from pathfinding_system.world.graph import Graph


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def robot_state_to_msg(state):
    msg = RobotStateMsg()
    msg.robot_id = state.id
    msg.pose = state.pose
    msg.velocity = state.velocity
    msg.status = int(state.status)
    msg.stamp = state.stamp if state.stamp is not None else rospy.Time.now()
    return msg


class TurtleBotNode:
    def __init__(
        self,
        robot: TurtleBot,
        graph: Graph | None = None,
        state_publish_rate_hz: float = 10.0,
    ) -> None:
        if state_publish_rate_hz <= 0:
            raise ValueError('state_publish_rate_hz must be positive')

        self._robot = robot
        self._graph = graph
        self._action_server = None
        self.cmd_vel_publisher = rospy.Publisher(robot.cmd_vel_topic, Twist, queue_size=1)
        self.state_publisher = rospy.Publisher(robot.state_topic, RobotStateMsg, queue_size=1)
        self._odom_subscriber = rospy.Subscriber(
            f'/{robot.id}/odom',
            Odometry,
            self._on_odom,
        )
        self._emergency_stop_subscriber = rospy.Subscriber(
            f'/{robot.id}/emergency_stop',
            Empty,
            self._on_emergency_stop,
        )
        self._state_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / state_publish_rate_hz),
            self._on_state_timer,
        )

    def start(self) -> None:
        if self._graph is None:
            return

        from pathfinding_system.msg import FollowPathAction  # type: ignore[import]

        self._action_server = actionlib.SimpleActionServer(
            f'/{self._robot.id}/follow_path',
            FollowPathAction,
            execute_cb=self._on_follow_path,
            auto_start=False,
        )
        self._action_server.start()
        rospy.loginfo(f"TurtleBotNode for {self._robot.id} started.")

    def publish_drive_result(self, result: DriveResult) -> None:
        cmd = Twist()
        cmd.linear.x = result.linear_x
        cmd.angular.z = result.angular_z
        self.cmd_vel_publisher.publish(cmd)

    def publish_stop(self) -> None:
        self.cmd_vel_publisher.publish(Twist())

    def _on_state_timer(self, event) -> None:
        self.state_publisher.publish(robot_state_to_msg(self._robot.state_snapshot()))

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._robot.update_pose(
            x=pose.position.x,
            y=pose.position.y,
            theta=_yaw_from_quaternion(pose.orientation),
            velocity=msg.twist.twist,
            stamp=msg.header.stamp,
        )

    def _on_emergency_stop(self, msg: Empty) -> None:
        self._robot.request_stop()
        rospy.logwarn(f"{self._robot.id}: emergency stop received.")
        self.publish_stop()

    def _on_follow_path(self, goal) -> None:
        from pathfinding_system.msg import (  # type: ignore[import]
            FollowPathFeedback,
            FollowPathResult,
        )

        waypoints = [self._graph.get_node(nid) for nid in goal.node_ids]
        self._robot.start_path(waypoints)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self._action_server.is_preempt_requested():
                self._robot.cancel_path()
                self.publish_stop()
                self._action_server.set_preempted()
                return
            if self._robot.stop_requested():
                self.publish_stop()
                self._action_server.set_aborted(
                    FollowPathResult(success=False, message="emergency stop")
                )
                return

            step = self._robot.step_path()
            self.publish_drive_result(step.drive_result)
            if step.completed:
                self.publish_stop()
                self._action_server.set_succeeded(
                    FollowPathResult(success=True, message="reached goal")
                )
                return

            fb = FollowPathFeedback()
            fb.current_index = step.current_index
            fb.current_pose = self._robot.current_pose()
            self._action_server.publish_feedback(fb)
            rate.sleep()
