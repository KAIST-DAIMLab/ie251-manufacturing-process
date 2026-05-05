from __future__ import annotations
import math
import threading
from typing import Any

import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from pathfinding_system.msg import RobotState as RobotStateMsg  # type: ignore[import]
from pathfinding_system.robot.motion_controller import MotionController
from pathfinding_system.robot.path_follower import PathFollower
from pathfinding_system.robot.robot_state import RobotState
from pathfinding_system.robot.turtlebot import TurtleBot
from pathfinding_system.world.graph import Graph


def _yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def robot_state_to_msg(state: RobotState) -> RobotStateMsg:
    """Convert a RobotState dataclass to a RobotState ROS message."""
    msg = RobotStateMsg()
    msg.robot_id = state.id
    msg.pose = state.pose
    msg.velocity = state.velocity
    msg.status = int(state.status)
    msg.stamp = state.stamp if state.stamp is not None else rospy.Time.now()
    return msg


class TurtleBotNode:
    """ROS adapter: wires topics, action server, and motion controller for one TurtleBot."""

    def __init__(
        self,
        robot: TurtleBot,
        graph: Graph | None = None,
        state_publish_rate_hz: float = 10.0,
        linear_speed: float = 0.22,
        angular_speed: float = 1.5,
        motion_rate_hz: float = 5.0,
    ) -> None:
        if state_publish_rate_hz <= 0:
            raise ValueError('state_publish_rate_hz must be positive')

        self._robot = robot
        self._graph = graph
        self._action_server = None

        self.cmd_vel_publisher = rospy.Publisher(robot.cmd_vel_topic, Twist, queue_size=1)
        self.state_publisher = rospy.Publisher(robot.state_topic, RobotStateMsg, queue_size=1)
        self._odom_subscriber = rospy.Subscriber(
            robot.odom_topic,
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

        self._motion_controller = MotionController(
            cmd_vel_publisher=self.cmd_vel_publisher,
            pose_provider=robot.current_pose,
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            rate_hz=motion_rate_hz,
        )
        self._path_follower = PathFollower(self._motion_controller)

    def start(self) -> None:
        """Start the FollowPath action server (no-op if no graph was provided)."""
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

    def publish_stop(self) -> None:
        """Publish a zero Twist to halt the robot."""
        self.cmd_vel_publisher.publish(Twist())

    def _on_state_timer(self, event: Any) -> None:
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
        self._motion_controller.stop()
        rospy.logwarn(f"{self._robot.id}: emergency stop received.")

    def _on_follow_path(self, goal: Any) -> None:
        from pathfinding_system.msg import (  # type: ignore[import]
            FollowPathFeedback,
            FollowPathResult,
        )

        waypoints = [self._graph.get_node(nid) for nid in goal.node_ids]

        if self._action_server.is_preempt_requested():
            self._robot.mark_idle()
            self.publish_stop()
            self._action_server.set_preempted()
            return

        self._robot.clear_stop()
        self._robot.mark_moving()

        result_container: list[bool] = []
        follow_thread = threading.Thread(
            target=lambda: result_container.append(self._path_follower.follow(waypoints)),
            daemon=True,
        )
        follow_thread.start()

        rate = rospy.Rate(20)
        while follow_thread.is_alive():
            if self._action_server.is_preempt_requested():
                self._motion_controller.stop()
                follow_thread.join()
                self._robot.mark_idle()
                self.publish_stop()
                self._action_server.set_preempted()
                return

            if self._robot.stop_requested():
                self._motion_controller.stop()
                follow_thread.join()
                self.publish_stop()
                self._action_server.set_aborted(
                    FollowPathResult(success=False, message="emergency stop")
                )
                return

            fb = FollowPathFeedback()
            fb.current_index = self._path_follower.current_index
            fb.current_pose = self._robot.current_pose()
            self._action_server.publish_feedback(fb)
            rate.sleep()

        follow_thread.join()
        if result_container and result_container[0]:
            self._robot.mark_reached()
            self.publish_stop()
            self._action_server.set_succeeded(
                FollowPathResult(success=True, message="reached goal")
            )
        else:
            self.publish_stop()
            self._action_server.set_aborted(
                FollowPathResult(success=False, message="interrupted")
            )
