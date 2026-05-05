from __future__ import annotations

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from pathfinding_system.robot.follow_path_action_server import FollowPathActionServer
from pathfinding_system.robot.motion_controller import MotionParameters
from pathfinding_system.robot.turtlebot import TurtleBot
from pathfinding_system.world.graph import Graph


class TurtleBotNode:
    """ROS adapter: creates and wires topics and action server for one TurtleBot."""

    def __init__(
        self,
        robot_id: str,
        namespace: str,
        graph: Graph | None = None,
        params: MotionParameters = MotionParameters(),
        motion_rate_hz: float = 5.0,
    ) -> None:
        self._namespace = namespace.strip('/')
        rospy.Subscriber(self.topic_odom, Odometry, self._robot.update_pose)
        rospy.Subscriber(f'/{robot_id}/emergency_stop', Empty, self._on_emergency_stop)
        cmd_vel_publisher = rospy.Publisher(f'/{self._namespace}/cmd_vel', Twist, queue_size=1)
        
        self._robot = TurtleBot(robot_id, cmd_vel_publisher=cmd_vel_publisher, params=params, motion_rate_hz=motion_rate_hz)
        self._follow_path_server = FollowPathActionServer(self._robot, graph) if graph is not None else None

    @property
    def topic_odom(self) -> str:
        """Topic name for the odometry subscriber."""
        return f'/{self._namespace}/odom'

    def start(self) -> None:
        """Start the FollowPath action server (no-op if no graph was provided)."""
        if self._follow_path_server is None:
            return
        self._follow_path_server.start()
        rospy.loginfo(f"TurtleBotNode for {self._robot.id} started.")

    def _on_emergency_stop(self, msg: Empty) -> None:
        self._robot.stop()
        rospy.logwarn(f"{self._robot.id}: emergency stop received.")
