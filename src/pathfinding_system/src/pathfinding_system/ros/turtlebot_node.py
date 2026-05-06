from __future__ import annotations

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from pathfinding_system.ros.follow_path_action_server import FollowPathActionServer
from pathfinding_system.robot.motion_controller import MotionParameters
from pathfinding_system.robot.turtlebot import TurtleBot
from pathfinding_system.ros.robot_command_action_server import RobotCommandActionServer
from pathfinding_system.world.graph import Graph


class TurtleBotNode:
    """ROS adapter: creates and wires topics and action server for one TurtleBot."""

    def __init__(
        self,
        robot_id: str,
        namespace: str | None = None,
        graph: Graph | None = None,
        params: MotionParameters = MotionParameters(),
        motion_rate_hz: float = 5.0,
    ) -> None:
        self._robot_id = robot_id
        self._namespace = (namespace or robot_id).strip('/')
        cmd_vel_publisher = rospy.Publisher(f'/{self._namespace}/cmd_vel', Twist, queue_size=1)
        self._robot = TurtleBot(robot_id, cmd_vel_publisher=cmd_vel_publisher, params=params, motion_rate_hz=motion_rate_hz)
        rospy.Subscriber(self.topic_odom, Odometry, self._robot.update_pose)
        rospy.Subscriber(self.topic_stop, Empty, self._on_stop)
        self._user_command_server = RobotCommandActionServer(self._robot, robot_id)
        self._follow_path_server = FollowPathActionServer(self._robot, graph, self._namespace) if graph is not None else None

    @property
    def topic_odom(self) -> str:
        """Topic name for the odometry subscriber."""
        return f'/{self._namespace}/odom'

    @property
    def topic_stop(self) -> str:
        """Topic name for stop requests."""
        return f'/{self._namespace}/stop'

    def start(self) -> None:
        """Start executor action servers."""
        self._user_command_server.start()
        if self._follow_path_server is not None:
            self._follow_path_server.start()
        rospy.loginfo(f"TurtleBotNode for {self._robot.id} started.")

    def _on_stop(self, msg: Empty) -> None:
        self._robot.stop()
        rospy.logwarn(f"{self._robot.id}: stop received.")
