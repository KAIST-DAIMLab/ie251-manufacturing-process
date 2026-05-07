from __future__ import annotations

import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from pathfinding_system.ros.follow_path_action_server import FollowPathActionServer
from pathfinding_system.robot.motion_controller import MotionController, MotionParameters
from pathfinding_system.robot.path_follower import PathFollower
from pathfinding_system.robot.robot_state import RobotState
from pathfinding_system.robot.turtlebot import TurtleBot
from pathfinding_system.ros.robot_command_action_server import RobotCommandActionServer
from pathfinding_system.world.graph import Graph


class TurtleBotNode:
    """ROS adapter: assembles state, motion components, publishers, subscribers, and action servers for one TurtleBot."""

    def __init__(
        self,
        robot_id: str,
        namespace: str | None = None,
        graph: Graph | None = None,
        params: MotionParameters = MotionParameters(),
        motion_rate_hz: float = 5.0,
        origin: Pose2D | None = None,
    ) -> None:
        self._robot_id = robot_id
        self._namespace = (namespace or robot_id).strip('/')

        cmd_vel_publisher = rospy.Publisher(f'/{self._namespace}/cmd_vel', Twist, queue_size=1)
        state = RobotState(id=robot_id, origin=origin)
        motion_controller = MotionController(
            cmd_vel_publisher=cmd_vel_publisher,
            pose_provider=state.current_pose,
            params=params,
        )
        path_follower = PathFollower(motion_controller, rate_hz=motion_rate_hz)
        self._robot = TurtleBot(
            robot_id,
            state=state,
            motion_controller=motion_controller,
            path_follower=path_follower,
            motion_rate_hz=motion_rate_hz,
        )

        rospy.Subscriber(self.topic_odom, Odometry, self._robot.update_pose)
        rospy.Subscriber(self.topic_stop, Empty, self._on_stop)
        self._user_command_server = RobotCommandActionServer(self._robot, robot_id)
        self._follow_path_server = FollowPathActionServer(self._robot, graph, robot_id) if graph is not None else None

    @property
    def topic_odom(self) -> str:
        """Topic name for the odometry subscriber."""
        return f'/{self._namespace}/odom'

    @property
    def topic_stop(self) -> str:
        """Topic name for stop requests."""
        return f'/{self._robot_id}/stop'

    def start(self) -> None:
        """Start executor action servers."""
        self._user_command_server.start()
        if self._follow_path_server is not None:
            self._follow_path_server.start()
        rospy.loginfo(f"TurtleBotNode for {self._robot.id} started.")

    def _on_stop(self, message: Empty) -> None:
        self._robot.stop()
        rospy.logwarn(f"{self._robot.id}: stop received.")
