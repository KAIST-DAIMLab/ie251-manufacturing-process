from __future__ import annotations

import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from pathfinder.ros.follow_path_action_server import FollowPathActionServer
from pathfinder.robot.motion_controller import MotionController, MotionParameters
from pathfinder.robot.path_follower import PathFollower
from pathfinder.robot.robot_state import RobotState
from pathfinder.robot.turtlebot import TurtleBot
from pathfinder.ros.robot_command_action_server import RobotCommandActionServer
from pathfinder.utils.physics import yaw_from_quaternion
from pathfinder.world.graph import Graph


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
        self._state = RobotState(id=robot_id, origin=origin or Pose2D())
        state = self._state
        motion_controller = MotionController(
            cmd_vel_publisher=cmd_vel_publisher,
            pose_provider=state.get_pose,
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

        rospy.Subscriber(self.topic_odom, Odometry, self._on_odom)
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

    def _on_odom(self, msg: Odometry) -> None:
        odom_pose = msg.pose.pose
        self._state.pose.x = odom_pose.position.x + self._state.origin.x
        self._state.pose.y = odom_pose.position.y + self._state.origin.y
        self._state.pose.theta = yaw_from_quaternion(odom_pose.orientation) + self._state.origin.theta
        self._state.velocity = msg.twist.twist

    def _on_stop(self, _: Empty) -> None:
        self._robot.stop()
        rospy.logwarn(f"{self._robot.id}: stop received.")
