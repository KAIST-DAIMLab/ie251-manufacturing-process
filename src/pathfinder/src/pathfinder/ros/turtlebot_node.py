from __future__ import annotations

import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty

from pathfinder.ros.follow_path_action_server import FollowPathActionServer
from pathfinder.robot.motion_engine import MotionEngine, MotionParameters
from pathfinder.robot.motion_controller import MotionController
from pathfinder.robot.path_follower import PathFollower
from pathfinder.robot.robot_state import RobotState
from pathfinder.robot.turtlebot import TurtleBot
from pathfinder.ros.robot_command_action_server import RobotCommandActionServer
from pathfinder.safety.obstacle_detector import ObstacleDetector
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
        obstacle_enabled: bool = True,
        obstacle_stop_distance: float = 0.5,
    ) -> None:
        self._robot_id = robot_id
        self._namespace = (namespace or robot_id).strip('/')
        self._obstacle_detector = ObstacleDetector(
            stop_distance=obstacle_stop_distance,
            detect_degree=20,
        ) if obstacle_enabled else None

        cmd_vel_publisher = rospy.Publisher(self.topic_cmd_vel, Twist, queue_size=1)

        self._state = RobotState(id=robot_id, origin=origin or Pose2D())
        state = self._state
        engine = MotionEngine(
            cmd_vel_publisher=cmd_vel_publisher,
            pose_provider=state.get_pose,
            params=params,
        )
        motion_controller = MotionController(engine, rate_hz=motion_rate_hz)
        path_follower = PathFollower(motion_controller)
        self._robot = TurtleBot(
            robot_id,
            state=state,
            motion_controller=motion_controller,
            path_follower=path_follower,
        )

        rospy.Subscriber(self.topic_odom, Odometry, self._on_odom)
        rospy.Subscriber(self.topic_stop, Empty, self._on_stop)
        if self._obstacle_detector is not None:
            rospy.Subscriber(self.topic_scan, LaserScan, self._on_scan)
        self._user_command_server = RobotCommandActionServer(self._robot, self.topic_user_command)
        self._follow_path_server = FollowPathActionServer(self._robot, graph, self.topic_follow_path) if graph is not None else None

    @property
    def topic_cmd_vel(self) -> str:
        """Topic name for velocity commands."""
        return f'/{self._namespace}/cmd_vel'

    @property
    def topic_odom(self) -> str:
        """Topic name for the odometry subscriber."""
        return f'/{self._namespace}/odom'

    @property
    def topic_stop(self) -> str:
        """Topic name for stop requests."""
        return f'/{self._namespace}/stop'

    @property
    def topic_scan(self) -> str:
        """Topic name for the LiDAR subscriber."""
        return f'/{self._namespace}/scan'

    @property
    def topic_user_command(self) -> str:
        """Topic name for the user command action server."""
        return f'/{self._namespace}/user_command'

    @property
    def topic_follow_path(self) -> str:
        """Topic name for the follow path action server."""
        return f'/{self._namespace}/follow_path'

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

    def _on_scan(self, message: LaserScan) -> None:
        detected = self._obstacle_detector.detect(message)
        self._robot.set_pause(detected)
        if detected:
            rospy.logwarn(f"{self._robot.id}: obstacle detected, pausing")
