from __future__ import annotations

import rospy
from geometry_msgs.msg import Pose2D, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from pathfinder.ros.path_follow_action_server import PathFollowActionServer
from pathfinder.robot.motion_engine import MotionEngine, MotionParameters
from pathfinder.robot.motion_controller import MotionController
from pathfinder.robot.path_follower import PathFollower
from pathfinder.robot.robot_state import RobotState
from pathfinder.robot.turtlebot import TurtleBot
from pathfinder.ros.motion_control_action_server import MotionControlActionServer
from pathfinder.safety.obstacle_detector import ObstacleDetector
from pathfinder.safety.forward_gate import ForwardGate
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
        odom_pose_enabled: bool = False,
        obstacle_enabled: bool = True,
        obstacle_stop_distance: float = 0.5,
        obstacle_detect_degree: int = 20,
    ) -> None:
        self._robot_id = robot_id
        self._namespace = (namespace or robot_id).strip('/')
        self._odom_pose_enabled = odom_pose_enabled
        self._obstacle_detector = ObstacleDetector(
            stop_distance=obstacle_stop_distance,
            detect_degree=obstacle_detect_degree,
        ) if obstacle_enabled else None

        cmd_vel_publisher = rospy.Publisher(self.topic_cmd_vel, Twist, queue_size=1)
        self._pose_publisher = rospy.Publisher(self.topic_pose, Pose2D, queue_size=1, latch=True)
        self._forward_gate = ForwardGate(cmd_vel_publisher)
        self._obstacle_blocked_publisher = rospy.Publisher(self.topic_obstacle_blocked, Bool, queue_size=1, latch=True)
        self._obstacle_blocked_publisher.publish(Bool(data=False))
        self._last_obstacle_blocked = False

        self._state = RobotState(id=robot_id, origin=origin or Pose2D())
        state = self._state
        engine = MotionEngine(
            cmd_vel_publisher=self._forward_gate,
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
        rospy.Subscriber(self.topic_amcl_pose, PoseWithCovarianceStamped, self._on_amcl_pose)
        if self._obstacle_detector is not None:
            rospy.Subscriber(self.topic_scan, LaserScan, self._on_scan)
        self._motion_control_server = MotionControlActionServer(self._robot, self.topic_user_command)
        self._path_follow_server = PathFollowActionServer(self._robot, graph, self.topic_follow_path) if graph is not None else None

    @property
    def topic_pose(self) -> str:
        """Topic name for the world-frame pose publisher."""
        return f'/{self._namespace}/pose'

    @property
    def topic_cmd_vel(self) -> str:
        return f'/{self._namespace}/cmd_vel'

    @property
    def topic_odom(self) -> str:
        return f'/{self._namespace}/odom'

    @property
    def topic_scan(self) -> str:
        """Topic name for the LiDAR subscriber."""
        return f'/{self._namespace}/scan'
    
    def topic_stop(self) -> str:
        return f'/{self._namespace}/stop'

    @property
    def topic_amcl_pose(self) -> str:
        return f'/{self._namespace}/amcl_pose'

    @property
    def topic_user_command(self) -> str:
        return f'/{self._namespace}/user_command'

    @property
    def topic_follow_path(self) -> str:
        return f'/{self._namespace}/follow_path'

    @property
    def topic_obstacle_blocked(self) -> str:
        """Topic name for the obstacle-blocked state publisher."""
        return f'/{self._namespace}/obstacle_blocked'

    def start(self) -> None:
        """Start executor action servers."""
        self._motion_control_server.start()
        if self._path_follow_server is not None:
            self._path_follow_server.start()
        rospy.loginfo(f"TurtleBotNode for {self._robot.id} started.")

    def _on_odom(self, msg: Odometry) -> None:
        self._state.velocity = msg.twist.twist
        odom_pose = msg.pose.pose
        # Heading comes from wheel-encoder odom in every mode: it's smooth and
        # high-rate, where AMCL is quantized at update_min_a (~5.7°) and makes
        # the P-controller overshoot on turns.
        self._state.pose.theta = yaw_from_quaternion(odom_pose.orientation) + self._state.origin.theta
        if self._odom_pose_enabled:
            self._state.pose.x = odom_pose.position.x + self._state.origin.x
            self._state.pose.y = odom_pose.position.y + self._state.origin.y
        self._pose_publisher.publish(self._state.get_pose())

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self._state.pose.x = pose.position.x
        self._state.pose.y = pose.position.y
        self._pose_publisher.publish(self._state.get_pose())

    def _on_scan(self, message: LaserScan) -> None:
        detected = self._obstacle_detector.detect(message)
        self._forward_gate.set_blocked(detected)
        if detected != self._last_obstacle_blocked:
            self._last_obstacle_blocked = detected
            self._obstacle_blocked_publisher.publish(Bool(data=detected))
        if detected:
            rospy.logwarn(f"{self._robot.id}: obstacle detected, forward motion blocked")
