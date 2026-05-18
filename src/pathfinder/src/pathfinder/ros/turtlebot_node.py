from __future__ import annotations

import math

import rospy
import tf
from geometry_msgs.msg import Pose2D, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8

from pathfinder.ros.path_follow_action_server import PathFollowActionServer
from pathfinder.robot.pose_estimator import PoseEstimator
from pathfinder.robot.motion_engine import MotionEngine, MotionParameters
from pathfinder.robot.motion_controller import MotionController
from pathfinder.robot.path_follower import PathFollower
from pathfinder.robot.robot_state import RobotState
from pathfinder.robot.turtlebot import TurtleBot
from pathfinder.ros.motion_control_action_server import MotionControlActionServer
from pathfinder.safety.obstacle_detector import ObstacleDetector
from pathfinder.safety.forward_gate import ForwardGate
from pathfinder.world.graph import Graph

ODOM_FRESHNESS_SEC = 2.0
RELOCALIZE_XY_VARIANCE = 0.25
RELOCALIZE_THETA_VARIANCE = 0.0685


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
        self._state_publisher = rospy.Publisher(self.topic_state, Int8, queue_size=1, latch=True)
        self._initial_pose_publisher = rospy.Publisher(self.topic_initial_pose, PoseWithCovarianceStamped, queue_size=1)
        self._state_timer = None
        self._relocalize_service = None
        self._last_odom_received_at: rospy.Time | None = None

        self._state = RobotState(id=robot_id)
        state = self._state
        tf_listener = tf.TransformListener()
        self._estimator = PoseEstimator(tf_listener, self._namespace, origin or Pose2D(), sim=odom_pose_enabled)
        engine = MotionEngine(
            cmd_vel_publisher=self._forward_gate,
            pose_provider=self._estimator.get_pose,
            params=params,
        )
        motion_controller = MotionController(engine, rate_hz=motion_rate_hz)
        path_follower = PathFollower(motion_controller)
        self._robot = TurtleBot(
            robot_id,
            state=state,
            motion_controller=motion_controller,
            path_follower=path_follower,
            initial_pose_publisher=self._publish_initial_pose,
            pose_provider=self._estimator.get_pose,
        )

        rospy.Subscriber(self.topic_odom, Odometry, self._on_odom)
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

    @property
    def topic_state(self) -> str:
        """Topic name for the RobotMode heartbeat publisher (UI infers OFFLINE from its absence)."""
        return f'/{self._namespace}/state'

    @property
    def topic_initial_pose(self) -> str:
        """Topic name for the AMCL initial-pose seed publisher."""
        return f'/{self._namespace}/initialpose'

    @property
    def service_relocalize(self) -> str:
        """Service name for the (x, y, theta) relocalize endpoint."""
        return f'/{self._namespace}/relocalize'

    def start(self) -> None:
        """Start executor action servers."""
        from pathfinder.srv import Relocalize  # lazy: avoids circular import on test stubs

        self._motion_control_server.start()
        if self._path_follow_server is not None:
            self._path_follow_server.start()
        self._relocalize_service = rospy.Service(self.service_relocalize, Relocalize, self._on_relocalize)
        self._publish_state()
        self._state_timer = rospy.Timer(rospy.Duration(0.5), self._publish_state)
        rospy.loginfo(f"TurtleBotNode for {self._robot.id} started.")

    def _publish_initial_pose(self, x: float, y: float, theta: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        half = theta / 2.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        covariance = [0.0] * 36
        covariance[0] = RELOCALIZE_XY_VARIANCE
        covariance[7] = RELOCALIZE_XY_VARIANCE
        covariance[35] = RELOCALIZE_THETA_VARIANCE
        msg.pose.covariance = covariance
        self._initial_pose_publisher.publish(msg)

    def _on_relocalize(self, request):
        from pathfinder.srv import RelocalizeResponse  # lazy: avoids circular import on test stubs

        self._robot.relocalize(request.x, request.y, request.theta)
        message = f"{self._robot.id}: relocalized at ({request.x:.2f}, {request.y:.2f}, {math.degrees(request.theta):.1f}°)"
        return RelocalizeResponse(success=True, message=message)

    def _publish_state(self, _event=None) -> None:
        self._recompute_online_status()
        self._state_publisher.publish(Int8(data=int(self._state.status)))

    def _recompute_online_status(self) -> None:
        if self._last_odom_received_at is None:
            fresh = False
        else:
            elapsed = (rospy.Time.now() - self._last_odom_received_at).to_sec()
            fresh = elapsed < ODOM_FRESHNESS_SEC
        self._robot.set_online(fresh)

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom_received_at = rospy.Time.now()
        self._estimator.on_odom(msg)
        self._pose_publisher.publish(self._estimator.get_pose())

    def _on_scan(self, message: LaserScan) -> None:
        detected = self._obstacle_detector.detect(message)
        self._forward_gate.set_blocked(detected)
        self._robot.set_obstacle(detected)
        if detected != self._last_obstacle_blocked:
            self._last_obstacle_blocked = detected
            self._obstacle_blocked_publisher.publish(Bool(data=detected))
            self._publish_state()
        if detected:
            rospy.logwarn(f"{self._robot.id}: obstacle detected, forward motion blocked")
