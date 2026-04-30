from __future__ import annotations
import math
import threading
import rospy
import actionlib
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from pathfinding_system.world.graph import Graph
from pathfinding_system.robot.robot_status import RobotStatus
from pathfinding_system.robot.robot_state import RobotState


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TurtleBotExecuter:
    def __init__(self, robot_id: str, graph: Graph, controller_params: dict) -> None:
        self._robot_id = robot_id
        self._ns = robot_id
        self._graph = graph
        self._params = controller_params
        self._state = RobotState(
            id=robot_id,
            pose=Pose2D(),
            velocity=Twist(),
            status=RobotStatus.IDLE,
            stamp=None,
        )
        self._active_path = None
        self._stop_requested = False
        self._lock = threading.Lock()
        self._cmd_pub = None
        self._state_pub = None
        self._action_server = None

    def start(self) -> None:
        from pathfinding_system.msg import FollowPathAction, RobotState as RobotStateMsg  # type: ignore[import]

        rospy.Subscriber(f'/{self._ns}/odom', Odometry, self._on_odom)
        rospy.Subscriber(f'/{self._ns}/emergency_stop', Empty, self._on_emergency_stop)

        self._cmd_pub = rospy.Publisher(f'/{self._ns}/cmd_vel', Twist, queue_size=1)
        self._state_pub = rospy.Publisher(
            f'/{self._ns}/robot_state', RobotStateMsg, queue_size=1
        )

        self._action_server = actionlib.SimpleActionServer(
            f'/{self._ns}/follow_path',
            FollowPathAction,
            execute_cb=self._on_follow_path,
            auto_start=False,
        )
        self._action_server.start()

        ctrl_hz = self._params.get('control_rate_hz', 20.0)
        state_hz = self._params.get('state_publish_rate_hz', 10.0)
        rospy.Timer(rospy.Duration(1.0 / ctrl_hz), self._control_tick)
        rospy.Timer(rospy.Duration(1.0 / state_hz), self._publish_state_tick)
        rospy.loginfo(f"TurtleBotExecuter for {self._robot_id} started.")

    def _on_follow_path(self, goal) -> None:
        from pathfinding_system.msg import FollowPathResult, FollowPathFeedback  # type: ignore[import]
        from pathfinding_system.planning.path import Path

        result = FollowPathResult()
        waypoints = [self._graph.get_node(nid) for nid in goal.node_ids]
        path = Path(waypoints)

        with self._lock:
            self._stop_requested = False
            self._active_path = path
            self._state.status = RobotStatus.MOVING

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._action_server.is_preempt_requested():
                with self._lock:
                    self._active_path = None
                    self._state.status = RobotStatus.IDLE
                self._cmd_pub.publish(Twist())
                self._action_server.set_preempted()
                return

            with self._lock:
                stopped = self._stop_requested
                complete = self._active_path is None
                if not complete and self._active_path is not None:
                    cur_idx = self._active_path.current_index()
                    cur_pose = self._state.pose
                else:
                    cur_idx = 0
                    cur_pose = self._state.pose

            if stopped:
                result.success = False
                result.message = "emergency stop"
                self._action_server.set_aborted(result)
                return

            if complete:
                result.success = True
                result.message = "reached goal"
                self._action_server.set_succeeded(result)
                return

            fb = FollowPathFeedback()
            fb.current_index = cur_idx
            fb.current_pose = cur_pose
            self._action_server.publish_feedback(fb)
            rate.sleep()

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        with self._lock:
            self._state.pose.x = msg.pose.pose.position.x
            self._state.pose.y = msg.pose.pose.position.y
            self._state.pose.theta = theta
            self._state.velocity = msg.twist.twist
            self._state.stamp = msg.header.stamp

    def _on_emergency_stop(self, msg: Empty) -> None:
        with self._lock:
            self._stop_requested = True
            self._state.status = RobotStatus.STOPPED
        if self._cmd_pub is not None:
            self._cmd_pub.publish(Twist())
        rospy.logwarn(f"{self._robot_id}: emergency stop received.")

    def _control_tick(self, event) -> None:
        with self._lock:
            if self._stop_requested or self._active_path is None:
                if self._cmd_pub is not None:
                    self._cmd_pub.publish(Twist())
                return
            if self._active_path.is_complete():
                self._active_path = None
                self._state.status = RobotStatus.REACHED
                if self._cmd_pub is not None:
                    self._cmd_pub.publish(Twist())
                return
            target = self._active_path.peek()
            px = self._state.pose.x
            py = self._state.pose.y
            theta = self._state.pose.theta

        tx, ty = target.x, target.y
        dx, dy = tx - px, ty - py
        dist = math.sqrt(dx * dx + dy * dy)

        p = self._params.get('controller', self._params)
        arrival_tol = p.get('arrival_tol', 0.10)
        heading_tol = p.get('heading_tol', 0.2)
        k_lin = p.get('k_lin', 0.5)
        k_ang = p.get('k_ang', 1.5)
        max_lin = p.get('max_lin_vel', 0.22)
        max_ang = p.get('max_ang_vel', 1.5)

        if dist < arrival_tol:
            with self._lock:
                if self._active_path is not None:
                    self._active_path.next()
                    if self._active_path.is_complete():
                        self._active_path = None
                        self._state.status = RobotStatus.REACHED
            if self._cmd_pub is not None:
                self._cmd_pub.publish(Twist())
            return

        theta_err = _wrap_angle(math.atan2(dy, dx) - theta)
        cmd = Twist()
        if abs(theta_err) > heading_tol:
            cmd.linear.x = 0.0
            cmd.angular.z = _clamp(k_ang * theta_err, max_ang)
        else:
            cmd.linear.x = _clamp(k_lin * dist, max_lin)
            cmd.angular.z = _clamp(k_ang * theta_err, max_ang)

        if self._cmd_pub is not None:
            self._cmd_pub.publish(cmd)

    def _publish_state_tick(self, event) -> None:
        with self._lock:
            msg = self._state.to_msg()
        if self._state_pub is not None:
            self._state_pub.publish(msg)
