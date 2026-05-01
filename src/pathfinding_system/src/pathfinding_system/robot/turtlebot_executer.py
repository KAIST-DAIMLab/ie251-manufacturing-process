from __future__ import annotations
import math
import threading
from dataclasses import dataclass
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
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class ControllerGains:
    arrival_tol: float = 0.10
    heading_tol: float = 0.20
    k_lin: float = 0.5
    k_ang: float = 1.5
    max_lin: float = 0.22
    max_ang: float = 1.5

    @classmethod
    def from_params(cls, params: dict) -> ControllerGains:
        p = params.get('controller', params) if isinstance(params, dict) else {}
        return cls(
            arrival_tol=p.get('arrival_tol', cls.arrival_tol),
            heading_tol=p.get('heading_tol', cls.heading_tol),
            k_lin=p.get('k_lin', cls.k_lin),
            k_ang=p.get('k_ang', cls.k_ang),
            max_lin=p.get('max_lin_vel', cls.max_lin),
            max_ang=p.get('max_ang_vel', cls.max_ang),
        )


class TurtleBotExecuter:
    def __init__(
        self,
        robot_id: str,
        graph: Graph,
        gains: ControllerGains,
        control_rate_hz: float = 20.0,
        state_publish_rate_hz: float = 10.0,
    ) -> None:
        self._robot_id = robot_id
        self._ns = robot_id
        self._graph = graph
        self._gains = gains
        self._control_rate_hz = control_rate_hz
        self._state_publish_rate_hz = state_publish_rate_hz
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

        rospy.Timer(rospy.Duration(1.0 / self._control_rate_hz), self._control_tick)
        rospy.Timer(
            rospy.Duration(1.0 / self._state_publish_rate_hz), self._publish_state_tick
        )
        rospy.loginfo(f"TurtleBotExecuter for {self._robot_id} started.")

    def _on_follow_path(self, goal) -> None:
        from pathfinding_system.msg import FollowPathResult, FollowPathFeedback  # type: ignore[import]
        from pathfinding_system.planning.path import Path

        waypoints = [self._graph.get_node(nid) for nid in goal.node_ids]
        path = Path(waypoints)

        with self._lock:
            self._stop_requested = False
            self._active_path = path
            self._state.status = RobotStatus.MOVING

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._action_server.is_preempt_requested():
                self._cancel_active_path()
                self._action_server.set_preempted()
                return

            with self._lock:
                stopped = self._stop_requested
                complete = self._active_path is None
                cur_idx = 0 if complete else self._active_path.current_index()
                cur_pose = self._state.pose

            if stopped:
                self._action_server.set_aborted(
                    FollowPathResult(success=False, message="emergency stop")
                )
                return
            if complete:
                self._action_server.set_succeeded(
                    FollowPathResult(success=True, message="reached goal")
                )
                return

            fb = FollowPathFeedback()
            fb.current_index = cur_idx
            fb.current_pose = cur_pose
            self._action_server.publish_feedback(fb)
            rate.sleep()

    def _cancel_active_path(self) -> None:
        with self._lock:
            self._active_path = None
            self._state.status = RobotStatus.IDLE
        if self._cmd_pub is not None:
            self._cmd_pub.publish(Twist())

    def _on_odom(self, msg: Odometry) -> None:
        theta = _yaw_from_quaternion(msg.pose.pose.orientation)
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
        target, pose = self._next_target_pose()
        if target is None:
            self._publish_cmd(Twist())
            return

        dx, dy = target.x - pose.x, target.y - pose.y
        dist = math.hypot(dx, dy)

        if dist < self._gains.arrival_tol:
            self._advance_waypoint()
            self._publish_cmd(Twist())
            return

        self._publish_cmd(self._compute_command(dx, dy, dist, pose.theta))

    def _next_target_pose(self):
        with self._lock:
            if self._stop_requested or self._active_path is None:
                return None, None
            if self._active_path.is_complete():
                self._active_path = None
                self._state.status = RobotStatus.REACHED
                return None, None
            return self._active_path.peek(), self._state.pose

    def _advance_waypoint(self) -> None:
        with self._lock:
            if self._active_path is None:
                return
            self._active_path.next()
            if self._active_path.is_complete():
                self._active_path = None
                self._state.status = RobotStatus.REACHED

    def _compute_command(self, dx: float, dy: float, dist: float, theta: float) -> Twist:
        g = self._gains
        theta_err = _wrap_angle(math.atan2(dy, dx) - theta)
        cmd = Twist()
        cmd.angular.z = _clamp(g.k_ang * theta_err, g.max_ang)
        cmd.linear.x = 0.0 if abs(theta_err) > g.heading_tol else _clamp(g.k_lin * dist, g.max_lin)
        return cmd

    def _publish_cmd(self, cmd: Twist) -> None:
        if self._cmd_pub is not None:
            self._cmd_pub.publish(cmd)

    def _publish_state_tick(self, event) -> None:
        with self._lock:
            msg = self._state.to_msg()
        if self._state_pub is not None:
            self._state_pub.publish(msg)
