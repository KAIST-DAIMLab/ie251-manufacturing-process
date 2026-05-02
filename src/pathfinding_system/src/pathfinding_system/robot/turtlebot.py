from __future__ import annotations
import math
import threading
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from pathfinding_system.robot.robot_status import RobotStatus
from pathfinding_system.robot.robot_state import RobotState
from pathfinding_system.world.node import Node


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class TurtleBot:
    def __init__(
        self,
        robot_id: str,
        state_publish_rate_hz: float = 10.0,
        k_lin: float = 0.5,
        k_ang: float = 1.5,
        max_lin_vel: float = 0.22,
        max_ang_vel: float = 1.5,
        arrival_tol: float = 0.10,
        heading_tol: float = 0.2,
    ) -> None:
        self.id = robot_id
        self._ns = robot_id
        self._state_publish_rate_hz = state_publish_rate_hz
        self._k_lin = k_lin
        self._k_ang = k_ang
        self._max_lin_vel = max_lin_vel
        self._max_ang_vel = max_ang_vel
        self._arrival_tol = arrival_tol
        self._heading_tol = heading_tol
        self._state = RobotState(
            id=robot_id,
            pose=Pose2D(),
            velocity=Twist(),
            status=RobotStatus.IDLE,
            stamp=None,
        )
        self._stop_requested = False
        self._lock = threading.Lock()
        self._state_pub = None
        self._cmd_vel_pub = None

    @property
    def cmd_vel_topic(self) -> str:
        return f'/{self._ns}/cmd_vel'

    def start(self) -> None:
        from pathfinding_system.msg import RobotState as RobotStateMsg  # type: ignore[import]

        rospy.Subscriber(f'/{self._ns}/odom', Odometry, self._on_odom)
        rospy.Subscriber(f'/{self._ns}/emergency_stop', Empty, self._on_emergency_stop)
        self._state_pub = rospy.Publisher(
            f'/{self._ns}/robot_state', RobotStateMsg, queue_size=1
        )
        self._cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        rospy.Timer(
            rospy.Duration(1.0 / self._state_publish_rate_hz), self._publish_state_tick
        )

    def drive_towards(self, node: Node) -> bool:
        if self._cmd_vel_pub is None:
            raise RuntimeError('TurtleBot.start() must be called before drive_towards().')
        with self._lock:
            x = self._state.pose.x
            y = self._state.pose.y
            theta = self._state.pose.theta
            self._state.status = RobotStatus.MOVING

        dx = node.x - x
        dy = node.y - y
        distance = math.hypot(dx, dy)
        cmd = Twist()

        if distance <= self._arrival_tol:
            self._cmd_vel_pub.publish(cmd)
            return True

        desired_heading = math.atan2(dy, dx)
        heading_error = _wrap_to_pi(desired_heading - theta)
        cmd.angular.z = _clamp(self._k_ang * heading_error, self._max_ang_vel)
        if abs(heading_error) <= self._heading_tol:
            cmd.linear.x = min(self._k_lin * distance, self._max_lin_vel)

        self._cmd_vel_pub.publish(cmd)
        return False

    def stop_motion(self) -> None:
        if self._cmd_vel_pub is not None:
            self._cmd_vel_pub.publish(Twist())

    def stop_requested(self) -> bool:
        with self._lock:
            return self._stop_requested

    def current_pose(self) -> Pose2D:
        with self._lock:
            pose = Pose2D()
            pose.x = self._state.pose.x
            pose.y = self._state.pose.y
            pose.theta = self._state.pose.theta
            return pose

    def set_status(self, status: RobotStatus) -> None:
        with self._lock:
            if status == RobotStatus.MOVING:
                self._stop_requested = False
            self._state.status = status

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
        self.stop_motion()
        rospy.logwarn(f"{self.id}: emergency stop received.")

    def _publish_state_tick(self, event) -> None:
        with self._lock:
            msg = self._state.to_msg()
        if self._state_pub is not None:
            self._state_pub.publish(msg)
