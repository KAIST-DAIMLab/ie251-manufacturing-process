import math
import threading
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

LINEAR_SPEED = 0.15   # m/s
ANGULAR_SPEED = 0.6   # rad/s
GOAL_TOL = 0.08       # m
ANGLE_TOL = 0.05      # rad
HEADING_KP = 0.4      # proportional gain (rad/s per rad) for heading correction while driving


class GoToGoal:
    """Drive the robot to a (x, y) coordinate using proportional heading control."""

    def __init__(self):
        self._lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self._odom_cb)

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        with self._lock:
            self._x, self._y, self._yaw = x, y, yaw

    def go_to(self, gx: float, gy: float, rate_hz: float = 20.0) -> bool:
        """
        Block until the robot reaches (gx, gy) or ROS shuts down.
        Returns True on success, False if rospy.is_shutdown() interrupted.
        """
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            with self._lock:
                x, y, yaw = self._x, self._y, self._yaw
            dx = gx - x
            dy = gy - y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < GOAL_TOL:
                self._stop()
                return True

            angle_to_goal = math.atan2(dy, dx)
            angle_err = self._wrap_angle(angle_to_goal - yaw)

            twist = Twist()
            if abs(angle_err) > ANGLE_TOL:
                twist.angular.z = ANGULAR_SPEED if angle_err > 0 else -ANGULAR_SPEED
            else:
                twist.linear.x = LINEAR_SPEED
                twist.angular.z = HEADING_KP * angle_err
            self._cmd_pub.publish(twist)
            rate.sleep()
        return False

    def _stop(self) -> None:
        msg = Twist()
        for _ in range(3):
            self._cmd_pub.publish(msg)

    @staticmethod
    def _wrap_angle(a: float) -> float:
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a
