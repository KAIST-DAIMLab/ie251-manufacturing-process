from __future__ import annotations

import threading

import rospy
import tf
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

from pathfinder.utils.physics import yaw_from_quaternion, yaw_from_xyzw


class PoseEstimator:
    """Map-frame pose from tf (real robot) or odom (simulation).

    Call on_odom() from the /odom subscriber callback; get_pose() is
    thread-safe and can be called from the motion-control thread at any time.
    """

    def __init__(
        self,
        tf_listener: tf.TransformListener,
        namespace: str,
        origin: Pose2D,
        sim: bool = False,
    ) -> None:
        self._tf_listener = tf_listener
        self._namespace = namespace.strip('/')
        self._origin = origin
        self._sim = sim
        self._lock = threading.Lock()
        self._pose = Pose2D()
        self._pose.x = origin.x
        self._pose.y = origin.y
        self._pose.theta = origin.theta

    def on_odom(self, msg: Odometry) -> None:
        """Update pose estimate from an odometry message."""
        odom_pose = msg.pose.pose
        if self._sim:
            x = odom_pose.position.x + self._origin.x
            y = odom_pose.position.y + self._origin.y
            theta = yaw_from_quaternion(odom_pose.orientation) + self._origin.theta
        else:
            try:
                base_frame = f'{self._namespace}/base_footprint'
                (trans, rot) = self._tf_listener.lookupTransform(
                    'map', base_frame, rospy.Time(0)
                )
                x = trans[0]
                y = trans[1]
                theta = yaw_from_xyzw(*rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                x = odom_pose.position.x + self._origin.x
                y = odom_pose.position.y + self._origin.y
                theta = yaw_from_quaternion(odom_pose.orientation) + self._origin.theta

        with self._lock:
            self._pose.x = x
            self._pose.y = y
            self._pose.theta = theta

    def get_pose(self) -> Pose2D:
        """Return a copy of the current map-frame pose. Thread-safe."""
        with self._lock:
            pose = Pose2D()
            pose.x = self._pose.x
            pose.y = self._pose.y
            pose.theta = self._pose.theta
            return pose
