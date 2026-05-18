import math
import os
import sys
import types
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = sys.modules.get('rospy') or types.ModuleType('rospy')
    rospy.Time = lambda secs=0: types.SimpleNamespace(secs=secs)
    sys.modules['rospy'] = rospy

    tf = sys.modules.get('tf') or types.ModuleType('tf')
    tf.LookupException = type('LookupException', (Exception,), {})
    tf.ConnectivityException = type('ConnectivityException', (Exception,), {})
    tf.ExtrapolationException = type('ExtrapolationException', (Exception,), {})
    sys.modules['tf'] = tf

    geometry_msgs = sys.modules.get('geometry_msgs') or types.ModuleType('geometry_msgs')
    geometry_msgs_msg = sys.modules.get('geometry_msgs.msg') or types.ModuleType('geometry_msgs.msg')

    class Pose2D:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0

    geometry_msgs_msg.Pose2D = Pose2D
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

    nav_msgs = sys.modules.get('nav_msgs') or types.ModuleType('nav_msgs')
    nav_msgs_msg = sys.modules.get('nav_msgs.msg') or types.ModuleType('nav_msgs.msg')
    nav_msgs_msg.Odometry = object
    sys.modules['nav_msgs'] = nav_msgs
    sys.modules['nav_msgs.msg'] = nav_msgs_msg


_install_ros_stubs()

from pathfinder.robot.pose_estimator import PoseEstimator  # noqa: E402
from pathfinder.utils.physics import yaw_from_xyzw  # noqa: E402


def _origin(x=0.0, y=0.0, theta=0.0):
    import geometry_msgs.msg as gm
    p = gm.Pose2D()
    p.x = x
    p.y = y
    p.theta = theta
    return p


def _odom_msg(x=0.0, y=0.0, yaw=0.0):
    half = yaw / 2.0
    return types.SimpleNamespace(
        pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=x, y=y, z=0.0),
                orientation=types.SimpleNamespace(
                    x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)
                ),
            )
        ),
        twist=types.SimpleNamespace(twist=types.SimpleNamespace()),
    )


def _tf_listener(trans=(0.0, 0.0, 0.0), rot=(0.0, 0.0, 0.0, 1.0)):
    return types.SimpleNamespace(lookupTransform=lambda *_: (trans, rot))


def _failing_tf_listener(exc_type=None):
    import tf as tf_mod
    exc = exc_type or tf_mod.LookupException

    def _raise(*_):
        raise exc()

    return types.SimpleNamespace(lookupTransform=_raise)


class PoseEstimatorSimModeTest(unittest.TestCase):
    def test_zero_origin_passes_odom_through(self):
        est = PoseEstimator(_tf_listener(), 'tb3_01', _origin(), sim=True)
        est.on_odom(_odom_msg(x=1.5, y=-0.3, yaw=0.8))
        pose = est.get_pose()
        self.assertAlmostEqual(pose.x, 1.5)
        self.assertAlmostEqual(pose.y, -0.3)
        self.assertAlmostEqual(pose.theta, 0.8)

    def test_non_zero_origin_adds_offset(self):
        est = PoseEstimator(_tf_listener(), 'tb3_01', _origin(x=1.0, y=2.0, theta=0.25), sim=True)
        est.on_odom(_odom_msg(x=1.2, y=-0.4, yaw=0.75))
        pose = est.get_pose()
        self.assertAlmostEqual(pose.x, 2.2)
        self.assertAlmostEqual(pose.y, 1.6)
        self.assertAlmostEqual(pose.theta, 1.0)

    def test_initialized_to_origin_before_first_odom(self):
        est = PoseEstimator(_tf_listener(), 'tb3_01', _origin(x=3.0, y=4.0, theta=0.5), sim=True)
        pose = est.get_pose()
        self.assertAlmostEqual(pose.x, 3.0)
        self.assertAlmostEqual(pose.y, 4.0)
        self.assertAlmostEqual(pose.theta, 0.5)


class PoseEstimatorRealModeTest(unittest.TestCase):
    def test_tf_success_uses_transform(self):
        yaw = math.pi / 4
        half = yaw / 2.0
        rot = (0.0, 0.0, math.sin(half), math.cos(half))
        listener = _tf_listener(trans=(2.5, -1.0, 0.0), rot=rot)
        est = PoseEstimator(listener, 'tb3_01', _origin(), sim=False)
        est.on_odom(_odom_msg(x=99.0, y=99.0, yaw=99.0))
        pose = est.get_pose()
        self.assertAlmostEqual(pose.x, 2.5)
        self.assertAlmostEqual(pose.y, -1.0)
        self.assertAlmostEqual(pose.theta, yaw, places=5)

    def test_lookup_exception_falls_back_to_odom_plus_origin(self):
        est = PoseEstimator(_failing_tf_listener(), 'tb3_01', _origin(x=1.0, y=2.0, theta=0.0), sim=False)
        est.on_odom(_odom_msg(x=0.5, y=0.3, yaw=0.4))
        pose = est.get_pose()
        self.assertAlmostEqual(pose.x, 1.5)
        self.assertAlmostEqual(pose.y, 2.3)
        self.assertAlmostEqual(pose.theta, 0.4)

    def test_connectivity_exception_falls_back(self):
        import tf as tf_mod
        est = PoseEstimator(_failing_tf_listener(tf_mod.ConnectivityException), 'tb3_01', _origin(), sim=False)
        est.on_odom(_odom_msg(x=1.0, y=2.0, yaw=0.5))
        pose = est.get_pose()
        self.assertAlmostEqual(pose.x, 1.0)
        self.assertAlmostEqual(pose.y, 2.0)
        self.assertAlmostEqual(pose.theta, 0.5)

    def test_extrapolation_exception_falls_back(self):
        import tf as tf_mod
        est = PoseEstimator(_failing_tf_listener(tf_mod.ExtrapolationException), 'tb3_01', _origin(), sim=False)
        est.on_odom(_odom_msg(x=1.0, y=2.0, yaw=0.5))
        pose = est.get_pose()
        self.assertAlmostEqual(pose.x, 1.0)

    def test_namespace_leading_slash_stripped(self):
        """Ensure /tb3_01 and tb3_01 produce the same base_frame string."""
        import tf as tf_mod
        calls = []

        def capturing_lookup(parent, child, time):
            calls.append(child)
            raise tf_mod.LookupException()

        listener = types.SimpleNamespace(lookupTransform=capturing_lookup)
        est = PoseEstimator(listener, '/tb3_01', _origin(), sim=False)
        est.on_odom(_odom_msg())
        self.assertEqual(calls[0], 'tb3_01/base_footprint')


class PoseEstimatorGetPoseTest(unittest.TestCase):
    def test_get_pose_returns_copy(self):
        est = PoseEstimator(_tf_listener(), 'tb3_01', _origin(), sim=True)
        est.on_odom(_odom_msg(x=1.0, y=2.0, yaw=0.5))
        pose = est.get_pose()
        pose.x = 999.0
        self.assertAlmostEqual(est.get_pose().x, 1.0)


class YawFromXyzwTest(unittest.TestCase):
    def test_identity_quaternion_gives_zero(self):
        self.assertAlmostEqual(yaw_from_xyzw(0.0, 0.0, 0.0, 1.0), 0.0)

    def test_positive_90_degrees(self):
        half = math.pi / 4
        self.assertAlmostEqual(yaw_from_xyzw(0.0, 0.0, math.sin(half), math.cos(half)), math.pi / 2, places=5)

    def test_negative_90_degrees(self):
        half = -math.pi / 4
        self.assertAlmostEqual(yaw_from_xyzw(0.0, 0.0, math.sin(half), math.cos(half)), -math.pi / 2, places=5)

    def test_180_degrees(self):
        result = abs(yaw_from_xyzw(0.0, 0.0, 1.0, 0.0))
        self.assertAlmostEqual(result, math.pi, places=5)


if __name__ == '__main__':
    unittest.main()
