from __future__ import annotations
import importlib.machinery
import importlib.util
import os
import sys
import types
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SCRIPT_PATH = os.path.join(PACKAGE_ROOT, 'scripts', 'real_odom_tf_bridge')


class RealOdomTfBridgeTest(unittest.TestCase):
    def test_bridge_publishes_prefixed_transform_from_odom(self):
        original_modules = {
            name: sys.modules.get(name)
            for name in ['rospy', 'tf2_ros', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg']
        }
        sent = []
        subscribers = []

        rospy = types.ModuleType('rospy')
        rospy.init_node = lambda name: None
        rospy.get_param = lambda name, default=None: {
            '~input': '/tb3_01/odom',
            '~parent_frame': 'tb3_01/odom',
            '~child_frame': 'tb3_01/base_footprint',
        }.get(name, default)
        rospy.Subscriber = lambda topic, msg_type, callback: subscribers.append((topic, msg_type, callback))
        rospy.spin = lambda: None
        sys.modules['rospy'] = rospy

        tf2_ros = types.ModuleType('tf2_ros')

        class TransformBroadcaster:
            def sendTransform(self, transform):
                sent.append(transform)

        tf2_ros.TransformBroadcaster = TransformBroadcaster
        sys.modules['tf2_ros'] = tf2_ros

        geometry_msgs = types.ModuleType('geometry_msgs')
        geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')

        class TransformStamped:
            def __init__(self):
                self.header = types.SimpleNamespace(stamp=None, frame_id='')
                self.child_frame_id = ''
                self.transform = types.SimpleNamespace(
                    translation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    rotation=None,
                )

        geometry_msgs_msg.TransformStamped = TransformStamped
        sys.modules['geometry_msgs'] = geometry_msgs
        sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

        nav_msgs = types.ModuleType('nav_msgs')
        nav_msgs_msg = types.ModuleType('nav_msgs.msg')
        nav_msgs_msg.Odometry = object
        sys.modules['nav_msgs'] = nav_msgs
        sys.modules['nav_msgs.msg'] = nav_msgs_msg

        try:
            loader = importlib.machinery.SourceFileLoader('real_odom_tf_bridge', SCRIPT_PATH)
            spec = importlib.util.spec_from_loader('real_odom_tf_bridge', loader)
            module = importlib.util.module_from_spec(spec)
            loader.exec_module(module)

            module.main()
            callback = subscribers[0][2]
            rotation = types.SimpleNamespace(x=0.0, y=0.0, z=0.1, w=0.99)
            msg = types.SimpleNamespace(
                header=types.SimpleNamespace(stamp='stamp'),
                pose=types.SimpleNamespace(
                    pose=types.SimpleNamespace(
                        position=types.SimpleNamespace(x=1.0, y=2.0, z=0.0),
                        orientation=rotation,
                    )
                ),
            )
            callback(msg)
        finally:
            for name, original in original_modules.items():
                if original is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = original

        self.assertEqual(subscribers[0][0], '/tb3_01/odom')
        self.assertEqual(sent[0].header.stamp, 'stamp')
        self.assertEqual(sent[0].header.frame_id, 'tb3_01/odom')
        self.assertEqual(sent[0].child_frame_id, 'tb3_01/base_footprint')
        self.assertEqual(sent[0].transform.translation.x, 1.0)
        self.assertEqual(sent[0].transform.translation.y, 2.0)
        self.assertIs(sent[0].transform.rotation, rotation)

    def test_bridge_can_stamp_transform_with_host_time(self):
        module, subscribers, sent, restore = self._load_bridge_for_callback({
            '~input': '/tb3_01/odom',
            '~parent_frame': 'tb3_01/odom',
            '~child_frame': 'tb3_01/base_footprint',
            '~stamp_with_now': True,
        })
        try:
            module.main()
            callback = subscribers[0][2]
            msg = types.SimpleNamespace(
                header=types.SimpleNamespace(stamp='robot-clock-stamp'),
                pose=types.SimpleNamespace(
                    pose=types.SimpleNamespace(
                        position=types.SimpleNamespace(x=1.0, y=2.0, z=0.0),
                        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                    )
                ),
            )
            callback(msg)
        finally:
            restore()

        self.assertEqual(sent[0].header.stamp, 'host-now')

    def test_bridge_ignores_odom_with_nan_pose(self):
        module, subscribers, sent, restore = self._load_bridge_for_callback()
        try:
            module.main()
            callback = subscribers[0][2]
            msg = types.SimpleNamespace(
                header=types.SimpleNamespace(stamp='stamp'),
                pose=types.SimpleNamespace(
                    pose=types.SimpleNamespace(
                        position=types.SimpleNamespace(x=float('nan'), y=2.0, z=0.0),
                        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                    )
                ),
            )
            callback(msg)
        finally:
            restore()

        self.assertEqual(sent, [])

    def _load_bridge_for_callback(self, params=None):
        original_modules = {
            name: sys.modules.get(name)
            for name in ['rospy', 'tf2_ros', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg']
        }
        sent = []
        subscribers = []

        rospy = types.ModuleType('rospy')
        rospy.init_node = lambda name: None
        param_values = {
            '~input': '/tb3_01/odom',
            '~parent_frame': 'tb3_01/odom',
            '~child_frame': 'tb3_01/base_footprint',
        }
        if params is not None:
            param_values.update(params)
        rospy.get_param = lambda name, default=None: param_values.get(name, default)
        rospy.Time = types.SimpleNamespace(now=lambda: 'host-now')
        rospy.Subscriber = lambda topic, msg_type, callback: subscribers.append((topic, msg_type, callback))
        rospy.spin = lambda: None
        rospy.logwarn_throttle = lambda *args, **kwargs: None
        sys.modules['rospy'] = rospy

        tf2_ros = types.ModuleType('tf2_ros')

        class TransformBroadcaster:
            def sendTransform(self, transform):
                sent.append(transform)

        tf2_ros.TransformBroadcaster = TransformBroadcaster
        sys.modules['tf2_ros'] = tf2_ros

        geometry_msgs = types.ModuleType('geometry_msgs')
        geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')

        class TransformStamped:
            def __init__(self):
                self.header = types.SimpleNamespace(stamp=None, frame_id='')
                self.child_frame_id = ''
                self.transform = types.SimpleNamespace(
                    translation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    rotation=None,
                )

        geometry_msgs_msg.TransformStamped = TransformStamped
        sys.modules['geometry_msgs'] = geometry_msgs
        sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

        nav_msgs = types.ModuleType('nav_msgs')
        nav_msgs_msg = types.ModuleType('nav_msgs.msg')
        nav_msgs_msg.Odometry = object
        sys.modules['nav_msgs'] = nav_msgs
        sys.modules['nav_msgs.msg'] = nav_msgs_msg

        loader = importlib.machinery.SourceFileLoader('real_odom_tf_bridge', SCRIPT_PATH)
        spec = importlib.util.spec_from_loader('real_odom_tf_bridge', loader)
        module = importlib.util.module_from_spec(spec)
        loader.exec_module(module)

        def restore():
            for name, original in original_modules.items():
                if original is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = original

        return module, subscribers, sent, restore


if __name__ == '__main__':
    unittest.main()
