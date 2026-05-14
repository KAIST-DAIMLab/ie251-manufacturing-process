from __future__ import annotations
import importlib.machinery
import importlib.util
import os
import sys
import types
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SCRIPT_PATH = os.path.join(PACKAGE_ROOT, 'scripts', 'scan_relay')


class ScanRelayLaunchTest(unittest.TestCase):
    def test_scan_relay_uses_roslaunch_node_name_for_private_params(self):
        original_rospy = sys.modules.get('rospy')
        original_sensor_msgs = sys.modules.get('sensor_msgs')
        original_sensor_msgs_msg = sys.modules.get('sensor_msgs.msg')
        init_calls = []

        rospy = types.ModuleType('rospy')
        rospy.init_node = lambda name, anonymous=False: init_calls.append((name, anonymous))
        rospy.get_param = lambda name, default=None: default
        rospy.Publisher = lambda *args, **kwargs: types.SimpleNamespace(publish=lambda msg: None)
        rospy.Subscriber = lambda *args, **kwargs: None
        rospy.spin = lambda: None
        sys.modules['rospy'] = rospy

        sensor_msgs = types.ModuleType('sensor_msgs')
        sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
        sensor_msgs_msg.LaserScan = object
        sys.modules['sensor_msgs'] = sensor_msgs
        sys.modules['sensor_msgs.msg'] = sensor_msgs_msg

        try:
            loader = importlib.machinery.SourceFileLoader('scan_relay', SCRIPT_PATH)
            spec = importlib.util.spec_from_loader('scan_relay', loader)
            module = importlib.util.module_from_spec(spec)
            loader.exec_module(module)

            module.main()
        finally:
            if original_rospy is None:
                sys.modules.pop('rospy', None)
            else:
                sys.modules['rospy'] = original_rospy
            if original_sensor_msgs is None:
                sys.modules.pop('sensor_msgs', None)
            else:
                sys.modules['sensor_msgs'] = original_sensor_msgs
            if original_sensor_msgs_msg is None:
                sys.modules.pop('sensor_msgs.msg', None)
            else:
                sys.modules['sensor_msgs.msg'] = original_sensor_msgs_msg

        self.assertEqual(init_calls, [('scan_relay', False)])

    def test_scan_relay_can_stamp_scan_with_host_time(self):
        original_rospy = sys.modules.get('rospy')
        original_sensor_msgs = sys.modules.get('sensor_msgs')
        original_sensor_msgs_msg = sys.modules.get('sensor_msgs.msg')
        published = []
        subscribers = []

        rospy = types.ModuleType('rospy')
        rospy.init_node = lambda name, anonymous=False: None
        rospy.get_param = lambda name, default=None: {
            '~frame_id': 'tb3_01/base_scan',
            '~stamp_with_now': True,
        }.get(name, default)
        rospy.Time = types.SimpleNamespace(now=lambda: 'host-now')
        rospy.Publisher = lambda *args, **kwargs: types.SimpleNamespace(publish=published.append)
        rospy.Subscriber = lambda topic, msg_type, callback: subscribers.append((topic, msg_type, callback))
        rospy.spin = lambda: None
        sys.modules['rospy'] = rospy

        sensor_msgs = types.ModuleType('sensor_msgs')
        sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
        sensor_msgs_msg.LaserScan = object
        sys.modules['sensor_msgs'] = sensor_msgs
        sys.modules['sensor_msgs.msg'] = sensor_msgs_msg

        try:
            loader = importlib.machinery.SourceFileLoader('scan_relay_stamp', SCRIPT_PATH)
            spec = importlib.util.spec_from_loader('scan_relay_stamp', loader)
            module = importlib.util.module_from_spec(spec)
            loader.exec_module(module)

            module.main()
            msg = types.SimpleNamespace(header=types.SimpleNamespace(stamp='robot-clock-stamp', frame_id='base_scan'))
            subscribers[0][2](msg)
        finally:
            if original_rospy is None:
                sys.modules.pop('rospy', None)
            else:
                sys.modules['rospy'] = original_rospy
            if original_sensor_msgs is None:
                sys.modules.pop('sensor_msgs', None)
            else:
                sys.modules['sensor_msgs'] = original_sensor_msgs
            if original_sensor_msgs_msg is None:
                sys.modules.pop('sensor_msgs.msg', None)
            else:
                sys.modules['sensor_msgs.msg'] = original_sensor_msgs_msg

        self.assertEqual(published[0].header.frame_id, 'tb3_01/base_scan')
        self.assertEqual(published[0].header.stamp, 'host-now')


if __name__ == '__main__':
    unittest.main()
