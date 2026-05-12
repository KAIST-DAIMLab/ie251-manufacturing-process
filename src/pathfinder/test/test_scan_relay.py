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


if __name__ == '__main__':
    unittest.main()
