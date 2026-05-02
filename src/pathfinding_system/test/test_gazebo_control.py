import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.waited_for = []
    rospy.service_proxy_calls = []
    rospy.called_services = []

    def wait_for_service(service_name):
        rospy.waited_for.append(service_name)

    def service_proxy(service_name, service_type):
        rospy.service_proxy_calls.append((service_name, service_type))

        def call_service():
            rospy.called_services.append(service_name)

        return call_service

    rospy.wait_for_service = wait_for_service
    rospy.ServiceProxy = service_proxy
    sys.modules['rospy'] = rospy

    std_srvs = types.ModuleType('std_srvs')
    std_srvs_srv = types.ModuleType('std_srvs.srv')
    std_srvs_srv.Empty = object
    sys.modules['std_srvs'] = std_srvs
    sys.modules['std_srvs.srv'] = std_srvs_srv


_install_ros_stubs()

from pathfinding_system.simulation.gazebo_control import unpause_gazebo


class GazeboControlTest(unittest.TestCase):
    def test_unpause_gazebo_waits_for_service_before_calling_it(self):
        import rospy
        from std_srvs.srv import Empty

        unpause_gazebo()

        self.assertEqual(rospy.waited_for, ['/gazebo/unpause_physics'])
        self.assertEqual(
            rospy.service_proxy_calls,
            [('/gazebo/unpause_physics', Empty)],
        )
        self.assertEqual(rospy.called_services, ['/gazebo/unpause_physics'])


if __name__ == '__main__':
    unittest.main()
