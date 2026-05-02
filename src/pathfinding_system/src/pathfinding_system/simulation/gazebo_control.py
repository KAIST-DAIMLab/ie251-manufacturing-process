from __future__ import annotations
import rospy
from std_srvs.srv import Empty


def unpause_gazebo(service_name: str = '/gazebo/unpause_physics') -> None:
    rospy.wait_for_service(service_name)
    unpause_physics = rospy.ServiceProxy(service_name, Empty)
    unpause_physics()
