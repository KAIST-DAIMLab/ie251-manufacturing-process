#!/usr/bin/env python3
import os
import rospy
import rospkg
from nav_msgs.msg import Odometry
from tb3_graph_nav.msg import NodeStatus
from tb3_graph_nav.graph import Graph


class PositionReporter:
    def __init__(self):
        rospy.init_node('position_reporter')
        graph_path = rospy.get_param(
            '~graph_config',
            os.path.join(
                rospkg.RosPack().get_path('tb3_graph_nav'), 'config', 'graph.yaml'
            )
        )
        self._graph = Graph.from_yaml(graph_path)
        self._pub = rospy.Publisher('/node_status', NodeStatus, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self._odom_cb)
        rospy.loginfo(
            f"position_reporter started — {len(self._graph.nodes)} nodes from {graph_path}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        nearest_id, dist = self._graph.nearest_node(x, y)
        status = NodeStatus()
        status.nearest_node_id = nearest_id
        status.x = x
        status.y = y
        status.distance_to_node = float(dist)
        self._pub.publish(status)

    def run(self) -> None:
        rospy.spin()


if __name__ == '__main__':
    PositionReporter().run()
