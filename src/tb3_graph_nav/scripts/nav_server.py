#!/usr/bin/env python3
import math
import os
import threading
import rospy
import rospkg
import actionlib
from nav_msgs.msg import Odometry
from tb3_graph_nav.msg import (
    NavigateToNodeAction,
    NavigateToNodeFeedback,
    NavigateToNodeResult,
)
from tb3_graph_nav.graph import Graph
from tb3_graph_nav.planner import dijkstra
from tb3_graph_nav.go_to_goal import GoToGoal


class NavServer:
    def __init__(self):
        rospy.init_node('nav_server')
        if rospy.has_param('~graph_config'):
            graph_path = rospy.get_param('~graph_config')
        else:
            graph_path = os.path.join(
                rospkg.RosPack().get_path('tb3_graph_nav'), 'config', 'graph.yaml'
            )
        self._graph = Graph.from_yaml(graph_path)
        self._gtg = GoToGoal()
        self._x = 0.0
        self._y = 0.0
        self._pos_lock = threading.Lock()
        rospy.Subscriber('/odom', Odometry, self._odom_cb)
        self._server = actionlib.SimpleActionServer(
            'navigate_to_node',
            NavigateToNodeAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._server.start()
        rospy.loginfo(
            f"nav_server ready — {len(self._graph.nodes)} nodes from {graph_path}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        with self._pos_lock:
            self._x = msg.pose.pose.position.x
            self._y = msg.pose.pose.position.y

    def _execute(self, goal) -> None:
        target = goal.target_node_id
        rospy.loginfo(f"Goal: navigate to '{target}'")

        if target not in self._graph.nodes:
            msg = f"Unknown node: '{target}'"
            rospy.logwarn(msg)
            self._server.set_aborted(
                NavigateToNodeResult(success=False, message=msg)
            )
            return

        with self._pos_lock:
            x, y = self._x, self._y
        start_id, _ = self._graph.nearest_node(x, y)
        path = dijkstra(self._graph, start_id, target)

        if path is None:
            msg = f"No path from '{start_id}' to '{target}'"
            rospy.logwarn(msg)
            self._server.set_aborted(
                NavigateToNodeResult(success=False, message=msg)
            )
            return

        rospy.loginfo(f"Path: {' -> '.join(path)}")

        for node_id in path[1:]:  # skip start node (already there)
            if self._server.is_preempt_requested():
                rospy.loginfo("Goal preempted")
                self._server.set_preempted()
                return

            node = self._graph.nodes[node_id]
            with self._pos_lock:
                fx, fy = self._x, self._y
            fb = NavigateToNodeFeedback()
            fb.current_node_id = node_id
            fb.x = fx
            fb.y = fy
            fb.distance_to_next = float(
                math.sqrt((node.x - fx) ** 2 + (node.y - fy) ** 2)
            )
            self._server.publish_feedback(fb)
            rospy.loginfo(f"Driving to {node_id} ({node.x:.2f}, {node.y:.2f})")
            if not self._gtg.go_to(node.x, node.y):
                self._server.set_aborted(
                    NavigateToNodeResult(success=False, message="ROS shutdown during navigation")
                )
                return
            rospy.loginfo(f"Reached {node_id}")

        result = NavigateToNodeResult(success=True, message=f"Reached '{target}'")
        self._server.set_succeeded(result)
        rospy.loginfo(f"Goal succeeded: reached '{target}'")

    def run(self) -> None:
        rospy.spin()


if __name__ == '__main__':
    try:
        NavServer().run()
    except rospy.ROSInterruptException:
        pass
