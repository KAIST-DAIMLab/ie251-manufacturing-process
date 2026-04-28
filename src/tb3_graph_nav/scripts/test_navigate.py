#!/usr/bin/env python3
"""
Quick integration test client. Run AFTER launching simulation.launch and graph_nav.launch.

Usage:
    python3 src/tb3_graph_nav/scripts/test_navigate.py [target_node]

Example:
    python3 src/tb3_graph_nav/scripts/test_navigate.py B3
"""
import sys
import rospy
import actionlib
from tb3_graph_nav.msg import NavigateToNodeAction, NavigateToNodeGoal


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else 'B3'
    rospy.init_node('test_navigate_client')
    client = actionlib.SimpleActionClient('navigate_to_node', NavigateToNodeAction)
    rospy.loginfo(f"Waiting for nav_server...")
    client.wait_for_server(timeout=rospy.Duration(10.0))
    rospy.loginfo(f"Sending goal: navigate to '{target}'")
    goal = NavigateToNodeGoal(target_node_id=target)
    client.send_goal(
        goal,
        feedback_cb=lambda fb: rospy.loginfo(
            f"  -> heading to {fb.current_node_id} "
            f"({fb.x:.2f}, {fb.y:.2f}), dist={fb.distance_to_next:.2f} m"
        )
    )
    client.wait_for_result()
    result = client.get_result()
    if result.success:
        rospy.loginfo(f"SUCCESS: {result.message}")
    else:
        rospy.logwarn(f"FAILED: {result.message}")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
