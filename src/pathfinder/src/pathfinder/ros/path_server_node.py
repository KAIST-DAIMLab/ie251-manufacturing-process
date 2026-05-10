from __future__ import annotations

import rospy
from nav_msgs.msg import Odometry

from pathfinder.planning.path_orchestrator import PathOrchestrator
from pathfinder.robot.robot_state import RobotState
from pathfinder.ros.path_follow_action_client import PathFollowActionClient
from pathfinder.ros.path_request_service import PathRequestService
from pathfinder.ros.pose_tracker import PoseTracker


class PathServerNode:
    """ROS adapter: wires odom subscribers and PathRequestService for multi-robot path coordination."""

    def __init__(
        self,
        orchestrator: PathOrchestrator,
        tracker: PoseTracker,
        clients: dict[str, PathFollowActionClient],
        robot_odom_topics: dict[str, str],
    ) -> None:
        """Create per-robot locks and assemble the PathRequestService."""
        self._tracker = tracker
        self._robot_odom_topics = robot_odom_topics
        self._request_server = PathRequestService(orchestrator, tracker, clients)

    def start(self) -> None:
        """Register odom subscribers and start the PathRequestService."""
        for robot_id, topic in self._robot_odom_topics.items():
            rospy.Subscriber(
                topic,
                Odometry,
                lambda message, namespace=robot_id: self._on_odom(namespace, message),
            )
        self._request_server.start()
        rospy.loginfo("PathServerNode started.")

    def _on_odom(self, namespace: str, message: Odometry) -> None:
        self._tracker.update(namespace, RobotState.from_odometry(namespace, message).get_pose())
