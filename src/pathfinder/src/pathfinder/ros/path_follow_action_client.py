from __future__ import annotations

import rospy
import actionlib

from pathfinder.msg import FollowPathAction, FollowPathGoal  # type: ignore[import]
from pathfinder.ros.action_client_base import ActionClientBase


class PathFollowActionClient(ActionClientBase):
    """Wraps an actionlib client to send and cancel FollowPath goals asynchronously."""

    def __init__(self, robot_id: str, action_namespace: str) -> None:
        self.robot_id = robot_id
        self._client = actionlib.SimpleActionClient(
            f'/{action_namespace}/follow_path', FollowPathAction
        )

    def send(self, node_ids: list[int]) -> bool:
        """Send a FollowPath goal asynchronously. Returns False if the server is unreachable."""
        if not self._client.wait_for_server(timeout=rospy.Duration(5.0)):
            return False
        goal = FollowPathGoal()
        goal.node_ids = node_ids
        self._client.send_goal(goal)
        return True
