from __future__ import annotations

import rospy
import actionlib

from pathfinder.msg import FollowPathAction, FollowPathGoal  # type: ignore[import]


class PathFollowActionClient:
    """Wraps an actionlib client to send and cancel FollowPath goals asynchronously."""

    def __init__(self, action_namespace: str) -> None:
        """Create a SimpleActionClient for the given action namespace."""
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

    def cancel(self) -> None:
        """Cancel any in-flight FollowPath goal."""
        self._client.cancel_goal()
