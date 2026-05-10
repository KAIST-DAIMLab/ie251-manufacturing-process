from __future__ import annotations
import rospy
import actionlib
from typing import Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from pathfinder.msg import FollowPathFeedback, FollowPathResult


class FollowPathClient:
    """Wraps an actionlib client; runs a synchronous dispatch with feedback and cancel callbacks."""

    def __init__(self, action_namespace: str) -> None:
        """Create a SimpleActionClient for the given action namespace."""
        from pathfinder.msg import FollowPathAction  # type: ignore[import]
        self._client = actionlib.SimpleActionClient(
            f'/{action_namespace}/follow_path', FollowPathAction
        )

    def dispatch(
        self,
        node_ids: list[int],
        on_feedback: Callable[[FollowPathFeedback], None],
        is_canceled: Callable[[], bool],
    ) -> FollowPathResult | None:
        """Send a FollowPath goal and block until done, canceled, or server unavailable."""
        from pathfinder.msg import FollowPathGoal  # type: ignore[import]

        if not self._client.wait_for_server(timeout=rospy.Duration(5.0)):
            return None

        latest_feedback = None

        def _store_feedback(feedback):
            nonlocal latest_feedback
            latest_feedback = feedback

        goal = FollowPathGoal()
        goal.node_ids = node_ids
        self._client.send_goal(goal, feedback_cb=_store_feedback)

        while not self._client.wait_for_result(timeout=rospy.Duration(0.1)):
            if is_canceled():
                self._client.cancel_goal()
                return None
            if latest_feedback is not None:
                on_feedback(latest_feedback)

        return self._client.get_result()
