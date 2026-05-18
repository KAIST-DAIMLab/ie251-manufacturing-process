from __future__ import annotations

import rospy
from actionlib_msgs.msg import GoalStatus


class ActionClientBase:
    """Shared cancel/is_active logic for actionlib SimpleActionClient wrappers."""

    def cancel(self) -> None:
        """Cancel any in-flight goal and wait for it to reach a terminal state."""
        if not self.is_active():
            return
        self._client.cancel_goal()
        self._client.wait_for_result(rospy.Duration(1.0))

    def is_active(self) -> bool:
        """Return whether a goal is currently in flight."""
        return self._client.get_state() in {
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING,
            GoalStatus.RECALLING,
        }
