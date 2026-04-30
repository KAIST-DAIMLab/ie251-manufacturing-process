from __future__ import annotations
import rospy
from std_msgs.msg import Empty
from pathfinding_system.safety.linear_predictor import LinearPredictor


class CollisionMonitor:
    def __init__(
        self,
        predictor: LinearPredictor,
        robot_namespaces: list[str],
        horizon: float,
        check_rate_hz: float,
    ) -> None:
        self._predictor = predictor
        self._namespaces = robot_namespaces
        self._horizon = horizon
        self._check_rate_hz = check_rate_hz
        self._states: dict[str, object] = {}
        self._stop_pubs: dict[str, rospy.Publisher] = {}

    def start(self) -> None:
        from pathfinding_system.msg import RobotState as RobotStateMsg  # type: ignore[import]
        for ns in self._namespaces:
            rospy.Subscriber(
                f'/{ns}/robot_state',
                RobotStateMsg,
                lambda msg, n=ns: self.update_state(n, msg),
            )
            self._stop_pubs[ns] = rospy.Publisher(
                f'/{ns}/emergency_stop', Empty, queue_size=1
            )
        rospy.Timer(
            rospy.Duration(1.0 / self._check_rate_hz),
            self._tick,
        )
        rospy.loginfo("CollisionMonitor started.")

    def update_state(self, ns: str, state) -> None:
        self._states[ns] = state

    def _tick(self, event) -> None:
        ns_list = self._namespaces
        for i in range(len(ns_list)):
            for j in range(i + 1, len(ns_list)):
                s1 = self._states.get(ns_list[i])
                s2 = self._states.get(ns_list[j])
                if s1 is None or s2 is None:
                    continue
                if self._predictor.will_collide(s1, s2, self._horizon):
                    rospy.logwarn(
                        f"Collision predicted between {ns_list[i]} and {ns_list[j]}! Stopping both."
                    )
                    self._stop_pubs[ns_list[i]].publish(Empty())
                    self._stop_pubs[ns_list[j]].publish(Empty())
