from __future__ import annotations
from itertools import combinations
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from pathfinding_system.robot.robot_state import RobotState
from pathfinding_system.safety.linear_predictor import LinearPredictor


class CollisionMonitor:
    def __init__(
        self,
        predictor: LinearPredictor,
        robot_namespaces: list[str],
        horizon: float,
        check_rate_hz: float,
        robot_odom_topics: dict[str, str] | None = None,
    ) -> None:
        self._predictor = predictor
        self._namespaces = robot_namespaces
        self._robot_odom_topics = robot_odom_topics or {
            ns: f'/{ns}/odom' for ns in robot_namespaces
        }
        self._horizon = horizon
        self._check_rate_hz = check_rate_hz
        self._states: dict[str, object] = {}
        self._stop_pubs: dict[str, rospy.Publisher] = {}

    def start(self) -> None:
        for ns in self._namespaces:
            rospy.Subscriber(
                self._robot_odom_topics[ns],
                Odometry,
                lambda msg, n=ns: self.update_odom(n, msg),
            )
            self._stop_pubs[ns] = rospy.Publisher(
                f'/{ns}/emergency_stop', Empty, queue_size=1
            )
        rospy.Timer(rospy.Duration(1.0 / self._check_rate_hz), self._tick)
        rospy.loginfo("CollisionMonitor started.")

    def update_state(self, ns: str, state) -> None:
        self._states[ns] = state

    def update_odom(self, ns: str, msg: Odometry) -> None:
        self.update_state(ns, RobotState.from_odometry(ns, msg))

    def _tick(self, event) -> None:
        for ns_a, ns_b in combinations(self._namespaces, 2):
            s_a, s_b = self._states.get(ns_a), self._states.get(ns_b)
            if s_a is None or s_b is None:
                continue
            if self._predictor.will_collide(s_a, s_b, self._horizon):
                rospy.logwarn(
                    f"Collision predicted between {ns_a} and {ns_b}! Stopping both."
                )
                self._stop_pubs[ns_a].publish(Empty())
                self._stop_pubs[ns_b].publish(Empty())
