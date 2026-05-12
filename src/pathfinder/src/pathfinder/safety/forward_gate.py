from __future__ import annotations


class ForwardGate:
    """Wraps a cmd_vel publisher and zeros positive linear.x when the forward path is blocked."""

    def __init__(self, publisher) -> None:
        self._publisher = publisher
        self._blocked = False

    def set_blocked(self, blocked: bool) -> None:
        self._blocked = blocked

    def publish(self, twist) -> None:
        if self._blocked and twist.linear.x > 0.0:
            twist.linear.x = 0.0
        self._publisher.publish(twist)
