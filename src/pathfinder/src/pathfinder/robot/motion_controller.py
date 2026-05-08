from __future__ import annotations

import rospy

from pathfinder.robot.motion_engine import MotionEngine
from pathfinder.world.node import Node


class MotionController:
    """Control loop that drives MotionEngine until arrival, cancel, or shutdown."""

    def __init__(self, engine: MotionEngine, rate_hz: float = 5.0) -> None:
        self._engine = engine
        self._rate_hz = rate_hz
        self._cancel = False
        self._pause = False

    def drive_to(self, target: Node) -> bool:
        """Loop engine.drive_towards until arrival or cancel; honors pause."""
        self._cancel = False
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown() and not self._cancel:
            while self._pause and not rospy.is_shutdown() and not self._cancel:
                self._engine.stop()
                rate.sleep()
            if self._engine.drive_towards(target):
                return True
            rate.sleep()
        return False

    def turn_to(self, heading: float) -> bool:
        """Loop engine.turn_towards until aligned or cancel; does not honor pause."""
        self._cancel = False
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown() and not self._cancel:
            if self._engine.turn_towards(heading):
                return True
            rate.sleep()
        return False

    def stop(self) -> None:
        """Cancel any in-flight drive_to or turn_to and halt the engine."""
        self._cancel = True
        self._engine.stop()

    def set_pause(self, paused: bool) -> None:
        """Pause or resume drive_to; has no effect on turn_to."""
        self._pause = paused
