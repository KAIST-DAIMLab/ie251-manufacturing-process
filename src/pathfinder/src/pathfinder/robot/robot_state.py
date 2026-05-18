from __future__ import annotations

from dataclasses import dataclass

from pathfinder.robot.robot_mode import RobotMode


@dataclass
class RobotState:
    """Status container for one robot."""
    id: str
    status: RobotMode = RobotMode.IDLE
