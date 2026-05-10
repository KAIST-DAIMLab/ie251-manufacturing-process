from __future__ import annotations

from pathfinder.world.robot import Robot


class RobotTopics:
    """Derives ROS topic names and action namespaces for a set of robots."""

    def __init__(self, robots: list[Robot], sim: bool) -> None:
        """Compute odom topics and action namespaces from robot ids and sim flag."""
        self.robot_ids: list[str] = [robot.id for robot in robots]
        self.odom_topics: dict[str, str] = {
            robot.id: f"/{robot.id}/sim/odom" if sim else f"/{robot.id}/odom"
            for robot in robots
        }
        self.action_namespaces: dict[str, str] = {
            robot.id: f"{robot.id}/sim" if sim else robot.id
            for robot in robots
        }
