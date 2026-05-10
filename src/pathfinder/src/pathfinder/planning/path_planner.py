from __future__ import annotations
from typing import Protocol
from pathfinder.world.node import Node


class PathPlanner(Protocol):
    """Plans an ordered list of waypoint Nodes between two graph nodes."""

    def plan(self, start: Node, goal: Node) -> list[Node]: ...
