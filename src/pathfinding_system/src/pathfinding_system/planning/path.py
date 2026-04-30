from __future__ import annotations
from pathfinding_system.world.node import Node


class Path:
    def __init__(self, waypoints: list[Node]) -> None:
        self._waypoints = waypoints
        self._cursor = 0

    def next(self) -> Node:
        if self._cursor >= len(self._waypoints):
            raise StopIteration
        node = self._waypoints[self._cursor]
        self._cursor += 1
        return node

    def peek(self) -> Node:
        if self._cursor >= len(self._waypoints):
            raise StopIteration
        return self._waypoints[self._cursor]

    def is_complete(self) -> bool:
        return self._cursor >= len(self._waypoints)

    def remaining(self) -> list[Node]:
        return self._waypoints[self._cursor:]

    def length(self) -> int:
        return len(self._waypoints)

    def current_index(self) -> int:
        return self._cursor
