from __future__ import annotations
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Node:
    id: int
    x: float
    y: float
    orientation: float | None = None
    station: int | None = None

    def distance_to(self, other: Node) -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def orientation_rad(self) -> float | None:
        if self.orientation is None:
            return None
        return math.radians(self.orientation)
