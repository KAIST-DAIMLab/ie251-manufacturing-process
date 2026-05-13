from __future__ import annotations
from dataclasses import dataclass
from pathfinder.world.node import Node


@dataclass(frozen=True)
class Station:
    id: int
    node: Node
    orientation: float
