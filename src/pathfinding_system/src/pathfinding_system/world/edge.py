from __future__ import annotations
from dataclasses import dataclass
from pathfinding_system.world.node import Node


@dataclass(frozen=True)
class Edge:
    from_node: Node
    to_node: Node

    @property
    def cost(self) -> float:
        return self.from_node.distance_to(self.to_node)
