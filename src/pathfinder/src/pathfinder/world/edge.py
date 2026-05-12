from __future__ import annotations
import json
from dataclasses import dataclass
from typing import TYPE_CHECKING
from pathfinder.world.node import Node

if TYPE_CHECKING:
    from pathfinder.world.graph import Graph


@dataclass(frozen=True)
class Edge:
    from_node: Node
    to_node: Node

    @property
    def cost(self) -> float:
        return self.from_node.distance_to(self.to_node)

    @classmethod
    def encode_wire(cls, from_id: int | None, to_id: int | None) -> str:
        """Serialize an edge as a JSON payload ([from, to]) or null."""
        if from_id is None or to_id is None:
            return json.dumps(None)
        return json.dumps([from_id, to_id])

    @classmethod
    def decode_wire(cls, payload: str, graph: Graph) -> Edge | None:
        """Resolve a wire payload back to an Edge against the given graph (or None)."""
        parsed = json.loads(payload)
        if parsed is None:
            return None
        return cls(graph.get_node(parsed[0]), graph.get_node(parsed[1]))
