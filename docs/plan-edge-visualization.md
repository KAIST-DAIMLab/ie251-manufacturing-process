# Edge Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Render the graph's edges as thin gray bars in Gazebo, alongside the existing green node markers, driven by `config/graph.yaml`.

**Architecture:** Extend the existing `spawn_simulation_markers` one-shot script. Add a pure helper that, given two `(x, y)` endpoints, returns the SDF box's midpoint, length, and yaw. Spawn one static SDF box per edge via `/gazebo/spawn_sdf_model`, sized to the edge length, oriented along the edge.

**Tech Stack:** ROS Noetic (rospy), Gazebo classic, PyYAML, `gazebo_msgs/SpawnModel` service.

---

## Context

`config/graph.yaml` defines `nodes:` (id, x, y) and `edges:` (from, to). Today only nodes are visualized: `scripts/spawn_simulation_markers` reads the `nodes` block and spawns a green disc (`cylinder`, radius 0.1, height 0.001) per node via `/gazebo/spawn_sdf_model`. Edges are silently absent from the Gazebo view, so the graph topology is not visible to a human running the simulation.

The fix mirrors the node-marker pattern: read the `edges` block, build a thin static SDF box per edge, spawn it. Pose math is a few lines (midpoint, Euclidean length, yaw via `atan2`), worth isolating as a pure helper so it can be unit-tested without rospy.

## Design choices (decided with the user)

- **Render target:** Gazebo SDF (consistent with current node markers, visible in Gazebo GUI).
- **Script layout:** extend `spawn_simulation_markers` rather than create a new script. The script's responsibility broadens slightly: it spawns all graph markers (nodes and edges).
- **Style:** thin gray box, length = edge distance, width 0.02 m, height 0.001 m, RGBA `0.5 0.5 0.5 1`.
- **Pure helper location:** `src/pathfinder/src/pathfinder/utils/physics.py` (already houses `yaw_from_quaternion`; keep math helpers together).

## File-by-file change summary

| File | Change |
|------|--------|
| `src/pathfinder/src/pathfinder/utils/physics.py` | Add `segment_pose(from_xy, to_xy) -> (midpoint, length, yaw)` |
| `src/pathfinder/test/test_physics_utils.py` | Add tests for `segment_pose` |
| `src/pathfinder/scripts/spawn_simulation_markers` | Add edge SDF template, edge-spawn loop, use `segment_pose` for pose |
| `src/pathfinder/CMakeLists.txt` | No change (`spawn_simulation_markers` already installed; `test_physics_utils.py` already registered) |

## Tasks

### Task 1: Add `segment_pose` pure helper (TDD)

**Files:**
- Modify: `src/pathfinder/src/pathfinder/utils/physics.py`
- Test: `src/pathfinder/test/test_physics_utils.py`

- [ ] **Step 1: Write the failing tests**

Append to `src/pathfinder/test/test_physics_utils.py` (above the `if __name__ == '__main__':` line):

```python
class SegmentPoseTest(unittest.TestCase):
    """Pure-math tests for the edge segment pose helper."""

    def test_horizontal_segment(self):
        from pathfinder.utils.physics import segment_pose

        midpoint, length, yaw = segment_pose((0.0, 0.0), (2.0, 0.0))

        self.assertAlmostEqual(midpoint[0], 1.0)
        self.assertAlmostEqual(midpoint[1], 0.0)
        self.assertAlmostEqual(length, 2.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_vertical_segment(self):
        from pathfinder.utils.physics import segment_pose

        midpoint, length, yaw = segment_pose((1.0, 1.0), (1.0, 4.0))

        self.assertAlmostEqual(midpoint[0], 1.0)
        self.assertAlmostEqual(midpoint[1], 2.5)
        self.assertAlmostEqual(length, 3.0)
        self.assertAlmostEqual(yaw, math.pi / 2.0)

    def test_diagonal_segment(self):
        from pathfinder.utils.physics import segment_pose

        midpoint, length, yaw = segment_pose((0.0, 0.0), (1.0, 1.0))

        self.assertAlmostEqual(midpoint[0], 0.5)
        self.assertAlmostEqual(midpoint[1], 0.5)
        self.assertAlmostEqual(length, math.sqrt(2.0))
        self.assertAlmostEqual(yaw, math.pi / 4.0)

    def test_reversed_endpoints_flip_yaw_by_pi(self):
        from pathfinder.utils.physics import segment_pose

        _, _, forward_yaw = segment_pose((0.0, 0.0), (1.0, 1.0))
        _, _, reverse_yaw = segment_pose((1.0, 1.0), (0.0, 0.0))

        self.assertAlmostEqual(abs(forward_yaw - reverse_yaw), math.pi)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 src/pathfinder/test/test_physics_utils.py`

Expected: 4 failures with `ImportError: cannot import name 'segment_pose'` (or `AttributeError`).

- [ ] **Step 3: Implement `segment_pose`**

Append to `src/pathfinder/src/pathfinder/utils/physics.py`:

```python
def segment_pose(
    from_xy: tuple[float, float],
    to_xy: tuple[float, float],
) -> tuple[tuple[float, float], float, float]:
    """Return (midpoint, length, yaw) for a planar segment between two points."""
    from_x, from_y = from_xy
    to_x, to_y = to_xy
    delta_x = to_x - from_x
    delta_y = to_y - from_y
    midpoint = ((from_x + to_x) / 2.0, (from_y + to_y) / 2.0)
    length = math.hypot(delta_x, delta_y)
    yaw = math.atan2(delta_y, delta_x)
    return midpoint, length, yaw
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 src/pathfinder/test/test_physics_utils.py`

Expected: `Ran 5 tests in 0.00Xs`, `OK`.

- [ ] **Step 5: Commit**

```bash
git add src/pathfinder/src/pathfinder/utils/physics.py src/pathfinder/test/test_physics_utils.py
git commit -m "feat(utils): add segment_pose helper for edge marker placement"
```

---

### Task 2: Extend `spawn_simulation_markers` to spawn edge boxes

**Files:**
- Modify: `src/pathfinder/scripts/spawn_simulation_markers`

- [ ] **Step 1: Replace the script with the extended version**

Overwrite `src/pathfinder/scripts/spawn_simulation_markers` with:

```python
#!/usr/bin/env python3
"""One-shot node: reads graph.yaml and spawns a marker per node and per edge in Gazebo."""
from __future__ import annotations
import math
import yaml

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion

from pathfinder.utils.physics import segment_pose


_NODE_MARKER_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="node_{node_id}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.001</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


_EDGE_MARKER_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="edge_{from_id}_{to_id}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>{length} 0.02 0.001</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    """Build a quaternion for a pure yaw rotation around the z-axis."""
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)
    return quaternion


def _spawn_node_marker(spawn_service, node: dict) -> None:
    """Spawn one green disc at the node's (x, y)."""
    node_id = node['id']
    x = float(node['x'])
    y = float(node['y'])

    sdf_string = _NODE_MARKER_SDF_TEMPLATE.format(node_id=node_id)

    pose = Pose(
        position=Point(x=x, y=y, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )

    request = SpawnModelRequest()
    request.model_name = f'node_{node_id}'
    request.model_xml = sdf_string
    request.robot_namespace = ''
    request.initial_pose = pose
    request.reference_frame = 'world'
    response = spawn_service(request)
    if not response.success:
        rospy.logerr(f"Failed to spawn marker for node {node_id}: {response.status_message}")
        raise RuntimeError(f"Marker spawn failed for node {node_id}: {response.status_message}")

    rospy.loginfo(f"Spawned marker node_{node_id} at (x={x}, y={y})")


def _spawn_edge_marker(
    spawn_service,
    edge: dict,
    node_map: dict[int, tuple[float, float]],
) -> None:
    """Spawn one gray bar between the edge's two endpoint nodes."""
    from_id = edge['from']
    to_id = edge['to']
    if from_id not in node_map:
        raise KeyError(f"edge references unknown 'from' node {from_id}")
    if to_id not in node_map:
        raise KeyError(f"edge references unknown 'to' node {to_id}")

    midpoint, length, yaw = segment_pose(node_map[from_id], node_map[to_id])

    sdf_string = _EDGE_MARKER_SDF_TEMPLATE.format(
        from_id=from_id,
        to_id=to_id,
        length=length,
    )

    pose = Pose(
        position=Point(x=midpoint[0], y=midpoint[1], z=0.0),
        orientation=_quaternion_from_yaw(yaw),
    )

    request = SpawnModelRequest()
    request.model_name = f'edge_{from_id}_{to_id}'
    request.model_xml = sdf_string
    request.robot_namespace = ''
    request.initial_pose = pose
    request.reference_frame = 'world'
    response = spawn_service(request)
    if not response.success:
        rospy.logerr(
            f"Failed to spawn marker for edge {from_id}->{to_id}: {response.status_message}"
        )
        raise RuntimeError(
            f"Marker spawn failed for edge {from_id}->{to_id}: {response.status_message}"
        )

    rospy.loginfo(
        f"Spawned marker edge_{from_id}_{to_id} at midpoint=({midpoint[0]:.3f}, "
        f"{midpoint[1]:.3f}), length={length:.3f}, yaw={yaw:.3f}"
    )


def main():
    rospy.init_node('simulation_marker_spawner')

    graph_config = rospy.get_param('~graph_config')

    with open(graph_config, 'r') as graph_file:
        graph_data = yaml.safe_load(graph_file)

    nodes = graph_data['nodes']
    edges = graph_data.get('edges', [])
    node_map = {n['id']: (float(n['x']), float(n['y'])) for n in nodes}

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    for node in nodes:
        _spawn_node_marker(spawn_service, node)

    for edge in edges:
        _spawn_edge_marker(spawn_service, edge, node_map)

    rospy.loginfo(
        f"All {len(nodes)} node markers and {len(edges)} edge markers spawned."
    )


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Verify the script is still executable**

Run: `ls -l src/pathfinder/scripts/spawn_simulation_markers`

Expected: starts with `-rwx` (executable bit preserved). If not, run `chmod +x src/pathfinder/scripts/spawn_simulation_markers`.

- [ ] **Step 3: Confirm imports resolve from the source tree**

Run: `python3 -c "import sys; sys.path.insert(0, 'src/pathfinder/src'); from pathfinder.utils.physics import segment_pose; print(segment_pose((0.0, 0.0), (1.0, 0.0)))"`

Expected: `((0.5, 0.0), 1.0, 0.0)`.

- [ ] **Step 4: Run the existing test suite to confirm nothing broke**

Run: `python3 src/pathfinder/test/test_physics_utils.py && python3 src/pathfinder/test/test_motion.py && python3 src/pathfinder/test/test_path_server.py`

Expected: all `OK`.

- [ ] **Step 5: Commit**

```bash
git add src/pathfinder/scripts/spawn_simulation_markers
git commit -m "feat(viz): spawn gray edge markers from graph.yaml"
```

---

### Task 3: Verify edge markers render correctly in simulation

**Files:** none modified.

This task is a manual / smoke verification because the script is the ROS adapter layer, which the project policy excludes from rospy-mock unit tests (see `feedback_node_testing_policy.md`).

- [ ] **Step 1: Build (no-op for pure-Python edits, but confirms the workspace still builds)**

Run: `catkin_make --only-pkg-with-deps pathfinder && source devel/setup.zsh`

Expected: build succeeds.

- [ ] **Step 2: Launch the simulation**

Run: `roslaunch pathfinder simulation.launch`

Expected log lines from `simulation_marker_spawner` (one per node, one per edge):
```
[INFO] ... Spawned marker node_1 at (x=0.0, y=0.0)
...
[INFO] ... Spawned marker edge_1_2 at midpoint=(0.600, 0.000), length=1.200, yaw=0.000
[INFO] ... Spawned marker edge_2_3 at midpoint=(1.750, 0.000), length=1.100, yaw=0.000
[INFO] ... Spawned marker edge_1_4 at midpoint=(0.000, 0.650), length=1.300, yaw=1.571
...
[INFO] ... All 9 node markers and 13 edge markers spawned.
```

- [ ] **Step 3: Visual check in Gazebo**

In the Gazebo GUI, confirm:
1. 9 green discs at the node coordinates (unchanged from before).
2. 13 thin gray bars connecting nodes per the `edges:` list in `config/graph.yaml`. Each bar's midpoint, length, and orientation match the line between its two endpoints.
3. The Models pane (left side) lists `edge_1_2`, `edge_2_3`, `edge_1_4`, `edge_2_5`, `edge_3_4`, `edge_4_5`, `edge_3_6`, `edge_5_6`, `edge_4_7`, `edge_5_8`, `edge_7_8`, `edge_8_9`, `edge_6_9`.

Cancel with Ctrl-C once verified.

- [ ] **Step 4: Capture a screenshot for the PR (optional)**

If the user is preparing a PR, save a Gazebo screenshot to `.images/` so the diff has a visual.

- [ ] **Step 5: No code commit needed for this task**

Verification only.

---

## Self-review notes

- Spec coverage: edges-from-graph.yaml are read (`graph_data.get('edges', [])`); each is rendered (`_spawn_edge_marker`); style matches the agreed gray thin-box; helper has tests.
- Type consistency: helper named `segment_pose` is used identically in test and script. SDF template placeholder names (`from_id`, `to_id`, `length`) match the `.format(...)` kwargs in `_spawn_edge_marker`.
- Edge cases: `edges:` block missing → `.get('edges', [])` returns `[]` and the loop is a no-op (matches the case where someone authors a node-only graph). Endpoint id missing from `node_map` → explicit `KeyError` with message identifying the offending edge.
- Backward compatibility: node-marker behavior is preserved verbatim (same SDF, same logs). The only addition is the edge loop after it.
