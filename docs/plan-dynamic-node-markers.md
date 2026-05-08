# Dynamic Node Markers from graph.yaml — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive the green cylinder node markers in Gazebo from `config/graph.yaml` instead of hardcoding them in the world file.

**Architecture:** Add a new one-shot ROS Python node `spawn_simulation_markers` that mirrors the pattern of the existing `spawn_simulation_robots`: read `graph.yaml`, build an SDF cylinder per node, call `/gazebo/spawn_sdf_model`. Strip the hardcoded `<model name="node_X">` blocks from `pathfinding_table.world`.

**Tech Stack:** ROS Noetic (rospy), Gazebo classic, PyYAML, `gazebo_msgs/SpawnModel` service.

---

## Context

`worlds/pathfinding_table.world` (lines 34–128) hardcodes 6 green cylinder markers at coordinates (1.0,0.75), (1.0,2.25), (3.0,0.75), (3.0,2.25), (5.0,0.75), (5.0,2.25). The active `config/graph.yaml` defines 9 nodes at completely different coordinates: (0.0,0.0), (1.2,0.0), (2.3,0.0), (0.0,1.3), (1.2,1.3), (2.3,1.3), (0.0,3.3), (1.2,3.3), (2.3,3.3). When the simulation runs, robots now spawn at the correct graph nodes (after the recent `spawn_simulation_robots` change) but the green markers sit at the stale, mismatched positions: visible in `.images/2605081216.jpg`.

Renumbering or moving graph nodes today silently desyncs the visual layer. We just removed the same class of bug for robot spawning; this plan finishes the job for markers.

## Design choices (decided with the user)

- **Where:** new dedicated `spawn_simulation_markers` script. Single responsibility, easy to disable independently. Mirrors the `spawn_simulation_robots` pattern.
- **Labels:** none in the 3D view. Spawned models are named `node_1` … `node_9` so they show in Gazebo's left-pane Models tree on hover/select.
- **Style:** unchanged from the original (green, radius 0.1 m, height 0.001 m, RGBA 0 1 0 1).

## File-by-file change summary

| File | Change |
|------|--------|
| `src/pathfinder/scripts/spawn_simulation_markers` | New file: SDF spawner driven by graph.yaml |
| `src/pathfinder/worlds/pathfinding_table.world` | Remove 6 hardcoded `<model name="node_X">` blocks |
| `src/pathfinder/launch/simulation.launch` | Add `<node>` to launch the new spawner |
| `src/pathfinder/CMakeLists.txt` | Add new script to `install(PROGRAMS ...)` |

## Tasks

### Task 1: Write spawn_simulation_markers script

**Files:**
- Create: `src/pathfinder/scripts/spawn_simulation_markers`

- [ ] **Step 1: Create the script**

```python
#!/usr/bin/env python3
"""One-shot node: reads graph.yaml and spawns a green disc per node in Gazebo."""
from __future__ import annotations
import yaml

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion


_MARKER_SDF_TEMPLATE = """<?xml version="1.0" ?>
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


def main():
    rospy.init_node('simulation_marker_spawner')

    graph_config = rospy.get_param('~graph_config')

    with open(graph_config, 'r') as graph_file:
        nodes = yaml.safe_load(graph_file)['nodes']

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    for node in nodes:
        node_id = node['id']
        x = float(node['x'])
        y = float(node['y'])

        sdf_string = _MARKER_SDF_TEMPLATE.format(node_id=node_id)

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

    rospy.loginfo(f"All {len(nodes)} node markers spawned.")


if __name__ == '__main__':
    main()
```

Notes:
- No `rospy.spin()` at the end. Static SDF models persist in Gazebo after the spawning process exits; unlike `spawn_simulation_robots`, this script does not start any roslaunch child processes that need the parent alive.
- Variable naming follows the project's "no shortened names" rule: `sdf_string`, not `sdf`; `graph_file`, not `f`.

- [ ] **Step 2: Make it executable**

Run: `chmod +x src/pathfinder/scripts/spawn_simulation_markers`

Expected: silent success. Verify with `ls -l` showing the `x` bit set.

### Task 2: Wire script into CMakeLists.txt

**Files:**
- Modify: `src/pathfinder/CMakeLists.txt` (lines 41–48)

- [ ] **Step 1: Add the script to install(PROGRAMS …)**

Replace the existing `install(PROGRAMS …)` block with:

```cmake
install(PROGRAMS
  scripts/path_server_node
  scripts/robot_executors_node
  scripts/spawn_simulation_robots
  scripts/spawn_simulation_markers
  scripts/client
  scripts/run_simulation
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### Task 3: Strip hardcoded markers from the world file

**Files:**
- Modify: `src/pathfinder/worlds/pathfinding_table.world`

- [ ] **Step 1: Delete the 6 `<model name="node_X">` blocks**

Open `src/pathfinder/worlds/pathfinding_table.world`. Find each `<model name="node_0">` through `<model name="node_5">` block (lines 34–128) and delete every line of each block, including its surrounding whitespace. Leave the rest of the world intact: table, lighting, ground plane, physics. Do not touch any other model.

After the edit, `grep -n 'name="node_' src/pathfinder/worlds/pathfinding_table.world` must return zero matches.

### Task 4: Add spawner node to simulation.launch

**Files:**
- Modify: `src/pathfinder/launch/simulation.launch`

- [ ] **Step 1: Add the marker spawner node after the robot spawner**

Insert this block immediately after the existing `<node name="simulation_spawner" .../>` `</node>` close, and before the `<include if="$(arg start_system)" .../>` block:

```xml
  <node name="simulation_marker_spawner" pkg="pathfinder" type="spawn_simulation_markers" output="screen">
    <param name="graph_config" value="$(arg graph_config)"/>
  </node>
```

The existing `<arg name="graph_config" default="$(find pathfinder)/config/graph.yaml"/>` declared at the top of the launch file is reused; no new arg is needed.

### Task 5: Build, run, verify

- [ ] **Step 1: Rebuild the catkin workspace**

Inside the noetic container at the workspace root:
```bash
catkin_make --only-pkg-with-deps pathfinder && source devel/setup.zsh
```

Expected: build succeeds, no errors mentioning `spawn_simulation_markers`.

- [ ] **Step 2: Launch the simulation**

```bash
roslaunch pathfinder simulation.launch
```

Expected console: `[simulation_marker_spawner] Spawned marker node_1 at (x=0.0, y=0.0)` … through `node_9`, then `All 9 node markers spawned.`

- [ ] **Step 3: Visually verify in Gazebo**

In the Gazebo viewport, count green discs: there must be **9**, not 6, and they must align with the 9 graph node coordinates above. No green disc should remain at the old positions (1.0,0.75), (3.0,0.75), (5.0,0.75), (1.0,2.25), (3.0,2.25), (5.0,2.25).

In the Gazebo World pane (left tree) under "Models", confirm entries `node_1` through `node_9` exist alongside `tb3_01` and `tb3_05`.

- [ ] **Step 4: Sanity check — graph edit propagates**

Edit one node's `(x, y)` in `src/pathfinder/config/graph.yaml`. Kill and relaunch `simulation.launch`. Confirm the corresponding green disc moved. Revert the YAML.

- [ ] **Step 5: Functional smoke test**

```bash
rosrun pathfinder client tb3_01 5
```

Expected: tb3_01 navigates between nodes as before. The marker change is purely visual; this only confirms we haven't regressed the rest.

### Task 6: Commit

- [ ] **Step 1: Stage and commit**

```bash
git add \
  src/pathfinder/scripts/spawn_simulation_markers \
  src/pathfinder/worlds/pathfinding_table.world \
  src/pathfinder/launch/simulation.launch \
  src/pathfinder/CMakeLists.txt

git commit -m "feat(sim): spawn node markers dynamically from graph.yaml

Strip the 6 hardcoded green cylinders from pathfinding_table.world
and add spawn_simulation_markers, a one-shot Python node that reads
graph.yaml and calls /gazebo/spawn_sdf_model for each node. The
markers now stay in lockstep with the graph definition."
```

Per the project's commit-after-modification convention, this single commit wraps the whole change. The four files form one logical unit and are not useful split apart (a half-applied state would leave Gazebo with no markers).

## Verification (end-to-end)

1. Build: `catkin_make --only-pkg-with-deps pathfinder && source devel/setup.zsh`.
2. Launch: `roslaunch pathfinder simulation.launch`.
3. Count: 9 green discs in Gazebo, at the 9 `graph.yaml` coordinates, none at the legacy positions.
4. Tree: `node_1` … `node_9` listed under Models in the Gazebo left pane.
5. Edit-replay: change one coordinate in `graph.yaml`, relaunch, confirm the disc moves; revert.
6. Functional smoke: `rosrun pathfinder client tb3_01 5` still drives the robot end to end.

## Out of scope

- 3D text labels per node. Gazebo classic has no native `<text>` SDF primitive, and the user opted for distinct model names (visible in the GUI tree) rather than texture-rendered labels.
- Configurable marker size/color via `params.yaml`. Hardcoded inside `_MARKER_SDF_TEMPLATE` to match the original visual.
- RViz visualization. The simulation remains Gazebo-only.
- Unit tests. This script is in the node-assembly layer per the project's testing policy: it only does ROS init, parameter reads, service calls. No rospy-mock unit tests, consistent with `spawn_simulation_robots`.
