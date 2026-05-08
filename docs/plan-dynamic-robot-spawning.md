# Plan: dynamic robot spawning from graph.yaml

Date: 2026-05-08

## Problem

`simulation.launch` hardcodes two `<group>` blocks for `tb3_01` (spawn at 1.0, 0.75) and `tb3_05` (spawn at 5.0, 2.25). The coordinates are stale: they match neither `graph.yaml` nor `robots.yaml`. Adding a third robot requires copy-pasting another `<group>` block. Renumbering graph nodes silently desyncs spawn poses.

## Goal

Drive simulated robot spawning from `robots.yaml` + `graph.yaml`, so the launch file is independent of how many robots there are or which graph nodes they start at.

## Schema change: `robots.yaml`

Replace `origin: {x, y, theta}` with `start_node: <id>` and optional `yaw: <radians>` (default `0.0`).

```yaml
robots:
  - id: tb3_01
    start_node: 4
  - id: tb3_05
    start_node: 5
```

Single source of truth: spawn coordinates always come from `graph.yaml`.

## New script: `scripts/spawn_simulation_robots`

A Python one-shot node launched by `simulation.launch`. Responsibilities:

1. Read params: `robots_config`, `graph_config`, `model` (default `waffle`).
2. Load both YAML files. Build `{node_id: (x, y)}` lookup from `graph.yaml`.
3. Render the TurtleBot3 xacro once via the `xacro` Python API to a URDF string (same URDF for all robots).
4. For each robot in `robots.yaml`:
   - Look up `(x, y)` from the graph by `start_node`. Build a `geometry_msgs/Pose` with quaternion from `yaw`.
   - Set `/{robot_id}/sim/robot_description` on the parameter server.
   - Launch `robot_state_publisher` in namespace `{robot_id}/sim` via `roslaunch.scriptapi`, with params `tf_prefix={robot_id}`, `publish_frequency=50.0`.
   - Call `/gazebo/spawn_urdf_model` service with the pose.
5. `rospy.spin()` to keep `roslaunch.scriptapi` child processes alive.

Failure modes:
- Missing `start_node` value: log error, raise.
- `start_node` not present in graph: log error listing valid ids, raise.
- xacro render failure: propagate.

## `simulation.launch` change

Remove both hardcoded `<group>` blocks. Add the spawner node and a `graph_config` arg:

```xml
<launch>
  <arg name="model" default="waffle"/>
  <arg name="robots_config" default="$(find pathfinder)/config/robots.yaml"/>
  <arg name="graph_config" default="$(find pathfinder)/config/graph.yaml"/>
  <arg name="start_system" default="true"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find pathfinder)/worlds/pathfinding_table.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <node name="simulation_spawner" pkg="pathfinder" type="spawn_simulation_robots" output="screen">
    <param name="robots_config" value="$(arg robots_config)"/>
    <param name="graph_config" value="$(arg graph_config)"/>
    <param name="model" value="$(arg model)"/>
  </node>

  <include if="$(arg start_system)" file="$(find pathfinder)/launch/robots.launch">
    <arg name="robots_config" value="$(arg robots_config)"/>
    <arg name="sim" value="true"/>
  </include>
</launch>
```

## Consumer change: `robot_executors_node`

Currently in non-sim mode it reads `robot_cfg['origin']`. After the schema change, it derives origin from `start_node` + `yaw`:

```python
if not sim:
    node = graph.get_node(robot_cfg['start_node'])
    origin = Pose2D()
    origin.x = node.x
    origin.y = node.y
    origin.theta = robot_cfg.get('yaw', 0.0)
```

`graph` is already loaded in this script, so no extra plumbing.

`path_server_node` only reads `id` from `robots.yaml`, so it needs no change.

## CMakeLists.txt

Add `scripts/spawn_simulation_robots` to the `install(PROGRAMS …)` list so `rosrun` and `<node type=...>` can find it.

## Out of scope / risks

- `roslaunch.scriptapi` child processes log to file rather than the parent's console. If we want `robot_state_publisher` logs in the simulator console, we'd need to wire stdout/stderr ourselves. Not doing this in v1.
- Tests: this is Node-layer wire-up (per the project testing policy), so no unit tests.
- README and CLAUDE.md mention the spawn pose hardcoding in `gazebo_world.launch` (stale reference); update those alongside the code change.

## File-by-file change summary

| File | Change |
|------|--------|
| `src/pathfinder/config/robots.yaml` | Replace `origin` with `start_node` (+ optional `yaw`) |
| `src/pathfinder/scripts/spawn_simulation_robots` | New file |
| `src/pathfinder/launch/simulation.launch` | Drop `<group>` blocks, add spawner node + `graph_config` arg |
| `src/pathfinder/scripts/robot_executors_node` | Compute `origin` from `start_node` + graph in non-sim mode |
| `src/pathfinder/CMakeLists.txt` | Install the new script |
| `README.md` / `CLAUDE.md` | Update any references to hardcoded spawn poses |

## Commit plan

1. Schema change to `robots.yaml` + `robot_executors_node` consumer update (one commit).
2. New `spawn_simulation_robots` script + CMakeLists install entry (one commit).
3. `simulation.launch` rewrite (one commit).
4. Doc updates (one commit).
