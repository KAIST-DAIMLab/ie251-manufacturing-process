# Plan: replace manual SE(2) pose composition with tf2

## Context

The robot executor today reconstructs the world-frame pose by element-wise adding `origin` (start_station + initial yaw) to `odom_pose` inside `_on_odom` ([turtlebot_node.py:104-110](../src/pathfinder/src/pathfinder/ros/turtlebot_node.py#L104-L110)). That addition is only correct when `origin.theta == 0`. With a non-zero initial yaw (e.g. `yaw: 90` in `robots.yaml`) the position offset is added in the wrong frame: when the robot drives forward in body-`+x`, odom reports `(1, 0)` but world-truth motion is rotated by `origin.theta`, so `pose.x` drifts in the wrong direction and the motion controller enters a small-circle limit cycle.

A short-term SE(2) fix was attempted directly in `_on_odom` but reverted; we want the proper solution: stop hand-rolling the transform and let `tf2` chain it.

## Discovery findings

Running `rostopic echo /tf` against the running sim revealed two facts that shape the design:

1. **Gazebo's diff_drive plugin publishes raw frame names**: `frame_id: "odom"`, `child_frame_id: "base_footprint"` — *not* the prefixed `tb3_01/odom` / `tb3_01/base_footprint`. Both robots emit to the same shared frames; the broadcast collides silently on every tick. `tf_prefix` set on `robot_state_publisher` only affects what *robot_state_publisher itself* emits, not the diff_drive plugin.
2. **`robot_state_publisher` emits a *disconnected* tree** rooted at `tb3_01/base_footprint` (no parent in `/tf`). So today there is no path from any "world" frame to any robot frame; `tf2` lookups would fail.

The system has been usable up to now only because pose is computed from the per-robot `/odom` *topic* (which is namespaced via the `tb3_01/sim/` prefix), bypassing `/tf` entirely. As soon as we switch to `tf2`, we have to give every robot a properly prefixed, properly connected TF tree.

The fix is to **ignore Gazebo's raw `odom → base_footprint`** (leave it noisy on `/tf`, no consumer of ours reads it) and **stand up our own prefixed tree on top of it** by re-publishing the odom topic as TF with prefixed frame names and anchoring it to a `world` root.

## Target frame tree

```
world  (root, no parent)
 ├── tb3_01/odom                       ← static, from start_station + initial yaw
 │    └── tb3_01/base_footprint        ← dynamic, from /tb3_01/sim/odom topic
 │         └── tb3_01/base_link → ...  ← already exists, from robot_state_publisher
 └── tb3_05/odom
      └── tb3_05/base_footprint
           └── tb3_05/base_link → ...
```

The leaf chain (`tb3_01/base_footprint → tb3_01/base_link → ...`) is unchanged — `robot_state_publisher` already publishes it with `tf_prefix=tb3_01`. We only have to add the two top edges per robot.

## Changes

### 1. New: `WorldBridge` ROS adapter + `world_bridge` script

A single process that handles all robots: publishes the static `world → <robot_id>/odom` for each robot at startup, then forwards every `/odom` message into `/tf` with prefixed frame names. One responsibility — "make the world frame reachable from each robot's base via TF".

**New file:** `src/pathfinder/src/pathfinder/ros/world_bridge.py`

```python
class WorldBridge:
    """Republish per-robot /odom as TF (<robot>/odom → <robot>/base_footprint)
    and broadcast a static world → <robot>/odom anchor from start_station + yaw."""

    def __init__(
        self,
        robots: list[Robot],
        graph: Graph,
        static_broadcaster: StaticTransformBroadcasterLike,
        dynamic_broadcaster: TransformBroadcasterLike,
    ) -> None:
        ...

    def start(self) -> None:
        """Publish the static anchors and register /odom subscribers for each robot."""

    def _on_odom(self, robot_id: str, msg: Odometry) -> None:
        """Forward odom pose as TF: <robot_id>/odom → <robot_id>/base_footprint."""
```

`StaticTransformBroadcasterLike` / `TransformBroadcasterLike` are structural protocols (`def sendTransform(self, t) -> None`) so the unit test can pass fakes without importing `tf2_ros`. The constructor takes them as collaborators (DI), per the architecture rule.

**New file:** `src/pathfinder/scripts/world_bridge`

```python
#!/usr/bin/env python3
from typing import cast
import rospy
import tf2_ros
from pathfinder.ros.world_bridge import WorldBridge
from pathfinder.world.graph import Graph
from pathfinder.world.robot import Robot


def main():
    rospy.init_node('world_bridge')
    graph_file = cast(str, rospy.get_param('~graph_file'))
    sim = cast(bool, rospy.get_param('~sim', False))
    robots = [Robot.from_dict(r, sim=sim) for r in cast(list, rospy.get_param('~robots'))]

    bridge = WorldBridge(
        robots=robots,
        graph=Graph.load_from_yaml(graph_file),
        static_broadcaster=tf2_ros.StaticTransformBroadcaster(),
        dynamic_broadcaster=tf2_ros.TransformBroadcaster(),
    )
    bridge.start()
    rospy.spin()


if __name__ == '__main__':
    main()
```

The script reads the same `~robots` rosparam as `path_server` / `robot_executors` so all three nodes share the YAML loaded by `<rosparam file="..." command="load"/>` in the launch file.

### 2. Extract `quaternion_from_yaw` into `utils/physics.py`

It already exists as a local helper in two scripts and we need it a third time:

| Existing | Action |
|---|---|
| [scripts/simulation_robots:38-45](../src/pathfinder/scripts/simulation_robots#L38-L45) (`_quaternion_from_yaw`) | Replace with `from pathfinder.utils.physics import quaternion_from_yaw`. |
| [scripts/simulation_markers:59-66](../src/pathfinder/scripts/simulation_markers#L59-L66) (`_quaternion_from_yaw`) | Same. |
| New use in `WorldBridge` | Imports the shared one. |

Add to `utils/physics.py`:

```python
def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Build a quaternion for a pure yaw rotation around the z-axis."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q
```

Add a test in `test/test_physics_utils.py`: round-trip `yaw_from_quaternion(quaternion_from_yaw(θ)) ≈ θ` for a handful of θ values (0, π/2, -π/2, π, π/4).

### 3. Refactor `TurtleBotNode` to read pose from `tf2`

**File:** `src/pathfinder/src/pathfinder/ros/turtlebot_node.py`

- Add `tf2_ros.Buffer()` + `tf2_ros.TransformListener(buffer)` in `__init__`. Inject `tf_buffer` as a constructor argument so tests can pass a fake (same DI pattern as everywhere else).
- Replace the pose computation in `_on_odom` with a `rospy.Timer(rospy.Duration(1.0 / motion_rate_hz), self._on_pose_tick)`.
- `_on_pose_tick` does:
  ```python
  try:
      t = self._tf_buffer.lookup_transform(
          'world',
          f'{self._robot_id}/base_footprint',
          rospy.Time(0),
          rospy.Duration(0.0),  # non-blocking; skip tick if not ready
      )
  except (tf2_ros.LookupException,
          tf2_ros.ConnectivityException,
          tf2_ros.ExtrapolationException):
      return
  self._state.pose.x = t.transform.translation.x
  self._state.pose.y = t.transform.translation.y
  self._state.pose.theta = yaw_from_quaternion(t.transform.rotation)
  self._pose_publisher.publish(self._state.get_pose())
  ```
- `_on_odom` keeps only velocity:
  ```python
  def _on_odom(self, msg: Odometry) -> None:
      self._state.velocity = msg.twist.twist
  ```
- Drop the `origin` constructor parameter and the `origin or Pose2D()` initialization line.

The `/tb3_X/sim/pose` topic still exists and still publishes `Pose2D` for `path_server` and the web UI — no change for downstream consumers.

### 4. Delete dead code

| File | Change |
|---|---|
| [scripts/robot:22-30](../src/pathfinder/scripts/robot#L22-L30) | Delete the `if not sim: origin = …` block entirely. Remove `import math` (now unused) and `from geometry_msgs.msg import Pose2D`. The `sim` param is still passed to `Robot.from_dict` for the namespace (`f"{robot_id}/sim"`), so keep `sim = rospy.get_param('~sim', False)`. |
| [src/pathfinder/robot/robot_state.py:15](../src/pathfinder/src/pathfinder/robot/robot_state.py#L15) | Drop the `origin: Pose2D` field. Nothing else reads it once `_on_odom` stops writing through it. |
| [src/pathfinder/src/pathfinder/ros/turtlebot_node.py:30,45](../src/pathfinder/src/pathfinder/ros/turtlebot_node.py#L30) | Drop `origin: Pose2D | None = None` constructor param and the `origin=origin or Pose2D()` call. |

### 5. Launch wiring

**File:** `src/pathfinder/launch/robots.launch`

Add the bridge after `path_server` and before `robot_executors`:

```xml
<node name="world_bridge" pkg="pathfinder" type="world_bridge" output="screen">
  <param name="graph_file" value="$(find pathfinder)/config/graph.yaml"/>
  <param name="sim" value="$(arg sim)"/>
  <rosparam file="$(arg robots_config)" command="load"/>
</node>
```

Same node is launched in both sim and non-sim modes — the only role-dependent piece is `Robot.from_dict(..., sim=sim)` adjusting the namespace for the `/odom` topic name (`/tb3_01/sim/odom` vs `/tb3_01/odom`).

`simulation.launch` already includes `robots.launch`, so no change needed there.

### 6. Register the script in `CMakeLists.txt`

Add `scripts/world_bridge` to the `install(PROGRAMS …)` block:

```cmake
install(PROGRAMS
  scripts/path_server
  scripts/robot
  scripts/world_bridge      # <-- new
  scripts/simulation_robots
  ...
)
```

## Tests

### `test/test_world_bridge.py` (new)

Stubs follow the established pattern (`_install_ros_stubs` ahead of import). Cases:

1. **Static anchors**: construct a `WorldBridge` with two robots whose start_stations resolve to known graph coords and known `yaw` (radians). Call `start()` with a fake `static_broadcaster`. Assert it received exactly two `TransformStamped` items with:
   - `header.frame_id == 'world'`
   - `child_frame_id == f'{robot.id}/odom'`
   - `transform.translation.{x,y}` match start_station coords, `z == 0`
   - `transform.rotation` round-trips via `yaw_from_quaternion` back to `robot.yaw` within `1e-9`.
2. **Dynamic forward**: feed a fake `Odometry` with `pose.position=(0.3, -0.2, 0)` and `pose.orientation=quaternion_from_yaw(0.4)` to `_on_odom('tb3_01', msg)`. Assert the dynamic broadcaster received a `TransformStamped` with:
   - `frame_id == 'tb3_01/odom'`
   - `child_frame_id == 'tb3_01/base_footprint'`
   - translation and rotation copied verbatim from the message.
3. **Unknown robot**: `_on_odom('tb3_99', msg)` is a no-op (no exception, no publish). Defensive — keeps a stray topic remap from crashing the bridge.

Add to `CMakeLists.txt`:
```cmake
catkin_add_nosetests(test/test_world_bridge.py)
```

### `test/test_physics_utils.py` (extend)

Round-trip test for `quaternion_from_yaw` ↔ `yaw_from_quaternion` over `[0, π/4, π/2, π-1e-6, -π/2, -π/4]`. Tolerance `1e-9`.

### `test/test_turtlebot.py` and any existing `test_turtlebot_node` (adjust)

- Remove assertions on `state.origin` (field is gone).
- Remove the `origin=...` argument from any constructor call sites.

### Manual smoke test

After bringing the bridge up alongside `robot_executors`:

```bash
rosrun tf2_tools view_frames.py   # writes frames.pdf — confirm tree matches the diagram
rosrun tf tf_echo world tb3_01/base_footprint   # should print a live transform
rostopic echo -n 1 /tb3_01/sim/pose   # should match the tf_echo output
```

Then issue a `MoveToNode` to a neighbour node and confirm the robot drives straight to it, no circle.

## Rollout order

The intent is a **working system at every step** with a clear rollback point:

1. **Add the bridge in parallel.** Land the `world_bridge` script, register it, launch it. The old `_on_odom` math still runs in `TurtleBotNode`; the new TF tree is published but no one reads it yet. Verify with `tf2_echo` and `frames.pdf` that the tree is correct.
2. **Switch `TurtleBotNode` to read tf2.** Replace `_on_odom`'s pose math with the timer-based lookup; keep velocity from `/odom`. Verify navigation works for `yaw: 0` (regression check) and `yaw: 90` (the bug).
3. **Delete dead code.** Drop `origin` from `TurtleBotNode`, `RobotState`, and `scripts/robot`. Run all tests.
4. **Extract `quaternion_from_yaw`.** Replace the two local copies. (Could also be step 0 — it's independent.)

Each step is its own commit / PR.

## Risks and open questions

- **First few ticks have no transform.** The static anchor lands on `/tf_static` before `TurtleBotNode` starts polling, and the dynamic edge needs the first `/odom` message before lookups succeed. The `try/except` on `LookupException` covers this — `_on_pose_tick` silently skips until ready. `fleet_service.MoveToNode` already short-circuits with `"no pose for {robot_id}"` while `robot.pose is None`, so the system fails gracefully.
- **Polling rate vs publish rate.** Timer at `motion_rate_hz` (5 Hz default) is slower than `/odom` (typically 30 Hz from Gazebo). The motion controller already runs at 5 Hz, so this matches. We trade a tiny amount of timing precision for not running pose math at 30 Hz unnecessarily.
- **Cross-namespace TF.** `tf2_ros.Buffer`/`TransformListener` subscribe to `/tf` and `/tf_static` (global, by convention). All robots and the bridge share those topics — exactly what we want.
- **Suppressing the raw `odom → base_footprint` from Gazebo.** Out of scope. The duplicate publishers cause noisy TF warnings but no consumer of ours reads those raw frames. If it becomes intolerable, the fix is to xacro-arg the diff_drive plugin's `<odometryFrame>` and `<robotBaseFrame>` to the prefixed names — but that means modifying the vendored `turtlebot3_description` xacro pipeline. Defer.
- **Real robots (non-sim).** A real TurtleBot publishes its own `odom` and TF tree. The bridge's `<robot_id>/odom → <robot_id>/base_footprint` edge will conflict with whatever the real robot emits *if* its TF also uses prefixed frame names. We'll know when we try; the fallback is to disable the bridge's dynamic edge for non-sim and rely on the real robot's TF + an explicit `world → <robot_id>/odom` static. Not blocking — sim is the immediate target.
