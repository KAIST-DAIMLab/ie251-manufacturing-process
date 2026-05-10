# Obstacle Pause for Forward Motion (revised)

## Goal

Pause forward driving while a static obstacle is inside a narrow front cone of the LDS LiDAR. Resume automatically once the cone clears. No replanning, no costmaps, no new action API.

## Approach

A new pure-logic class `ObstacleGate` is owned by `TurtleBotNode`, fed `LaserScan` messages, and queried by `MotionController` before every forward command. The gate is the single source of truth for "is the front clear?". It carries no rospy dependency so it stays unit-testable.

```
LaserScan ──▶ TurtleBotNode._on_scan ──▶ ObstacleGate.update(scan)
                                                   │
MotionController.drive_towards ──▶ gate.is_blocked() ──▶ publish 0 if blocked
```

## Files touched

| File | Change |
|------|--------|
| `src/pathfinder/src/pathfinder/safety/obstacle_gate.py` | new — `ObstacleGate` class |
| `src/pathfinder/src/pathfinder/robot/motion_controller.py` | accept optional gate, consult it before forward commands |
| `src/pathfinder/src/pathfinder/ros/turtlebot_node.py` | construct gate, subscribe `/{namespace}/scan`, add `topic_scan` property |
| `src/pathfinder/scripts/robot_executors_node` | read four new rosparams, build the gate, pass to motion controller |
| `src/pathfinder/launch/robots.launch` | inline `<param>` tags for the four new keys |
| `src/pathfinder/config/params.yaml` | mirror the four new keys under `executor:` |
| `src/pathfinder/package.xml` | add `sensor_msgs` build/exec depend |
| `src/pathfinder/CMakeLists.txt` | add `sensor_msgs` to `find_package` and `catkin_package` |
| `src/pathfinder/test/test_obstacle_gate.py` | new unit tests for the gate |
| `src/pathfinder/test/test_motion.py` | add cases for the gated controller |
| `src/pathfinder/CMakeLists.txt` | register `test_obstacle_gate.py` |

## ObstacleGate (new)

`src/pathfinder/src/pathfinder/safety/obstacle_gate.py`

Public surface (kept minimal):

```python
class ObstacleGate:
    """Pauses forward motion when a LaserScan return is inside the front cone."""

    def __init__(
        self,
        stop_distance: float,
        error_margin: float,
        stale_timeout_seconds: float,
        cone_half_width_radian: float,
        time_provider: Callable[[], float],
    ) -> None: ...

    def update(self, scan: LaserScan) -> None: ...
    def is_blocked(self) -> bool: ...
```

Behavior of `is_blocked()`:

| Condition | Result |
|-----------|--------|
| No scan ever received | blocked |
| Last `update()` older than `stale_timeout_seconds` | blocked |
| `scan.ranges` empty or `scan.angle_increment <= 0` | blocked |
| Any NaN inside the front cone | blocked |
| Any range inside the cone `< stop_distance + error_margin` | blocked |
| Inf inside the cone | treated as clear at max range |
| No beam falls inside the cone (config error) | blocked |
| Otherwise | clear |

Cone selection: iterate `scan.ranges`, compute each beam's angle as `wrap_to_pi(scan.angle_min + index * scan.angle_increment)`, and keep beams with `abs(angle) <= cone_half_width_radian`. Cone half-width default is `radians(10)` so the total cone is 20 degrees. Threshold is measured at the LiDAR origin, not the bumper.

`time_provider` is injected so unit tests don't need `rospy.Time.now()`. Production wiring passes `lambda: rospy.get_time()`.

## MotionController changes

`src/pathfinder/src/pathfinder/robot/motion_controller.py`

Add one optional collaborator. Default `None` keeps every existing test path unchanged.

```python
def __init__(
    self,
    cmd_vel_publisher: CmdVelPublisher,
    pose_provider: Callable[[], Pose2D],
    params: MotionParameters = MotionParameters(),
    obstacle_gate: ObstacleGate | None = None,
) -> None:
```

`drive_towards()` ordering, written for readability:

1. If within `arrival_tolerance` → publish zero, return `True`. (Arrival wins over the gate so a goal next to a wall still completes.)
2. Compute `speed_linear`, `speed_angular` exactly as today.
3. If `speed_linear > 0` and `obstacle_gate is not None` and `obstacle_gate.is_blocked()` → publish zero, return `False`.
4. Otherwise publish `(speed_linear, speed_angular)`, return `False`.

Step 3 sits *after* the heading-tolerance gate that already zeroes `speed_linear` while rotating in place, so in-place turning is naturally never blocked. `turn_towards()` is untouched. `move_backward` is untouched (out of scope per discussion).

Update the docstring on `drive_towards` to name the three outcomes: arrived, in-progress, blocked.

## TurtleBotNode changes

`src/pathfinder/src/pathfinder/ros/turtlebot_node.py`

- Add constructor parameter `obstacle_gate: ObstacleGate | None = None`. When provided, pass it into `MotionController` and subscribe to scan; when `None`, behave as today.
- Add `topic_scan` property mirroring `topic_odom` / `topic_stop`:

  ```python
  @property
  def topic_scan(self) -> str:
      """Topic name for the LiDAR subscriber."""
      return f'/{self._namespace}/scan'
  ```

- Add `_on_scan(self, message: LaserScan) -> None` that calls `self._obstacle_gate.update(message)`.
- Add `rospy.Subscriber(self.topic_scan, LaserScan, self._on_scan)` only when a gate is supplied.
- Per `feedback_node_testing_policy.md`, this class still has no unit tests.

## robot_executors_node changes

`src/pathfinder/scripts/robot_executors_node`

Read four new private rosparams (with defaults that match `params.yaml`):

```python
obstacle_pause_enabled    = rospy.get_param('~obstacle_pause_enabled', True)
obstacle_stop_distance    = rospy.get_param('~obstacle_stop_distance', 0.20)
lidar_error_margin        = rospy.get_param('~lidar_error_margin', 0.05)
scan_stale_timeout_seconds = rospy.get_param('~scan_stale_timeout_seconds', 1.0)
```

Build one gate per robot (or `None` when disabled) and pass it through:

```python
gate = ObstacleGate(
    stop_distance=obstacle_stop_distance,
    error_margin=lidar_error_margin,
    stale_timeout_seconds=scan_stale_timeout_seconds,
    cone_half_width_radian=math.radians(10.0),
    time_provider=rospy.get_time,
) if obstacle_pause_enabled else None

node = TurtleBotNode(robot_id, ..., obstacle_gate=gate)
```

The 10° half-width stays a code constant for v1 (one fewer knob to expose). Promote to a rosparam later if needed.

## Launch and config

`src/pathfinder/launch/robots.launch` — add inline params on the `robot_executors` node, matching the existing style:

```xml
<param name="obstacle_pause_enabled" value="true"/>
<param name="obstacle_stop_distance" value="0.20"/>
<param name="lidar_error_margin" value="0.05"/>
<param name="scan_stale_timeout_seconds" value="1.0"/>
```

`src/pathfinder/config/params.yaml` — mirror under `executor:` (file is documentation-only today; keep it in sync per CLAUDE.md):

```yaml
executor:
  linear_speed: 0.22
  angular_speed: 1.5
  motion_rate_hz: 5.0
  obstacle_pause_enabled: true
  obstacle_stop_distance: 0.20
  lidar_error_margin: 0.05
  scan_stale_timeout_seconds: 1.0
```

## Package metadata

`package.xml`:

```xml
<build_depend>sensor_msgs</build_depend>
<exec_depend>sensor_msgs</exec_depend>
```

`CMakeLists.txt`: add `sensor_msgs` to `find_package(catkin REQUIRED COMPONENTS ...)` and `catkin_package(CATKIN_DEPENDS ...)`. Then `catkin_make --only-pkg-with-deps pathfinder`.

## Tests

### `test/test_obstacle_gate.py` (new)

A tiny `FakeScan` namespace and a controllable `time_provider` cover everything; no rospy stub needed.

- `test_blocked_when_no_scan_received`
- `test_blocked_when_scan_is_stale`
- `test_blocked_when_ranges_empty`
- `test_blocked_when_front_range_below_threshold`
- `test_clear_when_front_range_above_threshold`
- `test_inf_in_cone_treated_as_clear`
- `test_nan_in_cone_treated_as_blocked`
- `test_obstacle_outside_cone_does_not_block` (place return at 90°)
- `test_threshold_uses_stop_distance_plus_error_margin`

### `test/test_motion.py` (extend)

Reuse the existing rospy-stub harness. Add a `FakeGate` with a `is_blocked` flag.

- `test_drive_towards_publishes_zero_and_returns_false_when_gate_blocked`
- `test_drive_towards_publishes_normal_command_when_gate_clear`
- `test_drive_towards_arrival_within_tolerance_ignores_gate` (gate blocked but already arrived → returns True, publishes zero)
- `test_in_place_rotation_is_not_blocked` (heading off → speed_linear is already 0 → gate is not consulted, angular command is published)

### `CMakeLists.txt`

Add `catkin_add_nosetests(test/test_obstacle_gate.py)`.

### Skipped (per `feedback_node_testing_policy.md`)

No `TurtleBotNode` rospy-mock test. The `topic_scan` string is reviewed alongside `topic_odom`/`topic_stop`; launch wiring is covered by `test_launch_split.py`.

## Accepted limitations

- If an obstacle blocks the goal indefinitely, `FollowPath` stays active and the executor publishes zeros until the user cancels. v1 has no preemption timeout, no replanning.
- Threshold is at the LiDAR origin; the front bumper is ~0.14 m closer to obstacles. Tune `obstacle_stop_distance` accordingly.
- Backward motion is unguarded.
- Cone half-width is a code constant for v1.
- Coexists with `safety/collision_monitor.py`'s latched `topic_stop` — the two mechanisms are independent on purpose.

## Verification

1. `catkin_make --only-pkg-with-deps pathfinder` succeeds after adding `sensor_msgs`.
2. `python3 src/pathfinder/test/test_obstacle_gate.py` and `python3 src/pathfinder/test/test_motion.py` both pass.
3. `catkin_make run_tests_pathfinder && catkin_test_results build/test_results` is green.
4. `roslaunch pathfinder simulation.launch`, drive a robot toward a spawned obstacle, observe `cmd_vel` zero out around 0.25 m and resume after the obstacle is removed.
