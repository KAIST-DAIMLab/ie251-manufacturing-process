# Plan — Add closed-loop `DriveTowards` to `MotionController`

## Context

Open-loop blocking primitives (`MoveTowards`, `turnLeft`, …) accumulate error across a multi-leg path: each leg snapshots `start` and runs to a fixed travelled distance, with no correction toward the *actual* waypoint. Drift inside a single leg propagates to the next.

We're switching path following to closed loop. PathFollower will own the control loop and tick MotionController each iteration with fresh odometry. MotionController exposes a single-tick proportional method that publishes one Twist toward the current target waypoint and reports whether arrival was reached.

The seven imperative primitives stay — they remain useful for direct, scripted manual control ("turn 90° then advance 50 cm"). They simply aren't what PathFollower will use.

## Target API addition

One new public method on `MotionController`:

```python
def DriveTowards(self, target: Node) -> bool:
    """Publish one proportional cmd_vel toward target; True when within arrival tolerance."""
```

Single tick. No internal loop. Reads `self._pose_provider()` once. Publishes one `Twist`. Returns `True` when the robot is inside `arrival_tolerance` of `target` (and publishes zero in that case), `False` otherwise. Intended to be called from PathFollower's loop at `rate_hz`.

Naming: `DriveTowards` (Pascal) for symmetry with `MoveTowards` / `MoveBackwards`. The existing imperative primitives use the same convention.

## New constructor parameters

Four optional kwargs on `__init__`, all with defaults matching the values from the original pre-rewrite controller:

```python
linear_gain: float = 0.5,
angular_gain: float = 1.5,
arrival_tolerance: float = 0.10,
heading_tolerance: float = 0.2,
```

Total constructor surface goes from 5 to 9 args — acceptable, all are independent scalar tunables. Not worth re-introducing a `MotionParameters` dataclass for.

## Method body (proportional control, identical math to the original `drive_towards`)

```python
def DriveTowards(self, target: Node) -> bool:
    pose = self._pose_provider()
    delta_x = target.x - pose.x
    delta_y = target.y - pose.y
    distance = math.hypot(delta_x, delta_y)

    if distance <= self._arrival_tolerance:
        self._publish(0.0, 0.0)
        return True

    desired_heading = math.atan2(delta_y, delta_x)
    heading_error = self._wrap_to_pi(desired_heading - pose.theta)
    angular_z = self._clamp(self._angular_gain * heading_error, self._angular_speed)

    if abs(heading_error) <= self._heading_tolerance:
        linear_x = min(self._linear_gain * distance, self._linear_speed)
    else:
        linear_x = 0.0

    self._publish(linear_x, angular_z)
    return False
```

`DriveTowards` does **not** consult `self._stop`. Cancellation is the caller's job — PathFollower already owns the loop and decides when to stop ticking. The `_stop` flag stays internal to the imperative blocking primitives.

`_clamp` is one new private static helper:

```python
@staticmethod
def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
```

## File layout after the change

Public method order (per project conventions, public first then private):

1. `__init__`
2. `turnLeft`, `turnRight`
3. `MoveTowards`, `MoveBackwards`
4. **`DriveTowards`** ← new
5. `SetLinearSpeed`, `SetAngularSpeed`
6. `stop`
7. `_move`, `_turn`
8. `_publish`
9. `_wrap_to_pi`, **`_clamp`** ← new

Imports: only `Node` is new. Add `from pathfinder.world.node import Node` at the top alongside the existing imports. No new ROS imports.

## PathFollower update

`src/pathfinder/src/pathfinder/robot/path_follower.py` is rewritten to own the loop. The existing tick-based `step(pose)` API and the `DriveResult` / `PathStep` indirection go away — PathFollower no longer needs them, since MotionController publishes directly.

Sketch:

```python
class PathFollower:
    """Drives a robot through an ordered list of waypoints by ticking MotionController each cycle."""

    def __init__(self, motion_controller: MotionController, rate_hz: float = 5.0) -> None:
        self._motion_controller = motion_controller
        self._rate_hz = rate_hz
        self._cancel = False

    def follow(self, waypoints: list[Node]) -> bool:
        """Drive through waypoints in order; True if all reached, False if cancelled or shutdown."""
        self._cancel = False
        rate = rospy.Rate(self._rate_hz)
        for waypoint in waypoints:
            while not rospy.is_shutdown() and not self._cancel:
                if self._motion_controller.DriveTowards(waypoint):
                    break
                rate.sleep()
            if self._cancel or rospy.is_shutdown():
                self._motion_controller.stop()
                return False
        return True

    def cancel(self) -> None:
        """Interrupt the active follow() and stop the robot."""
        self._cancel = True
        self._motion_controller.stop()
```

Notes:
- `follow()` blocks until done — matches the imperative-blocking style of the rest of MotionController, and matches what the `FollowPath` action server expects from the executor side.
- `DriveResult` and `PathStep` dataclasses go away. `turtlebot_node.py:11` (`from ... import DriveResult`) and `turtlebot_node.py:77` (`publish_drive_result`) need follow-up cleanup.
- `TurtleBot.start_path` / `step_path` / `cancel_path` need to be re-aimed at the new `follow` / `cancel` API. Likely `TurtleBot.follow_path(waypoints) -> bool` running on the action server's thread.

## Critical files

- **Modify:** `src/pathfinder/src/pathfinder/robot/motion_controller.py` — add `DriveTowards`, four constructor kwargs, `_clamp`, `Node` import.
- **Modify:** `src/pathfinder/src/pathfinder/robot/path_follower.py` — replace tick-based `start`/`step`/`cancel` with blocking `follow`/`cancel`.
- **Modify:** `src/pathfinder/src/pathfinder/robot/turtlebot.py` — collapse `start_path` + `step_path` into a single `follow_path(waypoints) -> bool`; drop `DriveResult` / `PathStep` imports.
- **Modify:** `src/pathfinder/src/pathfinder/robot/turtlebot_node.py` — drop `DriveResult` import and `publish_drive_result`; update the `FollowPath` action callback to call `follow_path` directly.
- **Modify:** `src/pathfinder/test/test_motion.py`, `test_turtlebot.py`, `test_turtlebot_node.py` — replace `drive_towards`-era assertions with `DriveTowards` tick assertions and `follow`-based path-follower assertions.

Reference (read-only):
- `src/pathfinder/src/pathfinder/robot/motion_controller.py:14-110` (current state, after the rewrite + linter pass) — anchors the file conventions: file-level imports, `_pose_provider` injection, `_publish` helper, `_wrap_to_pi` helper.
- Git: pre-rewrite `drive_towards` in commit `3a316e6` is the reference for the proportional math we are restoring.

## Verification

1. **Unit-test `DriveTowards` arithmetic** (no ROS, no threading):
   - Distance below `arrival_tolerance` → returns `True`, publishes `(0, 0)`.
   - Heading error above `heading_tolerance` → publishes `linear_x == 0`, non-zero angular toward target.
   - Heading aligned, large distance → `linear_x` clamped to `linear_speed`.
   - Large heading error → `angular_z` clamped to `angular_speed`.
   - Wrap-around (target behind robot at θ ≈ ±π) → angular sign chooses the short way.
2. **Unit-test `PathFollower.follow`** with a fake `MotionController` that returns `True` after N ticks per waypoint; assert all waypoints visited in order, assert `cancel()` interrupts mid-leg and calls `motion_controller.stop()`.
3. **Smoke run in container:** `roslaunch pathfinder gazebo_world.launch`, send a `MoveToNode` goal that requires several legs, and confirm the robot stays on the planned path (no visible drift between waypoints) and stops cleanly on emergency stop.

## Out of scope

- Acceleration / jerk limits on the published Twist. The original code didn't have these and the user hasn't asked.
- Switching to PID (integral, derivative). P-only matches the previous behavior.
- Removing the seven imperative blocking primitives. They stay for direct manual-control use cases.
