# Plan — Rewrite `MotionController` as imperative blocking primitives

## Context

`src/pathfinder/src/pathfinder/robot/motion_controller.py` currently mixes two unrelated responsibilities into one class:

1. A pure proportional controller (`drive_towards`) consumed tick-by-tick by `PathFollower`.
2. Imperative blocking primitives (`turnLeft`, `MoveTowards`, …) that block a thread, watch `/odom`, and stream `/cmd_vel`.

The constructor's optional `cmd_vel_topic`/`odom_topic` and the method-level `import rospy` / `from geometry_msgs.msg import Twist` exist only because those two responsibilities have different ROS coupling. The result is a 200-line class that is hard to read and violates the project rule against in-method imports.

This plan rewrites `MotionController` to do only #2 — imperative blocking primitives — with file-level imports and a single, mandatory pair of topic arguments. The proportional-control path (`drive_towards`, `MotionParameters`, `DriveResult`, `PathStep`) is removed from this class.

**Scope (confirmed with user):** rewrite only `motion_controller.py`. Downstream consumers (`path_follower.py`, `turtlebot.py`, `turtlebot_node.py`, `test_motion.py`, `test_turtlebot.py`, `test_turtlebot_node.py`) will break and are handled in a follow-up — out of scope here.

## Target API

```python
class MotionController:
    def __init__(
        self,
        cmd_vel_topic: str,
        odom_topic: str,
        linear_speed: float = 0.22,
        angular_speed: float = 1.5,
        rate_hz: float = 20.0,
    ) -> None: ...

    def turnLeft(self, rad: float) -> bool: ...
    def turnRight(self, rad: float) -> bool: ...
    def MoveTowards(self, meter: float) -> bool: ...
    def MoveBackwards(self, meter: float) -> bool: ...
    def SetLinearSpeed(self, speed: float) -> None: ...
    def SetAngularSpeed(self, speed: float) -> None: ...
    def stop(self) -> None: ...
```

Return-value contract for the four blocking primitives: `True` when the requested distance/angle is reached, `False` when interrupted by `stop()` or `rospy` shutdown. Zero / negative inputs are treated as no-ops returning `True`.

## File-level imports

All ROS imports go to the top of the file. No `import rospy`, no `from geometry_msgs.msg import Twist`, no `from nav_msgs.msg import Odometry` inside any method.

```python
from __future__ import annotations
import math
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
```

Note: this changes the `import rospy` timing relative to the existing `_install_ros_stubs()` test pattern in `test_motion.py`. That is the test author's problem to solve in the follow-up (out of scope here, per user).

## Internal design

State held on the instance:
- `self._linear_speed`, `self._angular_speed` — set by the `Set…` methods, read each loop tick.
- `self._rate_hz` — loop rate for both pose-wait and velocity-publish loops.
- `self._stop_event: threading.Event` — set by `stop()`, checked each loop tick. Cleared at the start of each primitive.
- `self._pose_lock: threading.Lock` — guards `_latest_pose`.
- `self._latest_pose: tuple[float, float, float] | None` — `(x, y, yaw)` from latest `/odom`.
- `self._cmd_vel_pub` — created in `__init__`. The `/odom` subscriber is also created in `__init__` and bound to `_on_odom`.

Two private helpers do the work:
- `_move(distance, direction)` — wait for first pose, then loop publishing `(±linear_speed, 0)` until Euclidean travel from start ≥ distance, or stop, or shutdown. Always finishes with a zero `Twist`.
- `_turn(angle, direction)` — same pattern with `(0, ±angular_speed)` and `abs(wrap_to_pi(yaw - start_yaw)) ≥ angle` as the exit condition. (This caps a single `_turn` call at π radians of useful tracking, matching the existing implementation; out-of-scope to fix here.)

Two more helpers stay private:
- `_yaw_from_quaternion(q)` and `_wrap_to_pi(angle)` — pure math, identical formulas to the current file (lines 202–208).
- `_publish(linear_x, angular_z)` — one-line `Twist` publisher. Replaces the existing `_publish_twist` / `_publish_stop` / `_require_ros_io` trio.

Removed entirely:
- `MotionParameters`, `DriveResult`, `PathStep` dataclasses.
- `drive_towards()` method.
- The `_begin_primitive` / `_require_ros_io` indirection — with topics now mandatory, the publisher always exists.
- Optional-topic branching in `__init__`.

## Method ordering and docstrings

Per project conventions in user memory:
- Public methods first, private after.
- One-line docstring on every public method and on the class.
- No docstrings on private methods.

Order in the file:
1. `class MotionController:` + class docstring
2. `__init__`
3. `turnLeft`, `turnRight`, `MoveTowards`, `MoveBackwards` (four blocking primitives, in user-listed order)
4. `SetLinearSpeed`, `SetAngularSpeed`
5. `stop`
6. `_on_odom`
7. `_move`, `_turn`
8. `_wait_for_pose`, `_current_pose`, `_publish`
9. `_yaw_from_quaternion`, `_wrap_to_pi` (staticmethods)

## Critical files

- **Modify:** `src/pathfinder/src/pathfinder/robot/motion_controller.py` — full rewrite.
- **Read-only references** (used to verify behavior parity with the old blocking primitives, not edited):
  - `src/pathfinder/src/pathfinder/robot/motion_controller.py:88-208` — current `_turn`/`_move` loop logic and quaternion math we are preserving.
  - `src/pathfinder/test/test_motion.py:223-321` — the imperative-primitive tests (constructor topics, set-speed, MoveTowards/Backwards, turnLeft/Right, stop). These describe the contract the new class must satisfy.

## Verification

The codebase will not compile-test cleanly until the follow-up updates the consumers, so end-to-end tests are out of scope. For this PR:

1. **Lint / static smell check** — confirm `grep -n 'import ' src/pathfinder/src/pathfinder/robot/motion_controller.py` shows imports only in the top block (no `import` keyword appearing inside a `def`).
2. **API surface check** — `python3 -c "import ast,sys; tree=ast.parse(open('src/pathfinder/src/pathfinder/robot/motion_controller.py').read()); cls=[n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name=='MotionController'][0]; print(sorted(m.name for m in cls.body if isinstance(m, ast.FunctionDef) and not m.name.startswith('_')))"` should print exactly `['MoveBackwards', 'MoveTowards', 'SetAngularSpeed', 'SetLinearSpeed', 'stop', 'turnLeft', 'turnRight']`.
3. **Behavior parity** — read-through against `test_motion.py:223-321` to confirm each assertion still holds with the new code (manual review; tests themselves will fail to import until the follow-up).
4. **Smoke run (in container, after follow-up lands)** — `roslaunch pathfinder gazebo_world.launch` and exercise a primitive from a Python REPL; confirm the robot moves and `stop()` halts it.

## Out of scope / follow-up work

These break with this rewrite and are deliberately deferred:
- `path_follower.py` imports `DriveResult`, `MotionController`, `PathStep` and calls `drive_towards`. Either delete `path_follower.py` or re-home the proportional logic.
- `turtlebot.py` imports `DriveResult`, `MotionController`, `MotionParameters`, `PathStep` and instantiates `MotionController(motion_parameters)` (no topics).
- `turtlebot_node.py:11` imports `DriveResult` for `publish_drive_result`.
- `test_motion.py`, `test_turtlebot.py`, `test_turtlebot_node.py` reference the removed types and the old constructor signature.
