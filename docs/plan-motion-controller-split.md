# Plan: split motion control into MotionController + MotionEngine

## Current pain

Loop logic is duplicated across three classes, each with its own cancel/pause flag:

- `TurtleBot._move_to` — has `_cancel`, `_pause`, rospy.Rate, inner pause loop
- `TurtleBot._turn_to` — has `_cancel`, rospy.Rate
- `PathFollower.follow` — has `_cancel`, `_pause_check`, rospy.Rate, identical inner structure

`TurtleBot.set_pause()` was wired to `_move_to` only after the recent edits removed `path_follower.set_pause_check(...)` from `TurtleBot.__init__`. The pause flag never reaches `PathFollower.follow()`. The two pause systems are out of sync.

## Target architecture

Two motion-control layers, plus an unchanged sequencer:

### `MotionEngine` (renamed from current `MotionController`)
Pure single-tick proportional logic. No rospy, no state, no loop.
- `drive_towards(node) -> bool`
- `turn_towards(heading) -> bool`
- `stop()`

`MotionParameters`, `CmdVelPublisher`, `_clamp` stay alongside.

### `MotionController` (new role)
Owns the control loop, the target, `_cancel`, `_pause`. Wraps a `MotionEngine`.

Public surface:
- `drive_to(target: Node) -> bool` — loops `engine.drive_towards` until arrival, cancel, or shutdown. Honors `_pause` (inner wait loop with `engine.stop()`).
- `turn_to(heading: float) -> bool` — loops `engine.turn_towards` until aligned, cancel, or shutdown. Does NOT honor `_pause`.
- `stop()` — sets `_cancel = True`, calls `engine.stop()`.
- `set_pause(paused: bool)` — sets `_pause`.

Constructor: `MotionController(engine: MotionEngine, rate_hz: float = 5.0)`.

`_cancel` resets at the start of each `drive_to` / `turn_to` call (matches current `_move_to`).

### `PathFollower` (simplified)
Becomes a pure sequencer that calls `MotionController.drive_to` per waypoint. Drops its own loop, rate, `_pause_check`, `set_pause_check`.

- `follow(waypoints: list[Node]) -> bool` — iterate, return False on first `drive_to` failure.
- `cancel()` — delegates to `motion_controller.stop()`.

### `TurtleBot` (thinned)
Becomes a thin facade. No `_cancel`, `_pause`, `_rate_hz`, `_move_to`, `_turn_to`.

- `move_forward(meter)` / `move_backward(meter)` — compute target Node, call `motion_controller.drive_to(target)`.
- `turn_left(rad)` / `turn_right(rad)` — compute absolute heading, call `motion_controller.turn_to(heading)`.
- `set_pause(paused)` — delegates to `motion_controller.set_pause(paused)`.
- `stop()` — delegates to `motion_controller.stop()`, also `path_follower.cancel()`.
- `follow_path(nodes)` — unchanged, via `path_follower.follow(nodes)`.

## File-by-file edits

1. **New file** `src/pathfinder/robot/motion_engine.py`
   Move `MotionParameters`, `CmdVelPublisher`, the renamed `MotionEngine` class, and `_clamp` here. No rospy import.

2. **Rewrite** `src/pathfinder/robot/motion_controller.py`
   New `MotionController` class as described. Imports rospy + `MotionEngine`. Owns the loop.

3. **Edit** `src/pathfinder/robot/path_follower.py`
   - Constructor: take `MotionController`, drop `rate_hz`.
   - Drop `_pause_check`, `set_pause_check`, rospy import, rospy.Rate.
   - `follow`: iterate and call `motion_controller.drive_to`.
   - `cancel`: delegate to `motion_controller.stop()`.

4. **Edit** `src/pathfinder/robot/turtlebot.py`
   - Drop `_cancel`, `_pause`, `_rate_hz`, `_move_to`, `_turn_to`, rospy import.
   - Methods become one-line delegators.

5. **Edit** `src/pathfinder/ros/turtlebot_node.py`
   - Build `MotionEngine`, wrap in `MotionController`, pass `MotionController` to both `PathFollower` and `TurtleBot`.

6. **Update tests**
   - `test/test_motion.py`: rename `MotionController` references to `MotionEngine` for tick-level tests. Add new `MotionController` tests covering: `drive_to` returns True on arrival; `set_pause(True)` causes `drive_to`'s engine to receive `stop()` instead of `drive_towards`; `set_pause(True)` does NOT pause `turn_to`; `stop()` causes both to return False; `_cancel` resets between calls.
   - `test/test_path_server.py`, `test/test_turtlebot_node.py`, `test/test_gazebo_control.py`: update any `MotionController` import / construction to match.
   - `test/test_path_follower.py` if present: rewrite to inject a fake `MotionController`.

## Out of scope

- Renaming files or classes outside motion logic.
- Changing `MotionParameters` fields.
- Touching `ObstacleDetector` or `TurtleBotNode._on_scan`.

## Success criteria

- All existing tests pass; new `MotionController` loop tests pass.
- `TurtleBot` contains no rospy.Rate or while-loop.
- `PathFollower` contains no rospy.Rate or pause-check.
- `MotionController.set_pause(True)` halts an in-flight `drive_to`; `turn_to` continues unaffected.
- `set_pause` on `TurtleBot` propagates to both single-shot moves and `follow_path` (via the shared `MotionController`).

## Open questions

- File name for the new engine module: `motion_engine.py` (proposed) vs. keeping both classes in `motion_controller.py`. Two files keeps the layering visible; one file keeps imports terser. Picking two files unless you say otherwise.
- Should `MotionController` expose `is_paused` / `is_cancelled` getters for tests, or should tests inspect via behavior only? Behavior-only is cleaner; will add getters if a test demands them.
