# Plan: TurtleBot publishes world-frame pose

## Context

`PathServerNode` currently subscribes to each robot's `/odom` and reconstructs a pose via `RobotState.from_odometry(robot_id, message).get_pose()` — but it never passes `origin`, so the helper applies a zero origin. In sim mode this is fine (Gazebo odom is already world-frame), but in non-sim mode it silently uses pose relative to the robot's start position. The robot itself, in `TurtleBotNode._on_odom`, applies origin correctly. Two consumers, two different pose semantics — the path server's planning starts from the wrong place on a real robot.

The fix is single-source-of-truth: `TurtleBot` already knows its origin and corrects pose; let it publish that corrected pose on `/{namespace}/pose`. Consumers subscribe to a plain `Pose2D`, no odometry math, no origin coupling.

This is not duplication of `/odom`. `/odom` is raw robot-frame odometry (Pose + Twist, sensor-level). `/pose` is derived world-frame state. Same distinction as `/cmd_vel` vs `/joint_states`.

## Changes

### 1. Publish `Pose2D` on `/{namespace}/pose` from `TurtleBotNode`

File: `src/pathfinder/src/pathfinder/ros/turtlebot_node.py`

- Add `topic_pose` property: `f'/{self._namespace}/pose'` (alongside the other `topic_*` properties).
- In `__init__`, create the publisher: `self._pose_publisher = rospy.Publisher(self.topic_pose, Pose2D, queue_size=1)`.
- In `_on_odom`, after the existing state update, append one line: `self._pose_publisher.publish(self._state.get_pose())`.

No new class, no timer, no separate publisher object. `TurtleBotNode` is the assembly/adapter layer (per CLAUDE.md); adding one publisher fits that role. Publishing inside `_on_odom` matches the odom rate naturally.

### 2. `PathServerNode` subscribes to `Pose2D`

File: `src/pathfinder/src/pathfinder/ros/path_server_node.py`

- Replace the `Odometry` subscriber with `Pose2D`:
  ```python
  rospy.Subscriber(
      f"/{robot.namespace}/pose",
      Pose2D,
      lambda message, robot_id=robot.id: self._tracker.update(robot_id, message),
  )
  ```
- Delete the `_on_odom` method and the `RobotState`/`Odometry`/`nav_msgs.msg` imports.
- The class no longer touches odometry math at all — pure state subscriber.

### 3. Delete `RobotState.from_odometry` and its test

After step 2, the method has no callers in production code.

- File: `src/pathfinder/src/pathfinder/robot/robot_state.py` — delete the `from_odometry` classmethod, drop the now-unused `Any` import.
- File: `src/pathfinder/test/test_turtlebot.py:141-148` — delete `test_robot_state_can_be_created_from_odometry`. Keeping a test only because it references a now-unused method is the wrong direction.

## SRP outcome

| Component | Owns |
|---|---|
| `TurtleBotNode._on_odom` | odom → world-frame state (origin math) → publish pose |
| `PathServerNode` | subscribe to pose, feed tracker. Knows nothing about origin or odometry. |
| `RobotState` | mutable pose/velocity container. No I/O, no message decoding. |

Three components, three distinct responsibilities, zero overlap.

## Verification

- `python3 src/pathfinder/test/test_path_server.py` — passes unchanged (service tests inject a `PoseTracker` directly, don't exercise the subscriber).
- `python3 src/pathfinder/test/test_turtlebot.py` — passes after the one deletion.
- `grep -r "from_odometry" src/pathfinder` — should return zero results after the change.
- Smoke test in container: `roslaunch pathfinder simulation.launch`, then `rostopic echo /tb3_01/sim/pose` should emit `Pose2D` at the odom rate. Then `rosrun pathfinder client tb3_01 5` should plan and dispatch as before.
- Behaviour parity in sim: origin defaults to `Pose2D()` (zero), so `/pose` matches what the path server used to compute — no regression.
- Behaviour fix in non-sim: `/pose` reflects world frame, so planning starts from the correct node — bug closed.

## What I'm not doing

- **No `PosePublisher` class.** Two lines inside `_on_odom` is simpler than a new class. Adding indirection now would obscure the data flow.
- **No timer-driven publishing.** Coupling to the odom callback gives the right rate for free.
- **No backwards-compat: `/odom` keeps existing.** I'm only changing `PathServerNode`'s subscription. Nothing else in the system reads odom in the path server.
