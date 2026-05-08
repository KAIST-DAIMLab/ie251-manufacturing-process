# TurtleBot Refactoring Plan

## Goal

Split the current `TurtleBot` class into focused modules with single responsibilities. The current class mixes pure motion math, mutable state, and ROS I/O — making it hard to test and slow to extend. After this refactor, each piece should be independently testable, and adding a new subscriber or swapping the control algorithm should touch one file.

## Current state

`src/pathfinder/robot/turtlebot.py` (~150 lines) does three things at once:

1. **Domain logic** — `drive_towards`, motion math, `MotionParameters`
2. **Mutable state** — pose, status, velocity, threading lock
3. **ROS I/O** — `rospy.Publisher`, `rospy.Subscriber`, `rospy.Timer`, message imports

Symptoms this causes:

- `pathfinder.msg` is lazy-imported inside a method to avoid circular-import side effects.
- `__init__` starts a `rospy.Timer` as a side effect, so the class can't be constructed without a running ROS node.
- `set_status(MOVING)` silently clears `_stop_requested` — coupling that's invisible from the method name.
- `nav_msgs` and `std_msgs` are imported by what should be pure logic.
- No unit tests are practical without a ROS master.

## Target structure

```
src/pathfinder/robot/
├── __init__.py
├── motion.py              # MotionParameters, compute_drive (pure functions)
├── motion_controller.py   # MotionController class — added when stateful control is needed
├── turtlebot.py           # TurtleBot — state container + domain methods, no rospy
└── turtlebot_node.py      # TurtleBotNode — ROS adapter (subs, pubs, timer)

scripts/
└── turtlebot_node         # thin executable launcher (chmod +x, no .py)
```

Responsibility per file:

| File | Imports rospy? | Holds state? | Purpose |
|------|----------------|--------------|---------|
| `motion.py` | No | No | Pure functions: pose + target → velocity |
| `motion_controller.py` | No | Yes | Stateful control (PID, accel limits) — *added later* |
| `turtlebot.py` | No (only types) | Yes | Robot state + thread-safe accessors |
| `turtlebot_node.py` | Yes | No (delegates) | ROS subscribers, publishers, timer |
| `scripts/turtlebot_node` | Yes | No | `init_node` + construct + `spin` |

## Phased plan

The phases are ordered so each one ends with a working, runnable system. Don't merge them into a single PR — each phase is independently reviewable.

### Phase 1 — Extract pure motion logic to `motion.py`

**What changes:** Pull the kinematics out of `TurtleBot.drive_towards` into a free function. Keep `TurtleBot.drive_towards` as a thin wrapper that snapshots state under the lock and delegates.

**New file `motion.py`:**

```python
from dataclasses import dataclass
import math
from geometry_msgs.msg import Pose2D
from pathfinder.world.node import Node


@dataclass(frozen=True)
class MotionParameters:
    linear_gain: float = 0.5
    angular_gain: float = 1.5
    max_linear_velocity: float = 0.22
    max_angular_velocity: float = 1.5
    arrival_tolerance: float = 0.10
    heading_tolerance: float = 0.2


@dataclass(frozen=True)
class DriveResult:
    arrived: bool
    linear_x: float
    angular_z: float


def compute_drive(pose: Pose2D, target: Node, params: MotionParameters) -> DriveResult:
    dx = target.x - pose.x
    dy = target.y - pose.y
    distance = math.hypot(dx, dy)

    if distance <= params.arrival_tolerance:
        return DriveResult(arrived=True, linear_x=0.0, angular_z=0.0)

    desired_heading = math.atan2(dy, dx)
    heading_error = _wrap_to_pi(desired_heading - pose.theta)
    angular_z = _clamp(params.angular_gain * heading_error, params.max_angular_velocity)

    if abs(heading_error) <= params.heading_tolerance:
        linear_x = min(params.linear_gain * distance, params.max_linear_velocity)
    else:
        linear_x = 0.0

    return DriveResult(arrived=False, linear_x=linear_x, angular_z=angular_z)


def _wrap_to_pi(angle: float) -> float: ...
def _clamp(value: float, limit: float) -> float: ...
```

Note `DriveResult` carries scalars, not `Twist`. Keeping `geometry_msgs.Twist` out of the math module means tests can assert on plain floats.

**Validation:** Write `tests/test_motion.py` with at least:

- Arrival case: distance below tolerance returns `arrived=True`, zero velocities.
- Heading mismatch: heading error above tolerance returns `linear_x == 0`.
- Saturation: large distance or heading error gets clamped to max velocities.
- Wrap-around: target behind robot turns the short way.

Run with plain `pytest`. No rospy needed.

**Done when:** All existing behavior preserved, `compute_drive` has unit tests, `TurtleBot.drive_towards` is a 5-line wrapper.

### Phase 2 — Extract ROS I/O to `turtlebot_node.py`

**What changes:** Move publishers, subscribers, and the timer into a new `TurtleBotNode` class. `TurtleBot` keeps state and domain methods; it stops importing `rospy` for anything except types it doesn't actually need.

**New `turtlebot.py` (after the move):**

```python
class TurtleBot:
    def __init__(self, robot_id: str, params: MotionParameters = MotionParameters()):
        self.id = robot_id
        self._params = params
        self._state = RobotState(id=robot_id)
        self._stop_requested = False
        self._lock = threading.Lock()

    def update_pose(self, x, y, theta, velocity, stamp) -> None:
        with self._lock:
            self._state.pose.x = x
            self._state.pose.y = y
            self._state.pose.theta = theta
            self._state.velocity = velocity
            self._state.stamp = stamp

    def request_stop(self) -> None:
        with self._lock:
            self._stop_requested = True
            self._state.status = RobotStatus.STOPPED

    def clear_stop(self) -> None:
        with self._lock:
            self._stop_requested = False

    def drive_towards(self, node: Node) -> DriveResult:
        with self._lock:
            pose = Pose2D(x=self._state.pose.x, y=self._state.pose.y, theta=self._state.pose.theta)
            self._state.status = RobotStatus.MOVING
        return compute_drive(pose, node, self._params)

    def stop_requested(self) -> bool:
        with self._lock:
            return self._stop_requested

    def state_message(self):
        with self._lock:
            return self._state.to_msg()

    # ... other accessors ...
```

Notice `set_status` is gone. The hidden coupling (`set_status(MOVING)` clearing the stop flag) is replaced with two explicit methods: `clear_stop()` and `drive_towards()` (which sets `MOVING` because that's what driving means). If you need to set status to something else, add a method named for that specific transition.

**New `turtlebot_node.py`:**

```python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from pathfinder.msg import RobotState as RobotStateMsg
from pathfinder.robot.turtlebot import TurtleBot
from pathfinder.robot.motion import DriveResult


class TurtleBotNode:
    def __init__(self, robot: TurtleBot, state_publish_rate_hz: float = 10.0):
        if state_publish_rate_hz <= 0:
            raise ValueError('state_publish_rate_hz must be positive')

        self.robot = robot
        ns = robot.id

        self._cmd_vel_pub = rospy.Publisher(f'/{ns}/cmd_vel', Twist, queue_size=10)
        self._state_pub = rospy.Publisher(f'/{ns}/robot_state', RobotStateMsg, queue_size=1)

        rospy.Subscriber(f'/{ns}/odom', Odometry, self._on_odom)
        rospy.Subscriber(f'/{ns}/emergency_stop', Empty, self._on_emergency_stop)

        self._timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / state_publish_rate_hz),
            self._publish_state,
        )

    def _on_odom(self, msg: Odometry) -> None:
        theta = _yaw_from_quaternion(msg.pose.pose.orientation)
        self.robot.update_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            theta,
            msg.twist.twist,
            msg.header.stamp,
        )

    def _on_emergency_stop(self, msg: Empty) -> None:
        self.robot.request_stop()
        self._cmd_vel_pub.publish(Twist())
        rospy.logwarn(f"{self.robot.id}: emergency stop received.")

    def _publish_state(self, event) -> None:
        self._state_pub.publish(self.robot.state_message())

    def send_drive_command(self, result: DriveResult) -> None:
        cmd = Twist()
        cmd.linear.x = result.linear_x
        cmd.angular.z = result.angular_z
        self._cmd_vel_pub.publish(cmd)
```

The `pathfinder.msg` import lives only here now. The lazy-import workaround in `start_state_publisher` goes away.

**New `scripts/turtlebot_node`** (no `.py` extension, `chmod +x`):

```python
#!/usr/bin/env python3
import rospy
from pathfinder.robot.turtlebot import TurtleBot
from pathfinder.robot.turtlebot_node import TurtleBotNode

if __name__ == '__main__':
    rospy.init_node('turtlebot_node')
    robot_id = rospy.get_param('~robot_id', 'tb3_0')
    rate_hz = rospy.get_param('~state_publish_rate_hz', 10.0)

    robot = TurtleBot(robot_id=robot_id)
    node = TurtleBotNode(robot, state_publish_rate_hz=rate_hz)
    rospy.spin()
```

Update `CMakeLists.txt`:

```cmake
catkin_install_python(PROGRAMS
  scripts/turtlebot_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

**Validation:** Run `catkin build` (or `catkin_make`), launch the node, and verify behavior matches before/after on the real or simulated TurtleBot. Subscribe to `/<id>/robot_state` to confirm publishing rate. Trigger `/<id>/emergency_stop` and confirm `cmd_vel` goes zero.

**Done when:** `TurtleBot` has zero `rospy.Publisher`/`Subscriber`/`Timer` calls, `TurtleBotNode` owns all I/O, the launcher is < 15 lines.

### Phase 3 — Introduce `MotionController` *only if needed*

This phase is **conditional**. Do it only when one of these triggers fires:

1. You need acceleration or jerk limiting on the velocity output (requires remembering the previous command).
2. You want PID control on heading instead of P-control (requires integral and derivative state).
3. You want to swap algorithms (pure-pursuit, DWA, etc.) at runtime.

Until one of these is real, `motion.py` as a pure-function module is the better design. Introducing a class without state is ceremony.

**When the trigger fires**, structure looks like:

```python
# motion_controller.py
class MotionController:
    def __init__(self, params: MotionParameters):
        self._params = params
        self._prev_linear_x = 0.0
        self._prev_angular_z = 0.0
        # PID state if needed:
        self._heading_integral = 0.0
        self._prev_heading_error = 0.0

    def compute(self, pose: Pose2D, target: Node, dt: float) -> DriveResult:
        # use compute_drive() for the base calculation
        # then apply state-dependent corrections (accel limits, integral term)
        ...

    def reset(self) -> None:
        """Clear integral windup and previous commands. Called on e-stop or new goal."""
        self._prev_linear_x = 0.0
        self._prev_angular_z = 0.0
        self._heading_integral = 0.0
        self._prev_heading_error = 0.0
```

`TurtleBot` would then hold a `MotionController` instead of just `MotionParameters`, and `request_stop()` / `clear_stop()` would call `controller.reset()`. The pure `compute_drive` function stays — `MotionController.compute` calls it as the base case and wraps it with state-dependent behavior.

If multiple algorithms become real, promote `MotionController` to a `Protocol` and have concrete classes (`PController`, `PurePursuitController`, etc.) implement `compute`. Don't do this speculatively.

**Validation:** Unit-test the controller with a fake clock — feed `dt` values and a sequence of pose inputs, assert the output respects the new constraints (e.g., `|linear_x[n] - linear_x[n-1]| / dt <= max_accel`).

## Items deliberately not changed

- The `RobotStatus` enum and `RobotState` dataclass are fine; they're already pure data.
- The `_yaw_from_quaternion` helper can stay in either `turtlebot_node.py` (where it's used) or in a small `utils.py` if it's needed elsewhere. Don't agonize over this.
- The threading lock stays. Rospy callbacks come from a thread pool, so any access to `_state` and `_stop_requested` must remain locked. Don't try to remove the lock as part of this refactor.

## Risk and rollback

Each phase is a separate commit. If Phase 2 breaks something at integration time, revert it without losing Phase 1's gains. The riskiest step is Phase 2 because it touches the launch path and topic wiring; do that one with the robot in a safe state and `cmd_vel` echo'd to the terminal so you can sanity-check zero output during e-stop.

## Open questions

1. Does anything else in `pathfinder` construct `TurtleBot` directly, or is it only created from a launcher? If only from a launcher, `TurtleBot` and `TurtleBotNode` could be merged into one file (`turtlebot_node.py`) with the class renamed back — keeping `motion.py` separate is still the main win.
2. Is there a planned multi-robot supervisor that would instantiate multiple `TurtleBotNode` objects in one process? If yes, the `init_node` separation in this plan is essential. If no, it's still good practice but lower priority.
3. Does `pathfinder.msg.RobotState` get imported anywhere else, or only inside `TurtleBot.start_state_publisher`? If only there, moving it to `turtlebot_node.py` is clean. If elsewhere, no harm — that import was always going to live somewhere.