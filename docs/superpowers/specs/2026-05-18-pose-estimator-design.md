# PoseEstimator — Design Spec

**Date:** 2026-05-18  
**Branch:** `wonseok/fix/ensure-using-slam`  
**Problem:** Robots overturn on ~90° turns because the current dual-callback pose scheme (`_on_odom` + `_on_amcl_pose` composing a `_yaw_offset`) is fragile — any bias in AMCL's yaw estimate propagates directly into the angular error signal fed to `MotionEngine.turn_towards`.  
**Goal:** Fix the overturning bug and simplify `TurtleBotNode` by extracting pose computation into a focused, testable class.

---

## 1. Root Cause

`turtlebot_node.py` currently maintains two pieces of shared mutable state across two callbacks:

- `_latest_odom_yaw` — written by `_on_odom`, read by `_on_amcl_pose`
- `_yaw_offset` — written by `_on_amcl_pose`, read by `_on_odom`

`_on_amcl_pose` sets `_yaw_offset = wrap_to_pi(amcl_yaw - latest_odom_yaw)`. Subsequent odom callbacks compute `theta = wrap_to_pi(odom_yaw + _yaw_offset)`. If AMCL's yaw has any bias (particle filter quantization, sparse `update_min_a` triggers), that bias is locked into every theta reading until the next AMCL update, causing `turn_towards` to correct against a wrong reference and overshoot.

---

## 2. Approach

Replace the dual-callback composition with a `PoseEstimator` class that uses a tf lookup (`map → {ns}/base_footprint`) as the single source of truth. AMCL already publishes the `map→odom` transform to the tf tree, so tf lookup works with the current AMCL setup and would also work with SLAM if the localization system changes later.

---

## 3. Architecture

```
Before:
  _on_odom ──┐
             ├─ _yaw_offset (shared state) ──→ state.pose.theta
  _on_amcl_pose ─┘                             │
                                    MotionEngine(pose_provider=state.get_pose)

After:
  _on_odom ──→ PoseEstimator.on_odom()
                  │  tf lookup ('map' → '{ns}/base_footprint')
                  │  fallback: odom + origin
                  └─ PoseEstimator.get_pose() ──→ MotionEngine(pose_provider=estimator.get_pose)
                                               └─→ /pose publisher
```

The `MotionEngine` constructor already accepts `pose_provider: Callable[[], Pose2D]`, so no change is needed to the motion stack.

---

## 4. Files Changed

| Action | File | Notes |
|--------|------|-------|
| **New** | `src/pathfinder/src/pathfinder/ros/pose_estimator.py` | `PoseEstimator` class |
| **New** | `src/pathfinder/test/test_pose_estimator.py` | Unit tests |
| **Modify** | `src/pathfinder/src/pathfinder/ros/turtlebot_node.py` | Remove dual-callback scheme, wire estimator |
| **Modify** | `src/pathfinder/src/pathfinder/utils/physics.py` | Add `yaw_from_xyzw()` |

---

## 5. `PoseEstimator` Class

**File:** `ros/pose_estimator.py`

```
PoseEstimator(tf_listener, namespace, origin, sim=False)
  ├── on_odom(msg: Odometry) → None
  └── get_pose() → Pose2D
```

**Constructor parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `tf_listener` | `tf.TransformListener` | Injected — not constructed inside the class |
| `namespace` | `str` | Robot namespace, e.g. `tb3_01` |
| `origin` | `Pose2D` | Start position used for odom fallback and sim mode |
| `sim` | `bool` | `True` → odom direct; `False` → tf lookup with odom fallback |

**`on_odom(msg)`:**

- In sim mode: `x = odom.x + origin.x`, `y = odom.y + origin.y`, `theta = yaw_from_quaternion(odom.orientation) + origin.theta`
- In real-robot mode: attempt `tf_listener.lookupTransform('map', '{ns}/base_footprint', Time(0))`
  - Success → `x, y` from translation; `theta` from `yaw_from_xyzw(*rotation)`
  - `LookupException / ConnectivityException / ExtrapolationException` → fall back to odom + origin (pose stays at last valid value until tf is ready)
- Updates internal `_pose` under `threading.Lock`

**`get_pose()`:**

- Returns a `Pose2D` copy under `threading.Lock`
- Safe to call from the motion control thread while `on_odom` fires on the ROS callback thread
- Returns `origin` before first odom message arrives (initialized in `__init__`)

---

## 6. `TurtleBotNode` Changes

**Deleted:**

- `_yaw_offset: float` field
- `_latest_odom_yaw: float` field
- `_on_amcl_pose()` method
- `rospy.Subscriber(self.topic_amcl_pose, ...)` line
- `topic_amcl_pose` property
- The `if self._odom_pose_enabled / else` pose update block inside `_on_odom`
- Comment block explaining the yaw composition scheme

**Added:**

- `tf.TransformListener()` constructed once in `__init__`
- `PoseEstimator(tf_listener, namespace, origin, sim=odom_pose_enabled)` constructed in `__init__`
- `pose_provider=self._estimator.get_pose` passed to `MotionEngine` (replaces `state.get_pose`)

**`_on_odom` after change:**

```python
def _on_odom(self, msg: Odometry) -> None:
    self._last_odom_received_at = rospy.Time.now()
    self._state.velocity = msg.twist.twist
    self._estimator.on_odom(msg)
    self._pose_publisher.publish(self._estimator.get_pose())
```

**`RobotState.pose`:** No longer written or read by `TurtleBotNode`. Verify during implementation whether other code reads it; if unused, remove the `pose` field from `RobotState` in the same pass.

---

## 7. `utils/physics.py` Addition

```python
def yaw_from_xyzw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from a quaternion given as (x, y, z, w) floats."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
```

Pure math, no ROS dependency. Placed alongside `yaw_from_quaternion` and `wrap_to_pi`.

---

## 8. Tests — `test/test_pose_estimator.py`

All tests follow the existing pattern of stubbing ROS at the module level; no roscore required.

| Test | Verifies |
|------|----------|
| Sim mode, zero origin | pose.x/y/theta equal odom values directly |
| Sim mode, non-zero origin | pose = odom + origin for x, y, theta |
| Real mode, tf succeeds | pose comes from mock `lookupTransform` return value |
| Real mode, `LookupException` | falls back to odom + origin |
| Real mode, `ExtrapolationException` | falls back to odom + origin |
| `get_pose()` returns copy | mutating return value does not change internal state |
| `yaw_from_xyzw(0,0,0,1)` | returns 0.0 (identity) |
| `yaw_from_xyzw(0,0,1,0)` | returns ±π (180°) |
| `yaw_from_xyzw` for ±90° | spot-check against known values |

The mock tf listener is a plain Python object with a `lookupTransform` method — no ROS stubs needed for this class.

---

## 9. Out of Scope

- Changes to `MotionEngine`, `MotionController`, `PathFollower`, or `PathFollower` — the motion stack is unchanged.
- Angular gain tuning — the overturning fix is in the pose feedback accuracy, not the gain value.
- SLAM integration — `PoseEstimator` is compatible with SLAM (tf-based) and AMCL; no further change needed to switch.
