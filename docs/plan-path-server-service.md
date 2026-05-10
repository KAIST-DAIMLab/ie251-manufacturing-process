# Plan: Replace `PathRequestActionServer` with ROS services

## Context

`PathRequestActionServer` exposes the user-facing `MoveToNode` action. The server spawns a daemon thread per goal, blocks on `PathFollowActionClient.dispatch()` (which itself polls in a 0.1s loop), transforms `FollowPath` feedback into `MoveToNode` feedback, deduplicates it, and manages goal-handle state machines (succeed / abort / cancel). All of that mechanism exists to deliver progress feedback the user-side client never actually surfaces (it just `loginfo`s) and to support cancellation.

The user has decided that an action protocol is overkill: the client only needs to ask "plan and dispatch a path" and "cancel a path". A request/response service fits the actual contract. Feedback from `FollowPath` will be consumed silently on the path-server side and not relayed.

## Outcome

- `client.send_goal(robot_id, target)` calls a `MoveToNode` service. The service plans, dispatches `FollowPath`, and returns immediately. It does NOT wait for the robot to arrive.
- `client.cancel(robot_id)` calls a new `CancelPath` service. The path server cancels the in-flight `FollowPath` for that robot.
- Calling `MoveToNode` while a `FollowPath` is in flight for the same robot auto-preempts the previous goal before dispatching the new one.
- `/{robot_id}/stop` topic is removed entirely (publisher in client, subscriber and handler in `TurtleBotNode`). `CancelPath` service is the only cancellation path.
- `MoveToNode.action` and all feedback-transformation code are deleted.

## Plan

### 1. Service definitions

Create:

- `src/pathfinder/srv/MoveToNode.srv`
  ```
  string  robot_id
  int32   target_node_id
  ---
  bool    success
  string  message
  ```
- `src/pathfinder/srv/CancelPath.srv`
  ```
  string  robot_id
  ---
  bool    success
  string  message
  ```

Update [CMakeLists.txt](../src/pathfinder/CMakeLists.txt):

- Drop `MoveToNode.action` from `add_action_files(...)` (keep `FollowPath.action`, `RobotCommand.action`).
- Add `add_service_files(FILES MoveToNode.srv CancelPath.srv)`.
- `generate_messages(...)` already lists the right deps; no change.

Delete [src/pathfinder/action/MoveToNode.action](../src/pathfinder/action/MoveToNode.action).

### 2. Simplify `PathFollowActionClient`

File: [src/pathfinder/src/pathfinder/ros/path_follow_action_client.py](../src/pathfinder/src/pathfinder/ros/path_follow_action_client.py)

Current `dispatch(node_ids, on_feedback, is_canceled)` is a polling loop with feedback relay and external cancel-check callback. Replace with two non-blocking methods:

- `send(node_ids: list[int]) -> bool` — waits up to 5s for the action server, then `self._client.send_goal(goal)` and returns `True` if reachable.
- `cancel() -> None` — calls `self._client.cancel_goal()` (no-op when no goal is active; SimpleActionClient handles that gracefully).

Remove the polling loop, `_store_feedback`, and the callback wiring entirely. No feedback is consumed any more.

### 3. Replace `PathRequestActionServer` with `PathRequestService`

Rename [path_request_action_server.py](../src/pathfinder/src/pathfinder/ros/path_request_action_server.py) → `path_request_service.py`. Class becomes `PathRequestService`.

**Constructor** — same dependencies minus action specifics:
```python
def __init__(
    self,
    orchestrator: PathOrchestrator,
    tracker: PoseTracker,
    clients: dict[str, PathFollowActionClient],
    robot_locks: dict[str, threading.Lock],
    move_service_name: str,
    cancel_service_name: str,
) -> None:
```

**`start()`** — register two services:
```python
rospy.Service(self._move_name, MoveToNode, self._handle_move)
rospy.Service(self._cancel_name, CancelPath, self._handle_cancel)
```

**`_handle_move(request) -> MoveToNodeResponse`**:
1. Validate `request.robot_id` is in `self._clients`; unknown → `MoveToNodeResponse(False, "unknown robot ...")`.
2. `pose = self._tracker.wait_for(request.robot_id, timeout)`; `None` → `(False, "no pose for ...")`.
3. `node_ids = self._orchestrator.plan(request.robot_id, pose, request.target_node_id)`; catch `UnknownRobotError`, `NodeNotFoundError`, `NoPathError` → `(False, str(error))`.
4. With `self._robot_locks[request.robot_id]`:
   - `self._clients[request.robot_id].cancel()` (auto-preempt; idempotent when no goal active).
   - `ok = self._clients[request.robot_id].send(node_ids)`; `False` → `(False, "follow server unreachable")`.
5. Return `(True, f"dispatched {len(node_ids)} waypoints")`.

**`_handle_cancel(request) -> CancelPathResponse`**:
1. Validate `request.robot_id`; unknown → `(False, "unknown robot")`.
2. With `self._robot_locks[request.robot_id]`: `self._clients[ns].cancel()`.
3. Return `(True, "canceled")`.

Per-robot lock is retained: the `move` handler does a cancel-then-send pair that must be atomic relative to a concurrent `move` or `cancel` for the same robot, and rospy dispatches service callbacks from a thread pool.

Removed: daemon-thread spawning, goal-handle plumbing, `_on_goal_received`, `_on_cancel`, `_publish_feedback`, `_finish`, `_succeed`, `_abort`, `actionlib.ActionServer` import.

### 4. Wire `PathServerNode` to the new service

File: [path_server_node.py](../src/pathfinder/src/pathfinder/ros/path_server_node.py)

- Replace `PathRequestActionServer(...)` instantiation with `PathRequestService(...)`, passing service names `/path_server/move_to_node` and `/path_server/cancel_path`.
- Everything else (PoseTracker, `robot_locks` dict construction, odometry subscriptions) is unchanged.

### 5. Update `Client`

File: [client.py](../src/pathfinder/src/pathfinder/client/client.py)

- Drop `actionlib.SimpleActionClient` for MoveToNode. Use `rospy.ServiceProxy('/path_server/move_to_node', MoveToNode)`.
- `send_goal(robot_id, target_node_id)`: `wait_for_service`, call proxy with `(robot_id, target_node_id)`, log `success`/`message`, return `success`. No feedback callback.
- `cancel(robot_id)`: switch from `Publisher('/{ns}/stop', Empty).publish(Empty())` to `ServiceProxy('/path_server/cancel_path', CancelPath)(robot_id)`.
- Remove `_on_feedback` method and the `Empty` publisher entirely.
- Imports at module level (per project convention): drop `MoveToNodeAction/Goal/Feedback`, drop `std_msgs.msg.Empty` if otherwise unused; add `from pathfinder.srv import MoveToNode, CancelPath`.

### 6. Remove `/{robot_id}/stop` topic from `TurtleBotNode`

File: [turtlebot_node.py](../src/pathfinder/src/pathfinder/ros/turtlebot_node.py)

- Delete the `topic_stop` param assignment, the `rospy.Subscriber(self.topic_stop, Empty, self._on_stop)` line (~L61), and the `_on_stop` handler (~L111).
- Drop `from std_msgs.msg import Empty` if no other use remains.
- Confirm no launch param overrides reference `topic_stop`.

`CancelPath` is the sole cancellation path after this.

### 7. Tests

- [test/test_path_server.py](../src/pathfinder/test/test_path_server.py) — rewrite for `PathRequestService`:
  - Stub `pathfinder.srv` (`MoveToNode`, `MoveToNodeRequest`, `MoveToNodeResponse`, `CancelPath`, `CancelPathRequest`, `CancelPathResponse`) alongside the existing `pathfinder.msg` stubs.
  - Tests call `service._handle_move(request)` and `service._handle_cancel(request)` directly. No `FakeGoalHandle`, no daemon threads.
  - Cover: happy path (response.success True, `client.send` called with expected node_ids); pose timeout; `UnknownRobotError`; `NodeNotFoundError`; `NoPathError`; `client.send` returns False; **auto-preempt** (verify `client.cancel` is called before `client.send` on a fresh move); cancel handler calls `client.cancel`; unknown-robot guard on cancel.
  - Replace `FakePathFollowActionClient.dispatch(...)` with `.send(...)` and `.cancel()` recording.
- [test/test_path_follow_action_client.py](../src/pathfinder/test/test_path_follow_action_client.py) — rewrite for the new `send` / `cancel` API. Drop polling-loop, feedback-arrival, cancel-mid-poll cases. Keep the server-unavailable case (`send` returns False).
- [test/test_client.py](../src/pathfinder/test/test_client.py) — replace MoveToNode action stubs with a `ServiceProxy` stub that records calls; replace the `Empty` publication assertion with a `CancelPath` service-call assertion.
- [test/test_path_orchestrator.py](../src/pathfinder/test/test_path_orchestrator.py) — no change.
- CMakeLists.txt: while editing, add `test_path_orchestrator.py` and `test_path_follow_action_client.py` to the `catkin_add_nosetests` block (they exist but are not currently registered).

Per the project memory, `*Node` assembly classes are not unit-tested; `PathRequestService` is a pure DI-driven component (only `start()` touches rospy) and IS unit-testable, like the existing `PathRequestActionServer` is.

### 8. Build & verify

```bash
catkin_make --only-pkg-with-deps pathfinder
source devel/setup.zsh
catkin_make run_tests_pathfinder && catkin_test_results build/test_results
python3 src/pathfinder/test/test_path_server.py
python3 src/pathfinder/test/test_path_follow_action_client.py
python3 src/pathfinder/test/test_client.py
```

End-to-end (in container):
```bash
roslaunch pathfinder simulation.launch
# new shell
rosrun pathfinder client tb3_0 5      # returns ~immediately, robot starts moving
rosrun pathfinder client tb3_0 12     # mid-drive: should auto-preempt and head to 12
rosservice call /path_server/cancel_path "robot_id: 'tb3_0'"   # robot stops
```

## Critical files

| File | Change |
|------|--------|
| `src/pathfinder/srv/MoveToNode.srv` | **new** |
| `src/pathfinder/srv/CancelPath.srv` | **new** |
| `src/pathfinder/CMakeLists.txt` | swap action→srv entry; add 2 nosetests |
| `src/pathfinder/action/MoveToNode.action` | **delete** |
| `src/pathfinder/src/pathfinder/ros/path_request_service.py` | rename from `path_request_action_server.py`, rewrite as service handler |
| `src/pathfinder/src/pathfinder/ros/path_follow_action_client.py` | simplify to `send` / `cancel` |
| `src/pathfinder/src/pathfinder/ros/path_server_node.py` | wire `PathRequestService` |
| `src/pathfinder/src/pathfinder/ros/turtlebot_node.py` | remove `/stop` subscriber + handler |
| `src/pathfinder/src/pathfinder/client/client.py` | service proxies; drop Empty publisher and feedback cb |
| `src/pathfinder/test/test_path_server.py` | rewrite for service handlers |
| `src/pathfinder/test/test_path_follow_action_client.py` | rewrite for `send`/`cancel` |
| `src/pathfinder/test/test_client.py` | service-proxy stub; cancel-service assertion |

## Reused existing code

- `PathOrchestrator.plan` (`planning/path_orchestrator.py`) — unchanged.
- `PoseTracker.wait_for` — unchanged.
- Per-robot `threading.Lock` dict in `PathServerNode` — same shape, repurposed to guard cancel-then-send.
- Existing `sys.modules` stub pattern in tests — extend with `pathfinder.srv` stubs alongside the existing `pathfinder.msg` ones.

## Notes

- Auto-preempt semantics: a second `MoveToNode` for the same robot calls `cancel()` on the action client and immediately calls `send()` with the new goal. `SimpleActionClient` cancels asynchronously, so the executor may briefly see a cancel followed by a new goal; that's the same behavior `actionlib`'s preempt path produces today.
- The `MoveToNode` service returns after dispatch, not after completion. The user explicitly chose this; the trade-off is that the user no longer learns end-of-drive success/failure from a single client invocation. If they later want completion notification, that's a follow-up (e.g. an `arrival` topic per robot) and out of scope here.
