# Plan: Refactor `planning/` Module

## Current Shape

The `planning/` module has 2 files after the recent cleanup:

- **`path_planner.py`** - pure A*, no `rospy` ✅ (already layer-pure)
- **`path_server.py`** - ROS god class: subscribers + action server + domain orchestration + action client + feedback translation

`path_server.py` violates the layered architecture rule (files importing `rospy` at module level should be thin adapters). It currently mixes 5 concerns:

| Concern | Methods | Layer |
|---|---|---|
| Odometry tracking | `_on_odom`, `_wait_for_robot_state`, `_state_available` | domain (threading only) |
| Action-server lifecycle | `_on_goal_received`, `_on_cancel`, `_execute` | ROS adapter |
| Start-node + path planning | `_resolve_start_node`, `_plan_node_ids` | domain |
| Action-client dispatch + feedback poll | `_dispatch_to_executor`, `_publish_feedback` | ROS adapter |
| Result translation | `_succeed`, `_abort`, `_finish` | ROS adapter |

The first and third rows are domain logic trapped inside a ROS file. They become ~70 % of the testable behavior once extracted, none of which needs `_install_ros_stubs`.

## Target Architecture

```
planning/                          ← pure domain (no rospy, no threading)
  path_planner.py                  ← Protocol only (algorithm interface)
  a_star_planner.py                ← NEW: current concrete implementation, renamed
  path_orchestrator.py             ← NEW: (pose, target) → list[int]; raises domain errors

ros/                               ← ROS-side classes (rospy or ROS-driven threading)
  pose_tracker.py                  ← NEW: dict + Condition; update / wait_for
  follow_path_client.py            ← NEW: action client + feedback poll
  path_server_node.py              ← MOVED + thinned: wires the above + ROS-result translation

scripts/path_server                ← thin wire-up: build the pieces, spin
```

This mirrors the existing `ros/turtlebot_node.py` + `robot/turtlebot.py` split that PR #14 established for the executor side.

**Why extract two classes from the adapter, not just one.** A class with a long list of private methods is itself a SRP smell: each cluster of related privates is a sub-responsibility waiting to be named. After only extracting the orchestrator, `PathServerNode` would still hold ~10 privates split across two distinct concerns - pose tracking (dict + Condition + `_on_odom` + `_wait_for_pose`) and action-client dispatch (client + feedback poll + cancel polling). Both deserve their own class.

**Threading lives in the adapter, not the domain.** The pose dict and `threading.Condition` exist only because rospy's subscriber thread and the goal-execution thread are different - a ROS plumbing concern. `PoseTracker` therefore lives in `ros/` even though it never imports `rospy`; the threading is ROS-driven, not domain logic. The orchestrator stays single-threaded and takes the pose as a parameter.

## Steps

### Step 1 - Extract `PathOrchestrator` (pure)

Single-method class that resolves the start node from a given pose and plans a route. No threading, no `rospy`, no I/O.

```python
# planning/path_orchestrator.py
class PathOrchestrator:
    """Resolves start node from a pose, plans a route, returns waypoint ids."""

    def __init__(
        self,
        graph: Graph,
        planner: PathPlanner,
        known_robots: Iterable[str],
    ) -> None: ...

    def plan(self, robot_id: str, current_pose: Pose2D, target_node_id: int) -> list[int]: ...
```

Raises explicit domain exceptions (inline in `path_orchestrator.py` until they outgrow it):

- `UnknownRobotError`
- `NodeNotFoundError` (wraps `KeyError` from `Graph.get_node`)
- `NoPathError` (wraps `ValueError` from `PathPlanner.plan`)

`_resolve_start_node` and `_plan_node_ids` are deleted from the adapter.

**Tests:** new `test_path_orchestrator.py`, no ROS stubs, no threading.

### Step 2 - Extract `PoseTracker`

Owns the pose dict + Condition that today lives in `PathServer`.

```python
# ros/pose_tracker.py
class PoseTracker:
    """Latest pose per robot, with a wait until the first one arrives."""

    def __init__(self, timeout_sec: float = 1.0) -> None: ...
    def update(self, robot_id: str, pose: Pose2D) -> None: ...
    def wait_for(self, robot_id: str) -> Pose2D | None: ...
```

**Why it lives in `ros/`:** no `rospy` import, but the threading exists because rospy's callback thread and the goal-execution thread are different. The lifetime and concurrency model are ROS-driven, so it sits next to other ROS-side classes.

**Effect on adapter:** `_on_odom` is one line (`self._tracker.update(ns, build_pose(message))`); `_wait_for_pose`, `_state_available`, `_robot_states`, and the related fields disappear.

**Tests:** new `test_pose_tracker.py`, stdlib `threading` only.

### Step 3 - Extract `FollowPathClient`

Wraps `actionlib.SimpleActionClient` plus the feedback-poll + preempt loop that today lives in `_dispatch_to_executor`.

```python
# ros/follow_path_client.py
class FollowPathClient:
    """Wraps an actionlib client; runs a synchronous dispatch with feedback + cancel callbacks."""

    def __init__(self, action_namespace: str) -> None: ...
    def dispatch(
        self,
        node_ids: list[int],
        on_feedback: Callable[[FollowPathFeedback], None],
        is_canceled: Callable[[], bool],
    ) -> FollowPathResult | None: ...
```

**Why now (revised from "optional"):** even though `_dispatch_to_executor` is only ~25 lines, leaving it inline keeps three more privates in `PathServerNode` (`_dispatch_to_executor`, the change-detection bookkeeping, the latest-feedback closure). Pulling them out is what makes the adapter readable.

### Step 4 - Move `path_server.py` → `ros/path_server_node.py`

After Steps 1-3 the adapter is small: subscribers wire `Odometry → PoseTracker.update`; the action server `goal_cb` spawns a thread that runs `tracker.wait_for → orchestrator.plan → client.dispatch`; result translation (`_finish` / `_succeed` / `_abort`) stays here because it's a ROS message-shape concern. Final private count drops to ~4, all of them narrow ROS-translation helpers. Update imports in `scripts/path_server` and `test_path_server.py`.

### Step 5 - Define `PathPlanner` interface; rename concrete class

Promote `PathPlanner` to a Protocol that any algorithm can satisfy. The current concrete class becomes `AStarPlanner`. The orchestrator depends on the protocol, not the concrete class - a clean swap point for tests (fake planner) and future algorithms.

```python
# planning/path_planner.py   (interface only)
from typing import Protocol

class PathPlanner(Protocol):
    """Plans an ordered list of waypoint Nodes between two graph nodes."""
    def plan(self, start: Node, goal: Node) -> list[Node]: ...
```

```python
# planning/a_star_planner.py
class AStarPlanner:
    """A* over a Graph using straight-line distance as the heuristic."""
    def __init__(self, graph: Graph) -> None: ...
    def plan(self, start: Node, goal: Node) -> list[Node]: ...
```

`PathOrchestrator.__init__` types the dependency as `planner: PathPlanner`.

**Order:** can run before, between, or after Steps 1-4. Suggest running it first if Step 1 hasn't started yet (so the orchestrator depends on the Protocol from day one), otherwise tack it on at the end.

**On adding `DijkstraPlanner` now:** I'd defer. A* with `h(n) = 0` *is* Dijkstra; for a ~10-node Euclidean graph the existing heuristic strictly dominates, so a Dijkstra class today would be dead code with dead tests. The Protocol alone gives us the swap point - a second implementation can land in a focused follow-up PR when a real motivation appears (heuristic-free fallback, multi-source shortest paths, benchmarking).

## Wire-up After Refactor

```python
# scripts/path_server
graph        = Graph.load_from_yaml(graph_file)
planner      = AStarPlanner(graph)
orchestrator = PathOrchestrator(graph, planner, known_robots=robot_ids)
tracker      = PoseTracker()
clients      = {rid: FollowPathClient(robot_action_namespaces[rid]) for rid in robot_ids}

server = PathServerNode(
    orchestrator,
    tracker,
    clients,
    robot_odom_topics=robot_odom_topics,
)
server.start()
rospy.spin()
```

Top-to-bottom: build the pieces, hand them to the adapter, spin. No business logic in the script.

## Test Impact

| Module | Test file | ROS stubs? | New / existing |
|---|---|---|---|
| `path_planner.py` (Protocol) | n/a (interface only) | n/a | shrunk to interface |
| `a_star_planner.py` | `test_a_star_planner.py` | no | NEW (move + light tests) |
| `path_orchestrator.py` | `test_path_orchestrator.py` | no | NEW |
| `ros/pose_tracker.py` | `test_pose_tracker.py` | no | NEW |
| `ros/follow_path_client.py` | `test_follow_path_client.py` | yes (small) | NEW |
| `ros/path_server_node.py` | `test_path_server.py` | yes | shrinks - pose-wait coverage moves to `test_pose_tracker.py` |

After this refactor the orchestrator and pose-tracker tests need zero ROS stubs. The remaining ROS-stubbed test is the action-client wrapper plus a thin adapter integration test.

## Open Questions

1. **Errors module.** Inline error classes in `path_orchestrator.py`, or separate `planning/errors.py`? Inline while count stays small (≤3 today).
2. **`PoseTracker` location.** It uses no `rospy`, so it could go in `planning/`. I'm putting it in `ros/` because its threading model is ROS-driven and the tracker has no purpose outside that context. Reasonable to revisit if a non-ROS consumer appears.
