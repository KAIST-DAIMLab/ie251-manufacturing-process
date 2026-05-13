# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repo shape

ROS 1 Noetic catkin workspace (Python, rospy). The workspace root is the catkin root: `src/`, `build/`, `devel/` live here. The single first-party package is [src/pathfinder/](src/pathfinder/). [src/turtlebot3/](src/turtlebot3/) is the vendored upstream TurtleBot3 stack — treat it as read-only.

All development happens inside the `noetic` Docker container defined in [docker/docker-compose.yml](docker/docker-compose.yml). The host repo is bind-mounted at `/home/ubuntu/workspace`, so edits on the host and inside the container share the same files.

## Common commands

All commands assume you're inside the container (`sudo docker exec -it noetic zsh`) and at the workspace root.

```bash
# First-time setup (the /etc/hosts line silences a rospy hostname warning)
echo "127.0.0.1 noetic" | sudo tee -a /etc/hosts
catkin_make --only-pkg-with-deps pathfinder
source devel/setup.zsh

# After editing .action / .msg / CMakeLists.txt
catkin_make --only-pkg-with-deps pathfinder && source devel/setup.zsh

# Pure-Python edits under src/pathfinder/src/ require no rebuild —
# scripts/* import from the source tree directly via setup.py / catkin_python_setup.

# Run the full system (Gazebo + 2 robots + path_server + 2 executors)
roslaunch pathfinder simulation.launch

# Send a goal (in a second sourced shell)
rosrun pathfinder client tb3_0 5

# Run all tests (uses catkin_add_nosetests entries in CMakeLists.txt)
catkin_make run_tests_pathfinder && catkin_test_results build/test_results

# Run a single test file directly (fast — no catkin needed; tests stub rospy)
python3 src/pathfinder/test/test_motion.py
python3 -m unittest src.pathfinder.test.test_path_server
```

The unit tests deliberately do **not** require a roscore. Each test that touches a module which imports `rospy` installs fake `rospy` / `actionlib` / `pathfinder.msg` modules into `sys.modules` before importing the code under test (see `_install_ros_stubs` in [test_path_server.py](src/pathfinder/test/test_path_server.py), [test_turtlebot_node.py](src/pathfinder/test/test_turtlebot_node.py), [test_gazebo_control.py](src/pathfinder/test/test_gazebo_control.py)). Follow the same pattern when adding tests for any module that lives in a "ROS-touching" layer.

## Architecture

Three-tier action pipeline. The user talks to the path server; the path server talks to per-robot executors; executors drive Gazebo via `cmd_vel`/`odom`.

```
client ──MoveToNode──▶ path_server ──FollowPath──▶ tb3_X executor ──cmd_vel──▶ Gazebo
                                │                              ▲
                                │                              │ /tb3_X/emergency_stop
                                └── CollisionMonitor (10 Hz) ──┘
```

- **`MoveToNode`** ([action/MoveToNode.action](src/pathfinder/action/MoveToNode.action)) — `(robot_id, target_node_id) → (success, message)`. Goal/feedback for the user.
- **`FollowPath`** ([action/FollowPath.action](src/pathfinder/action/FollowPath.action)) — `(node_ids[]) → (success, message)`. Server → executor.
- **`RobotState`** msg ([msg/RobotState.msg](src/pathfinder/msg/RobotState.msg)) — published at 10 Hz on `/tb3_X/robot_state`. Both the path_server (for start-node resolution) and the CollisionMonitor (for prediction) subscribe.

### Path server — concurrency

[planning/path_server.py](src/pathfinder/src/pathfinder/planning/path_server.py) uses `actionlib.ActionServer` (not `SimpleActionServer`) so it can handle goals for `tb3_0` and `tb3_1` in parallel. Each goal runs on its own thread; `_robot_locks[ns]` serializes goals targeting the same robot. Start-node resolution waits up to `first_state_timeout_sec` on a `threading.Condition` for the first `RobotState` message — don't replace this with a busy-wait or a fixed sleep.

A* lives in [planning/path_planner.py](src/pathfinder/src/pathfinder/planning/path_planner.py) using straight-line distance as the heuristic; the graph is loaded once at startup from [config/graph.yaml](src/pathfinder/config/graph.yaml).

### Robot side — pure-logic / ROS-adapter split

The `robot/` package is intentionally split so the domain logic is unit-testable without ROS:

| File | Imports `rospy`? | Role |
|------|------------------|------|
| [robot/motion.py](src/pathfinder/src/pathfinder/robot/motion.py) | no | Pure proportional controller (`compute_drive`) + `PathFollower` |
| [robot/turtlebot.py](src/pathfinder/src/pathfinder/robot/turtlebot.py) | no | State container + thread-safe accessors; no I/O |
| [robot/turtlebot_node.py](src/pathfinder/src/pathfinder/robot/turtlebot_node.py) | yes | ROS adapter: subscribers, publishers, timer, `FollowPath` action server |
| [scripts/robot_executor_node](src/pathfinder/scripts/robot_executor_node) | yes | `init_node` + wire-up + `spin` |

When extending the robot: keep new motion math in `motion.py` (no rospy); keep new state in `turtlebot.py` behind the existing lock; keep new topics/timers in `turtlebot_node.py`. The lazy `from pathfinder.msg import …` inside methods exists to break a circular import; preserve it when adding new message references in the adapter layer.

### Safety — emergency stop

[safety/collision_monitor.py](src/pathfinder/src/pathfinder/safety/collision_monitor.py) ticks at `check_rate_hz` (default 10 Hz). For each pair of robots it asks [safety/linear_predictor.py](src/pathfinder/src/pathfinder/safety/linear_predictor.py) to extrapolate poses forward over `horizon` seconds in `time_step` increments using the unicycle model and fires a `std_msgs/Empty` on `/tb3_X/emergency_stop` if any predicted distance drops below `safety_radius`.

The stop is **latched on the executor side** ([turtlebot.py:103-106](src/pathfinder/src/pathfinder/robot/turtlebot.py#L103-L106)): `request_stop()` sets `_stop_requested = True` and the executor aborts the current `FollowPath` goal. The flag clears only when a *new* `start_path` runs — i.e. the user must send a fresh `MoveToNode` goal to resume. This is intentional, not a bug.

## Configuration

Two files in [config/](src/pathfinder/config/):

- **`graph.yaml`** — nodes (id, x, y) and undirected edges. Loaded by both `path_server` (planning) and each executor (waypoint lookup in `FollowPath` callback).
- **`params.yaml`** — tunables for path server (`safety_radius`, `horizon`) and executor (`k_lin`, `k_ang`, `arrival_tol`, ...).

The launch files currently inline these as `<param>` tags rather than loading `params.yaml`. If you change a default, update both [launch/simulation.launch](src/pathfinder/launch/simulation.launch) and [launch/robots.launch](src/pathfinder/launch/robots.launch).

## Code quality rules (binding)

These exist because past output drifted from them and required dozens of follow-up refactor commits. Treat them as pre-commit constraints, not aspirations.

- **Framework at the edges.** Anything importing `rospy` belongs only in [src/pathfinder/src/pathfinder/ros/](src/pathfinder/src/pathfinder/ros/) and acts as a thin adapter (subscribers, publishers, action servers, lifecycle). Domain logic must run with `rospy` removed and be unit-testable without a roscore. If a new class needs `rospy` AND has logic worth testing, split it.
- **One responsibility per class.** Pick from {I/O adapter, domain logic, state container, coordinator, lifecycle owner}. If a class spans two of these, split it before the first commit, not after a god-class refactor request.
- **Dependency injection over construction-inside-class.** Classes accept their collaborators (planner, graph, controller, publisher) as constructor arguments. Wiring happens in `*Node` adapters or `scripts/`.
- **Every numeric tunable lives in `robots.yaml` or `params.yaml`** and is threaded YAML → dataclass (`MotionConfig`/`ObstacleConfig` in [world/robot.py](src/pathfinder/src/pathfinder/world/robot.py)) → script (`scripts/robot`) → node constructor in the same change. Hardcoded literals in constructors are bugs unless they are mathematical constants.
- **No speculative abstractions.** Don't add a class, lock, dict, parameter, or layer until the *current* code requires it. Removing a wrong abstraction costs more than adding it when the need actually appears.
- **No placeholder code.** If a class has no real implementation, no callers, or duplicates existing functionality, it doesn't ship. Diagrams describe what *exists*, not what was once planned.
- **Plan mechanism-words are suggestions, not commands.** When a plan prescribes a *how* (e.g. "read field-by-field", "use a dict here"), evaluate it against the architecture rules above before implementing. If the prescribed mechanism violates a rule, implement the goal via the rule-compliant approach instead. The architecture rules win.
- **"Whose job is this?" check.** Before placing logic in a file, ask which responsibility bucket it belongs to. Serialization of a domain object belongs on that object, not on the caller. If the answer is obvious and conflicts with where you were about to put it, move it.

## Conventions

- Robot namespaces are `tb3_0` and `tb3_1` everywhere (topics, action names, params). The README's collision-avoidance example only fires when both clients run within ~1 s of each other.
- Spawn poses are driven by `config/robots.yaml` (`start_station`) and resolved against station entries in `config/graph.yaml` at runtime by `spawn_simulation_robots`. To move a robot's start position, change `start_station` in `robots.yaml` to a valid graph station number.
- Python 3.8+ syntax is fine (Noetic ships Python 3.8); `from __future__ import annotations` is used throughout to allow `list[int]`-style hints.
- `scripts/` executables are extension-less and are listed in `install(PROGRAMS …)` in CMakeLists.txt — when adding a new node, add it there or `rosrun` won't find it.
