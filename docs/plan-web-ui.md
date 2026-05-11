# Plan: Web UI for live robot monitoring + control

## Context

Today the only way to drive a robot is `rosrun pathfinder client <robot_id> <target_node_id>` and the only way to inspect state is `rostopic echo`. We want a browser dashboard that:

1. Renders the pathfinding graph (nodes + edges) and shows each robot's live position on it.
2. Lets the user click a robot, then click a target node, to issue a `MoveToNode` command.

`rosbridge_websocket` is already wired in `src/pathfinder/launch/robots.launch:19-21` on port `9090`, so the browser can talk to ROS directly via [roslibjs](https://github.com/RobotWebTools/roslibjs). What's missing on the ROS side is read-access to the graph and robots list (today both are server-only YAML); the live pose feed at `/<ns>/pose` already exists in `src/pathfinder/src/pathfinder/ros/turtlebot_node.py:43,110`.

Decisions confirmed with the user:

- Frontend: **React + Vite + roslibjs**.
- Static data (graph/robots): **two new ROS services** on the central server.
- Rename: **`PathRequestService` → `FleetService`** (the ROS API surface; URL prefix moves to `/fleet/...`). `PathOrchestrator` keeps its pure-domain role; `PathServerNode` keeps its name.
- Robot status (idle/moving/paused): **out of scope for v1**, pose-only.
- Interaction model: **click a robot, then click a target node**.
- Hosting: a **separate `web-ui/docker-compose.yml`** that the host runs directly (independent of `docker/`).

## Changes

### 1. ROS: rename `PathRequestService` → `FleetService` (full propagation)

The "complete renames" rule means every reference moves in the same commit and the build passes clean. The chosen name is role-based: this class is the public API surface for managing the fleet (read graph/robots, dispatch moves, cancel them). The URL prefix moves to `/fleet/...` and reads naturally for both reads (`/fleet/get_graph`) and writes (`/fleet/move_to_node`). The launch *node* keeps the name `path_server`; only the class, file, and service namespace move.

| File | Change |
|---|---|
| `src/pathfinder/src/pathfinder/ros/path_request_service.py` | Rename file → `fleet_service.py`. Rename class to `FleetService`. Constants change to `'/fleet/move_to_node'` and `'/fleet/cancel_path'`. |
| `src/pathfinder/src/pathfinder/ros/path_server_node.py:9,28` | Update import + constructor call. |
| `src/pathfinder/test/test_path_server.py` | Rename file → `test_fleet_service.py`. Update class references and any service-name assertions. |
| `src/pathfinder/CMakeLists.txt:61` | Update `catkin_add_nosetests` entry to the new test filename. |
| `src/pathfinder/src/pathfinder/client/client.py` | Grep + update any reference to `/path_server/move_to_node`. |
| `src/pathfinder/test/test_client.py` | Same. |
| `README.md` | Update service path references. The "central server" prose framing can stay; only the URL paths and class name change. |

### 2. ROS: two new services on `FleetService`

Add two srv files. JSON-string responses are deliberate: the only consumer is the web UI which speaks JSON natively, and going JSON keeps us from inventing new `.msg` types just to mirror the YAML shape.

`src/pathfinder/srv/GetGraph.srv`
```
---
string graph_json
```
Response payload: `{"nodes":[{"id":int,"x":float,"y":float},...], "edges":[{"from":int,"to":int},...]}`.

`src/pathfinder/srv/GetRobots.srv`
```
---
string robots_json
```
Response payload: `{"robots":[{"id":"tb3_01","namespace":"tb3_01","start_node":int},...]}`.

`src/pathfinder/CMakeLists.txt:21-25` — add both files to `add_service_files`.

`src/pathfinder/src/pathfinder/ros/fleet_service.py`:
- Constructor gains a `graph: Graph` parameter (already loaded once in `PathServerNode`; just pass it in).
- New constants `GRAPH_SERVICE_NAME = '/fleet/get_graph'`, `ROBOTS_SERVICE_NAME = '/fleet/get_robots'`.
- `start()` registers both new `rospy.Service` handlers.
- New private handlers walk `self._graph.all_nodes()` / `self._graph.all_edges()` and the existing `self._robots` dict, serialise via `json.dumps`, and return the response.

`src/pathfinder/src/pathfinder/ros/path_server_node.py:28` — pass `graph` into the new constructor signature.

### 3. New `web-ui/` folder

```
web-ui/
  docker-compose.yml       # node service, network_mode: host, runs `npm run dev`
  Dockerfile               # FROM node:20-alpine; WORKDIR /app
  package.json             # react, react-dom, roslib, vite, @vitejs/plugin-react
  vite.config.js           # server.host=true, server.port=5173
  index.html
  .env.example             # VITE_ROSBRIDGE_URL=ws://localhost:9090
  .gitignore               # node_modules, dist, .env
  src/
    main.jsx
    App.jsx                # holds selectedRobotId state + layout
    components/
      GraphCanvas.jsx      # SVG with viewBox derived from graph bounds; renders edges, NodeMarkers, RobotDots
      NodeMarker.jsx       # circle + id label; onClick triggers move when a robot is selected
      RobotDot.jsx         # circle (with heading triangle); onClick selects/deselects
    ros/
      rosClient.js         # singleton ROSLIB.Ros connection
      fetchGraph.js        # one-shot get_graph service call → JSON.parse
      fetchRobots.js       # one-shot get_robots service call → JSON.parse
      subscribePose.js     # roslibjs Topic on /<ns>/pose, returns unsubscribe()
      moveToNode.js        # service call to /fleet/move_to_node
```

Behaviour:

- On mount, call `fetchGraph` + `fetchRobots` once. Compute SVG `viewBox` from node min/max with a small padding. Flip Y so the on-screen layout matches "y-up" intuition.
- For each robot returned by `fetchRobots`, subscribe to `/<namespace>/pose` and store the latest `{x, y, theta}` in React state keyed by robot id. The robot dot re-renders on each message.
- Click a `RobotDot` → `setSelectedRobotId(id)`; visual highlight (e.g. ring). Click the same dot or press `Escape` → clear.
- Click a `NodeMarker` while a robot is selected → call `moveToNode(selectedRobotId, nodeId)` and surface the service `success`/`message` as a transient banner. Clear selection after dispatch.
- Robots render as small filled circles with a heading tick using `pose.theta`.

### 4. `web-ui/docker-compose.yml`

Mirrors the existing `docker/docker-compose.yml` style:

- `network_mode: host` so the container can reach `ws://localhost:9090` without extra hostname mapping. (Same pattern as the noetic service.)
- Bind mount `./` → `/app`; named volume for `node_modules` to keep host clean.
- Command runs `npm install` (idempotent) then `npm run dev`.
- Vite is configured with `server.host=true` and `server.port=5173`; with host networking, the dev server is reachable at `http://localhost:5173`.

Browser only ever talks to ROS via `ws://localhost:9090` (rosbridge) — no other backend.

### 5. Tests

- Extend `test_fleet_service.py` (renamed in step 1) with two cases: `_handle_get_graph` returns parseable JSON whose `nodes`/`edges` match a fake `Graph`; `_handle_get_robots` returns parseable JSON listing the injected robots. Use the existing rospy-stub pattern at `test_path_server.py` (the `*_install_ros_stubs` helpers) — `FleetService` is a service handler, not a `*Node`, so unit tests with mocked rospy are still appropriate per the project's testing policy.
- No frontend test framework in v1 (per "no speculative abstractions"). Manual verification covers the click flows; the React surface area is small enough to verify by exercising it.

## Critical files (modify or create)

Modify:
- `src/pathfinder/src/pathfinder/ros/path_request_service.py` (rename + new handlers + new constructor arg)
- `src/pathfinder/src/pathfinder/ros/path_server_node.py` (import + pass `graph`)
- `src/pathfinder/test/test_path_server.py` (rename + extend)
- `src/pathfinder/CMakeLists.txt` (test filename + new srv files)
- `src/pathfinder/src/pathfinder/client/client.py` (service path)
- `src/pathfinder/test/test_client.py` (service path)
- `README.md` (service paths + new web-ui section)

Create:
- `src/pathfinder/srv/GetGraph.srv`
- `src/pathfinder/srv/GetRobots.srv`
- `web-ui/` (full tree above)

## Verification

1. **Build clean.** Inside the noetic container:
   ```
   catkin_make --only-pkg-with-deps pathfinder && source devel/setup.zsh
   ```
2. **Unit tests green.**
   ```
   catkin_make run_tests_pathfinder && catkin_test_results build/test_results
   ```
3. **Service paths exposed.** With `roslaunch pathfinder simulation.launch` running:
   ```
   rosservice list | grep ^/fleet/
   # expect: /fleet/get_graph /fleet/get_robots
   #         /fleet/move_to_node /fleet/cancel_path
   rosservice call /fleet/get_graph
   rosservice call /fleet/get_robots
   ```
4. **CLI client still works (proves the rename propagated).**
   ```
   rosrun pathfinder client tb3_01 5
   ```
5. **Web UI end-to-end.** From the host (outside any container):
   ```
   cd web-ui
   cp .env.example .env
   docker compose up
   ```
   Open `http://localhost:5173`. Confirm:
   - Graph (9 nodes, 12 edges from `config/graph.yaml`) renders with sensible spacing.
   - Both robots appear at their `start_node` positions and update live as Gazebo moves them.
   - Click `tb3_01` → highlight ring appears. Click node `5` → robot drives 4 → ... → 5; the dot tracks the motion. Banner shows the service response.
   - Click `tb3_05` and pick a different node while the first move is in flight; both robots move concurrently (path server already supports this).
6. **Pre-commit walk** of project CLAUDE.md and the memory rules (no shortened names, public-method docstrings, type hints, full rename propagation, no placeholder code) before declaring done.
