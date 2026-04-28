# IE251 Manufacturing Process — TurtleBot3 Graph Navigation

A ROS Noetic system where a TurtleBot3 Waffle navigates a user-defined node graph in Gazebo simulation. Send a target node ID; the robot plans a Dijkstra path and drives waypoint-by-waypoint, publishing its position continuously.

## Overview

```
User sends goal (node ID)
        │
        ▼
  nav_server.py          ← NavigateToNode action server
  ├─ finds nearest node (via /odom)
  ├─ Dijkstra path plan
  └─ drives node-by-node via GoToGoal
        │
        ▼
  /cmd_vel  ──►  TurtleBot3 Waffle (Gazebo)
                        │
                        ▼
  /odom  ──►  position_reporter.py  ──►  /node_status
```

**Workspace:** 6 m × 3 m grid — 18 nodes (A1–A6, B1–B6, C1–C6) at 1 m spacing.

```
C1 — C2 — C3 — C4 — C5 — C6
|    |    |    |    |    |
B1 — B2 — B3 — B4 — B5 — B6
|    |    |    |    |    |
A1 — A2 — A3 — A4 — A5 — A6
↑
robot spawns here (0.5, 0.5)
```

---

## Prerequisites

### Host machine

- Docker + Docker Compose
- An X server running (any Linux desktop, or XQuartz on macOS)

### Install (once)

```bash
# Allow Docker containers to open GUI windows on your display
xhost +local:docker

# Create xauth file for the container
touch /tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
```

> **macOS:** Use XQuartz. Set `DISPLAY=host.docker.internal:0` and enable "Allow connections from network clients" in XQuartz preferences.

---

## Quick Start

### 1. Start the Docker container

```bash
cd /path/to/ie251-manufacturing-process
docker compose -f docker/docker-compose.yml up -d
```

### 2. Open terminal tabs in the container

Open 3 separate terminals and attach to the running container in each:

```bash
docker exec -it noetic zsh
```

### 3. Build the workspace (first time only)

In **any one** of the terminals:

```bash
cd ~/repositories/kaist/ie251-manufacturing-process
source /opt/ros/noetic/setup.zsh
catkin_make
```

---

## Running the Simulation

### Terminal 1 — Launch Gazebo

```bash
source /opt/ros/noetic/setup.zsh
cd ~/repositories/kaist/ie251-manufacturing-process
source devel/setup.zsh
roslaunch tb3_graph_nav simulation.launch
```

Gazebo opens with a TurtleBot3 Waffle at node A1 (0.5, 0.5). Wait until you see the robot in the scene before proceeding.

### Terminal 2 — Launch navigation stack

```bash
source /opt/ros/noetic/setup.zsh
cd ~/repositories/kaist/ie251-manufacturing-process
source devel/setup.zsh
roslaunch tb3_graph_nav graph_nav.launch
```

Expected output:
```
[nav_server]: nav_server ready — 18 nodes loaded from .../graph.yaml
[position_reporter]: position_reporter started — 18 nodes from .../graph.yaml
```

### Terminal 3 — Send a navigation goal

```bash
source /opt/ros/noetic/setup.zsh
cd ~/repositories/kaist/ie251-manufacturing-process
source devel/setup.zsh

# Navigate to node C6 (far corner)
python3 src/tb3_graph_nav/scripts/test_navigate.py C6
```

The robot plans A1 → C6, drives through each intermediate node, and logs progress:

```
[INFO] Goal: navigate to 'C6'
[INFO] Path: A1 -> A2 -> A3 -> A4 -> A5 -> A6 -> B6 -> C6
[INFO] Driving to A2 (1.50, 0.50)
[INFO] Reached A2
[INFO] Driving to A3 (2.50, 0.50)
...
[INFO] Goal succeeded: reached 'C6'
```

### Monitor current position (any terminal)

```bash
rostopic echo /node_status
```

```
nearest_node_id: "A2"
x: 1.482
y: 0.501
distance_to_node: 0.018
```

---

## Sending Goals — All Options

### Option A: Test script

```bash
python3 src/tb3_graph_nav/scripts/test_navigate.py <NODE_ID>

# Examples
python3 src/tb3_graph_nav/scripts/test_navigate.py B3
python3 src/tb3_graph_nav/scripts/test_navigate.py C6
python3 src/tb3_graph_nav/scripts/test_navigate.py A6
```

### Option B: Interactive GUI (axclient)

```bash
rosrun actionlib axclient.py /navigate_to_node
```

Set `target_node_id` to any node ID and click **Send Goal**.

### Option C: Command line (rostopic)

```bash
rostopic pub /navigate_to_node/goal tb3_graph_nav/NavigateToNodeActionGoal \
  "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
goal_id:
  stamp: {secs: 0, nsecs: 0}
  id: ''
goal:
  target_node_id: 'B3'" --once
```

### Cancel a goal in flight

```bash
rostopic pub /navigate_to_node/cancel actionlib_msgs/GoalID "{}" --once
```

---

## Node Graph Reference

| Node | x (m) | y (m) | Neighbors |
|------|--------|--------|-----------|
| A1   | 0.5    | 0.5    | A2, B1    |
| A2   | 1.5    | 0.5    | A1, A3, B2 |
| A3   | 2.5    | 0.5    | A2, A4, B3 |
| A4   | 3.5    | 0.5    | A3, A5, B4 |
| A5   | 4.5    | 0.5    | A4, A6, B5 |
| A6   | 5.5    | 0.5    | A5, B6    |
| B1   | 0.5    | 1.5    | A1, B2, C1 |
| B2   | 1.5    | 1.5    | B1, B3, A2, C2 |
| B3   | 2.5    | 1.5    | B2, B4, A3, C3 |
| B4   | 3.5    | 1.5    | B3, B5, A4, C4 |
| B5   | 4.5    | 1.5    | B4, B6, A5, C5 |
| B6   | 5.5    | 1.5    | B5, A6, C6 |
| C1   | 0.5    | 2.5    | B1, C2    |
| C2   | 1.5    | 2.5    | C1, C3, B2 |
| C3   | 2.5    | 2.5    | C2, C4, B3 |
| C4   | 3.5    | 2.5    | C3, C5, B4 |
| C5   | 4.5    | 2.5    | C4, C6, B5 |
| C6   | 5.5    | 2.5    | C5, B6    |

To modify the graph, edit [src/tb3_graph_nav/config/graph.yaml](src/tb3_graph_nav/config/graph.yaml).

---

## Running Tests

Pure Python tests (no ROS or Gazebo needed):

```bash
cd src/tb3_graph_nav
python3 -m pytest tests/ -v
```

Expected: **20 passed**

---

## Package Structure

```
src/tb3_graph_nav/
├── src/tb3_graph_nav/
│   ├── graph.py          # Graph + Node data model, YAML loader
│   ├── planner.py        # Dijkstra shortest path
│   └── go_to_goal.py     # Proportional heading controller (library)
├── scripts/
│   ├── nav_server.py     # NavigateToNode action server (ROS node)
│   ├── position_reporter.py  # Publishes /node_status (ROS node)
│   └── test_navigate.py  # Quick integration test client
├── msg/NodeStatus.msg
├── action/NavigateToNode.action
├── config/graph.yaml     # 18-node 6×3 grid
└── launch/
    ├── simulation.launch # Gazebo + TurtleBot3 Waffle
    └── graph_nav.launch  # nav_server + position_reporter
```

---

## Troubleshooting

**Gazebo window doesn't open**
```bash
# On host, re-run:
xhost +local:docker
touch /tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
```

**`[ERROR] Failed to load model 'waffle'`**
The `TURTLEBOT3_MODEL` env var is already set in `simulation.launch`. If it still fails:
```bash
export TURTLEBOT3_MODEL=waffle
roslaunch tb3_graph_nav simulation.launch
```

**`nav_server` can't find the graph YAML**
```bash
# Rebuild and re-source:
catkin_make && source devel/setup.zsh
```

**Robot doesn't move / drifts off course**
Odometry drift is normal in Gazebo over long runs. Restart the simulation and try a shorter path first (e.g. A1 → A3).

**`rospack find tb3_graph_nav` fails**
Make sure you sourced the devel overlay:
```bash
source devel/setup.zsh
```
