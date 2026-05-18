# Roscore Recovery Design

## Problem

When roscore restarts, the pathfinder system does not recover automatically:

- The pathfinder container exits and Docker restarts it, which re-runs `catkin_make` (~40s) even though roscore was the only thing that died.
- The web UI gets stuck on "Connecting to rosbridge..." indefinitely because `fetchGraph` and `fetchRobots` are called only once on mount. If rosbridge is down at that moment, graph state stays `null` forever.

## Scope

- Pathfinder container: survive roscore restarts without rebuilding.
- Web UI: show a non-blocking reconnecting banner, auto-dismiss on recovery.
- Out of scope: pathfinder container crash recovery, robot bringup recovery.

## Design

### Layer 1 — Pathfinder Container (`entrypoint.sh`)

Replace the single `exec roslaunch` with a build-once + reconnect loop.

```bash
#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps pathfinder -C /workspace
source /workspace/devel/setup.bash

while true; do
    echo "[pathfinder] Waiting for roscore at $ROS_MASTER_URI..."
    until rostopic list > /dev/null 2>&1; do sleep 2; done

    echo "[pathfinder] roscore detected, launching..."
    roslaunch pathfinder robots.launch "$@" || true

    echo "[pathfinder] roslaunch exited, waiting for roscore..."
    sleep 2
done
```

**Behavior:**
- `catkin_make` runs once per container start.
- After a successful build, the loop probes `rostopic list` every 2 seconds until roscore is reachable.
- `roslaunch` exits when roscore dies. `|| true` keeps the loop alive.
- The container never restarts just because roscore did — no rebuild on roscore recovery.

### Layer 2 — Web UI (`App.jsx`)

**Connection state:**

Add `rosConnected` boolean state (default `false`). Wire three `ros` event listeners in a single `useEffect`:

- `ros.on('connection')` → set `rosConnected = true`, re-fetch graph + robots.
- `ros.on('close')` → set `rosConnected = false`.
- `ros.on('error')` → set `rosConnected = false`.

Remove the existing one-shot `useEffect([], [])` that calls `fetchGraph`/`fetchRobots`. Data fetching moves entirely into the `'connection'` handler, so both first load and reconnect use the same path.

**Banner:**

- `graph === null` (never loaded): keep existing "Connecting to rosbridge..." splash.
- `graph !== null && !rosConnected`: show a non-blocking banner at the top of the page. The canvas, robot panel, and last known state remain visible.
- `graph !== null && rosConnected`: normal UI, no banner.

The banner auto-dismisses as soon as the connection handler fires and the re-fetch succeeds.

## Files Changed

| File | Change |
|------|--------|
| `docker/pathfinder/entrypoint.sh` | Replace single launch with build-once + reconnect loop |
| `web-ui/src/App.jsx` | Add `rosConnected` state, move fetch into connection handler, add reconnect banner |
