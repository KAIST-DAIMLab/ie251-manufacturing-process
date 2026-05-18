# Roscore Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the pathfinder container and web UI survive roscore restarts without manual intervention.

**Architecture:** Two independent layers — the container entrypoint gains a reconnect loop so `catkin_make` runs once per container start and `roslaunch` restarts automatically when roscore recovers; the web UI tracks rosbridge connection state and re-fetches graph + robots on every reconnect.

**Tech Stack:** Bash (entrypoint), React + ROSLIB (web UI)

**Spec:** `docs/superpowers/specs/2026-05-18-roscore-recovery-design.md`

---

## Files Changed

| File | Change |
|------|--------|
| `docker/pathfinder/entrypoint.sh` | Replace `exec roslaunch` with build-once + reconnect loop |
| `web-ui/src/App.jsx` | Import `ros`, add `rosConnected` state, move fetch to connection handler, add reconnect banner |

---

## Task 1: Reconnect Loop in entrypoint.sh

**Files:**
- Modify: `docker/pathfinder/entrypoint.sh`

- [ ] **Step 1: Replace the entrypoint**

Replace the entire content of `docker/pathfinder/entrypoint.sh` with:

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

- [ ] **Step 2: Verify manually**

Start both containers:
```bash
cd docker/roscore && docker compose up -d
cd ../pathfinder && docker compose up
```

Confirm `catkin_make` runs and `roslaunch` starts. Then kill roscore:
```bash
docker compose -f docker/roscore/docker-compose.yml stop
```

Expected: pathfinder container prints `[pathfinder] roslaunch exited, waiting for roscore...` and starts polling. No `catkin_make` output appears.

Restart roscore:
```bash
docker compose -f docker/roscore/docker-compose.yml start
```

Expected: pathfinder prints `[pathfinder] roscore detected, launching...` and roslaunch restarts. No `catkin_make`.

- [ ] **Step 3: Commit**

```bash
git add docker/pathfinder/entrypoint.sh
git commit -m "Fix: Restart Roslaunch on Roscore Recovery Without Rebuilding"
```

---

## Task 2: rosConnected State and Connection Handler in App.jsx

**Files:**
- Modify: `web-ui/src/App.jsx`

- [ ] **Step 1: Add `ros` import**

In `web-ui/src/App.jsx`, add this import after the existing ros imports (line 12):

```jsx
import ros from './ros/rosClient.js'
```

- [ ] **Step 2: Add `rosConnected` state**

After line 27 (`const [relocalizeState, setRelocalizeState] = useState(null)`), add:

```jsx
const [rosConnected, setRosConnected] = useState(false)
```

- [ ] **Step 3: Replace the one-shot fetch useEffect**

Replace lines 29-36 (the `useEffect` that calls `fetchGraph`/`fetchRobots` once) with:

```jsx
useEffect(() => {
    const onConnection = () => {
      setRosConnected(true)
      Promise.all([fetchGraph(), fetchRobots()])
        .then(([graphData, robotsData]) => {
          setGraph(graphData)
          setRobots(robotsData.robots)
        })
        .catch((error) => console.error('init error:', error))
    }
    const onClose = () => setRosConnected(false)
    const onError = () => setRosConnected(false)

    ros.on('connection', onConnection)
    ros.on('close', onClose)
    ros.on('error', onError)

    return () => {
      ros.off('connection', onConnection)
      ros.off('close', onClose)
      ros.off('error', onError)
    }
  }, [])
```

- [ ] **Step 4: Add reconnect banner to the render**

In the main `return` block, after the `<h1>` tag (line 183) and before the `<p>` instruction text, insert:

```jsx
{!rosConnected && (
    <div style={{
        marginBottom: 8,
        padding: '6px 12px',
        background: '#7f1d1d',
        borderLeft: '3px solid #ef4444',
        color: '#fca5a5',
        fontSize: 13,
    }}>
        Reconnecting to rosbridge...
    </div>
)}
```

- [ ] **Step 5: Verify manually**

Start the web UI:
```bash
cd web-ui && npm run dev
```

With rosbridge up: confirm the UI loads normally and the banner is absent.

Stop rosbridge (stop pathfinder container): confirm the banner `Reconnecting to rosbridge...` appears over the last known fleet state.

Restart rosbridge: confirm the banner disappears and the graph + robots reload automatically.

Also verify first-load behavior: stop rosbridge, refresh the page. Confirm the existing `Connecting to rosbridge...` splash appears. Start rosbridge: confirm the UI loads fully without a manual refresh.

- [ ] **Step 6: Commit**

```bash
git add web-ui/src/App.jsx
git commit -m "Feat: Auto-Reconnect Web UI to Rosbridge After Roscore Recovery"
```
