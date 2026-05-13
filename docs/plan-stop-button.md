# Plan: Stop button for selected robot in web UI

## Context

The web UI currently lets the user drag a robot onto a node to send it there (`MoveToNode` service), but there is no way to cancel that motion mid-flight from the browser. The user has to wait for arrival or kill the action via CLI. We want a "Stop" button on the selected robot's side panel that immediately halts the robot — cancelling both the in-flight `FollowPath` goal *and* any motion the `MotionController` is performing.

**Key finding from exploration:** the backend stop chain already exists end-to-end. No backend code changes are required.

- `TurtleBot.stop()` ([turtlebot.py:69-72](../src/pathfinder/src/pathfinder/robot/turtlebot.py#L69-L72)) calls both `motion_controller.stop()` and `path_follower.cancel()`.
- `CancelPath.srv` is fully wired: `/fleet/cancel_path` → `fleet_service._handle_cancel` ([fleet_service.py:65-73](../src/pathfinder/src/pathfinder/ros/fleet_service.py#L65-L73)) → `PathFollowActionClient.cancel()` → executor's `PathFollowActionServer` preempts → calls `self._robot.stop()` ([path_follow_action_server.py:49-53](../src/pathfinder/src/pathfinder/ros/path_follow_action_server.py#L49-L53)) → `MotionController._cancel = True`, `engine.stop()` publishes zero `Twist`.

So a single call to `/fleet/cancel_path` from the browser is sufficient. Work is purely in `web-ui/`.

## Changes

### 1. New: `web-ui/src/ros/cancelPath.js`

Mirror [moveToNode.js](../web-ui/src/ros/moveToNode.js). One service-call helper, returning a Promise.

```js
import ROSLIB from 'roslib'
import { ros } from './rosClient.js'

export function cancelPath(robotId) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros,
      name: '/fleet/cancel_path',
      serviceType: 'pathfinder/CancelPath',
    })
    client.callService(
      new ROSLIB.ServiceRequest({ robot_id: robotId }),
      resolve,
      reject,
    )
  })
}
```

Request shape `{ robot_id }` and response `{ success, message }` match [CancelPath.srv](../src/pathfinder/srv/CancelPath.srv).

### 2. `web-ui/src/App.jsx` — add `handleStop` callback

Parallel to existing `handleDrop` ([App.jsx:50-55](../web-ui/src/App.jsx#L50-L55)):

```js
const handleStop = useCallback((robotId) => {
  cancelPath(robotId)
    .then((response) => setBanner(`${robotId}: ${response.message}`))
    .catch((error) => setBanner(`Error: ${error}`))
}, [])
```

Pass `onStop={handleStop}` into `<RobotPanel>` ([App.jsx:104-107](../web-ui/src/App.jsx#L104-L107)).

### 3. `web-ui/src/components/RobotPanel.jsx` — add Stop button

Currently a read-only display ([RobotPanel.jsx](../web-ui/src/components/RobotPanel.jsx)). Accept `onStop` prop and render a destructive-styled button just below the robot ID header (line 36, before the IDENTITY section). When `robot` is null, the panel already short-circuits to the "Select a robot…" message, so the button only ever renders when a robot is selected.

Button style (matches existing inline-style convention, red destructive accent — no red token exists in the codebase yet, so introduce `#ef4444`):

```jsx
<button
  onClick={() => onStop(robot.id)}
  style={{
    marginBottom: 12,
    padding: '6px 12px',
    background: '#ef4444',
    color: '#fff',
    border: 'none',
    fontSize: 13,
    cursor: 'pointer',
  }}
>
  Stop
</button>
```

Always enabled — calling `cancel_goal()` on an idle action client is a no-op on the backend and returns `success=true, message="canceled"`, so there's no harm in letting the user click when nothing is moving.

## Files touched

| File | Change |
|---|---|
| `web-ui/src/ros/cancelPath.js` | **new** — service-call wrapper |
| `web-ui/src/App.jsx` | import `cancelPath`, add `handleStop`, pass `onStop` to RobotPanel |
| `web-ui/src/components/RobotPanel.jsx` | accept `onStop`, render Stop button |

No backend changes. No new ROS messages, services, topics, or tests on the backend.

## Verification

End-to-end test in the running simulator:

1. Start the system: `roslaunch pathfinder simulation.launch`
2. Open `http://localhost:5173` in a browser, wait for "Connecting to rosbridge…" to clear.
3. Click a robot to select it — confirm the Stop button appears in the right-hand panel.
4. Drag the robot to a far node to start a long path.
5. While it's moving, click **Stop**.
   - Expected: robot decelerates to a halt within one motion tick (~200 ms at 5 Hz).
   - Banner shows `tb3_01: canceled` (or whatever the service returns).
   - `rostopic echo /tb3_01/cmd_vel -n 5` should show zero Twist after the stop.
6. Click Stop when no goal is active — banner should still show success, no crash.
7. Resume by dragging to a new node — robot should move normally (cancel doesn't latch any state past the current goal, per [motion_controller.py:43-47](../src/pathfinder/src/pathfinder/robot/motion_controller.py#L43-L47) which resets `_cancel` on next loop entry).

No unit-test changes required — the backend code being exercised is unchanged and already covered by `test_fleet_service.py` and `test_client.py`.
