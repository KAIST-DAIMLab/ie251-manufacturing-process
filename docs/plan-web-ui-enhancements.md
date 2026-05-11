# Plan: Web UI v2 — config panel, detection cone, drag-to-move

## Context

The v1 web UI (`docs/plan-web-ui.md`, shipped) renders the graph and lets the user click a robot, then click a target node. Three follow-ups:

1. **Right-side config panel** showing the selected robot's static configuration (motion params, obstacle settings, start node) and live pose.
2. **Obstacle-detection cone** drawn in front of every robot, sized by its `obstacle.stop_distance` and `obstacle.detect_degree`. Helps the user see at a glance which robots will stop and how close.
3. **Drag-to-move** replacing click-to-target. The user grabs a robot and drops it on a node; clicking is reserved for selection (which controls the panel).

Confirmed UX:

- **Drag-to-move; click-to-select.** Two distinct gestures. Selection persists after a successful drop so the panel stays useful for inspection.
- **Disabled cone shows a dimmed outline.** When `obstacle.enabled = false`, the cone is drawn as a dashed grey outline (no fill), so the operator still sees direction/range but the visual signals "not active".

## Changes

### 1. ROS: extend `_handle_get_robots` JSON payload

Today it returns `{id, namespace}`. The panel needs the full per-robot config that's already loaded in `Robot` from `robots.yaml`.

`src/pathfinder/src/pathfinder/ros/fleet_service.py:_handle_get_robots` — extend the payload to:

```json
{
  "robots": [
    {
      "id": "tb3_01",
      "namespace": "tb3_01",
      "start_node": 1,
      "yaw": 0.0,
      "motion": {
        "linear_speed": 0.22,
        "angular_speed": 1.5,
        "move_rate_hz": 5.0,
        "arrival_tolerance": 0.05
      },
      "obstacle": {
        "enabled": true,
        "stop_distance": 0.4,
        "detect_degree": 20
      }
    }
  ]
}
```

The srv shape (`string robots_json`) does not change. `Robot` already exposes `motion: MotionConfig` and `obstacle: ObstacleConfig` ([world/robot.py:25-35](../src/pathfinder/src/pathfinder/world/robot.py#L25-L35)) — read field-by-field rather than `dataclasses.asdict` so we have explicit control over the wire shape.

`src/pathfinder/test/test_fleet_service.py` — the fake robot in `_make_service` is `types.SimpleNamespace(id, namespace, pose)`. Extend it with `start_node`, `yaw`, `motion`, `obstacle` (using nested `SimpleNamespace`s) and assert the new fields appear in the get_robots response JSON.

### 2. Web UI: right-side config panel

New file `web-ui/src/components/RobotPanel.jsx`:
- Props: `robot` (the selected robot record, including the new fields above) and `pose` (live `{x, y, theta}` or undefined).
- Renders three sections:
  - **Identity**: id, namespace, start_node, yaw.
  - **Motion**: linear_speed, angular_speed, move_rate_hz, arrival_tolerance.
  - **Obstacle**: enabled, stop_distance, detect_degree.
  - **Live pose**: x, y, theta (degrees, derived from radians).
- Pure read-only.

`web-ui/src/App.jsx` — switch the outer layout to `display: flex` with the SVG canvas on the left and the panel on the right. Panel shows a placeholder ("Select a robot to see config") when `selectedRobotId` is null.

### 3. Web UI: detection cone per robot

`web-ui/src/components/RobotDot.jsx` — add a `<polygon>` (apex at robot center, two flank vertices) for the cone. Props gain:
- `obstacle: { enabled, stop_distance, detect_degree }`
- `worldScale: number` (pixels per meter — see step 5 below)

Geometry (in SVG coords; SVG y grows downward, world y grows upward, so the y-component of the heading flips):

```
apex   = (svgX, svgY)
radius = obstacle.stop_distance * worldScale
half   = (obstacle.detect_degree / 2) * Math.PI / 180
theta  = pose.theta                                   // robot heading, radians (world frame)

leftAngle  = theta + half
rightAngle = theta - half

leftPoint  = (apex.x + radius * cos(leftAngle),  apex.y - radius * sin(leftAngle))
rightPoint = (apex.x + radius * cos(rightAngle), apex.y - radius * sin(rightAngle))
```

Style:
- `enabled = true`: fill `rgba(248,113,113,0.18)`, no stroke.
- `enabled = false`: no fill, stroke `rgba(160,160,160,0.6)` dashed (e.g. `strokeDasharray="4 4"`).

The cone renders *behind* the robot dot (so the dot stays clickable). Order in JSX: cone first, then dot, then heading tick, then label.

### 4. Web UI: drag-to-move replaces click-on-node

State changes in `web-ui/src/App.jsx`:

```js
const [dragState, setDragState] = useState(null)
// dragState shape: { robotId, pointerSvg: {x, y}, hoverNodeId } | null
```

Event flow (handlers live on `<svg>` in `GraphCanvas.jsx` and bubble up via callbacks):

| Event | Where | Action |
|---|---|---|
| `mousedown` on a robot dot | `RobotDot` `onMouseDown` | `setDragState({ robotId, pointerSvg: {x,y}, hoverNodeId: null })` and `setSelectedRobotId(robotId)` (so panel updates immediately). |
| `mousemove` on SVG | `GraphCanvas` `onMouseMove` | If dragging, compute pointer SVG coords from `event.clientX/Y` minus the SVG bounding rect; find the nearest node within ~`NODE_HIT_RADIUS` and update `hoverNodeId`. |
| `mouseup` on SVG | `GraphCanvas` `onMouseUp` | If `hoverNodeId` is set, call `moveToNode(dragState.robotId, hoverNodeId)`, banner the result; clear `dragState`. Else just clear `dragState`. |
| `mouseleave` on SVG | `GraphCanvas` | Clear `dragState`. |

Visualization while dragging (in `GraphCanvas`):
- Dashed line from the dragged robot's current SVG position to `dragState.pointerSvg`.
- The hovered node renders with an extra highlight ring (yellow, slightly larger). `NodeMarker.jsx` accepts a new `dropTarget` boolean prop.

Click vs drag disambiguation: a click is a `mousedown` + `mouseup` with no node hovered in between. Since `mouseup` without `hoverNodeId` just clears the drag state and selection has already been set on `mousedown`, "click to select" is naturally implied — no extra logic needed. Removing `NodeMarker.onSelect` (the old click-to-target path) is part of this step.

### 5. World-to-SVG scale shared between canvas and dot

The cone math needs `worldScale` (pixels per meter). Today, `GraphCanvas.jsx` computes a per-render scale inside `toSvg`. Extract:

```js
const scale = useMemo(() => {
  const scaleX = (SVG_W - PADDING * 2) / (bounds.width || 1)
  const scaleY = (SVG_H - PADDING * 2) / (bounds.height || 1)
  return Math.min(scaleX, scaleY)
}, [bounds])
```

Pass `scale` down to `RobotDot` as `worldScale`. `toSvg` keeps using the same value internally.

## Critical files

Modify:
- `src/pathfinder/src/pathfinder/ros/fleet_service.py` (extend `_handle_get_robots`)
- `src/pathfinder/test/test_fleet_service.py` (extend fake robot + assertions)
- `web-ui/src/App.jsx` (layout, drag state, mouse handlers, panel wiring)
- `web-ui/src/components/GraphCanvas.jsx` (drag tracking, drop-target detection, ghost line, expose `scale`)
- `web-ui/src/components/RobotDot.jsx` (cone polygon, mousedown handler)
- `web-ui/src/components/NodeMarker.jsx` (`dropTarget` prop, drop the old click-to-target hook)

Create:
- `web-ui/src/components/RobotPanel.jsx`

## Verification

1. **Unit tests still green.**
   ```
   python3 src/pathfinder/test/test_fleet_service.py
   ```
   (11 existing tests; the get_robots one gets stronger assertions for the new fields.)

2. **Build clean inside the noetic container.**
   ```
   catkin_make --only-pkg-with-deps pathfinder && source devel/setup.zsh
   ```

3. **Service payload check.**
   ```
   roslaunch pathfinder simulation.launch    # in container
   rosservice call /fleet/get_robots
   ```
   Expect a `robots_json` containing the `motion` and `obstacle` sub-objects per robot.

4. **Web UI end-to-end** (host, outside container):
   ```
   cd web-ui && docker compose up
   # open http://localhost:5173
   ```
   Verify:
   - Each robot dot has a translucent red cone in front, oriented along the robot's heading. As `tb3_01` rotates (e.g. when reaching a waypoint), the cone rotates with it. The cone's tip distance equals `obstacle.stop_distance` in world units.
   - If `config/robots.yaml` is edited to `obstacle.enabled: false` for one robot and the system is restarted, that robot's cone renders as a dashed grey outline.
   - Click `tb3_01` → right panel populates: id `tb3_01`, namespace, start_node, motion fields, obstacle fields, live pose updating in real time. Click again or click empty space → panel clears.
   - Drag `tb3_01` toward node 5: a dashed line follows the pointer, node 5 highlights as the pointer enters its hit radius. Release on node 5 → banner reads "tb3_01: dispatched N waypoints", robot drives to node 5, cone tracks heading, panel still shows tb3_01 (selection persists).
   - Drag from `tb3_05` and release in empty space → no dispatch, ghost line vanishes.
   - The graph nodes themselves no longer respond to plain clicks (drag is the only way to dispatch a goal).
