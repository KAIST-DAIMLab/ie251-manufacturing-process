import React, { useEffect, useState, useCallback } from 'react'
import GraphCanvas from './components/GraphCanvas.jsx'
import RobotPanel from './components/RobotPanel.jsx'
import { fetchGraph } from './ros/fetchGraph.js'
import { fetchRobots } from './ros/fetchRobots.js'
import { subscribePose } from './ros/subscribePose.js'
import { subscribePathStatus } from './ros/subscribePathStatus.js'
import { subscribeState } from './ros/subscribeState.js'
import { moveToNode } from './ros/moveToNode.js'
import { cancelPath } from './ros/cancelPath.js'

const STATE_FRESHNESS_MS = 2500
const ONLINE_CHECK_INTERVAL_MS = 500

export default function App() {
  const [graph, setGraph] = useState(null)
  const [robots, setRobots] = useState([])
  const [poses, setPoses] = useState({})
  const [pathStatuses, setPathStatuses] = useState({})
  const [robotStates, setRobotStates] = useState({})
  const [onlineMap, setOnlineMap] = useState({})
  const [selectedRobotId, setSelectedRobotId] = useState(null)
  const [banner, setBanner] = useState(null)
  const [dragState, setDragState] = useState(null)

  useEffect(() => {
    Promise.all([fetchGraph(), fetchRobots()])
      .then(([graphData, robotsData]) => {
        setGraph(graphData)
        setRobots(robotsData.robots)
      })
      .catch((error) => console.error('init error:', error))
  }, [])

  useEffect(() => {
    if (robots.length === 0) return
    const unsubscribers = robots.map((robot) =>
      subscribePose(robot.namespace, (message) => {
        setPoses((previous) => ({
          ...previous,
          [robot.id]: { x: message.x, y: message.y, theta: message.theta },
        }))
      }),
    )
    return () => unsubscribers.forEach((unsubscribe) => unsubscribe())
  }, [robots])

  useEffect(() => {
    if (robots.length === 0) return
    const unsubscribers = robots.map((robot) =>
      subscribePathStatus(robot.namespace, (status) => {
        setPathStatuses((previous) => ({ ...previous, [robot.id]: status }))
      }),
    )
    return () => unsubscribers.forEach((unsubscribe) => unsubscribe())
  }, [robots])

  useEffect(() => {
    if (robots.length === 0) return
    const unsubscribers = robots.map((robot) =>
      subscribeState(robot.namespace, (message) => {
        setRobotStates((previous) => ({
          ...previous,
          [robot.id]: { value: message.data, receivedAt: Date.now() },
        }))
      }),
    )
    return () => unsubscribers.forEach((unsubscribe) => unsubscribe())
  }, [robots])

  useEffect(() => {
    if (robots.length === 0) return
    const tick = () => {
      const now = Date.now()
      setOnlineMap((previous) => {
        let changed = false
        const next = { ...previous }
        for (const robot of robots) {
          const entry = robotStates[robot.id]
          const online = entry !== undefined && now - entry.receivedAt < STATE_FRESHNESS_MS
          if (next[robot.id] !== online) {
            next[robot.id] = online
            changed = true
          }
        }
        return changed ? next : previous
      })
    }
    tick()
    const handle = setInterval(tick, ONLINE_CHECK_INTERVAL_MS)
    return () => clearInterval(handle)
  }, [robots, robotStates])

  useEffect(() => {
    if (!banner) return
    const timer = setTimeout(() => setBanner(null), 3000)
    return () => clearTimeout(timer)
  }, [banner])

  const handleRobotMouseDown = useCallback((robotId, pointerSvg) => {
    setSelectedRobotId(robotId)
    setDragState({ robotId, pointerSvg, hoverNodeId: null })
  }, [])

  const getRobotLabel = useCallback((robotId) => {
    return robots.find((robot) => robot.id === robotId)?.name || robotId
  }, [robots])

  const handleDrop = useCallback((robotId, nodeId) => {
    setDragState(null)
    moveToNode(robotId, nodeId)
      .then((response) => setBanner(`${getRobotLabel(robotId)}: ${response.message}`))
      .catch((error) => setBanner(`Error: ${error}`))
  }, [getRobotLabel])

  const handleStop = useCallback((robotId) => {
    cancelPath(robotId)
      .then((response) => setBanner(`${getRobotLabel(robotId)}: ${response.message}`))
      .catch((error) => setBanner(`Error: ${error}`))
  }, [getRobotLabel])

  const handleJogError = useCallback((message) => {
    setBanner(`Error: ${message}`)
  }, [])

  const handleDragCancel = useCallback(() => {
    setDragState(null)
  }, [])

  const selectedRobot = robots.find((r) => r.id === selectedRobotId) ?? null

  const stateMap = Object.fromEntries(
    robots.map((robot) => [
      robot.id,
      onlineMap[robot.id] ? (robotStates[robot.id]?.value ?? 1) : 0,
    ])
  )

  if (!graph) {
    return (
      <div style={{ padding: 24 }}>Connecting to rosbridge...</div>
    )
  }

  return (
    <div style={{ padding: 16 }}>
      <h1 style={{ marginBottom: 12, fontSize: 16, letterSpacing: 1 }}>PATHFINDER MONITOR</h1>

      <p style={{ marginBottom: 8, color: '#888' }}>
        {dragState ? `dragging ${getRobotLabel(dragState.robotId)}…` : 'Click a robot to select, drag to a node to move'}
      </p>

      <div
        style={{
          marginBottom: 8,
          padding: '6px 12px',
          background: banner ? '#1f2937' : 'transparent',
          borderLeft: `3px solid ${banner ? '#4ade80' : 'transparent'}`,
          fontSize: 13,
          minHeight: 18,
          visibility: banner ? 'visible' : 'hidden',
        }}
      >
        {banner || ' '}
      </div>

      <div style={{ display: 'flex', alignItems: 'flex-start', gap: 16 }}>
        <GraphCanvas
          graph={graph}
          robots={robots}
          poses={poses}
          pathStatuses={pathStatuses}
          stateMap={stateMap}
          selectedRobotId={selectedRobotId}
          dragState={dragState}
          setDragState={setDragState}
          onRobotMouseDown={handleRobotMouseDown}
          onDrop={handleDrop}
          onDragCancel={handleDragCancel}
        />
        <div style={{ background: '#1e1e1e', minWidth: 220 }}>
          <RobotPanel
            robot={selectedRobot}
            pose={selectedRobotId ? poses[selectedRobotId] : null}
            pathStatus={selectedRobotId ? pathStatuses[selectedRobotId] : null}
            serverStateValue={selectedRobotId ? robotStates[selectedRobotId]?.value : null}
            online={selectedRobotId ? onlineMap[selectedRobotId] === true : false}
            onStop={handleStop}
            onJogError={handleJogError}
          />
        </div>
      </div>
    </div>
  )
}
