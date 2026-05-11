import React, { useEffect, useState, useCallback } from 'react'
import GraphCanvas from './components/GraphCanvas.jsx'
import RobotPanel from './components/RobotPanel.jsx'
import { fetchGraph } from './ros/fetchGraph.js'
import { fetchRobots } from './ros/fetchRobots.js'
import { subscribePose } from './ros/subscribePose.js'
import { moveToNode } from './ros/moveToNode.js'

export default function App() {
  const [graph, setGraph] = useState(null)
  const [robots, setRobots] = useState([])
  const [poses, setPoses] = useState({})
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
    if (!banner) return
    const timer = setTimeout(() => setBanner(null), 3000)
    return () => clearTimeout(timer)
  }, [banner])

  const handleRobotMouseDown = useCallback((robotId, pointerSvg) => {
    setSelectedRobotId(robotId)
    setDragState({ robotId, pointerSvg, hoverNodeId: null })
  }, [])

  const handleDrop = useCallback((robotId, nodeId) => {
    setDragState(null)
    moveToNode(robotId, nodeId)
      .then((response) => setBanner(`${robotId}: ${response.message}`))
      .catch((error) => setBanner(`Error: ${error}`))
  }, [])

  const handleDragCancel = useCallback(() => {
    setDragState(null)
  }, [])

  const selectedRobot = robots.find((r) => r.id === selectedRobotId) ?? null

  if (!graph) {
    return (
      <div style={{ padding: 24 }}>Connecting to rosbridge...</div>
    )
  }

  return (
    <div style={{ padding: 16 }}>
      <h1 style={{ marginBottom: 12, fontSize: 16, letterSpacing: 1 }}>PATHFINDER MONITOR</h1>

      <p style={{ marginBottom: 8, color: '#888' }}>
        {dragState ? `dragging ${dragState.robotId}…` : 'Click a robot to select, drag to a node to move'}
      </p>

      {banner && (
        <div
          style={{
            marginBottom: 8,
            padding: '6px 12px',
            background: '#1f2937',
            borderLeft: '3px solid #4ade80',
            fontSize: 13,
          }}
        >
          {banner}
        </div>
      )}

      <div style={{ display: 'flex', alignItems: 'flex-start', gap: 16 }}>
        <GraphCanvas
          graph={graph}
          robots={robots}
          poses={poses}
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
          />
        </div>
      </div>
    </div>
  )
}
