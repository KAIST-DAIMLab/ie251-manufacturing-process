import React, { useMemo, useRef } from 'react'
import NodeMarker from './NodeMarker.jsx'
import RobotDot from './RobotDot.jsx'

const PADDING = 60
const NODE_HIT_RADIUS = 28

function buildViewBox(nodes) {
  const xs = nodes.map((n) => n.x)
  const ys = nodes.map((n) => n.y)
  const minX = Math.min(...xs)
  const maxX = Math.max(...xs)
  const minY = Math.min(...ys)
  const maxY = Math.max(...ys)
  return { minX, maxX, minY, maxY, width: maxX - minX, height: maxY - minY }
}

const PATH_COLORS = ['#f87171', '#60a5fa', '#a78bfa', '#34d399']

function robotColor(robotId) {
  return PATH_COLORS[parseInt(robotId.replace(/\D/g, ''), 10) % PATH_COLORS.length]
}

export default function GraphCanvas({
  graph,
  robots,
  poses,
  pathStatuses,
  stateMap,
  selectedRobotId,
  dragState,
  setDragState,
  onRobotMouseDown,
  onDrop,
  onDragCancel,
}) {
  const SVG_W = 700
  const SVG_H = 500
  const svgRef = useRef(null)

  const nodeMap = useMemo(
    () => Object.fromEntries(graph.nodes.map((n) => [n.id, n])),
    [graph],
  )

  const stationByNode = useMemo(
    () => Object.fromEntries((graph.stations ?? []).map((station) => [station.node, station])),
    [graph],
  )

  const bounds = useMemo(() => buildViewBox(graph.nodes), [graph])

  const scale = useMemo(() => {
    const scaleX = (SVG_W - PADDING * 2) / (bounds.width || 1)
    const scaleY = (SVG_H - PADDING * 2) / (bounds.height || 1)
    return Math.min(scaleX, scaleY)
  }, [bounds])

  function toSvg(worldX, worldY) {
    const offsetX = (SVG_W - bounds.width * scale) / 2
    const offsetY = (SVG_H - bounds.height * scale) / 2
    return {
      x: offsetX + (worldX - bounds.minX) * scale,
      y: SVG_H - (offsetY + (worldY - bounds.minY) * scale),
    }
  }

  function clientToSvg(clientX, clientY) {
    const rect = svgRef.current.getBoundingClientRect()
    return { x: clientX - rect.left, y: clientY - rect.top }
  }

  function findNearestNode(svgPos) {
    let nearest = null
    let minDist = NODE_HIT_RADIUS
    for (const node of graph.nodes) {
      const { x, y } = toSvg(node.x, node.y)
      const dist = Math.hypot(svgPos.x - x, svgPos.y - y)
      if (dist < minDist) {
        minDist = dist
        nearest = node.id
      }
    }
    return nearest
  }

  function handleMouseMove(event) {
    if (!dragState) return
    const pointerSvg = clientToSvg(event.clientX, event.clientY)
    const hoverNodeId = findNearestNode(pointerSvg)
    setDragState((prev) => ({ ...prev, pointerSvg, hoverNodeId }))
  }

  function handleMouseUp() {
    if (!dragState) return
    if (dragState.hoverNodeId !== null) {
      onDrop(dragState.robotId, dragState.hoverNodeId)
    } else {
      onDragCancel()
    }
  }

  function handleMouseLeave() {
    if (dragState) onDragCancel()
  }

  const draggingRobotSvgPos = dragState
    ? (() => {
        const pose = poses[dragState.robotId]
        return pose ? toSvg(pose.x, pose.y) : null
      })()
    : null

  return (
    <svg
      ref={svgRef}
      width={SVG_W}
      height={SVG_H}
      style={{ display: 'block', background: '#1e1e1e', cursor: dragState ? 'grabbing' : 'default' }}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onMouseLeave={handleMouseLeave}
    >
      {graph.edges.map((edge, index) => {
        const a = toSvg(nodeMap[edge.from].x, nodeMap[edge.from].y)
        const b = toSvg(nodeMap[edge.to].x, nodeMap[edge.to].y)
        return (
          <line key={index} x1={a.x} y1={a.y} x2={b.x} y2={b.y} stroke="#555" strokeWidth={2} />
        )
      })}

      {robots.map((robot) => {
        const status = pathStatuses?.[robot.id]
        if (!status || status.node_ids.length === 0) return null
        const color = robotColor(robot.id)
        return status.node_ids.map((nodeId, i) => {
          if (i === 0) return null
          const a = toSvg(nodeMap[status.node_ids[i - 1]].x, nodeMap[status.node_ids[i - 1]].y)
          const b = toSvg(nodeMap[nodeId].x, nodeMap[nodeId].y)
          const active = i - 1 === status.current_index || i === status.current_index
          return (
            <line
              key={`${robot.id}-${i}`}
              x1={a.x} y1={a.y} x2={b.x} y2={b.y}
              stroke={color}
              strokeWidth={active ? 3 : 1.5}
              strokeOpacity={active ? 0.9 : 0.4}
              strokeDasharray={active ? 'none' : '6 4'}
              pointerEvents="none"
            />
          )
        })
      })}

      {graph.nodes.map((node) => {
        const { x, y } = toSvg(node.x, node.y)
        return (
          <NodeMarker
            key={node.id}
            node={node}
            station={stationByNode[node.id]}
            svgX={x}
            svgY={y}
            dropTarget={dragState?.hoverNodeId === node.id}
          />
        )
      })}

      {robots.map((robot) => {
        const pose = poses[robot.id]
        if (!pose) return null
        const { x, y } = toSvg(pose.x, pose.y)
        return (
          <RobotDot
            key={robot.id}
            robot={robot}
            svgX={x}
            svgY={y}
            theta={pose.theta}
            selected={robot.id === selectedRobotId}
            stateValue={stateMap?.[robot.id] ?? 0}
            worldScale={scale}
            onMouseDown={(clientX, clientY) => {
              const pointerSvg = clientToSvg(clientX, clientY)
              onRobotMouseDown(robot.id, pointerSvg)
            }}
          />
        )
      })}

      {dragState && draggingRobotSvgPos && dragState.pointerSvg && (
        <line
          x1={draggingRobotSvgPos.x}
          y1={draggingRobotSvgPos.y}
          x2={dragState.pointerSvg.x}
          y2={dragState.pointerSvg.y}
          stroke="#facc15"
          strokeWidth={1.5}
          strokeDasharray="6 4"
          pointerEvents="none"
        />
      )}
    </svg>
  )
}
