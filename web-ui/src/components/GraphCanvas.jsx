import React, { useMemo } from 'react'
import NodeMarker from './NodeMarker.jsx'
import RobotDot from './RobotDot.jsx'

const PADDING = 60

function buildViewBox(nodes) {
  const xs = nodes.map((n) => n.x)
  const ys = nodes.map((n) => n.y)
  const minX = Math.min(...xs)
  const maxX = Math.max(...xs)
  const minY = Math.min(...ys)
  const maxY = Math.max(...ys)
  return { minX, maxX, minY, maxY, width: maxX - minX, height: maxY - minY }
}

export default function GraphCanvas({
  graph,
  robots,
  poses,
  selectedRobotId,
  onSelectRobot,
  onSelectNode,
}) {
  const SVG_W = 700
  const SVG_H = 500

  const nodeMap = useMemo(
    () => Object.fromEntries(graph.nodes.map((n) => [n.id, n])),
    [graph],
  )

  const bounds = useMemo(() => buildViewBox(graph.nodes), [graph])

  function toSvg(worldX, worldY) {
    const scaleX = (SVG_W - PADDING * 2) / (bounds.width || 1)
    const scaleY = (SVG_H - PADDING * 2) / (bounds.height || 1)
    const scale = Math.min(scaleX, scaleY)
    const offsetX = (SVG_W - bounds.width * scale) / 2
    const offsetY = (SVG_H - bounds.height * scale) / 2
    return {
      x: offsetX + (worldX - bounds.minX) * scale,
      y: SVG_H - (offsetY + (worldY - bounds.minY) * scale),
    }
  }

  return (
    <svg width={SVG_W} height={SVG_H} style={{ display: 'block', background: '#1e1e1e' }}>
      {graph.edges.map((edge, index) => {
        const a = toSvg(nodeMap[edge.from].x, nodeMap[edge.from].y)
        const b = toSvg(nodeMap[edge.to].x, nodeMap[edge.to].y)
        return (
          <line key={index} x1={a.x} y1={a.y} x2={b.x} y2={b.y} stroke="#555" strokeWidth={2} />
        )
      })}

      {graph.nodes.map((node) => {
        const { x, y } = toSvg(node.x, node.y)
        return (
          <NodeMarker
            key={node.id}
            node={node}
            svgX={x}
            svgY={y}
            selected={false}
            onSelect={selectedRobotId ? () => onSelectNode(node.id) : null}
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
            onSelect={() => onSelectRobot(robot.id)}
          />
        )
      })}
    </svg>
  )
}
