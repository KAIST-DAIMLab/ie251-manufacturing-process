import React from 'react'

const COLORS = ['#f87171', '#60a5fa', '#a78bfa', '#34d399']
const RADIUS = 10
const TICK = 16

function ObstacleCone({ svgX, svgY, theta, obstacle, worldScale }) {
  const radius = obstacle.stop_distance * worldScale
  const half = (obstacle.detect_degree / 2) * (Math.PI / 180)
  const leftAngle = theta + half
  const rightAngle = theta - half
  const leftX = svgX + radius * Math.cos(leftAngle)
  const leftY = svgY - radius * Math.sin(leftAngle)
  const rightX = svgX + radius * Math.cos(rightAngle)
  const rightY = svgY - radius * Math.sin(rightAngle)
  const points = `${svgX},${svgY} ${leftX},${leftY} ${rightX},${rightY}`

  if (obstacle.enabled) {
    return <polygon points={points} fill="rgba(248,113,113,0.18)" stroke="none" pointerEvents="none" />
  }
  return (
    <polygon
      points={points}
      fill="none"
      stroke="rgba(160,160,160,0.6)"
      strokeWidth={1}
      strokeDasharray="4 4"
      pointerEvents="none"
    />
  )
}

export default function RobotDot({ robot, svgX, svgY, theta, selected, online = true, worldScale, onMouseDown }) {
  const colorIndex = parseInt(robot.id.replace(/\D/g, ''), 10) % COLORS.length
  const label = robot.name || robot.id
  const fill = online ? COLORS[colorIndex] : '#6b7280'
  const ringStroke = selected ? '#fff' : 'transparent'
  const tickX = svgX + Math.cos(-theta) * TICK
  const tickY = svgY + Math.sin(-theta) * TICK
  const dotOpacity = online ? 1 : 0.35

  return (
    <g
      onMouseDown={(e) => { e.preventDefault(); e.stopPropagation(); onMouseDown(e.clientX, e.clientY) }}
      style={{ cursor: 'grab' }}
      opacity={dotOpacity}
    >
      {robot.obstacle && worldScale && (
        <ObstacleCone svgX={svgX} svgY={svgY} theta={theta} obstacle={robot.obstacle} worldScale={worldScale} />
      )}
      <circle cx={svgX} cy={svgY} r={RADIUS + 4} fill="none" stroke={ringStroke} strokeWidth={2} />
      <circle cx={svgX} cy={svgY} r={RADIUS} fill={fill} />
      <line x1={svgX} y1={svgY} x2={tickX} y2={tickY} stroke="#fff" strokeWidth={2} />
      <text x={svgX} y={svgY + RADIUS + 16} textAnchor="middle" fill={online ? '#eee' : '#9ca3af'} fontSize={14} fontWeight="700">
        {label}
      </text>
    </g>
  )
}
