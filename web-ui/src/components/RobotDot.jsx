import React from 'react'

const COLORS = ['#f87171', '#60a5fa', '#a78bfa', '#34d399']
const RADIUS = 10
const TICK = 16

export default function RobotDot({ robot, svgX, svgY, theta, selected, onSelect }) {
  const colorIndex = parseInt(robot.id.replace(/\D/g, ''), 10) % COLORS.length
  const fill = COLORS[colorIndex]
  const ringStroke = selected ? '#fff' : 'transparent'
  const tickX = svgX + Math.cos(-theta) * TICK
  const tickY = svgY + Math.sin(-theta) * TICK

  return (
    <g onClick={onSelect} style={{ cursor: 'pointer' }}>
      <circle cx={svgX} cy={svgY} r={RADIUS + 4} fill="none" stroke={ringStroke} strokeWidth={2} />
      <circle cx={svgX} cy={svgY} r={RADIUS} fill={fill} />
      <line x1={svgX} y1={svgY} x2={tickX} y2={tickY} stroke="#fff" strokeWidth={2} />
      <text x={svgX} y={svgY + RADIUS + 12} textAnchor="middle" fill="#eee" fontSize={9}>
        {robot.id}
      </text>
    </g>
  )
}
