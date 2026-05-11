import React from 'react'

const RADIUS = 10

export default function NodeMarker({ node, svgX, svgY, selected, onSelect }) {
  const fill = selected ? '#facc15' : '#4ade80'
  return (
    <g onClick={onSelect} style={{ cursor: onSelect ? 'pointer' : 'default' }}>
      <circle cx={svgX} cy={svgY} r={RADIUS} fill={fill} stroke="#fff" strokeWidth={1.5} />
      <text x={svgX} y={svgY - RADIUS - 4} textAnchor="middle" fill="#eee" fontSize={10}>
        {node.id}
      </text>
    </g>
  )
}
