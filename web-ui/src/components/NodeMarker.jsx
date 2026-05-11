import React from 'react'

const RADIUS = 10

export default function NodeMarker({ node, svgX, svgY, dropTarget }) {
  const fill = dropTarget ? '#facc15' : '#4ade80'
  return (
    <g style={{ cursor: 'default' }}>
      {dropTarget && (
        <circle cx={svgX} cy={svgY} r={RADIUS + 6} fill="none" stroke="#facc15" strokeWidth={2} />
      )}
      <circle cx={svgX} cy={svgY} r={RADIUS} fill={fill} stroke="#fff" strokeWidth={1.5} />
      <text x={svgX} y={svgY - RADIUS - 4} textAnchor="middle" fill="#eee" fontSize={10}>
        {node.id}
      </text>
    </g>
  )
}
