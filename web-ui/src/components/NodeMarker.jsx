import React from 'react'

const RADIUS = 10

export default function NodeMarker({ node, svgX, svgY, dropTarget }) {
  const isStation = node.station !== null && node.station !== undefined
  const fill = dropTarget ? '#facc15' : isStation ? '#fb923c' : '#4ade80'
  const label = isStation ? `Station ${node.station}` : node.id
  return (
    <g style={{ cursor: 'default' }}>
      {dropTarget && (
        <circle cx={svgX} cy={svgY} r={RADIUS + 6} fill="none" stroke="#facc15" strokeWidth={2} />
      )}
      <circle cx={svgX} cy={svgY} r={RADIUS} fill={fill} stroke="#fff" strokeWidth={1.5} />
      <text x={svgX} y={svgY - RADIUS - 4} textAnchor="middle" fill="#eee" fontSize={10}>
        {label}
      </text>
    </g>
  )
}
