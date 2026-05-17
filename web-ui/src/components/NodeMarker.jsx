import React from 'react'

const RADIUS = 10

export default function NodeMarker({ node, station, svgX, svgY, dropTarget, relocalizeTarget }) {
  const isStation = station !== null && station !== undefined
  const fill = dropTarget ? '#facc15' : relocalizeTarget ? '#a78bfa' : isStation ? '#fb923c' : '#4ade80'
  const label = isStation ? `Station ${station.id}` : node.id
  return (
    <g style={{ cursor: relocalizeTarget ? 'crosshair' : 'default' }}>
      {dropTarget && (
        <circle cx={svgX} cy={svgY} r={RADIUS + 6} fill="none" stroke="#facc15" strokeWidth={2} />
      )}
      {relocalizeTarget && (
        <circle cx={svgX} cy={svgY} r={RADIUS + 6} fill="none" stroke="#a78bfa" strokeWidth={2} />
      )}
      <circle cx={svgX} cy={svgY} r={RADIUS} fill={fill} stroke="#fff" strokeWidth={1.5} />
      <text x={svgX} y={svgY - RADIUS - 4} textAnchor="middle" fill="#eee" fontSize={10}>
        {label}
      </text>
    </g>
  )
}
