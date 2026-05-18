import React, { useRef, useState } from 'react'
import { rotateRobot } from '../ros/rotateRobot.js'

const SIZE = 110
const CENTER = SIZE / 2
const NEEDLE_RADIUS = 38
const HANDLE_RADIUS = 11
const DIAL_RADIUS = 48

function formatError(error) {
  if (!error) return 'rotation failed'
  if (typeof error === 'string') return error
  return error.message || String(error)
}

function pointerTheta(event, svgElement) {
  const rect = svgElement.getBoundingClientRect()
  const offsetX = event.clientX - rect.left - CENTER
  const offsetY = event.clientY - rect.top - CENTER
  return Math.atan2(-offsetY, offsetX)
}

export default function RotationDial({ robot, theta, disabled, onError }) {
  const svgRef = useRef(null)
  const activePointerRef = useRef(null)
  const [dragTheta, setDragTheta] = useState(null)
  const [pending, setPending] = useState(false)

  const displayTheta = dragTheta ?? theta ?? 0
  const handleX = CENTER + NEEDLE_RADIUS * Math.cos(displayTheta)
  const handleY = CENTER - NEEDLE_RADIUS * Math.sin(displayTheta)
  const busy = disabled || pending

  function handlePointerDown(event) {
    if (busy) return
    event.preventDefault()
    event.currentTarget.setPointerCapture(event.pointerId)
    activePointerRef.current = event.pointerId
    setDragTheta(pointerTheta(event, svgRef.current))
  }

  function handlePointerMove(event) {
    if (activePointerRef.current !== event.pointerId) return
    setDragTheta(pointerTheta(event, svgRef.current))
  }

  function handlePointerUp(event) {
    if (activePointerRef.current !== event.pointerId) return
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId)
    }
    const finalTheta = pointerTheta(event, svgRef.current)
    activePointerRef.current = null
    setPending(true)
    rotateRobot(robot.id, finalTheta)
      .then((response) => {
        if (!response.success) onError?.(response.message)
      })
      .catch((error) => onError?.(formatError(error)))
      .finally(() => {
        setPending(false)
        setDragTheta(null)
      })
  }

  const ringColor = busy ? '#333' : '#555'
  const needleColor = busy ? '#4b5563' : '#9ca3af'
  const handleFill = pending ? '#1d4ed8' : '#2563eb'

  return (
    <svg
      ref={svgRef}
      width={SIZE}
      height={SIZE}
      viewBox={`0 0 ${SIZE} ${SIZE}`}
      style={{ touchAction: 'none', opacity: disabled ? 0.5 : 1, flexShrink: 0 }}
      aria-label="Manual rotation dial"
    >
      <circle cx={CENTER} cy={CENTER} r={DIAL_RADIUS} fill="#141414" stroke={ringColor} strokeWidth="1" />
      <line x1={CENTER} y1={CENTER} x2={handleX} y2={handleY} stroke={needleColor} strokeWidth="2" strokeLinecap="round" />
      <circle cx={CENTER} cy={CENTER} r={3.5} fill={needleColor} />
      <circle
        cx={handleX}
        cy={handleY}
        r={HANDLE_RADIUS}
        fill={handleFill}
        stroke="#e0e7ff"
        strokeWidth="1.5"
        style={{ cursor: busy ? 'not-allowed' : 'grab' }}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
        onLostPointerCapture={handlePointerUp}
      />
    </svg>
  )
}
