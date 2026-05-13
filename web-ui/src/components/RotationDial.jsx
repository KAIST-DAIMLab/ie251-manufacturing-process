import React, { useMemo, useRef, useState } from 'react'

const SIZE = 112
const CENTER = SIZE / 2
const RING_RADIUS = 40
const MARKER_RADIUS = 31

function wrapToPi(value) {
  let wrapped = value
  while (wrapped <= -Math.PI) wrapped += Math.PI * 2
  while (wrapped > Math.PI) wrapped -= Math.PI * 2
  return wrapped
}

function toDeg(rad) {
  return ((rad * 180) / Math.PI).toFixed(1)
}

function markerPoint(theta, radius = MARKER_RADIUS) {
  return {
    x: CENTER + Math.cos(theta) * radius,
    y: CENTER - Math.sin(theta) * radius,
  }
}

function pointerAngle(event, element) {
  const rect = element.getBoundingClientRect()
  const x = event.clientX - rect.left
  const y = event.clientY - rect.top
  return wrapToPi(Math.atan2(CENTER - y, x - CENTER))
}

export default function RotationDial({ pose, disabled, onRotate }) {
  const svgRef = useRef(null)
  const activePointerIdRef = useRef(null)
  const dragThetaRef = useRef(null)
  const [dragTheta, setDragTheta] = useState(null)
  const activeTheta = dragTheta ?? pose?.theta ?? 0
  const currentMarker = useMemo(() => markerPoint(pose?.theta ?? 0), [pose?.theta])
  const targetMarker = useMemo(() => markerPoint(activeTheta), [activeTheta])

  function setPreviewTheta(theta) {
    dragThetaRef.current = theta
    setDragTheta(theta)
  }

  function clearPreviewTheta() {
    dragThetaRef.current = null
    setDragTheta(null)
  }

  function handlePointerDown(event) {
    if (disabled || !pose || !svgRef.current) return
    event.preventDefault()
    activePointerIdRef.current = event.pointerId
    event.currentTarget.setPointerCapture(event.pointerId)
    setPreviewTheta(pointerAngle(event, svgRef.current))
  }

  function handlePointerMove(event) {
    if (dragThetaRef.current === null || !svgRef.current || event.pointerId !== activePointerIdRef.current) return
    event.preventDefault()
    setPreviewTheta(pointerAngle(event, svgRef.current))
  }

  function handlePointerUp(event) {
    if (event.pointerId !== activePointerIdRef.current) return
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId)
    }
    activePointerIdRef.current = null
    const targetTheta = dragThetaRef.current
    clearPreviewTheta()
    if (targetTheta === null || disabled || !pose) return
    onRotate(targetTheta)
  }

  function cancelPreview(event) {
    if (event?.pointerId !== undefined && event.pointerId !== activePointerIdRef.current) return
    if (event?.currentTarget?.hasPointerCapture?.(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId)
    }
    activePointerIdRef.current = null
    if (dragThetaRef.current !== null) clearPreviewTheta()
  }

  const muted = disabled || !pose

  return (
    <div style={{ marginBottom: 14 }}>
      <div style={{ color: '#555', fontSize: 10, letterSpacing: 1, marginBottom: 6 }}>ROTATION</div>
      <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
        <svg
          ref={svgRef}
          width={SIZE}
          height={SIZE}
          viewBox={`0 0 ${SIZE} ${SIZE}`}
          onPointerDown={handlePointerDown}
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerUp}
          onPointerCancel={cancelPreview}
          style={{
            display: 'block',
            cursor: muted ? 'not-allowed' : dragTheta === null ? 'grab' : 'grabbing',
            opacity: muted ? 0.45 : 1,
            touchAction: 'none',
          }}
        >
          <circle cx={CENTER} cy={CENTER} r={RING_RADIUS} fill="#141414" stroke="#555" strokeWidth="1.5" />
          <circle cx={CENTER} cy={CENTER} r="11" fill="#60a5fa" stroke="#fff" strokeWidth="2" />

          <line
            x1={CENTER}
            y1={CENTER}
            x2={currentMarker.x}
            y2={currentMarker.y}
            stroke="#fff"
            strokeWidth="2"
            strokeLinecap="round"
          />
          <circle cx={currentMarker.x} cy={currentMarker.y} r="3.5" fill="#fff" />

          {dragTheta !== null && (
            <>
              <line
                x1={CENTER}
                y1={CENTER}
                x2={targetMarker.x}
                y2={targetMarker.y}
                stroke="#f97316"
                strokeWidth="2"
                strokeLinecap="round"
              />
              <circle cx={targetMarker.x} cy={targetMarker.y} r="4" fill="#f97316" />
            </>
          )}
        </svg>

        <div>
          <div style={{ color: '#888', marginBottom: 4 }}>current</div>
          <div style={{ color: '#eee' }}>{pose ? `${toDeg(pose.theta)}°` : '-'}</div>
          {dragTheta !== null && (
            <>
              <div style={{ color: '#888', marginTop: 8, marginBottom: 4 }}>target</div>
              <div style={{ color: '#f97316' }}>{toDeg(dragTheta)}°</div>
            </>
          )}
          {disabled && pose && <div style={{ color: '#666', marginTop: 8 }}>moving</div>}
        </div>
      </div>
    </div>
  )
}
