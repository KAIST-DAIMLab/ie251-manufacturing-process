import React, { useEffect, useMemo, useRef, useState } from 'react'
import { cancelPath } from '../ros/cancelPath.js'
import { createCmdVelPublisher } from '../ros/cmdVel.js'
import { teleopVelocity } from '../ros/cmdVelMessage.js'

const PUBLISH_INTERVAL_MS = 100

const COMMANDS = {
  forward: {
    label: 'Move forward',
    gridArea: 'forward',
  },
  backward: {
    label: 'Move backward',
    gridArea: 'backward',
  },
  left: {
    label: 'Turn left',
    gridArea: 'left',
  },
  right: {
    label: 'Turn right',
    gridArea: 'right',
  },
}

function TeleopIcon({ command }) {
  if (command === 'forward' || command === 'backward') {
    const points = command === 'forward' ? '12,3 21,19 3,19' : '3,5 21,5 12,21'
    return (
      <svg width="24" height="24" viewBox="0 0 24 24" aria-hidden="true">
        <polygon points={points} fill="currentColor" />
      </svg>
    )
  }

  const mirrored = command === 'right'
  return (
    <svg
      width="24"
      height="24"
      viewBox="0 0 24 24"
      aria-hidden="true"
      style={{ transform: mirrored ? 'scaleX(-1)' : 'none' }}
    >
      <path
        d="M8 7.5H4.5V4"
        fill="none"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <path
        d="M4.9 7.1A7.4 7.4 0 1 1 4.4 16"
        fill="none"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
      />
    </svg>
  )
}

function formatError(error) {
  if (!error) return 'teleop failed'
  if (typeof error === 'string') return error
  return error.message || String(error)
}

export default function TeleopControls({ robot, onError }) {
  const publisher = useMemo(() => createCmdVelPublisher(robot.namespace), [robot.namespace])
  const holdRef = useRef(null)
  const intervalRef = useRef(null)
  const publisherRef = useRef(publisher)
  const [activeCommand, setActiveCommand] = useState(null)
  const [pendingCommand, setPendingCommand] = useState(null)

  useEffect(() => {
    publisherRef.current = publisher
    return () => {
      stopTeleop()
      publisher.unadvertise()
    }
  }, [publisher])

  useEffect(() => {
    window.addEventListener('blur', stopTeleop)
    return () => window.removeEventListener('blur', stopTeleop)
  }, [])

  function publishStop() {
    publisherRef.current.publish(0, 0)
  }

  function stopTeleop() {
    if (intervalRef.current !== null) {
      clearInterval(intervalRef.current)
      intervalRef.current = null
    }
    if (holdRef.current !== null) {
      publishStop()
    }
    holdRef.current = null
    setActiveCommand(null)
    setPendingCommand(null)
  }

  function startPublishing(token, velocity, command) {
    if (holdRef.current !== token) return
    publisherRef.current.publish(velocity.linearX, velocity.angularZ)
    intervalRef.current = window.setInterval(() => {
      if (holdRef.current === token) {
        publisherRef.current.publish(velocity.linearX, velocity.angularZ)
      }
    }, PUBLISH_INTERVAL_MS)
    setPendingCommand(null)
    setActiveCommand(command)
  }

  function handlePointerDown(event, command) {
    event.preventDefault()
    if (holdRef.current !== null) return
    event.currentTarget.setPointerCapture(event.pointerId)

    const token = { pointerId: event.pointerId }
    holdRef.current = token
    setPendingCommand(command)

    cancelPath(robot.id)
      .then((response) => {
        if (!response.success) throw new Error(response.message)
        startPublishing(token, teleopVelocity(command, robot.motion), command)
      })
      .catch((error) => {
        if (holdRef.current === token) stopTeleop()
        onError?.(formatError(error))
      })
  }

  function handlePointerUp(event) {
    if (holdRef.current?.pointerId !== event.pointerId) return
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId)
    }
    event.preventDefault()
    stopTeleop()
  }

  const isBusy = holdRef.current !== null

  return (
    <div style={{ marginBottom: 14 }}>
      <div style={{ color: '#555', fontSize: 10, letterSpacing: 1, marginBottom: 6 }}>TELEOP</div>
      <div
        style={{
          display: 'grid',
          gridTemplateAreas: '". forward ." "left . right" ". backward ."',
          gridTemplateColumns: '40px 40px 40px',
          gridTemplateRows: '40px 40px 40px',
          gap: 8,
        }}
      >
        {Object.entries(COMMANDS).map(([command, config]) => (
          <button
            key={command}
            type="button"
            aria-label={config.label}
            title={config.label}
            onPointerDown={(event) => handlePointerDown(event, command)}
            onPointerUp={handlePointerUp}
            onPointerCancel={handlePointerUp}
            onLostPointerCapture={handlePointerUp}
            style={{
              ...buttonStyle(activeCommand === command || pendingCommand === command, isBusy),
              gridArea: config.gridArea,
            }}
          >
            <TeleopIcon command={command} />
          </button>
        ))}
      </div>
    </div>
  )
}

function buttonStyle(active, busy) {
  return {
    width: 40,
    height: 40,
    display: 'grid',
    placeItems: 'center',
    border: '1px solid #555',
    background: active ? '#2563eb' : '#141414',
    color: active ? '#fff' : '#d1d5db',
    cursor: busy && !active ? 'not-allowed' : 'pointer',
    opacity: busy && !active ? 0.45 : 1,
    touchAction: 'none',
  }
}
