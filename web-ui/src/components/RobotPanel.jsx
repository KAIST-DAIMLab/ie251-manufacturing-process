import React from 'react'
import TeleopControls from './TeleopControls.jsx'

const toDeg = (rad) => ((rad * 180) / Math.PI).toFixed(1)

function Row({ label, value }) {
  return (
    <tr>
      <td style={{ color: '#888', paddingRight: 12, paddingBottom: 4, verticalAlign: 'top' }}>{label}</td>
      <td style={{ color: '#eee', paddingBottom: 4 }}>{value}</td>
    </tr>
  )
}

function Section({ title, children }) {
  return (
    <div style={{ marginBottom: 14 }}>
      <div style={{ color: '#555', fontSize: 10, letterSpacing: 1, marginBottom: 6 }}>{title}</div>
      <table style={{ borderSpacing: 0 }}>
        <tbody>{children}</tbody>
      </table>
    </div>
  )
}

export default function RobotPanel({ robot, pose, pathStatus, onStop, onJogError }) {
  if (!robot) {
    return (
      <div style={{ padding: 16, color: '#444', fontSize: 13 }}>
        Select a robot to see config
      </div>
    )
  }

  const label = robot.name || robot.id

  return (
    <div style={{ padding: 16, fontSize: 13 }}>
      <div style={{ color: '#facc15', fontSize: 14, letterSpacing: 1, marginBottom: 10 }}>{label}</div>

      <button
        onClick={() => onStop(robot.id)}
        style={{
          marginBottom: 14,
          padding: '6px 12px',
          background: '#ef4444',
          color: '#fff',
          border: 'none',
          fontSize: 13,
          cursor: 'pointer',
        }}
      >
        Stop
      </button>

      <Section title="STATUS">
        {(() => {
          const moving = pathStatus && pathStatus.node_ids.length > 0
          return (
            <>
              <Row label="state" value={moving ? 'MOVING' : 'IDLE'} />
              <Row
                label="target"
                value={moving ? `node ${pathStatus.node_ids[pathStatus.node_ids.length - 1]}` : '—'}
              />
              <Row
                label="step"
                value={moving ? `${pathStatus.current_index + 1} / ${pathStatus.node_ids.length}` : '—'}
              />
            </>
          )
        })()}
      </Section>

      <Section title="IDENTITY">
        <Row label="namespace" value={robot.namespace} />
        <Row label="start_station" value={robot.start_station} />
      </Section>

      <Section title="MOTION">
        <Row label="linear_speed" value={`${robot.motion.linear_speed} m/s`} />
        <Row label="angular_speed" value={`${robot.motion.angular_speed} rad/s`} />
        <Row label="move_rate" value={`${robot.motion.move_rate_hz} Hz`} />
        <Row label="arrival_tol" value={`${robot.motion.arrival_tolerance} m`} />
        <Row label="turning_tol" value={`${(robot.motion.turning_tolerance * 180 / Math.PI).toFixed(1)}°`} />
      </Section>

      <Section title="OBSTACLE">
        <Row label="enabled" value={robot.obstacle.enabled ? 'yes' : 'no'} />
        <Row label="stop_dist" value={`${robot.obstacle.stop_distance} m`} />
        <Row label="detect" value={`${robot.obstacle.detect_degree}°`} />
      </Section>

      <TeleopControls robot={robot} onError={onJogError} />

      <Section title="LIVE POSE">
        {pose ? (
          <>
            <Row label="x" value={`${pose.x.toFixed(3)} m`} />
            <Row label="y" value={`${pose.y.toFixed(3)} m`} />
            <Row label="θ" value={`${toDeg(pose.theta)}°`} />
          </>
        ) : (
          <Row label="status" value="—" />
        )}
      </Section>
    </div>
  )
}
