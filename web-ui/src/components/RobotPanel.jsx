import React from 'react'

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

export default function RobotPanel({ robot, pose }) {
  if (!robot) {
    return (
      <div style={{ padding: 16, color: '#444', fontSize: 13 }}>
        Select a robot to see config
      </div>
    )
  }

  return (
    <div style={{ padding: 16, fontSize: 13 }}>
      <div style={{ color: '#facc15', fontSize: 14, letterSpacing: 1, marginBottom: 14 }}>{robot.id}</div>

      <Section title="IDENTITY">
        <Row label="namespace" value={robot.namespace} />
        <Row label="start_node" value={robot.start_node} />
        <Row label="yaw" value={`${robot.yaw} rad`} />
      </Section>

      <Section title="MOTION">
        <Row label="linear_speed" value={`${robot.motion.linear_speed} m/s`} />
        <Row label="angular_speed" value={`${robot.motion.angular_speed} rad/s`} />
        <Row label="move_rate" value={`${robot.motion.move_rate_hz} Hz`} />
        <Row label="arrival_tol" value={`${robot.motion.arrival_tolerance} m`} />
      </Section>

      <Section title="OBSTACLE">
        <Row label="enabled" value={robot.obstacle.enabled ? 'yes' : 'no'} />
        <Row label="stop_dist" value={`${robot.obstacle.stop_distance} m`} />
        <Row label="detect" value={`${robot.obstacle.detect_degree}°`} />
      </Section>

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
