import React from 'react'
import { colorForState } from '../util/robotColor.js'

export default function RobotList({ robots, stateMap, selectedRobotId, onSelect }) {
  return (
    <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap', padding: 8, background: '#1e1e1e' }}>
      {robots.map((robot) => {
        const stateValue = stateMap?.[robot.id] ?? 0
        const fill = colorForState(robot, stateValue)
        const selected = robot.id === selectedRobotId
        const offline = stateValue === 0
        return (
          <button
            key={robot.id}
            onClick={offline ? undefined : () => onSelect(robot.id)}
            disabled={offline}
            style={{
              display: 'flex',
              alignItems: 'center',
              gap: 6,
              padding: '4px 10px',
              background: '#262626',
              border: `1px solid ${selected ? '#fff' : '#333'}`,
              color: offline ? '#9ca3af' : '#eee',
              fontSize: 13,
              cursor: offline ? 'not-allowed' : 'pointer',
            }}
          >
            <span
              style={{
                width: 10,
                height: 10,
                borderRadius: '50%',
                background: fill,
                opacity: offline ? 0.35 : 1.0,
              }}
            />
            {robot.name || robot.id}
          </button>
        )
      })}
    </div>
  )
}
