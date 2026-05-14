import assert from 'node:assert/strict'
import { describe, it } from 'node:test'

import { buildTwist, cmdVelTopic, teleopVelocity } from './cmdVelMessage.js'

describe('cmdVelMessage', () => {
  it('builds a forward Twist with angular velocity zeroed', () => {
    assert.deepEqual(buildTwist(0.22), {
      linear: { x: 0.22, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    })
  })

  it('builds a backward Twist with negative linear velocity', () => {
    assert.equal(buildTwist(-0.22).linear.x, -0.22)
  })

  it('builds a left turn Twist with linear velocity zeroed', () => {
    assert.deepEqual(buildTwist(0, 1.5), {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 1.5 },
    })
  })

  it('builds a right turn Twist with negative angular velocity', () => {
    assert.equal(buildTwist(0, -1.5).angular.z, -1.5)
  })

  it('builds a stop Twist', () => {
    assert.deepEqual(buildTwist(), {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    })
  })

  it('builds a namespaced cmd_vel topic', () => {
    assert.equal(cmdVelTopic('/tb3_01/'), '/tb3_01/cmd_vel')
  })

  it('uses three-fifths speed for forward teleop', () => {
    assert.deepEqual(teleopVelocity('forward', { linear_speed: 0.5, angular_speed: 1.5 }), {
      linearX: 0.3,
      angularZ: 0,
    })
  })

  it('uses three-fifths speed for backward teleop', () => {
    assert.deepEqual(teleopVelocity('backward', { linear_speed: 0.5, angular_speed: 1.5 }), {
      linearX: -0.3,
      angularZ: 0,
    })
  })

  it('keeps full angular speed for turn teleop', () => {
    assert.deepEqual(teleopVelocity('right', { linear_speed: 0.5, angular_speed: 1.5 }), {
      linearX: 0,
      angularZ: -1.5,
    })
  })
})
