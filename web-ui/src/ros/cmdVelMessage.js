export const TELEOP_LINEAR_SCALE = 3 / 5

export function buildTwist(linearX = 0, angularZ = 0) {
  return {
    linear: { x: linearX, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angularZ },
  }
}

export function cmdVelTopic(namespace) {
  return `/${namespace.replace(/^\/+|\/+$/g, '')}/cmd_vel`
}

export function teleopVelocity(command, motion) {
  switch (command) {
    case 'forward':
      return { linearX: motion.linear_speed * TELEOP_LINEAR_SCALE, angularZ: 0 }
    case 'backward':
      return { linearX: -motion.linear_speed * TELEOP_LINEAR_SCALE, angularZ: 0 }
    case 'left':
      return { linearX: 0, angularZ: motion.angular_speed }
    case 'right':
      return { linearX: 0, angularZ: -motion.angular_speed }
    default:
      return { linearX: 0, angularZ: 0 }
  }
}
