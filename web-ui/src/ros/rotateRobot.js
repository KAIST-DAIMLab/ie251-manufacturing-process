import ROSLIB from 'roslib'
import ros from './rosClient.js'

export function rotateRobot(robotId, targetTheta) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros,
      name: '/fleet/rotate_robot',
      serviceType: 'pathfinder/RotateRobot',
    })
    client.callService(
      new ROSLIB.ServiceRequest({ robot_id: robotId, target_theta: targetTheta }),
      resolve,
      reject,
    )
  })
}
