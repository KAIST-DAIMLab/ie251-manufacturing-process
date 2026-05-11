import ROSLIB from 'roslib'
import ros from './rosClient.js'

export function cancelPath(robotId) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros,
      name: '/fleet/cancel_path',
      serviceType: 'pathfinder/CancelPath',
    })
    client.callService(
      new ROSLIB.ServiceRequest({ robot_id: robotId }),
      resolve,
      reject,
    )
  })
}
