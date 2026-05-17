import ROSLIB from 'roslib'
import ros from './rosClient.js'

export function relocalize(namespace, x, y, theta) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros,
      name: `/${namespace}/relocalize`,
      serviceType: 'pathfinder/Relocalize',
    })
    client.callService(new ROSLIB.ServiceRequest({ x, y, theta }), resolve, reject)
  })
}
