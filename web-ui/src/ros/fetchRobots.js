import ROSLIB from 'roslib'
import ros from './rosClient.js'

export function fetchRobots() {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros,
      name: '/fleet/get_robots',
      serviceType: 'pathfinder/GetRobots',
    })
    client.callService(new ROSLIB.ServiceRequest({}), (response) => {
      resolve(JSON.parse(response.robots_json))
    }, reject)
  })
}
