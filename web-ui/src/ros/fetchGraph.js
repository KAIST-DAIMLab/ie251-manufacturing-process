import ROSLIB from 'roslib'
import ros from './rosClient.js'

export function fetchGraph() {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros,
      name: '/fleet/get_graph',
      serviceType: 'pathfinder/GetGraph',
    })
    client.callService(new ROSLIB.ServiceRequest({}), (response) => {
      resolve(JSON.parse(response.graph_json))
    }, reject)
  })
}
