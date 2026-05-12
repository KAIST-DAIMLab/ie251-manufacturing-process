import ROSLIB from 'roslib'
import ros from './rosClient.js'

export function moveToNode(robotId, targetNodeId) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros,
      name: '/fleet/move_to_node',
      serviceType: 'pathfinder/MoveToNode',
    })
    client.callService(
      new ROSLIB.ServiceRequest({ robot_id: robotId, target_node_id: targetNodeId }),
      resolve,
      reject,
    )
  })
}
