import ROSLIB from 'roslib'
import ros from './rosClient.js'
import { buildTwist, cmdVelTopic } from './cmdVelMessage.js'

export function createCmdVelPublisher(namespace) {
  const topic = new ROSLIB.Topic({
    ros,
    name: cmdVelTopic(namespace),
    messageType: 'geometry_msgs/Twist',
  })

  return {
    publish(linearX = 0, angularZ = 0) {
      topic.publish(new ROSLIB.Message(buildTwist(linearX, angularZ)))
    },
    unadvertise() {
      topic.unadvertise()
    },
  }
}
