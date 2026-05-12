import ROSLIB from 'roslib'
import ros from './rosClient.js'

export function subscribePathStatus(namespace, onStatus) {
  const topic = new ROSLIB.Topic({
    ros,
    name: `/${namespace}/path_status`,
    messageType: 'std_msgs/String',
  })
  topic.subscribe((message) => onStatus(JSON.parse(message.data)))
  return () => topic.unsubscribe()
}
