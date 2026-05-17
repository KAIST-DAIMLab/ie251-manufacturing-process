import ROSLIB from 'roslib'
import ros from './rosClient.js'

export function subscribeState(namespace, onState) {
  const topic = new ROSLIB.Topic({
    ros,
    name: `/${namespace}/state`,
    messageType: 'std_msgs/Int8',
  })
  topic.subscribe(onState)
  return () => topic.unsubscribe()
}
