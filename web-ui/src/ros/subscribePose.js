import ROSLIB from 'roslib'
import ros from './rosClient.js'

export function subscribePose(namespace, onPose) {
  const topic = new ROSLIB.Topic({
    ros,
    name: `/${namespace}/pose`,
    messageType: 'geometry_msgs/Pose2D',
  })
  topic.subscribe(onPose)
  return () => topic.unsubscribe()
}
