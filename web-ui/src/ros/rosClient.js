import ROSLIB from 'roslib'

const url = import.meta.env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'

const ros = new ROSLIB.Ros({ url })

ros.on('connection', () => console.log('rosbridge connected'))
ros.on('error', (error) => console.error('rosbridge error:', error))
ros.on('close', () => console.warn('rosbridge disconnected'))

export default ros
