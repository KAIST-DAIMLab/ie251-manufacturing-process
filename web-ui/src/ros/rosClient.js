import ROSLIB from 'roslib'

const url = import.meta.env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
const RECONNECT_DELAY_MS = 1500

const ros = new ROSLIB.Ros({ url })

let reconnectTimer = null

function scheduleReconnect() {
  if (reconnectTimer || ros.isConnected) return
  reconnectTimer = setTimeout(() => {
    reconnectTimer = null
    if (ros.isConnected) return
    try {
      ros.connect(url)
    } catch (_error) {
      scheduleReconnect()
    }
  }, RECONNECT_DELAY_MS)
}

ros.on('connection', () => console.log('rosbridge connected'))
ros.on('error', (error) => console.error('rosbridge error:', error))
ros.on('close', () => {
  console.warn('rosbridge disconnected')
  scheduleReconnect()
})

export default ros
