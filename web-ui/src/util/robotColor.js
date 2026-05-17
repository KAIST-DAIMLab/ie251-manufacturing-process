const COLORS = ['#f87171', '#60a5fa', '#a78bfa', '#34d399']

export function colorForState(robot, stateValue) {
  if (stateValue === 0) return '#6b7280'
  if (stateValue === 3) return '#f59e0b'
  const index = parseInt(robot.id.replace(/\D/g, ''), 10) % COLORS.length
  return COLORS[index]
}
