#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo install -m 644 "$SCRIPT_DIR/turtlebot3.env.example" /etc/default/turtlebot3.example
if [[ ! -f /etc/default/turtlebot3 ]]; then
  sudo install -m 644 "$SCRIPT_DIR/turtlebot3.env.example" /etc/default/turtlebot3
  echo "Edit /etc/default/turtlebot3 to set ROS_NAMESPACE, ROS_HOSTNAME, and ROS_MASTER_CANDIDATES for this robot."
fi

sudo install -m 755 "$SCRIPT_DIR/turtlebot3-bringup" /usr/local/bin/turtlebot3-bringup
sudo install -m 644 "$SCRIPT_DIR/turtlebot3-bringup.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now turtlebot3-bringup.service

echo "Installed. Tail logs with:  journalctl -u turtlebot3-bringup -f"
