#!/usr/bin/env bash
set -euo pipefail

# Stop + disable the unit (no-op if already inactive/disabled).
if systemctl list-unit-files turtlebot3-bringup.service >/dev/null 2>&1; then
  sudo systemctl disable --now turtlebot3-bringup.service || true
fi

# Remove every file install.sh creates.
sudo rm -f /etc/systemd/system/turtlebot3-bringup.service
sudo rm -f /usr/local/bin/turtlebot3-bringup
sudo rm -f /etc/default/turtlebot3.example
sudo rm -f /etc/default/turtlebot3

sudo systemctl daemon-reload
sudo systemctl reset-failed turtlebot3-bringup.service 2>/dev/null || true

echo "Uninstalled. /etc/default/turtlebot3 (your edited env) was removed too."
