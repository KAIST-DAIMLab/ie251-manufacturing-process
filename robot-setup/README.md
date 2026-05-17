# Robot-side bringup auto-start

Ships `turtlebot3_bringup` as a systemd service on each physical TurtleBot.
The wrapper probes a list of candidate ROS masters at boot and reconnects
automatically if the master goes down.

## Prerequisites (on the Pi)

- ROS Noetic at `/opt/ros/noetic`
- `ros-noetic-turtlebot3-bringup` installed
- A built catkin workspace at `~/catkin_ws` (path configurable in the env file)
- User `ubuntu` with passwordless `sudo`

## Install

From a workstation:

```bash
rsync -a robot-setup/ ubuntu@<robot-ip>:/tmp/robot-setup/
ssh ubuntu@<robot-ip> 'bash /tmp/robot-setup/install.sh'
```

Then on the robot, edit `/etc/default/turtlebot3` and set this robot's
`ROS_NAMESPACE`, `ROS_HOSTNAME`, and `ROS_MASTER_CANDIDATES`:

```bash
sudo $EDITOR /etc/default/turtlebot3
sudo systemctl restart turtlebot3-bringup
```

## Operation

```bash
# follow the live log
journalctl -u turtlebot3-bringup -f

# restart after editing the env file
sudo systemctl restart turtlebot3-bringup

# disable auto-start
sudo systemctl disable --now turtlebot3-bringup
```

## Adding master candidates

`ROS_MASTER_CANDIDATES` is a space-separated list. The wrapper tries each
entry in order and uses the first one whose XML-RPC port is reachable.
Append entries as you add laptops or networks:

```
ROS_MASTER_CANDIDATES="http://192.168.0.4:11311 http://192.168.0.5:11311 http://laptop.local:11311"
```

If the active master disappears, `roslaunch` exits, systemd restarts the
service, and the probe loop picks up the next reachable candidate without
manual intervention.

## TF remap note

The wrapper remaps `/tf` → `/${ROS_NAMESPACE}/raw_tf` and
`/tf_static` → `/${ROS_NAMESPACE}/raw_tf_static`, matching
`src/pathfinder/scripts/real_robot_bringup`. This keeps the per-robot TF
tree from polluting the global one and stays compatible with the laptop's
`real_robot_localization` stack.
