#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps pathfinder -C /workspace
source /workspace/devel/setup.bash

while true; do
    echo "[pathfinder] Waiting for roscore at $ROS_MASTER_URI..."
    until rostopic list > /dev/null 2>&1; do sleep 2; done

    echo "[pathfinder] roscore detected, launching..."
    roslaunch pathfinder robots.launch "$@" || true

    echo "[pathfinder] roslaunch exited, waiting for roscore..."
    sleep 2
done
