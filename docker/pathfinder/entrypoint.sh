#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps pathfinder -C /workspace
source /workspace/devel/setup.bash

while true; do
    echo "[pathfinder] Waiting for roscore at $ROS_MASTER_URI..."
    until rostopic list > /dev/null 2>&1; do sleep 2; done

    echo "[pathfinder] roscore detected, launching..."
    roslaunch pathfinder robots.launch "$@" \
        > >(grep --line-buffered -v 'XmlRpcClient\|XmlRpcDispatch') 2>&1 &
    LAUNCH_PID=$!

    while kill -0 $LAUNCH_PID 2>/dev/null; do
        if ! rostopic list > /dev/null 2>&1; then
            echo "[pathfinder] roscore lost, stopping roslaunch..."
            kill $LAUNCH_PID 2>/dev/null || true
            wait $LAUNCH_PID 2>/dev/null || true
            break
        fi
        sleep 0.5
    done

    echo "[pathfinder] Waiting for roscore..."
    sleep 2
done
