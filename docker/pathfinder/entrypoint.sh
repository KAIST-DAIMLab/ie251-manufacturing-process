#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps pathfinder -C /workspace
source /workspace/devel/setup.bash

exec roslaunch pathfinder robots.launch "$@"
