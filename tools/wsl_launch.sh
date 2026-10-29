#!/bin/bash
# usage: wsl_launch.sh <launch args...>   (starts move_group etc., log in ~/launch.log)
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
exec ros2 launch painting_cell cell.launch.py "$@" > ~/launch.log 2>&1
