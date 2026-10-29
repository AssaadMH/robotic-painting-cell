#!/bin/bash
# usage: wsl_py.sh script.py [args]   -- run a script with the ROS + workspace environment
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
python3 "$@" 2>&1 | grep -v "^WARNING:root"
