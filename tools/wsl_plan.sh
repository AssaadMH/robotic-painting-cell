#!/bin/bash
# usage: wsl_plan.sh [plan_paint args]   (move_group must be running)
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
ros2 run painting_cell plan_paint "$@" 2>&1 | grep -v "^WARNING:root"
