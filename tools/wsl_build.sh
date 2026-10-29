#!/bin/bash
# Sync the Windows source tree into the WSL workspace and build it.
set -e
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ws/src
rsync -a --delete /mnt/c/Users/HP/yuktii/painting_cell/src/ ~/ws/src/
cd ~/ws
colcon build --symlink-install --event-handlers console_cohesion- 2>&1 | tail -15
