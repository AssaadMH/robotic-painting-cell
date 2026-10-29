#!/bin/bash
# usage: wsl_video.sh render|assemble [args]
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
export PYOPENGL_PLATFORM=egl
python3 /mnt/c/Users/HP/yuktii/painting_cell/tools/make_video.py "$@" 2>&1 | grep -v "^WARNING:root"
