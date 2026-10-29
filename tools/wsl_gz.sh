#!/bin/bash
# usage: wsl_gz.sh start | play [gz_play args] | stop
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
case "$1" in
  start) exec ros2 launch painting_cell gz_cell.launch.py > ~/gz_launch.log 2>&1 ;;
  play)  shift; ros2 run painting_cell gz_play "$@" 2>&1 | grep -v "^WARNING:root" ;;
  stop)  pkill -f "[g]z_cell.launch" ; pkill -f "[g]z sim" ; pkill -f "[p]arameter_bridge" ; pkill -f "[r]obot_state_publisher" ; pkill -f "[g]z_play"
         sleep 2; pkill -9 -f "[g]z sim"; pgrep -fa "gz sim|gz_play" | cut -c1-80; echo stopped ;;
esac
