#!/bin/bash
# stop all launched ROS processes (pattern trick: [m] keeps pkill from matching this script)
pkill -f "[r]os2 launch" ; pkill -f "[m]ove_group" ; pkill -f "[r]obot_state_publisher" ; pkill -f "[j]oint_state_publisher" ; pkill -f "[r]viz2" ; pkill -f "[p]lay_paint"
sleep 2
pgrep -fa "move_group|robot_state|rviz2|play_paint" | cut -c1-60 || true
echo stopped
