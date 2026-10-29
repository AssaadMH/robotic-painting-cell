#!/bin/bash
# Sweep rail offset x pedestal height; for each, start move_group and run layout_probe.py
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/HP/yuktii/painting_cell && bash tools/wsl_build.sh >/dev/null 2>&1
source ~/ws/install/setup.bash
pkill -f "ros2 launch" ; pkill -f move_group ; pkill -f robot_state_publisher ; pkill -f joint_state_publisher; sleep 2
for ped in 0.05 0.45 0.90; do
  for ry in -2.4 -2.6 -2.8 -3.0; do
    : > ~/sweep_launch.log
    ros2 launch painting_cell cell.launch.py rviz:=false mode:=plan rail_y:=$ry pedestal_h:=$ped > ~/sweep_launch.log 2>&1 &
    LP=$!
    for i in $(seq 1 60); do grep -q "You can start planning now" ~/sweep_launch.log && break; sleep 1; done
    python3 /mnt/c/Users/HP/yuktii/painting_cell/tools/layout_probe.py $ry $ped 2>&1 | grep RESULT
    pkill -f "ros2 launch" ; pkill -f move_group ; pkill -f robot_state_publisher ; pkill -f joint_state_publisher
    sleep 3
  done
done
echo SWEEP_DONE
