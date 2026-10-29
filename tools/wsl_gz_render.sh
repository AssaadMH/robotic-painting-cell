#!/bin/bash
# Full Gazebo render of the demo into ~/gz_render (detached from the caller; progress in ~/gz_render.log)
cd /mnt/c/Users/HP/yuktii/painting_cell
echo "build $(date)" > ~/gz_render.step; bash tools/wsl_build.sh >> ~/gz_render.step 2>&1; echo "start $(date)" >> ~/gz_render.step
bash tools/wsl_gz.sh stop > /dev/null
rm -rf ~/gz_render
setsid bash tools/wsl_gz.sh start &
sleep 30
source /opt/ros/jazzy/setup.bash; source ~/ws/install/setup.bash
PYTHONUNBUFFERED=1 ros2 run painting_cell gz_play --plan results/plan.json --out ~/gz_render > ~/gz_render.log 2>&1
bash tools/wsl_gz.sh stop > /dev/null
echo finished >> ~/gz_render.log
