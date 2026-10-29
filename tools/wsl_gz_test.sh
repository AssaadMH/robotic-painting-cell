#!/bin/bash
# Short Gazebo render of the first painting strokes (smoothness check): frames 0..N-1 into ~/gz_test
cd /mnt/c/Users/HP/yuktii/painting_cell
[ "$2" = nobuild ] || bash tools/wsl_build.sh > /dev/null
bash tools/wsl_gz.sh stop > /dev/null
rm -rf ~/gz_test
setsid bash tools/wsl_gz.sh start &
sleep 25
bash tools/wsl_gz.sh play --plan results/plan.json --out ~/gz_test --start-frame ${3:-0} --max-frames ${1:-200} > ~/gz_test.log 2>&1
bash tools/wsl_gz.sh stop
ls ~/gz_test/frames | wc -l
