#!/bin/bash
# (WSL) car meshes -> spray paths -> copy into the ROS package
cd /mnt/c/Users/HP/yuktii/painting_cell
python3 -c 'import rtree, scipy; print("rtree ok, scipy", scipy.__version__)' || exit 1
time python3 tools/gen_paths.py 2>&1 | grep -v WARNING
cp results/paths.json src/painting_cell/paths/paths.json
