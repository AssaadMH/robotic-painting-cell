#!/usr/bin/env python3
"""Map IK reachability for a horizontal gun (pointing +y, towards the car) vs distance from the robot base."""
import numpy as np
import rclpy
from painting_cell import cell_common as cc
from painting_cell.plan_paint import CellPlanner

rclpy.init()
p = CellPlanner()
p.apply_scene(0.0, first=True)
print('tool pointing +y (horizontal), TCP at x=0, rail=0, rail base y=-2.2')
print('dist from base axis [m] ->  z=0.5  z=0.9  z=1.3  (IK without collision)')
for dist in (0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8):
    row = []
    for z in (0.5, 0.9, 1.3):
        tcp = np.array([0.0, -2.2 + dist, z])
        pose = cc.pose_from(tcp, cc.tool_frame([0, -1, 0], [0, 0, 1]))
        ok = p.ik(pose, seed=dict(p.state), avoid=False) is not None
        row.append('ok  ' if ok else '--  ')
    print('   %.1f                  ' % dist, '   '.join(row))
print()
print('tool pointing DOWN (roof):')
for dist in (1.0, 1.4, 1.8, 2.2, 2.6, 2.9):
    row = []
    for z in (1.8, 2.0):
        tcp = np.array([0.0, -2.2 + dist, z])
        pose = cc.pose_from(tcp, cc.tool_frame([0, 0, 1], [0, 1, 0]))
        ok = p.ik(pose, seed=dict(p.state), avoid=False) is not None
        row.append('ok  ' if ok else '--  ')
    print('   %.1f                  ' % dist, '   '.join(row))
rclpy.shutdown()
