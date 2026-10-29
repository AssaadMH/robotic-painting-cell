#!/usr/bin/env python3
"""Score a rail/pedestal layout: IK reachability of side-panel, roof and hood poses (no collision check)."""
import sys
import numpy as np
import rclpy
from painting_cell import cell_common as cc
from painting_cell.plan_paint import CellPlanner

rail_y = float(sys.argv[1]); ped = float(sys.argv[2])
rclpy.init()
p = CellPlanner()
p.apply_scene(0.0, first=True)
rel = lambda y: y - rail_y            # distance from the base axis along y


def test(tcp, n, d):
    return p.ik(cc.pose_from(np.array(tcp), cc.tool_frame(n, d)), seed=dict(p.state), avoid=False) is not None


res = {}
# side panel (R side, y=-0.95): paint pose 0.25 off, pre-pose 0.45 off; four heights
side = []
for z in (0.45, 0.75, 1.05, 1.35):
    side.append(test([0, -0.95 - 0.25, z], [0, -1, 0], [0, 0, 1]))
    side.append(test([0, -0.95 - 0.45, z], [0, -1, 0], [0, 0, 1]))
roof, hood, wind = [], [], []
for y in (-0.72, -0.45, -0.2, 0.0):
    roof.append(test([0, y, 1.61 + 0.25], [0, 0, 1], [0, 1, 0]))
    roof.append(test([0, y, 1.61 + 0.45], [0, 0, 1], [0, 1, 0]))
    hood.append(test([1.4, y, 0.88 + 0.25 + 0.0], [0, 0, 1], [0, 1, 0]))
    hood.append(test([1.4, y, 0.88 + 0.45], [0, 0, 1], [0, 1, 0]))
print('RESULT rail_y=%.2f pedestal=%.2f  side %d/%d  roof %d/%d  hood %d/%d' %
      (rail_y, ped, sum(side), len(side), sum(roof), len(roof), sum(hood), len(hood)))
rclpy.shutdown()
