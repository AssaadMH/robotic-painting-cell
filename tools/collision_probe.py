#!/usr/bin/env python3
"""For one stroke: plan the Cartesian pass WITHOUT collision checking, then report which link pairs collide."""
import sys, collections
import numpy as np
import rclpy
from painting_cell import cell_common as cc
from painting_cell.plan_paint import CellPlanner

surf, idx = sys.argv[1], int(sys.argv[2])
cross = float(sys.argv[3]) if len(sys.argv) > 3 else None
rclpy.init()
p = CellPlanner()
p.standoff = 0.25
p.apply_scene(0.0, first=True)
data = cc.load_paths()
st = [s for ph in data['phases'] for s in ph['surfaces'] if s['name'] == surf][0]['strokes'][idx]
P = np.array(st['points']) + np.array([0, 0, cc.CAR_Z]); N = np.array(st['normals'])
d = np.gradient(P, axis=0); d /= np.linalg.norm(d, axis=1, keepdims=True)
tcp = P + N * 0.25
poses = [cc.pose_from(tcp[i], cc.tool_frame(N[i], d[i])) for i in range(len(P))]
state = dict(p.state)
state[cc.RAIL_JOINT] = st['rail_station']
state[cc.CROSS_JOINT] = cross if cross is not None else (cc.CROSS_SIDE if surf.startswith('side') else cc.CROSS_TOP)
pre = cc.pose_from(tcp[0] + N[0] * 0.2, cc.tool_frame(N[0], d[0]))
sol = p.ik(pre, seed=state, avoid=False)
print('IK(no collision) pre pose:', 'ok' if sol else 'FAIL')
if sol:
    frac, jt = p.cartesian(poses, seed=sol, avoid=False)
    print('cartesian fraction (no collision check): %.2f, %d points' % (frac, len(jt.points)))
    pairs = collections.Counter()
    for pt in jt.points[::2]:
        s = dict(sol)
        for n, q in zip(jt.joint_names, pt.positions):
            s[n] = q
        valid, contacts = p.state_valid(s)
        if not valid:
            for c in contacts:
                pairs[tuple(sorted((c.contact_body_1, c.contact_body_2)))] += 1
    print('colliding pairs along the pass:', dict(pairs) if pairs else 'none')
    valid, contacts = p.state_valid(sol)
    print('pre pose valid:', valid, [(c.contact_body_1, c.contact_body_2) for c in contacts[:4]])
rclpy.shutdown()
