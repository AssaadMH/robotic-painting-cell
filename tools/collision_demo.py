#!/usr/bin/env python3
"""Collision-checking demonstration data (needs move_group running).
Takes one hood stroke and plans it twice with MoveIt's collision-aware Cartesian planner:
  * BAD  : nozzle 4 cm *inside* the car surface  -> expected to be rejected
  * GOOD : nozzle at the 250 mm standoff         -> expected to pass
For the bad path the unconstrained trajectory is re-checked state by state to find the first collision."""
import json, sys
import numpy as np
import rclpy
from painting_cell import cell_common as cc
from painting_cell.plan_paint import CellPlanner

out = sys.argv[1] if len(sys.argv) > 1 else '/home/assaad/collision_demo.json'
rclpy.init()
p = CellPlanner()
p.standoff = 0.25
p.apply_scene(0.0, first=True)
data = cc.load_paths()
sf = [s for s in data['phases'][0]['surfaces'] if s['name'] == 'hood_R'][0]
st = sf['strokes'][2]
P = np.array(st['points']) + np.array([0, 0, cc.CAR_Z]); N = np.array(st['normals'])
d = np.gradient(P, axis=0); d /= np.linalg.norm(d, axis=1, keepdims=True)
state = dict(p.state); state[cc.RAIL_JOINT] = st['rail_station']; state[cc.CROSS_JOINT] = cc.CROSS_TOP


def run(offset, label):
    tcp = P + N * offset
    poses = [cc.pose_from(tcp[i], cc.tool_frame(N[i], d[i])) for i in range(len(P))]
    pre = cc.pose_from(tcp[0] + N[0] * 0.20, cc.tool_frame(N[0], d[0]))
    sol = p.ik_multi(pre, state)
    if sol is None:
        raise SystemExit('no IK for the pre pose')
    # same two-step procedure as the real planner: straight-in approach, then the surface-following pass
    def last(jt, base):
        d = dict(base)
        for n, q in zip(jt.joint_names, jt.points[-1].positions):
            d[n] = q
        return d
    fa, ja = p.cartesian([poses[0]], seed=sol, avoid=True)
    fa_free, ja_free = p.cartesian([poses[0]], seed=sol, avoid=False)
    start2 = last(ja, sol) if fa >= 0.995 else last(ja_free, sol)
    fp, jp = p.cartesian(poses[1:], seed=start2, avoid=True)
    fp_free, jp_free = p.cartesian(poses[1:], seed=last(ja_free, sol), avoid=False)
    f_chk = fa if fa < 0.995 else fp               # the first pass that fails decides
    jt_free = ja_free
    jt_free.points = list(ja_free.points) + list(jp_free.points)
    jt_free.joint_names = ja_free.joint_names
    states, valid, pairs = [], [], []
    for pt in jt_free.points:
        s = dict(sol)
        for n, q in zip(jt_free.joint_names, pt.positions):
            s[n] = q
        v, contacts = p.state_valid(s)
        states.append([s[j] for j in cc.ALL_JOINTS]); valid.append(bool(v))
        pairs.append(sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in contacts}))
    first_bad = next((i for i, v in enumerate(valid) if not v), None)
    print('%s: offset %+.2f m  collision-aware cartesian fraction %.2f ; unconstrained path has %d/%d invalid states ; first collision at point %s %s' %
          (label, offset, f_chk, valid.count(False), len(valid), first_bad, pairs[first_bad] if first_bad is not None else ''))
    return {'label': label, 'offset': offset, 'fraction_collision_aware': f_chk, 'n_points': len(states),
            'n_invalid': valid.count(False), 'first_invalid': first_bad,
            'contacts_at_first': [list(x) for x in pairs[first_bad]] if first_bad is not None else [],
            'states': states, 'valid': valid, 'start': [sol[j] for j in cc.ALL_JOINTS]}


res = {'stroke': 'hood_R[2]', 'bad': run(-0.04, 'BAD  (nozzle 4 cm inside the hood)'),
       'good': run(0.25, 'GOOD (250 mm standoff)')}
json.dump(res, open(out, 'w'))
print('wrote', out)
rclpy.shutdown()
