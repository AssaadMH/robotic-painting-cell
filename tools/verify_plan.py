#!/usr/bin/env python3
"""Independent verification of a recorded plan (needs move_group running):
  1. collision: every Nth sample of every segment is re-checked against the full scene (car at its yaw)
  2. paint passes: forward kinematics of the nozzle -> speed, and distance to the planned stroke polyline
"""
import json, sys, time
import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionFK
from painting_cell import cell_common as cc
from painting_cell.plan_paint import CellPlanner

plan_file = sys.argv[1] if len(sys.argv) > 1 else '/home/assaad/plan.json'
out_file = sys.argv[2] if len(sys.argv) > 2 else '/home/assaad/verify.json'
plan = json.load(open(plan_file))
paths = cc.load_paths()
standoff = paths['params']['standoff']
strokes = {}
for ph in paths['phases']:
    for sf in ph['surfaces']:
        for i, s in enumerate(sf['strokes']):
            strokes[(sf['name'], i)] = s

rclpy.init()
p = CellPlanner()
p.standoff = standoff
p.apply_scene(0.0, first=True)
fk = p.create_client(GetPositionFK, '/compute_fk')
fk.wait_for_service()


def tcp_of(q):
    req = GetPositionFK.Request()
    req.header.frame_id = cc.FRAME
    req.fk_link_names = [cc.TCP_LINK]
    req.robot_state = p.robot_state(dict(zip(cc.ALL_JOINTS, q)))
    r = p.call(fk, req)
    ps = r.pose_stamped[0].pose.position
    return np.array([ps.x, ps.y, ps.z])


STRIDE = 3
checked = invalid = 0
bad = []
t0 = time.time()
for si, seg in enumerate(plan['segments']):
    if seg['kind'] == 'rotate':
        continue
    if abs(seg['yaw'] - p.yaw) > 1e-6:
        p.apply_scene(seg['yaw'])
    pos = seg['positions']
    idxs = list(range(0, len(pos), STRIDE))
    if idxs and idxs[-1] != len(pos) - 1:
        idxs.append(len(pos) - 1)
    for k in idxs:
        valid, contacts = p.state_valid(dict(zip(cc.ALL_JOINTS, pos[k])))
        checked += 1
        if not valid:
            invalid += 1
            bad.append((si, seg['kind'], seg['surface'], k, [(c.contact_body_1, c.contact_body_2) for c in contacts[:2]]))

res = {'collision_checked': checked, 'collision_invalid': invalid, 'invalid_examples': bad[:5], 'paint': []}
# paint passes
yaw_now = None
for seg in plan['segments']:
    if seg['kind'] != 'paint':
        continue
    name, idx = seg['stroke_ref']
    st = strokes[(name, idx)]
    Rz = cc.rot_z(seg['yaw'])
    P = (Rz @ np.array(st['points']).T).T + np.array([0, 0, cc.CAR_Z])
    N = (Rz @ np.array(st['normals']).T).T
    target = P + N * standoff                    # nozzle path the stroke asks for
    pos, times = seg['positions'], seg['times']
    tcp = np.array([tcp_of(q) for q in pos])
    # tracking error: distance of each FK point to the target polyline (dense)
    dense = []
    for k in range(len(target) - 1):
        dense.append(target[k] + (target[k + 1] - target[k]) * np.linspace(0, 1, 40)[:, None])
    dense = np.vstack(dense)
    err = np.array([np.linalg.norm(dense - t, axis=1).min() for t in tcp])
    length = float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum())
    dur = times[-1] - times[0]
    # nozzle-to-surface distance along the normal: compare with standoff using planned normals
    res['paint'].append({'surface': name, 'stroke': idx, 'tcp_len_m': length, 'dur_s': dur,
                         'speed_mps': length / dur if dur > 0 else 0, 'max_err_mm': 1000 * float(err.max()),
                         'mean_err_mm': 1000 * float(err.mean())})
sp = np.array([x['speed_mps'] for x in res['paint']])
er = np.array([x['max_err_mm'] for x in res['paint']])
res['summary'] = {'paint_passes': len(sp), 'speed_mean': float(sp.mean()), 'speed_min': float(sp.min()),
                  'speed_max': float(sp.max()), 'track_err_max_mm': float(er.max()),
                  'track_err_mean_mm': float(np.mean([x['mean_err_mm'] for x in res['paint']])),
                  'wall_s': time.time() - t0}
json.dump(res, open(out_file, 'w'))
print('collision re-check: %d states checked, %d invalid' % (checked, invalid))
if bad:
    print('examples:', bad[:3])
s = res['summary']
print('paint passes %d: nozzle speed mean %.3f (min %.3f max %.3f) m/s ; tracking error max %.2f mm, mean %.2f mm  [%.0fs]' %
      (s['paint_passes'], s['speed_mean'], s['speed_min'], s['speed_max'], s['track_err_max_mm'], s['track_err_mean_mm'], s['wall_s']))
rclpy.shutdown()
