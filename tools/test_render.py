#!/usr/bin/env python3
import os, sys, subprocess, time, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render_cell as rc

HOME = os.path.expanduser('~')
urdf = '/tmp/cell_render.urdf'
xacro = os.path.join(rc.SHARE_CELL, 'urdf', 'painting_cell.urdf.xacro')
with open(urdf, 'w') as f:
    subprocess.run(['xacro', xacro], stdout=f, check=True)
plan = sys.argv[1] if len(sys.argv) > 1 else HOME + '/plan.json'
paths = os.path.join(rc.SHARE_CELL, 'paths', 'paths.json')
r = rc.CellRenderer(plan, paths, urdf, size=(1280, 720))
print('timeline total %.1f s, %d segments' % (r.total, len(r.segments)))

# --- FK validation: nozzle at the start/end of every paint pass vs the stroke target
errs = []
for seg in r.segments:
    if seg['kind'] != 'paint':
        continue
    ref = tuple(seg['stroke_ref'])
    P, N = r.stroke[ref]
    Rz = rc.rot_z4(seg['yaw'])[:3, :3]
    Pw = (Rz @ P.T).T + np.array([0, 0, rc.CAR_Z])
    Nw = (Rz @ N.T).T
    tgt = Pw + Nw * r.standoff
    names = r.joints_names
    for k, idx in ((0, 0), (-1, -1)):
        q = dict(zip(names, seg['positions'][idx]))
        pos = r.tcp_position(q)
        # distance to the nearest target sample (path is sampled every 4 cm, so compare to the polyline)
        d = min(np.linalg.norm(tgt[i] - pos) for i in range(len(tgt)))
        errs.append(d)
errs = np.array(errs) * 1000
print('FK check on %d paint-pass end points: nozzle to nearest target sample  mean %.1f mm  max %.1f mm' % (len(errs), errs.mean(), errs.max()))

# --- one frame
sidepaint = [r.tl[i][0] + 0.55 * (r.tl[i][1] - r.tl[i][0]) for i, sg in enumerate(r.segments)
             if sg['kind'] == 'paint' and sg['stroke_ref'][0] == 'side_R' and sg['stroke_ref'][1] == 12]
t = float(sys.argv[2]) if len(sys.argv) > 2 else sidepaint[0]
cam = rc.orbit_pose(math.radians(-60), math.radians(24), 11.5, (0.3, -0.4, 0.9))
t0 = time.time()
img, label, n_started, n_done = r.render(t, cam)
print('frame t=%.1f s : "%s"  strokes started %d done %d   (%.2f s to render)' % (t, label, n_started, n_done, time.time() - t0))
import cv2
out = os.path.join(HOME, 'test_frame.png')
cv2.imwrite(out, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
print('wrote', out)
