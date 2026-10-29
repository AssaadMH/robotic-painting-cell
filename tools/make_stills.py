#!/usr/bin/env python3
"""Render the report figures (clean stills, no overlays) into docs/screenshots/."""
import os, sys, math, json
import numpy as np, cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_video as mv

out = mv.ROOT + '/docs/screenshots'
os.makedirs(out, exist_ok=True)
d = mv.Director(12)
r = d.r


def save(name, img):
    cv2.imwrite('%s/%s.png' % (out, name), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    print('wrote', name, flush=True)


ready, *_ = r.state_at(0.0)
cam = d.cam
save('fig01_cell_overview', r.render_state(ready, 0.0, {}, None, '', cam(-55, 27, 13.0, (0.0, -0.6, 0.9)))[0])
save('fig02_cell_topview', r.render_state(ready, 0.0, {}, None, '', cam(-90, 87, 14.5, (0.0, -0.4, 0.0)))[0])
t_roof = d.pose_at_stroke_mid(('roof_R', 3))
q, yaw, pn, sp, lb = r.state_at(t_roof)
save('fig03_roof_pose_cross_in', r.render_state(q, 0.0, {}, None, '', cam(-98, 20, 8.0, (q['rail_joint'], -1.2, 1.0)))[0])
t_side = d.pose_at_stroke_mid(('side_R', 12))
q, yaw, pn, sp, lb = r.state_at(t_side)
save('fig04_side_pose_cross_out', r.render_state(q, 0.0, {}, None, '', cam(-90, 18, 8.0, (q['rail_joint'] * 0.8, -1.2, 1.0)))[0])
save('fig05_paths', r.render_state(ready, 0.0, {}, None, '', cam(-70, 36, 7.6, (0.0, 0.0, 0.7)), robot=False, paths=106)[0])
t = d.t_side_end - 0.45 * (d.t_side_end - d.t_top_end) * 0.0 - 0.0
t_paint = d.pose_at_stroke_mid(('side_R', 18))
q, yaw, pn, sp, lb = r.state_at(t_paint)
save('fig06_painting_side', r.render_state(q, yaw, pn, sp, lb, cam(-62, 17, 6.8, (q['rail_joint'] * 0.8, -0.9, 0.95)))[0])
t_hood = d.pose_at_stroke_mid(('trunk_R', 2))
q, yaw, pn, sp, lb = r.state_at(t_hood)
save('fig07_painting_top', r.render_state(q, yaw, pn, sp, lb, cam(-45, 28, 8.6, (0.3, -0.6, 0.9)))[0])
dm = d.demo
fi = dm['bad']['first_invalid']
q = dict(zip(r.joints_names, dm['bad']['states'][fi]))
save('fig08_collision_rejected', r.render_state(q, 0.0, {}, None, '', cam(28, 20, 5.2, (1.3, -0.6, 0.95)), tint=True)[0])
q = dict(zip(r.joints_names, dm['good']['states'][len(dm['good']['states']) // 2]))
save('fig09_collision_free', r.render_state(q, 0.0, {}, None, '', cam(28, 20, 5.2, (1.3, -0.6, 0.95)))[0])
t_mid_rot = 0.5 * (d.t_rot[0] + d.t_rot[1])
q, yaw, pn, sp, lb = r.state_at(t_mid_rot)
save('fig10_turntable', r.render_state(q, yaw, pn, sp, lb, cam(-48, 33, 11.8, (0.0, -0.4, 0.8)))[0])
q, yaw, pn, sp, lb = r.state_at(d.t_all - 0.01)
save('fig11_final', r.render_state(q, yaw, pn, None, '', cam(-50, 26, 9.5, (0.0, -0.6, 0.9)))[0])
save('fig12_final_other_side', r.render_state(q, yaw, pn, None, '', cam(55, 24, 9.5, (0.0, 0.0, 0.9)))[0])
print('done')
