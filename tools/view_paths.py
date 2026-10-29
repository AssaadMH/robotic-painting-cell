#!/usr/bin/env python3
"""Contact sheet: car body + spray strokes (coloured per surface) + uncovered sample points (red).
usage: view_paths.py out.png"""
import os, sys, json
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import numpy as np, trimesh, pyrender
from PIL import Image, ImageDraw
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_paths as gp

car = gp.Car()
paths = json.load(open(gp.OUT))
lim = gp.panel_limits(car)
cols = {'hood': (0.9, 0.2, 0.1), 'roof': (0.1, 0.5, 0.95), 'side': (0.1, 0.8, 0.3)}
pts, pc, strokes = [], [], {}
for ph in paths['phases']:
    for sf in ph['surfaces']:
        base = sf['name'].rsplit('_', 1)[0]
        for s in sf['strokes']:
            P = np.array(s['points']); N = np.array(s['normals'])
            strokes.setdefault(base, []).append((P, N))
            for k in range(len(P) - 1):
                for t in np.linspace(0, 1, 6):
                    pts.append(P[k] + (P[k + 1] - P[k]) * t + N[k] * 0.01); pc.append(cols[base])
miss = []
for name in ('hood', 'roof'):
    S = gp.surface_samples(car, 'top', (*lim[name], -1.0, 1.0))
    dense = np.vstack([P for P, _ in strokes[name]])
    from scipy.spatial import cKDTree
    d, _ = cKDTree(np.vstack([np.vstack([P[k] + (P[k+1]-P[k]) * t for t in np.linspace(0, 1, 5)]) for P, _ in strokes[name] for k in range(len(P)-1)])).query(S)
    miss.append(S[d > gp.FAN_WIDTH / 2])
for sign in (-1, 1):
    S = gp.surface_samples(car, 'side', sign)
    from scipy.spatial import cKDTree
    d, _ = cKDTree(np.vstack([np.vstack([P[k] + (P[k+1]-P[k]) * t for t in np.linspace(0, 1, 5)]) for P, _ in strokes['side'] for k in range(len(P)-1)])).query(S)
    miss.append(S[d > gp.FAN_WIDTH / 2])
miss = np.vstack(miss)
print('uncovered samples', len(miss))
body = pyrender.Mesh.from_trimesh(car.panels, material=pyrender.MetallicRoughnessMaterial(baseColorFactor=(0.7, 0.7, 0.72, 1), roughnessFactor=0.6))
win = pyrender.Mesh.from_trimesh(car.windows, material=pyrender.MetallicRoughnessMaterial(baseColorFactor=(0.1, 0.15, 0.25, 1)))
sp = pyrender.Mesh.from_points(np.array(pts), colors=np.array(pc))
mp = pyrender.Mesh.from_points(miss + 0.01 * np.sign(miss) * [0, 1, 0], colors=np.tile([1.0, 0, 0], (len(miss), 1)))
def look(eye, tgt=(0, 0, 0.8), up=(0, 0, 1)):
    eye, tgt, up = (np.asarray(v, float) for v in (eye, tgt, up))
    f = tgt - eye; f /= np.linalg.norm(f); s = np.cross(f, up); s /= np.linalg.norm(s); u = np.cross(s, f)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = s, u, -f, eye; return T
r = pyrender.OffscreenRenderer(960, 540, point_size=3)
sheet = Image.new('RGB', (1920, 1080))
for i, (name, eye, up) in enumerate((('top', (0, 0, 9), (1, 0, 0)), ('right side', (0, -8, 0.9), (0, 0, 1)),
                                     ('front-right', (6, -5, 4), (0, 0, 1)), ('rear-left', (-6, 5, 4), (0, 0, 1)))):
    sc = pyrender.Scene(ambient_light=(0.5, 0.5, 0.5), bg_color=(0.1, 0.1, 0.12))
    for m in (body, win, sp, mp):
        sc.add(m)
    sc.add(pyrender.PerspectiveCamera(yfov=0.6), pose=look(eye, up=up))
    sc.add(pyrender.DirectionalLight(intensity=2.5), pose=look((4, -3, 8)))
    img, _ = r.render(sc)
    im = Image.fromarray(img); ImageDraw.Draw(im).text((10, 10), name + ': strokes hood red / roof blue / side green, uncovered = red dots', fill=(255, 255, 0))
    sheet.paste(im, ((i % 2) * 960, (i // 2) * 540))
sheet.save(sys.argv[1]); print('->', sys.argv[1])
