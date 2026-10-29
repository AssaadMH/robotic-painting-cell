#!/usr/bin/env python3
"""Render 4 views of an STL (offscreen pyrender) into one contact sheet.
usage: view_stl.py mesh.stl out.png [--yup]"""
import os, sys
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import numpy as np, trimesh, pyrender
from PIL import Image

src, out = sys.argv[1], sys.argv[2]
m = trimesh.load(src, force='mesh')
if '--yup' in sys.argv:                      # y-up -> z-up
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
m.apply_translation(-m.bounds.mean(0))
ext = m.extents
print('extents', np.round(ext, 3))
mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=(0.75, 0.76, 0.8, 1), metallicFactor=0.1, roughnessFactor=0.5)
mesh = pyrender.Mesh.from_trimesh(m, material=mat, smooth=False)
R = 1.6 * float(np.linalg.norm(ext))


def look(eye, tgt=(0, 0, 0), up=(0, 0, 1)):
    eye, tgt, up = map(lambda v: np.asarray(v, float), (eye, tgt, up))
    f = tgt - eye; f /= np.linalg.norm(f)
    s = np.cross(f, up); s /= np.linalg.norm(s)
    u = np.cross(s, f)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = s, u, -f, eye
    return T

views = [('iso', (R*0.6, -R*0.6, R*0.45)), ('side -y', (0, -R, 0.05)), ('front +x', (R, 0, 0.05)), ('top', (0.001, 0, R))]
r = pyrender.OffscreenRenderer(640, 400)
tiles = []
for name, eye in views:
    sc = pyrender.Scene(ambient_light=(0.35, 0.35, 0.35), bg_color=(0.12, 0.13, 0.16))
    sc.add(mesh)
    sc.add(pyrender.PerspectiveCamera(yfov=0.5), pose=look(eye, up=(0, 1, 0) if name == 'top' else (0, 0, 1)))
    sc.add(pyrender.DirectionalLight(intensity=3.0), pose=look((R, -R*0.5, R)))
    sc.add(pyrender.DirectionalLight(intensity=1.5), pose=look((-R, R, R*0.5)))
    img, _ = r.render(sc)
    im = Image.fromarray(img); 
    from PIL import ImageDraw; ImageDraw.Draw(im).text((8, 8), name, fill=(255, 255, 0))
    tiles.append(im)
sheet = Image.new('RGB', (1280, 800))
for i, t in enumerate(tiles):
    sheet.paste(t, ((i % 2) * 640, (i // 2) * 400))
sheet.save(out)
print('->', out)
