#!/usr/bin/env python3
"""Prepare the Togg T10X model as the car of the painting cell.

Input : binary STL of the downloaded model (y up, length along z, metres, airport "follow me" light bar on the roof)
Output: src/painting_cell/meshes/
          car_body.stl      painted body panels, windows cut out (also the ray-casting target for the spray paths)
          car_full.stl      decimated body + windows + wheels + trim, the MoveIt collision object
          car_windows.stl, car_liner.stl, car_wheels.stl, car_trim.stl   visual parts for Gazebo
        results/car_parts.png  colour-coded contact sheet

Steps: y-up -> z-up, front -> +x, wheels on z = 0, light bar removed (a body in white has no roof equipment),
scaled to the 4800 x 1900 x 1600 mm envelope of the brief (body width, mirrors excluded).
usage: prep_car.py in.stl
"""
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
MESHES = os.path.join(HERE, '..', 'src', 'painting_cell', 'meshes')
RESULTS = os.path.join(HERE, '..', 'results')
TARGET = (4.8, 1.9, 1.6)                     # L x W x H of the brief [m]


def load(path):
    m = trimesh.load(path, force='mesh', process=True)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))   # y-up -> z-up
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1]))   # grille (-y) -> +x
    c = m.bounds.mean(0)
    m.apply_translation([-c[0], -c[1], -m.bounds[0][2]])
    return m


def flatness(p):
    """(thickness along the flattest principal direction, plate area of the two larger directions)."""
    v = p.vertices - p.vertices.mean(0)
    if len(v) < 4:
        return 1.0, 0.0
    s = np.linalg.svd(v[:: max(1, len(v) // 4000)], full_matrices=False)[2]
    if len(s) < 3:
        return 1.0, 0.0
    proj = v @ s.T
    ext = proj.max(0) - proj.min(0)
    return ext[2], ext[0] * ext[1]


def classify(parts):
    """body = largest shell (windows included), liner = second largest (roof lining behind the greenhouse), wheels = inside the 4 wheel boxes,
    bar = the follow-me light bar and sign on the roof, decal = livery and number plates (thin plates lying
    on the body), trim = the rest (grille, lamps, mirrors, rails, handles)."""
    parts = sorted(parts, key=lambda p: -len(p.faces))
    body = parts[0]
    pq = trimesh.proximity.ProximityQuery(body)
    rng = np.random.default_rng(0)
    groups = {'body': [body], 'liner': [parts[1]], 'wheels': [], 'bar': [], 'decal': [], 'trim': []}
    for p in parts[2:]:
        lo, hi = p.bounds
        cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
        if hi[2] < 0.76 and abs(abs(cx) - 1.45) < 0.45 and abs(cy) > 0.6:
            groups['wheels'].append(p)
        elif lo[2] > 1.55 and -0.5 < cx < 0.15 and abs(cy) < 0.62 and not (hi[0] - lo[0] > 1.0):
            groups['bar'].append(p)
        elif lo[2] > 1.66:
            groups['bar'].append(p)
        elif abs(cx) > 2.0 and abs(cy) < 0.15 and 0.35 < hi[1] - lo[1] < 0.65 and hi[2] - lo[2] < 0.2:
            groups['decal'].append(p)                       # number plates
        else:
            thick, area = flatness(p)
            if thick < 0.02 and area > 0.04 and len(p.faces) > 40:
                s = p.vertices[rng.choice(len(p.vertices), min(60, len(p.vertices)), replace=False)]
                if np.median(np.abs(pq.signed_distance(s))) < 0.03:
                    groups['decal'].append(p)
                    continue
            groups['trim'].append(p)
    return {k: (trimesh.util.concatenate(v) if v else None) for k, v in groups.items()}


def render_sheet(groups, out):
    os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
    import pyrender
    from PIL import Image, ImageDraw
    cols = {'body': (0.80, 0.80, 0.83, 1), 'windows': (0.15, 0.35, 0.65, 1), 'liner': (0.3, 0.3, 0.3, 1), 'wheels': (0.12, 0.12, 0.12, 1),
            'trim': (0.9, 0.55, 0.1, 1), 'bar': (0.9, 0.1, 0.1, 1), 'decal': (0.9, 0.1, 0.1, 1)}
    meshes = [pyrender.Mesh.from_trimesh(g, material=pyrender.MetallicRoughnessMaterial(baseColorFactor=cols[k], roughnessFactor=0.5),
                                         smooth=False) for k, g in groups.items() if g is not None]
    def look(eye, tgt=(0, 0, 0.8), up=(0, 0, 1)):
        eye, tgt, up = (np.asarray(v, float) for v in (eye, tgt, up))
        f = tgt - eye; f /= np.linalg.norm(f)
        s = np.cross(f, up); s /= np.linalg.norm(s)
        u = np.cross(s, f)
        T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = s, u, -f, eye
        return T
    r = pyrender.OffscreenRenderer(640, 400)
    sheet = Image.new('RGB', (1280, 800))
    for i, (name, eye, up) in enumerate((('front-left', (6, 5, 3.5), (0, 0, 1)), ('right side (-y)', (0, -9, 0.9), (0, 0, 1)),
                                         ('rear-right', (-6, -5, 3.5), (0, 0, 1)), ('top', (0, 0, 10), (1, 0, 0)))):
        sc = pyrender.Scene(ambient_light=(0.35, 0.35, 0.35), bg_color=(0.12, 0.13, 0.16))
        for m in meshes:
            sc.add(m)
        sc.add(pyrender.PerspectiveCamera(yfov=0.55), pose=look(eye, up=up))
        sc.add(pyrender.DirectionalLight(intensity=3.0), pose=look((5, -4, 8)))
        sc.add(pyrender.DirectionalLight(intensity=1.5), pose=look((-5, 4, 5)))
        img, _ = r.render(sc)
        im = Image.fromarray(img)
        ImageDraw.Draw(im).text((8, 8), name + '   grey painted body | blue windows | black wheels | orange trim',
                                fill=(255, 255, 0))
        sheet.paste(im, ((i % 2) * 640, (i // 2) * 400))
    sheet.save(out)


def split_windows(body, liner):
    """The shell is one surface, windows included. A shell face is a window if a ray cast inwards from it
    passes through an opening of the roof lining; thin strips (< 20 cm tall) found that way are fillets
    at the sill, cowl and tailgate, not glass."""
    c, n = body.triangles_center, body.face_normals
    lo, hi = liner.bounds
    cand = np.where((c[:, 2] > lo[2] - 0.02) & (c[:, 0] > lo[0] - 0.05) & (c[:, 0] < hi[0] - 0.02))[0]
    hit = liner.ray.intersects_any(c[cand] - n[cand] * 0.003, -n[cand])
    idx = cand[~hit]
    sub = body.submesh([idx], append=True, repair=False)
    win = np.zeros(len(body.faces), bool)
    for cc in trimesh.graph.connected_components(sub.face_adjacency, nodes=np.arange(len(sub.faces)), min_len=1):
        z = sub.triangles_center[cc][:, 2]
        if z.max() - z.min() > 0.20:
            win[idx[cc]] = True
    return body.submesh([np.where(~win)[0]], append=True), body.submesh([np.where(win)[0]], append=True)


def main(src):
    m = load(src)
    print('loaded', len(m.faces), 'faces, bounds', np.round(m.bounds, 3).tolist())
    groups = classify(m.split(only_watertight=False))
    for k, g in groups.items():
        print('  %-6s faces=%7d' % (k, 0 if g is None else len(g.faces)),
              '' if g is None else np.round(g.bounds, 3).tolist())
    # scale: length and height from everything that stays, width from the body shell (mirrors stick out)
    keep = trimesh.util.concatenate([g for k, g in groups.items() if g is not None and k not in ('bar', 'decal')])
    ext = keep.extents
    sx, sz = TARGET[0] / ext[0], TARGET[2] / ext[2]
    sy = TARGET[1] / groups['body'].extents[1]
    S = np.diag([sx, sy, sz, 1.0])
    print('scale x %.4f  y %.4f  z %.4f' % (sx, sy, sz))
    for g in groups.values():
        if g is not None:
            g.apply_transform(S)
    keep.apply_transform(S)
    lo = keep.bounds[0]
    for g in groups.values():
        if g is not None:
            g.apply_translation([0, 0, -lo[2]])
    keep.apply_translation([0, 0, -lo[2]])
    print('car after scaling', np.round(keep.extents, 3), 'body', np.round(groups['body'].extents, 3))
    os.makedirs(MESHES, exist_ok=True)
    panels, windows = split_windows(groups['body'], groups['liner'])
    print('painted panels %d faces, windows %d faces' % (len(panels.faces), len(windows.faces)))
    panels.export(os.path.join(MESHES, 'car_body.stl'))
    windows.export(os.path.join(MESHES, 'car_windows.stl'))
    for k in ('liner', 'wheels', 'trim'):
        groups[k].export(os.path.join(MESHES, 'car_%s.stl' % k))
    # collision object: whole car (no bar), decimated so FCL stays fast
    col = keep
    try:
        col = keep.simplify_quadric_decimation(face_count=40000)
    except Exception as e:                  # fast_simplification missing -> keep full detail
        print('decimation unavailable (%s), using full mesh' % e)
    col.export(os.path.join(MESHES, 'car_full.stl'))
    print('car_full.stl faces', len(col.faces))
    os.makedirs(RESULTS, exist_ok=True)
    render_sheet({'body': panels, 'windows': windows, 'liner': groups['liner'], 'wheels': groups['wheels'], 'trim': groups['trim']},
                 os.path.join(RESULTS, 'car_parts.png'))
    print('sheet -> results/car_parts.png')


if __name__ == '__main__':
    main(sys.argv[1])
