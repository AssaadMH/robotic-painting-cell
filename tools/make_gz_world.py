#!/usr/bin/env python3
"""Build the Gazebo Harmonic paint booth for the painting cell (WSL; numpy + trimesh).

Writes src/painting_cell/gz/:
  booth.sdf            world: booth, lights, turntable + car (static model, turned with set_pose),
                       paint strips, spray cone, cinema camera
  meshes/*.obj         car visuals (smooth normals, decimated for software rendering)
  paint/*.obj          one strip per stroke chunk, 250 mm wide (the spray fan), conformed to the body 3 mm
                       above it; drawn in primer grey and recoloured to paint when the gun passes
  paint_index.json     chunk -> (surface, stroke, fraction of the stroke where the chunk ends)
"""
import json
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', 'src', 'painting_cell')
MESHES = os.path.join(PKG, 'meshes')
GZ = os.path.join(PKG, 'gz')
PATHS = os.path.join(HERE, '..', 'results', 'paths.json')

CAR_Z = 0.01
FAN = 0.25                 # spray fan width [m]
CHUNK = 0.04               # paint strip chunk length along the stroke [m] (the gun moves ~0.1 m per frame)
LIFT = 0.003               # strip height above the body [m]
FPS = 2                    # camera rate in simulation time [Hz] (software rendering: ~3 s per frame)
STEPS = 10                 # physics steps per camera frame (0.05 s): a 0.5 s step lets the wrist dynamics blow up near a5 = 0
W, H = 1280, 720

PRIMER = (0.58, 0.59, 0.61)
PAINT = (0.62, 0.04, 0.06)


def obj(mesh, path):
    """OBJ with smooth vertex normals and no material file (the SDF sets the material)."""
    with open(path, 'w') as f:
        f.write(trimesh.exchange.obj.export_obj(mesh, include_normals=True, include_color=False,
                                                include_texture=False, write_texture=False, mtl_name=None))


def decimate(m, n):
    if len(m.faces) <= n:
        return m
    d = m.simplify_quadric_decimation(face_count=n)
    d.merge_vertices()
    return d


def strip_mesh(panels, ray, P, N, layer):
    """Ribbon over a polyline: 7 columns across the fan, each vertex projected onto the body."""
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    ss = np.linspace(0, s[-1], max(2, int(np.ceil(s[-1] / 0.02)) + 1))
    Pd = np.stack([np.interp(ss, s, P[:, k]) for k in range(3)], 1)
    Nd = np.stack([np.interp(ss, s, N[:, k]) for k in range(3)], 1)
    Nd /= np.linalg.norm(Nd, axis=1, keepdims=True)
    T = np.gradient(Pd, axis=0)
    T /= np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-9)
    B = np.cross(Nd, T)
    B /= np.maximum(np.linalg.norm(B, axis=1, keepdims=True), 1e-9)
    cols = np.linspace(-FAN / 2, FAN / 2, 7)
    Q = Pd[:, None, :] + B[:, None, :] * cols[None, :, None]               # (rows, cols, 3)
    org = (Q + Nd[:, None, :] * 0.10).reshape(-1, 3)
    d = np.repeat(-Nd, len(cols), axis=0)
    loc, ri, ti = ray.intersects_location(org, d, multiple_hits=False)
    V = np.full((len(org), 3), np.nan)
    ok = np.zeros(len(org), bool)
    if len(ri):
        good = np.linalg.norm(loc - org[ri], axis=1) < 0.25                  # not through a window opening
        ri, loc, ti = ri[good], loc[good], ti[good]
        V[ri] = loc + panels.face_normals[ti] * (LIFT + 0.00005 * layer)
        ok[ri] = True
    R, C = len(Pd), len(cols)
    F = []
    for i in range(R - 1):
        for j in range(C - 1):
            a, b, c, e = i * C + j, i * C + j + 1, (i + 1) * C + j, (i + 1) * C + j + 1
            if ok[a] and ok[b] and ok[c]:
                F.append((a, c, b))
            if ok[b] and ok[c] and ok[e]:
                F.append((b, c, e))
    if not F:
        return None, ss
    m = trimesh.Trimesh(np.nan_to_num(V), np.array(F), process=False)
    m.remove_unreferenced_vertices()
    if np.mean(m.face_normals @ Nd.mean(0)) < 0:          # face the strip outwards, like the surface
        m.invert()
    return m, ss


def chunks(P, N):
    """Split a stroke polyline into ~CHUNK long pieces; returns [(P_i, N_i, end_fraction)]."""
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    n = max(1, int(round(s[-1] / CHUNK)))
    edges = np.linspace(0, s[-1], n + 1)
    out = []
    for k in range(n):
        a, b = edges[k], edges[k + 1]
        ss = np.concatenate([[a], s[(s > a) & (s < b)], [b]])
        Pk = np.stack([np.interp(ss, s, P[:, i]) for i in range(3)], 1)
        Nk = np.stack([np.interp(ss, s, N[:, i]) for i in range(3)], 1)
        out.append((Pk, Nk, float(b / s[-1]) if s[-1] > 0 else 1.0))
    return out


def material(rgb, rough=0.5, metal=0.0, spec=0.3, emissive=None):
    r, g, b = rgb
    e = '<emissive>%.3f %.3f %.3f 1</emissive>' % emissive if emissive else ''
    return ('<material><ambient>%.3f %.3f %.3f 1</ambient><diffuse>%.3f %.3f %.3f 1</diffuse>'
            '<specular>%.2f %.2f %.2f 1</specular>%s<pbr><metal><metalness>%.2f</metalness>'
            '<roughness>%.2f</roughness></metal></pbr></material>') % (r, g, b, r, g, b, spec, spec, spec, e, metal, rough)


def box(name, size, xyz, mat, rpy=(0, 0, 0)):
    return ('<visual name="%s"><pose>%.3f %.3f %.3f %.3f %.3f %.3f</pose><geometry><box><size>%.3f %.3f %.3f</size>'
            '</box></geometry>%s</visual>') % (name, *xyz, *rpy, *size, mat)


def booth_sdf(car_visuals, strip_visuals):
    walls = []
    wall = material((0.74, 0.76, 0.78), rough=0.9)
    t, h = 0.15, 3.6
    walls.append(box('wall_back', (11.0, t, h), (0, 3.9, h / 2), wall))
    walls.append(box('wall_left', (t, 8.0, h), (-5.4, 0, h / 2), wall))
    walls.append(box('wall_right', (t, 8.0, h), (5.4, 0, h / 2), wall))
    walls.append(box('wall_front_low', (11.0, t, 1.0), (0, -3.9, 0.5), wall))
    # glazed upper front wall frame (posts only, the camera looks through it)
    for x in np.arange(-5.0, 5.01, 2.5):
        walls.append(box('post_%d' % int(x * 10 + 100), (0.12, 0.12, h - 1.0), (x, -3.9, 1.0 + (h - 1.0) / 2),
                         material((0.25, 0.27, 0.30), rough=0.6)))
    # light panels along the top of the side walls (emissive strips)
    for i, x in enumerate(np.arange(-4.5, 4.51, 1.5)):
        for y in (-3.75, 3.75):
            walls.append(box('lamp_%d_%d' % (i, int(y > 0)), (1.2, 0.08, 0.30), (x, y, 3.2),
                             material((1, 1, 1), rough=0.9, emissive=(1.0, 1.0, 0.97))))
    floor = material((0.32, 0.34, 0.37), rough=0.85)
    grate = []
    for i, x in enumerate(np.arange(-5.0, 5.01, 0.5)):
        grate.append(box('grate_%d' % i, (0.02, 7.6, 0.004), (x, 0, 0.002), material((0.22, 0.23, 0.25), rough=0.9)))
    lights = []
    for i, (x, y) in enumerate([(-2.5, -1.5), (2.5, 1.5)]):
        lights.append('<light type="point" name="lamp%d"><pose>%.1f %.1f 3.3 0 0 0</pose><diffuse>0.30 0.30 0.29 1</diffuse>'
                      '<specular>0.3 0.3 0.3 1</specular><attenuation><range>12</range><constant>0.6</constant>'
                      '<linear>0.05</linear><quadratic>0.01</quadratic></attenuation><cast_shadows>false</cast_shadows></light>' % (i, x, y))
    turntable = (box('turntable_ring', (0.01, 0.01, 0.01), (0, 0, -1), material((0, 0, 0))) +
                 '<visual name="ring"><pose>0 0 0.003 0 0 0</pose><geometry><cylinder><radius>2.75</radius><length>0.006</length>'
                 '</cylinder></geometry>%s</visual>' % material((0.95, 0.75, 0.10), rough=0.6) +
                 '<visual name="disc"><pose>0 0 0.005 0 0 0</pose><geometry><cylinder><radius>2.65</radius><length>0.008</length>'
                 '</cylinder></geometry>%s</visual>' % material((0.45, 0.47, 0.50), rough=0.7))
    cam = ('<model name="cine"><static>true</static><pose>4.5 -3.4 3.0 0 0.45 2.5</pose><link name="cine_link">'
           '<sensor name="cine_cam" type="camera"><update_rate>%d</update_rate><always_on>true</always_on><topic>cine</topic>'
           '<camera><horizontal_fov>1.05</horizontal_fov><image><width>%d</width><height>%d</height></image>'
           '<clip><near>0.05</near><far>60</far></clip><anti_aliasing>4</anti_aliasing></camera></sensor></link></model>') % (FPS, W, H)
    spray = ('<model name="spray"><static>true</static><pose>0 0 -20 0 0 0</pose><link name="spray_link">'
             '<visual name="cone"><pose>0 0 0.125 3.14159265 0 0</pose><geometry><cone><radius>0.125</radius><length>0.25</length></cone></geometry>'
             '%s</visual><visual name="mist"><pose>0 0 0.23 0 0 0</pose><geometry><cylinder><radius>0.14</radius><length>0.02</length></cylinder>'
             '</geometry>%s</visual></link></model>') % (
        material((0.95, 0.30, 0.30), rough=1.0, emissive=(0.45, 0.10, 0.10)), material((0.85, 0.25, 0.25), rough=1.0, emissive=(0.3, 0.06, 0.06)))
    return """<?xml version="1.0"?>
<sdf version="1.10">
  <world name="booth">
    <physics name="p600" type="dart"><max_step_size>%.10f</max_step_size><real_time_factor>1000</real_time_factor></physics>
    <gravity>0 0 0</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <scene><ambient>0.30 0.31 0.33 1</ambient><background>0.62 0.65 0.70 1</background><shadows>true</shadows><grid>false</grid></scene>
    <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><cast_shadows>true</cast_shadows>
      <diffuse>0.62 0.62 0.60 1</diffuse><specular>0.35 0.35 0.35 1</specular><direction>-0.35 0.45 -0.82</direction></light>
    %s
    <model name="booth"><static>true</static><link name="booth_link">
      <visual name="floor"><pose>0 0 -0.05 0 0 0</pose><geometry><box><size>11 8 0.1</size></box></geometry>%s</visual>
      %s
      %s
      %s
    </link></model>
    <model name="car"><static>true</static><pose>0 0 %.3f 0 0 0</pose><link name="car_link">
      %s
      %s
    </link></model>
    %s
    %s
  </world>
</sdf>
""" % (1.0 / (FPS * STEPS), '\n    '.join(lights), floor, '\n      '.join(walls), '\n      '.join(grate), turntable,
       CAR_Z, '\n      '.join(car_visuals), '\n      '.join(strip_visuals), spray, cam)


def export_kuka():
    """KUKA visuals for Gazebo: the Collada files are in inches with per-file axis conventions and their
    colours do not survive the ogre2 import. Each link is scaled, rotated onto its collision mesh (whose
    frame is the link frame) and split into an orange and a dark OBJ."""
    import itertools
    src = os.path.join(HERE, '..', 'src', 'kuka_kr210_support', 'meshes', 'kr210l150')
    out = os.path.join(GZ, 'kuka')
    os.makedirs(out, exist_ok=True)
    rots = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for i in range(3):
                R[i, perm[i]] = signs[i]
            if np.linalg.det(R) > 0:
                rots.append(R)
    for n in ('base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'):
        scene = trimesh.load(os.path.join(src, 'visual', n + '.dae'), force='scene')
        col = trimesh.load(os.path.join(src, 'collision', n + '.stl'), force='mesh')
        groups = {'orange': [], 'dark': []}
        for node in scene.graph.nodes_geometry:
            T, gname = scene.graph[node]
            g = scene.geometry[gname].copy()
            g.apply_transform(T)
            try:
                c = np.array(g.visual.material.main_color[:3], float)
            except Exception:
                c = np.array([30, 30, 30.0])
            groups['orange' if c[0] > 150 and c[1] < 140 else 'dark'].append(g)
        allv = trimesh.util.concatenate(groups['orange'] + groups['dark'])
        V = allv.vertices * 0.0254
        best = min(rots, key=lambda R: np.abs((V @ R.T).min(0) - col.bounds[0]).sum() + np.abs((V @ R.T).max(0) - col.bounds[1]).sum())
        err = np.abs((V @ best.T).min(0) - col.bounds[0]).max() + np.abs((V @ best.T).max(0) - col.bounds[1]).max()
        M = np.eye(4)
        M[:3, :3] = best * 0.0254
        for k, parts in groups.items():
            if not parts:
                continue
            m = trimesh.util.concatenate(parts)
            m.apply_transform(M)
            m.merge_vertices()
            obj(decimate(m, 12000), os.path.join(out, '%s_%s.obj' % (n, k)))
        print('kuka %-9s parts orange %d dark %d, bbox mismatch %.3f m' % (n, len(groups['orange']), len(groups['dark']), err))


def main():
    if '--kuka-only' in sys.argv:
        export_kuka()
        return
    export_kuka()
    os.makedirs(os.path.join(GZ, 'meshes'), exist_ok=True)
    os.makedirs(os.path.join(GZ, 'paint'), exist_ok=True)
    load = lambda n: trimesh.load(os.path.join(MESHES, n), force='mesh')
    panels = load('car_body.stl')
    parts = {'body': (decimate(panels, 110000), material(PRIMER, rough=0.75, spec=0.15)),
             'windows': (load('car_windows.stl'), material((0.05, 0.07, 0.09), rough=0.08, spec=0.9)),
             'liner': (decimate(load('car_liner.stl'), 8000), material((0.12, 0.12, 0.13), rough=0.9)),
             'wheels': (decimate(load('car_wheels.stl'), 30000), material((0.09, 0.09, 0.10), rough=0.6, spec=0.3)),
             'trim': (load('car_trim.stl'), material((0.06, 0.06, 0.07), rough=0.3, spec=0.6))}        # thin strips: no decimation
    car_visuals = []
    for name, (m, mat) in parts.items():
        obj(m, os.path.join(GZ, 'meshes', 'car_%s.obj' % name))
        car_visuals.append('<visual name="car_%s"><geometry><mesh><uri>model://painting_cell/gz/meshes/car_%s.obj</uri></mesh>'
                           '</geometry>%s</visual>' % (name, name, mat))
        print('car_%s: %d faces' % (name, len(m.faces)))
    ray = trimesh.ray.ray_triangle.RayMeshIntersector(panels)
    paths = json.load(open(PATHS))
    index, strips, layer = [], [], 0
    for ph in paths['phases']:
        for sf in ph['surfaces']:
            for si, st in enumerate(sf['strokes']):
                P, N = np.array(st['points']), np.array(st['normals'])
                for ci, (Pk, Nk, frac) in enumerate(chunks(P, N)):
                    m, _ = strip_mesh(panels, ray, Pk, Nk, layer)
                    if m is None:
                        continue
                    vis = 'paint_%s_%02d_%02d' % (sf['name'], si, ci)
                    obj(m, os.path.join(GZ, 'paint', vis + '.obj'))
                    strips.append('<visual name="%s"><geometry><mesh><uri>model://painting_cell/gz/paint/%s.obj</uri></mesh>'
                                  '</geometry>%s</visual>' % (vis, vis, material(PRIMER, rough=0.75, spec=0.15)))
                    index.append({'visual': vis, 'surface': sf['name'], 'stroke': si, 'chunk': ci, 'end_frac': frac})
                layer += 1
    print('paint strips: %d chunks over %d strokes' % (len(index), layer))
    json.dump({'paint_rgb': PAINT, 'primer_rgb': PRIMER, 'link': 'car_link', 'fps': FPS, 'chunks': index},
              open(os.path.join(GZ, 'paint_index.json'), 'w'))
    with open(os.path.join(GZ, 'booth.sdf'), 'w') as f:
        f.write(booth_sdf(car_visuals, strips))
    print('-> %s' % os.path.join(GZ, 'booth.sdf'))


if __name__ == '__main__':
    main()
