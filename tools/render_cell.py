#!/usr/bin/env python3
"""Offscreen renderer for the painting cell (pyrender / EGL), driven by the recorded plan.

* parses the processed URDF, forward kinematics in numpy (validated against the planner's stroke targets)
* draws the robot with the low-poly collision meshes + primitives, the car, floor, booth, turntable
* paint ribbons are attached to the CAR frame, so they turn with the turntable
Usage as a module:  r = CellRenderer(...);  img = r.render(t_sim, cam)
"""
import os
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import json
import math
import struct
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import pyrender

HOME = os.path.expanduser('~')
WS = os.path.join(HOME, 'ws')
SHARE_KUKA = os.path.join(WS, 'install', 'kuka_kr210_support', 'share', 'kuka_kr210_support')
SHARE_CELL = os.path.join(WS, 'install', 'painting_cell', 'share', 'painting_cell')

SURFACE_COLORS = {
    'hood': (0.90, 0.10, 0.10), 'windshield': (0.95, 0.55, 0.10), 'roof': (0.15, 0.45, 0.90),
    'rear_window': (0.60, 0.25, 0.85), 'trunk': (0.10, 0.75, 0.40), 'side': (0.95, 0.85, 0.15),
}
LINK_COLORS = {
    'base_link': (0.55, 0.55, 0.58), 'link_1': (0.95, 0.45, 0.05), 'link_2': (0.95, 0.45, 0.05),
    'link_3': (0.95, 0.45, 0.05), 'link_4': (0.95, 0.45, 0.05), 'link_5': (0.40, 0.40, 0.43),
    'link_6': (0.30, 0.30, 0.33), 'rail_base': (0.30, 0.33, 0.38), 'carriage': (0.95, 0.78, 0.10),
    'slide': (0.40, 0.43, 0.48), 'pedestal': (0.33, 0.36, 0.41), 'gun': (0.12, 0.12, 0.14),
}
CAR_Z = 0.01


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    return R


def tf(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def axis_angle(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    c, s = math.cos(ang), math.sin(ang)
    C = 1 - c
    R = np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                  [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                  [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])
    T = np.eye(4)
    T[:3, :3] = R
    return T


def rot_z4(deg):
    return axis_angle((0, 0, 1), math.radians(deg))


def look_at(eye, target, up=(0, 0, 1)):
    eye, target, up = np.asarray(eye, float), np.asarray(target, float), np.asarray(up, float)
    f = target - eye
    f /= np.linalg.norm(f)
    s = np.cross(f, up)
    s /= np.linalg.norm(s)
    u = np.cross(s, f)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = s, u, -f, eye
    return T


def orbit_pose(yaw, pitch, dist, target):
    t = np.asarray(target, float)
    eye = t + dist * np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch)])
    return look_at(eye, t)


class UrdfModel:
    """Minimal URDF: kinematic tree + collision/visual primitives & meshes."""

    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.links = {}
        for l in root.findall('link'):
            geoms = []
            elems = l.findall('visual') or l.findall('collision')
            for c in elems:
                g = c.find('geometry')
                if g is None:
                    continue
                o = c.find('origin')
                xyz = [float(v) for v in o.get('xyz', '0 0 0').split()] if o is not None else [0, 0, 0]
                rpy = [float(v) for v in o.get('rpy', '0 0 0').split()] if o is not None else [0, 0, 0]
                geoms.append((tf(xyz, rpy), g))
            self.links[l.get('name')] = geoms
        self.joints = []
        for j in root.findall('joint'):
            o = j.find('origin')
            xyz = [float(v) for v in o.get('xyz', '0 0 0').split()] if o is not None else [0, 0, 0]
            rpy = [float(v) for v in o.get('rpy', '0 0 0').split()] if o is not None else [0, 0, 0]
            ax = j.find('axis')
            self.joints.append({'name': j.get('name'), 'type': j.get('type'), 'parent': j.find('parent').get('link'),
                                'child': j.find('child').get('link'), 'T': tf(xyz, rpy),
                                'axis': [float(v) for v in ax.get('xyz').split()] if ax is not None else [0, 0, 1]})
        self.root = 'world'

    def fk(self, q):
        """q: dict joint name -> value. Returns dict link -> 4x4 world pose."""
        poses = {self.root: np.eye(4)}
        pending = list(self.joints)
        guard = 0
        while pending and guard < 1000:
            guard += 1
            rest = []
            for j in pending:
                if j['parent'] not in poses:
                    rest.append(j)
                    continue
                T = poses[j['parent']] @ j['T']
                if j['type'] in ('revolute', 'continuous'):
                    T = T @ axis_angle(j['axis'], q.get(j['name'], 0.0))
                elif j['type'] == 'prismatic':
                    M = np.eye(4)
                    M[:3, 3] = np.asarray(j['axis'], float) * q.get(j['name'], 0.0)
                    T = T @ M
                poses[j['child']] = T
            pending = rest
        return poses


def resolve(fn):
    if fn.startswith('package://kuka_kr210_support/'):
        return os.path.join(SHARE_KUKA, fn[len('package://kuka_kr210_support/'):])
    raise ValueError(fn)


def geom_to_trimesh(g):
    b, c, m = g.find('box'), g.find('cylinder'), g.find('mesh')
    if b is not None:
        return trimesh.creation.box([float(v) for v in b.get('size').split()])
    if c is not None:
        cyl = trimesh.creation.cylinder(radius=float(c.get('radius')), height=float(c.get('length')), sections=24)
        return cyl
    if m is not None:
        fn = resolve(m.get('filename'))
        if fn.endswith('.dae'):                       # detailed visual mesh -> decimate once, cache on disk
            cache_dir = os.path.join(HOME, 'mesh_cache')
            os.makedirs(cache_dir, exist_ok=True)
            cached = os.path.join(cache_dir, os.path.basename(fn).replace('.dae', '.stl'))
            if not os.path.exists(cached):
                import re
                head = open(fn, encoding='utf-8', errors='ignore').read(4000)
                mm = re.search(r'<unit[^>]*meter="([0-9.eE+-]+)"', head)
                unit = float(mm.group(1)) if mm else 1.0      # these DAE files declare inches (0.0254 m)
                mesh = trimesh.load(fn, force='mesh')
                mesh.apply_scale(unit)
                mesh.merge_vertices()                         # DAE stores unmerged vertices; decimating those tears the mesh apart
                mesh.update_faces(mesh.nondegenerate_faces())
                mesh.remove_unreferenced_vertices()
                target = max(1500, int(len(mesh.faces) * 0.35))
                small = mesh.simplify_quadric_decimation(face_count=target)
                ext0, ext1 = mesh.bounds[1] - mesh.bounds[0], small.bounds[1] - small.bounds[0]
                if np.abs(ext1 - ext0).max() > 0.10 * np.abs(ext0).max():
                    raise RuntimeError('decimation changed the shape of %s: %s -> %s' % (fn, ext0, ext1))
                small.export(cached)
            return trimesh.load(cached, force='mesh')
        return trimesh.load(fn, force='mesh')
    return None


def make_material(rgb, alpha=1.0, metallic=0.25, rough=0.55):
    return pyrender.MetallicRoughnessMaterial(baseColorFactor=[rgb[0], rgb[1], rgb[2], alpha], metallicFactor=metallic,
                                              roughnessFactor=rough, doubleSided=True,
                                              alphaMode='BLEND' if alpha < 1 else 'OPAQUE')


class CellRenderer:
    def __init__(self, plan_file, paths_file, urdf_file, size=(1280, 720), walls=True):
        self.W, self.H = size
        self.model = UrdfModel(urdf_file)
        self.joints_names = ['rail_joint', 'cross_joint', 'joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
        plan = json.load(open(plan_file))
        self.segments = plan['segments']
        self.meta = plan['meta']
        self.paths = json.load(open(paths_file))
        self.fan = self.paths['params']['fan_width']
        self.standoff = self.paths['params']['standoff']
        self.stroke = {}
        for ph in self.paths['phases']:
            for sf in ph['surfaces']:
                for i, s in enumerate(sf['strokes']):
                    self.stroke[(sf['name'], i)] = (np.array(s['points']), np.array(s['normals']))
        self._timeline()
        self.scene = pyrender.Scene(bg_color=[0.09, 0.10, 0.13, 1.0], ambient_light=[0.30, 0.30, 0.33])
        self._build_static(walls)
        self._build_robot()
        self._build_car()
        self.renderer = pyrender.OffscreenRenderer(self.W, self.H)
        self.cam_node = None
        self.dyn_nodes = []

    # ------------------------------------------------------------------ timeline
    def _timeline(self):
        t = 0.0
        self.tl = []        # (t0, t1, segment index)
        for i, s in enumerate(self.segments):
            if s['kind'] == 'rotate':
                d = 6.0
            else:
                d = s['times'][-1] if s['times'] else 0.0
            self.tl.append((t, t + d, i))
            t += d
        self.total = t
        self.starts = np.array([x[0] for x in self.tl])

    def state_at(self, t):
        """-> (joint dict, car yaw, painted dict ref->fraction, current spray (ref,frac) or None, label)"""
        t = min(max(t, 0.0), self.total - 1e-6)
        k = int(np.searchsorted(self.starts, t, side='right')) - 1
        t0, t1, si = self.tl[k]
        seg = self.segments[si]
        # joints: from this segment, or last known from earlier ones
        q = None
        yaw = 0.0
        painted = {}
        spray = None
        label = ''
        # state carried over from earlier segments (cheap: scan back for last joint state / yaw / painted)
        for j in range(k):
            s = self.segments[self.tl[j][2]]
            if s['kind'] == 'rotate':
                yaw = s['yaw']
            elif s['times']:
                q = s['positions'][-1]
                yaw = s['yaw']
                if s['kind'] == 'paint':
                    painted[tuple(s['stroke_ref'])] = 1.0
        if seg['kind'] == 'rotate':
            u = (t - t0) / max(t1 - t0, 1e-9)
            u = 0.5 - 0.5 * math.cos(math.pi * min(1, u))
            a0, a1 = seg['angles'][0], seg['angles'][-1]
            yaw = a0 + (a1 - a0) * u
            label = 'TURNTABLE: car indexes %d -> %d deg' % (round(a0), round(a1))
        else:
            tt = t - t0
            times, pos = seg['times'], seg['positions']
            if times:
                idx = int(np.searchsorted(times, tt, side='right')) - 1
                idx = max(0, min(idx, len(times) - 2)) if len(times) > 1 else 0
                if len(times) > 1:
                    u = (tt - times[idx]) / max(times[idx + 1] - times[idx], 1e-9)
                    a, b = np.array(pos[idx]), np.array(pos[idx + 1])
                    q = list(a + (b - a) * min(max(u, 0), 1))
                else:
                    q = pos[0]
            yaw = seg['yaw']
            label = {'rail': 'rail / cross-slide move', 'move': 'collision-free joint move', 'approach': 'approach to standoff',
                     'paint': 'PAINTING', 'retreat': 'retreat'}.get(seg['kind'], seg['kind'])
            if seg['kind'] == 'paint':
                ref = tuple(seg['stroke_ref'])
                frac = min(1.0, max(0.0, tt / max(times[-1], 1e-9)))
                painted[ref] = frac
                spray = (ref, frac)
                label = 'PAINTING  %s  stroke %d' % (ref[0], ref[1] + 1)
        if q is None:
            q = [0, 0, 0, 0.35, -1.10, 0, 1.0, 0]
        return dict(zip(self.joints_names, q)), yaw, painted, spray, label

    # ------------------------------------------------------------------ scene building
    def _build_static(self, walls):
        sc = self.scene
        floor = trimesh.creation.box((11.0, 8.0, 0.10))
        floor.apply_translation([0, 0, -0.05])
        sc.add(pyrender.Mesh.from_trimesh(floor, material=make_material((0.30, 0.32, 0.36), metallic=0.0, rough=0.95)))
        # turntable ring (flush) + safety markings
        ring = trimesh.creation.annulus(r_min=2.55, r_max=2.65, height=0.012)
        ring.apply_translation([0, 0, 0.006])
        sc.add(pyrender.Mesh.from_trimesh(ring, material=make_material((0.95, 0.78, 0.10), metallic=0.0, rough=0.9)))
        disc = trimesh.creation.cylinder(radius=2.55, height=0.006, sections=64)
        disc.apply_translation([0, 0, 0.003])
        sc.add(pyrender.Mesh.from_trimesh(disc, material=make_material((0.30, 0.32, 0.36), metallic=0.1, rough=0.8)))
        if walls:
            t, h = 0.15, 3.6
            for size, xyz in (((11, t, h), (0, 3.9, h / 2)), ((t, 8, h), (-5.4, 0, h / 2))):
                w = trimesh.creation.box(size)
                w.apply_translation(xyz)
                sc.add(pyrender.Mesh.from_trimesh(w, material=make_material((0.60, 0.75, 0.90), alpha=0.18, metallic=0.0, rough=1.0)))
        for d, (az, el, inten) in enumerate(((0.9, 0.9, 2.0), (3.6, 0.6, 0.9))):
            n = pyrender.Node(light=pyrender.DirectionalLight(color=[1, 0.98, 0.95], intensity=inten),
                              matrix=orbit_pose(az, el, 5.0, (0, 0, 0)))
            sc.add_node(n)

    def _build_robot(self):
        self.robot_nodes = []          # (link name, local geom transform, node)
        self.robot_meshes = {}         # node id -> (normal mesh, red mesh)
        self.tinted = False
        for link, geoms in self.model.links.items():
            col = LINK_COLORS.get(link, (0.6, 0.6, 0.6))
            for T, g in geoms:
                m = geom_to_trimesh(g)
                if m is None:
                    continue
                mesh = pyrender.Mesh.from_trimesh(m, material=make_material(col, metallic=0.35, rough=0.5), smooth=False)
                red = pyrender.Mesh.from_trimesh(m, material=make_material((0.95, 0.12, 0.10), metallic=0.1, rough=0.6), smooth=False)
                node = self.scene.add(mesh, pose=np.eye(4))
                self.robot_nodes.append((link, T, node))
                self.robot_meshes[id(node)] = (mesh, red)
        # the nozzle is red
        self.last_q = None

    def _build_car(self):
        car = trimesh.load(os.path.join(SHARE_CELL, 'meshes', 'car_full.stl'), force='mesh')
        self.car_mesh = pyrender.Mesh.from_trimesh(car, material=make_material((0.70, 0.72, 0.76), metallic=0.4, rough=0.4), smooth=False)
        self.car_node = self.scene.add(self.car_mesh, pose=np.eye(4))
        self.ribbon_nodes = {}         # ref -> (node, nverts)

    # ------------------------------------------------------------------ dynamic bits
    def ribbon_mesh(self, ref, frac):
        P, N = self.stroke[ref]
        m = max(2, int(round(frac * (len(P) - 1))) + 1)
        P, N = P[:m], N[:m]
        d = np.gradient(P, axis=0)
        d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
        b = np.cross(N, d)
        b /= np.maximum(np.linalg.norm(b, axis=1, keepdims=True), 1e-9)
        h = self.fan / 2
        L, R = P - b * h + N * 0.004, P + b * h + N * 0.004
        V = np.empty((2 * len(P), 3))
        V[0::2], V[1::2] = L, R
        F = []
        for i in range(len(P) - 1):
            a, bb, c, dd = 2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 3
            F += [(a, bb, c), (bb, dd, c)]
        base = ref[0].rsplit('_', 1)[0]
        col = np.array(SURFACE_COLORS.get(base, (0.8, 0.8, 0.8)))
        if ref[0].endswith('_L'):
            col = np.minimum(1.0, col * 0.8 + 0.2)
        tm = trimesh.Trimesh(V, np.array(F), process=False)
        return pyrender.Mesh.from_trimesh(tm, material=make_material(col, metallic=0.3, rough=0.3), smooth=False)

    def cone_mesh(self, ref, frac):
        P, N = self.stroke[ref]
        k = min(len(P) - 1, int(round(frac * (len(P) - 1))))
        tip = P[k] + N[k] * self.standoff
        d = np.gradient(P, axis=0)[k]
        d /= max(np.linalg.norm(d), 1e-9)
        b = np.cross(N[k], d)
        b /= max(np.linalg.norm(b), 1e-9)
        h = self.fan / 2
        base = [P[k] - b * h + N[k] * 0.006, P[k] + b * h + N[k] * 0.006, P[k] + d * 0.05 + N[k] * 0.006, P[k] - d * 0.05 + N[k] * 0.006]
        V = np.array([tip] + base)
        F = [(0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1)]
        tm = trimesh.Trimesh(V, np.array(F), process=False)
        base_name = ref[0].rsplit('_', 1)[0]
        col = SURFACE_COLORS.get(base_name, (0.8, 0.8, 0.8))
        return pyrender.Mesh.from_trimesh(tm, material=make_material(col, alpha=0.35, metallic=0.0, rough=1.0), smooth=False)

    def set_robot(self, q):
        poses = self.model.fk(q)
        for link, Tg, node in self.robot_nodes:
            self.scene.set_pose(node, poses[link] @ Tg)
        return poses

    def set_tint(self, red):
        if red == self.tinted:
            return
        for link, Tg, node in self.robot_nodes:
            # only the arm and the gun turn red (the reported contact is gun <-> car); rail, slide, pedestal stay normal
            hit = red and link in ('link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6', 'gun')
            node.mesh = self.robot_meshes[id(node)][1 if hit else 0]
        self.tinted = red

    def set_visible(self, robot=True):
        """Park the robot far below the floor to hide it (keeps the scene graph unchanged)."""
        self.robot_hidden = not robot

    def path_ribbons(self, n_show):
        """Thin ribbons for the first n_show strokes (all strokes of both halves, in car frame)."""
        if not hasattr(self, 'path_nodes'):
            self.path_nodes = {}
            self.path_order = [ref for ref in self.stroke]
        for k, ref in enumerate(self.path_order):
            if k < n_show and ref not in self.path_nodes:
                P, N = self.stroke[ref]
                d = np.gradient(P, axis=0)
                d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
                b = np.cross(N, d)
                b /= np.maximum(np.linalg.norm(b, axis=1, keepdims=True), 1e-9)
                L, R = P - b * 0.008 + N * 0.012, P + b * 0.008 + N * 0.012
                V = np.empty((2 * len(P), 3)); V[0::2], V[1::2] = L, R
                F = []
                for i in range(len(P) - 1):
                    F += [(2 * i, 2 * i + 1, 2 * i + 2), (2 * i + 1, 2 * i + 3, 2 * i + 2)]
                base = ref[0].rsplit('_', 1)[0]
                col = SURFACE_COLORS.get(base, (0.8, 0.8, 0.8))
                tm = trimesh.Trimesh(V, np.array(F), process=False)
                mesh = pyrender.Mesh.from_trimesh(tm, material=make_material(col, metallic=0.0, rough=0.4), smooth=False)
                self.path_nodes[ref] = self.scene.add(mesh, pose=self.car_pose)
            elif k >= n_show and ref in self.path_nodes:
                self.scene.remove_node(self.path_nodes.pop(ref))

    def tcp_position(self, q):
        poses = self.model.fk(q)
        return poses['paint_tcp'][:3, 3]

    # ------------------------------------------------------------------ rendering
    def render(self, t, cam, caption=None, painted_override=None, hide_spray=False):
        q, yaw, painted, spray, label = self.state_at(t)
        return self.render_state(q, yaw, painted, spray, label, cam)

    def render_state(self, q, yaw, painted, spray, label, cam, tint=False, robot=True, paths=0):
        self.set_tint(tint)
        poses = self.set_robot(q)
        if not robot:                                  # hide the robot by sinking it below the floor
            for link, Tg, node in self.robot_nodes:
                P = poses[link] @ Tg
                P[2, 3] -= 50.0
                self.scene.set_pose(node, P)
        Tc = rot_z4(yaw)
        Tc[2, 3] = CAR_Z
        self.car_pose = Tc
        self.scene.set_pose(self.car_node, Tc)
        self.path_ribbons(paths)
        for ref, nd in getattr(self, 'path_nodes', {}).items():
            self.scene.set_pose(nd, Tc)
        # ribbons ride with the car: completed ones are cached, the in-progress one is rebuilt
        for ref, frac in painted.items():
            cur = self.ribbon_nodes.get(ref)
            n_pts = max(2, int(round(frac * (len(self.stroke[ref][0]) - 1))) + 1)
            if cur is None or cur[1] != n_pts:
                if cur is not None:
                    self.scene.remove_node(cur[0])
                node = self.scene.add(self.ribbon_mesh(ref, frac), pose=Tc)
                self.ribbon_nodes[ref] = (node, n_pts)
            else:
                self.scene.set_pose(cur[0], Tc)
        for ref in [r for r in self.ribbon_nodes if r not in painted]:       # time moved backwards
            self.scene.remove_node(self.ribbon_nodes.pop(ref)[0])
        for n in self.dyn_nodes:
            self.scene.remove_node(n)
        self.dyn_nodes = []
        if spray is not None:
            self.dyn_nodes.append(self.scene.add(self.cone_mesh(*spray), pose=Tc))
        if self.cam_node is None:
            self.cam_node = self.scene.add(pyrender.PerspectiveCamera(yfov=math.radians(40), aspectRatio=self.W / self.H), pose=cam)
        else:
            self.scene.set_pose(self.cam_node, cam)
        color, _ = self.renderer.render(self.scene)
        return color, label, len(painted), sum(1 for v in painted.values() if v >= 0.999)
