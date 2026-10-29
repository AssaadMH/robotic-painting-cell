#!/usr/bin/env python3
"""Generate spray-gun paths on the car body by ray casting the real mesh (Togg T10X body panels).

For every paintable surface a boustrophedon (zig-zag) raster is built:
  * stroke pitch  = spray fan width * (1 - overlap)
  * every path point is a ray hit on the body panels, so the path follows the surface exactly
  * the gun axis is the inverse (interpolated) surface normal, at a fixed standoff
Surfaces: hood and roof (strokes across the car, cast from above) and the two side panels (vertical strokes,
cast from the side). The body is painted before glass and trim are fitted, so only the panels and the wheels
(which the car stands on) are hit by the rays; window openings let the rays through. The panel limits come
from the mesh itself: the hood ends at the windshield, the roof between the windshield and the rear window,
the sides run from the sill to the window line (the pillars between the windows need a separate short program).
The car turns on a turntable, so each surface is split into the part painted at yaw 0 deg (near side faces the
robot rail) and the part painted after a 180 deg turn.

Output: results/paths.json   (car frame, metres)  +  coverage statistics.  Needs numpy + trimesh (+ rtree).
"""
import json
import os

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
MESHES = os.path.join(HERE, '..', 'src', 'painting_cell', 'meshes')
OUT = os.path.join(HERE, '..', 'results', 'paths.json')

FAN_WIDTH = 0.25          # spray fan width on the surface at the standoff [m]
OVERLAP = 0.30            # stroke overlap
PITCH = FAN_WIDTH * (1 - OVERLAP)
STANDOFF = 0.25           # nozzle to surface [m]
STEP = 0.04               # path point spacing along a stroke [m]
EDGE = 0.04               # keep-out from panel edges [m]
RAIL_PITCH = 0.7          # rail stations are multiples of this [m]
TOP_NZ = 0.55             # a top-surface hit counts if its normal points this much up
SIDE_NY = 0.55            # a side hit counts if its normal points this much sideways
SIDE_NZ_MIN = 0.0         # on the inward-sloping lower doors the gun stays level (sprays up to ~25 deg off normal)


class Car:
    """Body panels (painted) + wheels (occluders) with one ray caster; windows kept for the panel limits."""

    def __init__(self):
        load = lambda n: trimesh.load(os.path.join(MESHES, n), force='mesh')
        self.panels = load('car_body.stl')
        self.windows = load('car_windows.stl')
        parts = [self.panels, load('car_wheels.stl')]
        self.n_panel = len(self.panels.faces)
        self.all = trimesh.util.concatenate(parts)
        self.vn = self.all.vertex_normals                       # smooth normals for the gun axis
        self.ray = trimesh.ray.ray_triangle.RayMeshIntersector(self.all)
        self.glass_ray = trimesh.ray.ray_triangle.RayMeshIntersector(trimesh.util.concatenate([self.panels, self.windows]))

    def cast(self, origins, d):
        """First hit of each ray. Returns (hit_on_panel, point, smooth normal facing the ray origin)."""
        origins = np.asarray(origins, float)
        d = np.broadcast_to(np.asarray(d, float), origins.shape)
        loc, ri, ti = self.ray.intersects_location(origins, d, multiple_hits=False)
        hit = np.zeros(len(origins), bool)
        P = np.zeros((len(origins), 3))
        N = np.zeros((len(origins), 3))
        if len(ri):
            tri = self.all.faces[ti]
            bary = trimesh.triangles.points_to_barycentric(self.all.vertices[tri], loc)
            n = np.einsum('ij,ijk->ik', bary, self.vn[tri])
            n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
            flip = np.einsum('ij,ij->i', n, d[ri]) > 0
            n[flip] *= -1
            hit[ri] = ti < self.n_panel
            P[ri], N[ri] = loc, n
        return hit, P, N


def runs(mask):
    """(start, stop) index pairs of the contiguous True runs of a boolean array."""
    m = np.concatenate([[False], mask, [False]]).astype(int)
    e = np.flatnonzero(np.diff(m))
    return list(zip(e[::2], e[1::2]))


def longest_run(mask, p=None):
    """Longest run of True; with points p, a run is also cut where neighbours are more than 2.5 steps apart
    (a ray that slips through a window opening lands far away on the inside of the body)."""
    if p is not None:
        jump = np.linalg.norm(np.diff(p, axis=0), axis=1) > 2.5 * STEP
        mask = mask.copy()
        r = []
        for a, b in runs(mask):
            cuts = [a] + [k + 1 for k in range(a, b - 1) if jump[k]] + [b]
            r += [(cuts[i], cuts[i + 1]) for i in range(len(cuts) - 1)]
        return max(r, key=lambda ab: ab[1] - ab[0]) if r else None
    r = runs(mask)
    return max(r, key=lambda ab: ab[1] - ab[0]) if r else None


def panel_limits(car):
    """x limits of the hood and the roof from the windshield and rear-window glass."""
    w = car.windows
    c, n = w.triangles_center, w.face_normals
    ws = c[(n[:, 2] > 0.4) & (c[:, 0] > 0) & (np.abs(c[:, 1]) < 0.4)]
    rw = c[(n[:, 2] > 0.2) & (n[:, 0] < 0) & (c[:, 0] < -1.0) & (np.abs(c[:, 1]) < 0.4)]
    lim = {'hood': (ws[:, 0].max() + 0.03, car.panels.bounds[1][0]),
           'roof': (rw[:, 0].max() + 0.03, ws[:, 0].min() - 0.03)}
    return lim


def ztop_profile(car, xs):
    """Height of the car top along the centre line (glass included, so the profile has no holes)."""
    loc, ri, _ = car.glass_ray.intersects_location(np.stack([xs, np.zeros_like(xs), np.full_like(xs, 4.0)], 1),
                                             np.tile([0, 0, -1.0], (len(xs), 1)), multiple_hits=False)
    z = np.full(len(xs), np.nan)
    z[ri] = loc[:, 2]
    return z


def rail_station(xw):
    return float(np.clip(np.round(xw / RAIL_PITCH) * RAIL_PITCH, -3.5, 3.5))


def top_strokes(car, x0, x1, y_lo, y_hi):
    """Strokes along y over a top surface (cast straight down) between y_lo..y_hi (car frame)."""
    # space the strokes by arc length along the centre-line profile (slopes would otherwise open gaps)
    xf = np.linspace(x0 + EDGE, x1 - EDGE, 300)
    zf = ztop_profile(car, xf)
    ok = np.isfinite(zf)
    xf, zf = xf[ok], zf[ok]
    arc = np.concatenate([[0], np.cumsum(np.hypot(np.diff(xf), np.diff(zf)))])
    n_str = max(1, int(np.ceil(arc[-1] / PITCH - 1e-9)) + 1)
    xs = np.interp(np.linspace(0, arc[-1], n_str), arc, xf)
    strokes = []
    for x in xs:
        ys = np.arange(y_lo, y_hi + 1e-9, STEP)
        hit, p, n = car.cast(np.stack([np.full_like(ys, x), ys, np.full_like(ys, 4.0)], 1), (0, 0, -1))
        keep = hit & (n[:, 2] > TOP_NZ)
        r = longest_run(keep, p)
        if r is None:
            continue
        a, b = r
        if abs(ys[a] - ys[0]) > 1e-6:          # trim the outer panel edge (the centre line is not an edge)
            a += 1
        if abs(ys[b - 1] - ys[-1]) > 1e-6:
            b -= 1
        if b - a < 3:
            continue
        p, n = p[a:b], n[a:b]
        if len(strokes) % 2 == 1:              # boustrophedon
            p, n = p[::-1], n[::-1]
        strokes.append((p, n))
    return strokes


def window_line(car, sign):
    """z of the window sill along x on one side (3 cm below the lowest glass); 1.36 m where there is no glass."""
    c = car.windows.triangles_center
    c = c[(np.sign(c[:, 1]) == sign) & (np.abs(car.windows.face_normals[:, 1]) > 0.5)]
    xs = np.arange(-2.5, 2.501, 0.05)
    z = np.array([c[np.abs(c[:, 0] - x) < 0.12, 2].min() - 0.03 if np.any(np.abs(c[:, 0] - x) < 0.12) else 1.36 for x in xs])
    return lambda x: float(np.interp(x, xs, z))


def side_strokes(car, sign):
    """Vertical strokes on the side at y = sign*|y| (cast along -sign*y), sill to window line."""
    zcap = window_line(car, sign)
    xs = np.arange(-2.25, 2.25 + 1e-9, PITCH)
    strokes = []
    for x in xs:
        zs = np.arange(0.28, zcap(x) + 1e-9, STEP)
        hit, p, n = car.cast(np.stack([np.full_like(zs, x), np.full_like(zs, sign * 3.0), zs], 1), (0, -sign, 0))
        keep = hit & (np.abs(n[:, 1]) > SIDE_NY) & (p[:, 1] * sign > 0.6)
        r = longest_run(keep, p)
        if r is None or r[1] - r[0] < 4:
            continue
        a, b = r[0] + 1, r[1] - 1                   # keep off the run ends (sill and window edge)
        p, n = p[a:b], n[a:b].copy()
        n[:, 2] = np.maximum(n[:, 2], SIDE_NZ_MIN)
        n /= np.linalg.norm(n, axis=1, keepdims=True)
        if len(strokes) % 2 == 1:
            p, n = p[::-1], n[::-1]
        strokes.append((p, n))
    return strokes


def surface_samples(car, kind, region):
    """2 cm grid of points on a surface, selected with the same rules as the strokes."""
    pts = []
    if kind == 'top':
        x0, x1, y0, y1 = region
        for x in np.arange(x0, x1, 0.02):
            ys = np.arange(y0, y1, 0.02)
            hit, p, n = car.cast(np.stack([np.full_like(ys, x), ys, np.full_like(ys, 4.0)], 1), (0, 0, -1))
            r = longest_run(hit & (n[:, 2] > TOP_NZ), p)
            if r is not None:
                pts.append(p[r[0]:r[1]])
    else:
        sign = region
        zcap = window_line(car, sign)
        for x in np.arange(-2.40, 2.40, 0.02):
            zs = np.arange(0.28, zcap(x), 0.02)
            hit, p, n = car.cast(np.stack([np.full_like(zs, x), np.full_like(zs, sign * 3.0), zs], 1), (0, -sign, 0))
            r = longest_run(hit & (np.abs(n[:, 1]) > SIDE_NY) & (p[:, 1] * sign > 0.6), p)   # sill .. window line
            if r is not None:
                pts.append(p[r[0]:r[1]])
    return np.vstack(pts)


def coverage(pts, strokes):
    """Fraction of surface points within half a fan width of a stroke polyline (densified to 1 cm)."""
    dense = []
    for p, _ in strokes:
        for k in range(len(p) - 1):
            m = max(2, int(np.linalg.norm(p[k + 1] - p[k]) / 0.01))
            dense.append(p[k] + (p[k + 1] - p[k]) * np.linspace(0, 1, m)[:, None])
    dense = np.vstack(dense)
    try:
        from scipy.spatial import cKDTree
        d, _ = cKDTree(dense).query(pts)
    except ImportError:
        d = np.concatenate([np.linalg.norm(pts[s:s + 300, None] - dense[None], axis=2).min(1)
                            for s in range(0, len(pts), 300)])
    return float((d <= FAN_WIDTH / 2).mean())


def stroke_record(p, n, yaw):
    """Serialise one stroke; rail station from the stroke centre in world x."""
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    cx_world = float(c * p[:, 0].mean() - s * p[:, 1].mean())
    return {
        'points': np.round(p, 5).tolist(),
        'normals': np.round(n, 5).tolist(),
        'length': float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()),
        'rail_station': rail_station(cx_world),
    }


def main():
    car = Car()
    lim = panel_limits(car)
    print('panel limits (x): ' + ', '.join('%s %.2f..%.2f' % (k, *v) for k, v in lim.items()))
    plan = {'params': {'fan_width': FAN_WIDTH, 'overlap': OVERLAP, 'pitch': PITCH, 'standoff': STANDOFF,
                       'step': STEP, 'rail_pitch': RAIL_PITCH},
            'car': 'Togg T10X body panels (meshes/car_body.stl), 4.80 x 1.90 x 1.60 m',
            'panel_limits_x': {k: [round(v[0], 3), round(v[1], 3)] for k, v in lim.items()},
            'phases': []}
    stats, all_strokes = [], {}
    # phase 0: yaw 0  -> right side (y<0) faces the rail; phase 1: yaw 180 -> left side faces the rail
    for yaw, side_sign, half in ((0, -1, (-1.0, 0.0)), (180, +1, (0.0, 1.0))):
        tag = 'R' if side_sign < 0 else 'L'
        surfaces = []
        for name in ('hood', 'roof'):
            x0, x1 = lim[name]
            st = top_strokes(car, x0, x1, *half)
            if not st:
                continue
            cov = coverage(surface_samples(car, 'top', (x0, x1, half[0], half[1])), st)
            surfaces.append({'name': '%s_%s' % (name, tag), 'strokes': [stroke_record(p, n, yaw) for p, n in st]})
            stats.append((yaw, surfaces[-1]['name'], len(st), sum(s['length'] for s in surfaces[-1]['strokes']), cov))
            all_strokes.setdefault(name, []).extend(st)
        st = side_strokes(car, side_sign)
        cov = coverage(surface_samples(car, 'side', side_sign), st)
        surfaces.append({'name': 'side_%s' % tag, 'strokes': [stroke_record(p, n, yaw) for p, n in st]})
        stats.append((yaw, surfaces[-1]['name'], len(st), sum(s['length'] for s in surfaces[-1]['strokes']), cov))
        plan['phases'].append({'turntable_yaw_deg': yaw, 'surfaces': surfaces})
    # union of both turntable positions over the whole panel (car frame)
    union = []
    for name in ('hood', 'roof'):
        x0, x1 = lim[name]
        cov = coverage(surface_samples(car, 'top', (x0, x1, -1.0, 1.0)), all_strokes[name])
        union.append({'surface': name, 'strokes': len(all_strokes[name]), 'coverage': round(cov, 4)})
    plan['union_coverage'] = union
    plan['coverage'] = [{'yaw': a, 'surface': b, 'strokes': c, 'path_m': round(d, 2), 'coverage': round(e, 4)}
                        for a, b, c, d, e in stats]
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        json.dump(plan, f)
    print('pitch %.3f m  standoff %.2f m' % (PITCH, STANDOFF))
    print('%-5s %-16s %7s %9s %9s' % ('yaw', 'surface', 'strokes', 'path[m]', 'coverage'))
    for a, b, c, d, e in stats:
        print('%-5d %-16s %7d %9.2f %8.1f%%' % (a, b, c, d, 100 * e))
    print()
    print('union of both turntable positions, whole panel:')
    for u in union:
        print('  %-12s strokes=%2d coverage=%5.1f%%' % (u['surface'], u['strokes'], 100 * u['coverage']))
    print('total strokes %d, path length %.1f m  ->  %s' % (sum(s[2] for s in stats), sum(s[3] for s in stats), OUT))


if __name__ == '__main__':
    main()
