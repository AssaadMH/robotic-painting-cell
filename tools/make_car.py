#!/usr/bin/env python3
"""Generate the reference car (4800 x 1900 x 1600 mm) as watertight STL meshes.

The body is a loft of 6-point cross-sections along a side profile (hood, windshield, roof,
rear window, trunk).  Because the geometry is generated, the paint surfaces are known exactly.

Frame: x = length (front = +x), y = width, z = up from the ground (turntable top = 0).
Outputs (metres): meshes/car_body.stl (painted shell) and meshes/car_full.stl (body + 4 wheels,
used as the collision object).
"""
import os
import struct
import sys
import numpy as np

L, W, H = 4.8, 1.9, 1.6
ZBOT = 0.28                      # underbody height above ground

# side profile of the top line: (x, z)
TOP = [(2.40, 0.66), (1.95, 0.80), (0.90, 0.95), (0.25, 1.58),
       (-1.15, 1.60), (-1.80, 1.12), (-2.40, 1.08)]


def ztop(x):
    xs = [p[0] for p in TOP][::-1]
    zs = [p[1] for p in TOP][::-1]
    return float(np.interp(x, xs, zs))


def section(x):
    """6 vertices (left A,B,C then right C,B,A) of the cross-section at x."""
    zt = ztop(x)
    zb = min(0.80, zt - 0.10)                        # widest line
    # half-width of the top strip: hood/trunk 0.85, greenhouse tapers to 0.72 (tumblehome)
    hw_top = float(np.interp(x, [-1.80, -1.15, 0.25, 0.90], [0.85, 0.72, 0.72, 0.85]))
    A, B, C = (0.86, ZBOT), (W / 2, zb), (hw_top, zt)
    pts = [(-A[0], A[1]), (-B[0], B[1]), (-C[0], C[1]), (C[0], C[1]), (B[0], B[1]), (A[0], A[1])]
    return [(x, y, z) for y, z in pts]


def xs_samples():
    brk = [p[0] for p in TOP]
    xs = set(np.round(np.arange(-2.40, 2.4001, 0.1), 4)) | set(brk)
    return sorted(xs)


def build_body():
    xs = xs_samples()
    secs = [section(x) for x in xs]
    V, F = [], []
    for s in secs:
        V.extend(s)
    n = 6
    for i in range(len(secs) - 1):
        for k in range(n):
            a, b = i * n + k, i * n + (k + 1) % n
            c, d = (i + 1) * n + k, (i + 1) * n + (k + 1) % n
            # outward normal: sections are ordered A_l -> B_l -> C_l -> C_r -> B_r -> A_r (clockwise seen from +x? fixed below)
            F.append((a, c, b))
            F.append((b, c, d))
    # end caps (fans), rear = first section, front = last
    for sec_i, flip in ((0, False), (len(secs) - 1, True)):
        base = sec_i * n
        for k in range(1, n - 1):
            tri = (base, base + k, base + k + 1)
            F.append(tri[::-1] if flip else tri)
    V = np.array(V, float)
    F = np.array(F, int)
    return orient_outward(V, F)


def orient_outward(V, F):
    """Flip triangles so signed volume is positive (normals outward)."""
    v0, v1, v2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    vol = np.einsum('ij,ij->i', v0, np.cross(v1, v2)).sum() / 6.0
    if vol < 0:
        F = F[:, ::-1]
    return V, F


def cylinder(center, radius, width, axis_y=True, seg=24):
    cx, cy, cz = center
    V, F = [], []
    for side in (-0.5, 0.5):
        for k in range(seg):
            a = 2 * np.pi * k / seg
            V.append((cx + radius * np.cos(a), cy + side * width, cz + radius * np.sin(a)))
    c0 = len(V)
    V.append((cx, cy - 0.5 * width, cz))
    V.append((cx, cy + 0.5 * width, cz))
    for k in range(seg):
        k2 = (k + 1) % seg
        F += [(k, k2, seg + k), (k2, seg + k2, seg + k), (c0, k2, k), (c0 + 1, seg + k, seg + k2)]
    V = np.array(V, float)
    F = np.array(F, int)
    return orient_outward(V, F)


def write_stl(path, V, F):
    n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = n / np.where(ln == 0, 1, ln)
    with open(path, 'wb') as f:
        f.write(b'painting_cell car'.ljust(80, b' '))
        f.write(struct.pack('<I', len(F)))
        for i, tri in enumerate(F):
            f.write(struct.pack('<3f', *n[i]))
            for j in tri:
                f.write(struct.pack('<3f', *V[j]))
            f.write(b'\0\0')


def merge(parts):
    V, F, off = [], [], 0
    for v, f in parts:
        V.append(v)
        F.append(f + off)
        off += len(v)
    return np.vstack(V), np.vstack(F)


def main(outdir):
    os.makedirs(outdir, exist_ok=True)
    body = build_body()
    wheels = [cylinder((x, y, 0.33), 0.33, 0.22) for x in (-1.45, 1.45) for y in (-0.83, 0.83)]
    write_stl(os.path.join(outdir, 'car_body.stl'), *body)
    write_stl(os.path.join(outdir, 'car_full.stl'), *merge([body] + wheels))
    full = merge([body] + wheels)
    for name, (V, F) in (('body', body), ('full', full)):
        edges = {}
        for tri in F:
            for k in range(3):
                e = tuple(sorted((int(tri[k]), int(tri[(k + 1) % 3]))))
                edges[e] = edges.get(e, 0) + 1
        open_edges = sum(1 for c in edges.values() if c != 2)
        v0, v1, v2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        vol = np.einsum('ij,ij->i', v0, np.cross(v1, v2)).sum() / 6.0
        print('%-5s triangles=%d  non-manifold edges=%d  signed volume=%.3f m3' % (name, len(F), open_edges, vol))
    ext = full[0].max(0) - full[0].min(0)
    print('car (with wheels) L=%.3f W=%.3f H=%.3f m   (target 4.800 x 1.900 x 1.600)' % tuple(ext))

if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), '..', 'src', 'painting_cell', 'meshes'))
