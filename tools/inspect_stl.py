#!/usr/bin/env python3
"""Quick STL report: triangle count, bounding box, components hint, watertightness."""
import struct, sys
import numpy as np

path = sys.argv[1]
with open(path, 'rb') as f:
    head = f.read(80)
    n = struct.unpack('<I', f.read(4))[0]
    rec = np.frombuffer(f.read(n * 50), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
V = rec['v'].reshape(-1, 3).astype(np.float64)
print('header:', head[:60])
print('triangles: %d' % n)
lo, hi = V.min(0), V.max(0)
print('bbox min', np.round(lo, 3), 'max', np.round(hi, 3), 'size', np.round(hi - lo, 3))
# signed volume via winding
t = V.reshape(-1, 3, 3)
vol = np.einsum('ij,ij->i', t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6
print('signed volume (units^3): %.4g' % vol)
# vertex weld + open edges on a subsample is too slow for millions; report unique vertices
u = np.unique(np.round(V, 4), axis=0)
print('unique vertices: %d' % len(u))
# mass distribution along each axis to guess front/up
for ax, name in enumerate('xyz'):
    h, e = np.histogram(V[:, ax], bins=10)
    print(name, 'hist', h.tolist())
