#!/usr/bin/env python3
"""ASCII STL -> binary STL (vertices only, normals recomputed from winding).
usage: stl_ascii2bin.py in.stl out.stl"""
import struct, sys
import numpy as np

src, dst = sys.argv[1], sys.argv[2]
vals = []
with open(src, 'r', errors='ignore') as f:
    for line in f:
        s = line.lstrip()
        if s.startswith('vertex'):
            a = s.split()
            vals.append((float(a[1]), float(a[2]), float(a[3])))
V = np.array(vals, np.float32).reshape(-1, 3, 3)
del vals
n = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0])
ln = np.linalg.norm(n, axis=1, keepdims=True)
n = (n / np.where(ln == 0, 1, ln)).astype(np.float32)
rec = np.zeros(len(V), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
rec['n'] = n
rec['v'] = V
with open(dst, 'wb') as f:
    f.write(b'binary from ascii'.ljust(80, b' '))
    f.write(struct.pack('<I', len(V)))
    f.write(rec.tobytes())
print('triangles', len(V), '->', dst)
