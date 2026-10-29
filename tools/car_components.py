#!/usr/bin/env python3
"""List connected components of the Togg mesh in the car frame (x front, z up, ground z=0)."""
import sys
import numpy as np, trimesh
m = trimesh.load(sys.argv[1], force='mesh', process=True)        # process=True merges vertices
m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))   # y-up -> z-up
m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1]))   # front (-y) -> +x
m.apply_translation([-m.bounds.mean(0)[0], -m.bounds.mean(0)[1], -m.bounds[0][2]])
print('whole', np.round(m.bounds, 3).tolist(), 'faces', len(m.faces))
parts = m.split(only_watertight=False)
print('components', len(parts))
rows = sorted(((len(p.faces), p) for p in parts), key=lambda r: -r[0])
for i, (nf, p) in enumerate(rows[:40]):
    b = p.bounds
    print('%3d faces=%7d  x[%6.2f %6.2f] y[%6.2f %6.2f] z[%5.2f %5.2f] wt=%d' % (i, nf, b[0][0], b[1][0], b[0][1], b[1][1], b[0][2], b[1][2], p.is_watertight))
small = rows[40:]
print('... %d more components, %d faces total' % (len(small), sum(r[0] for r in small)))
if small:
    zs = np.array([r[1].bounds[1][2] for r in small]); print('   their zmax range %.2f..%.2f' % (zs.min(), zs.max()))
