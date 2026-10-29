#!/usr/bin/env python3
"""Pick report figures from the Gazebo render (timeline.jsonl) and copy them to docs/screenshots as PNG,
plus results/gz_summary.json.   usage: gz_figures.py [render_dir]"""
import json
import os
import shutil
import sys

from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/gz_render')
tl = [json.loads(l) for l in open(os.path.join(R, 'timeline.jsonl'))]
have = {int(f[:-4]) for f in os.listdir(os.path.join(R, 'frames')) if f.endswith('.jpg')}
tl = [r for r in tl if r['k'] in have]


def pick(cond, where=0.5):
    c = [r for r in tl if cond(r)]
    return c[int(len(c) * where)] if c else None


figs = {
    'fig01_gz_cell': pick(lambda r: r['kind'] == 'intro', 0.62),
    'fig02_gz_hood': pick(lambda r: r['kind'] == 'paint' and r['surface'] == 'hood_R', 0.8),
    'fig05_gz_side': pick(lambda r: r['kind'] == 'paint' and r['surface'] == 'side_R', 0.55),
    'fig06_gz_final': pick(lambda r: r['kind'] == 'outro', 0.35),
}
out = os.path.join(ROOT, 'docs', 'screenshots')
for name, r in figs.items():
    if r is None:
        print('%s: no frame yet' % name)
        continue
    Image.open(os.path.join(R, 'frames', '%05d.jpg' % r['k'])).save(os.path.join(out, name + '.png'))
    print('%s <- frame %d (t=%.1f s, %s %s)' % (name, r['k'], r['t'], r['kind'], r['surface']))
s = os.path.join(R, 'summary.json')
if os.path.exists(s):
    shutil.copy(s, os.path.join(ROOT, 'results', 'gz_summary.json'))
    print('summary ->', os.path.join(ROOT, 'results', 'gz_summary.json'))
