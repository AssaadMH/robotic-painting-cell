#!/usr/bin/env python3
"""Second pass on the demo video: steady captions drawn AFTER the motion interpolation.

make_video_gz.py draws a caption into every 10 fps frame before minterpolate, so each caption change is blended
over a couple of 24 fps frames, and on short strokes the caption flips between "Painting" and "Moving" twice a
second. This pass covers the caption bar of the Gazebo window with a clean bar and one caption per surface group
("Painting the roof (left), strokes 55 to 62 of 94"); transit frames belong to the group of the next stroke.

usage (WSL): recaption.py [--render DIR] [--src video/demo_interp.mp4] [--out video/demo.mp4]
"""
import argparse
import json
import os
import shutil
import subprocess

from PIL import Image, ImageDraw

from make_video_gz import FFMPEG, FPS_IN, HDR, ROOT, VH, VID, VW, surface_label, tfont, wl

T_TITLE = 5.0        # title card in front of the Gazebo frames (make_video_gz.main)
EDGE = 0.1           # reach into the cards so the blended card/window frames are covered too
MIN_SEG = 2.0        # shorter caption segments are merged into the previous one


def captions(tl, n_str):
    """[(caption, first_k, last_k)] for the Gazebo frames, in order."""
    nxt, cur = {}, ''
    for r in reversed(tl):                       # surface of the next paint frame, for transit frames
        if r['kind'] == 'paint':
            cur = r['surface']
        nxt[r['k']] = cur
    raw = []
    for r in tl:
        t, kind = r['t'], r['kind']
        if kind == 'intro':
            key = ('Paint booth with the car on a turntable' if t < 24 else
                   'KUKA KR 210 L150 on a rail and cross slide' if t < 48 else 'Car body in primer, 4.8 x 1.9 x 1.6 m', None)
        elif kind == 'outro':
            key = ('Done: %d of %d strokes painted' % (n_str, n_str), None)
        elif kind == 'rotate':
            key = ('Turning the car 180 degrees', None)
        else:
            s = r['surface'] if kind == 'paint' else nxt[r['k']]
            key = (s, r['strokes_done'] + 1 if kind == 'paint' else None)
        raw.append((key, r['k']))
    segs = []                                    # [label, k0, k1, strokes]
    for (label, stroke), k in raw:
        if segs and segs[-1][0] == label:
            segs[-1][2] = k
        else:
            segs.append([label, k, k, set()])
        if stroke is not None:
            segs[-1][3].add(stroke)
    merged = []
    for s in segs:
        if merged and (s[2] - s[1] + 1) / FPS_IN < MIN_SEG:
            merged[-1][2] = s[2]
            merged[-1][3] |= s[3]
        else:
            merged.append(s)
    out = []
    for label, k0, k1, st in merged:
        if st:
            a, b = min(st), max(st)
            rng = 'stroke %d' % a if a == b else 'strokes %d to %d' % (a, b)
            label = 'Painting the %s, %s of %d' % (surface_label(label), rng, n_str)
        out.append((label, k0, k1))
    return out


def bar(text, speed, path):
    im = Image.new('RGB', (VW, 44), (0, 0, 0))
    d = ImageDraw.Draw(im)
    d.text((18, 9), text, font=tfont(24), fill=(255, 255, 255))
    if speed:
        tw = d.textlength(speed, font=tfont(20))
        d.text((VW - tw - 18, 12), speed, font=tfont(20), fill=(220, 220, 220))
    im.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--render', default=os.path.expanduser('~/gz_render'))
    ap.add_argument('--src', default=os.path.join(VID, 'demo_interp.mp4'))
    ap.add_argument('--out', default=os.path.join(VID, 'demo.mp4'))
    a = ap.parse_args()
    tl = [json.loads(l) for l in open(os.path.join(a.render, 'timeline.jsonl'))]
    n_str = len(json.load(open(os.path.join(ROOT, 'results', 'plan.json')))['metrics']['strokes'])
    ks = sorted(int(f[:-4]) for f in os.listdir(os.path.join(a.render, 'frames')) if f.endswith('.jpg'))
    have = set(ks)
    tl = [r for r in tl if r['k'] in have]
    pos = {r['k']: i for i, r in enumerate(tl)}  # composed frame index - title
    segs = captions(tl, n_str)

    tmp = os.path.join(VID, 'captions')
    if os.path.exists(tmp):
        shutil.rmtree(tmp)
    os.makedirs(tmp)
    inputs, chain, last = [], [], '0:v'
    for i, (text, k0, k1) in enumerate(segs):
        t0 = T_TITLE + pos[k0] / FPS_IN
        t1 = T_TITLE + (pos[k1] + 1) / FPS_IN
        if i == 0:
            t0 -= EDGE
        if i == len(segs) - 1:
            t1 += EDGE
        intro = tl[pos[k0]]['kind'] in ('intro', 'outro')
        p = os.path.join(tmp, '%03d.png' % i)
        bar(text, '' if intro else '%gx speed' % (FPS_IN / 2.0), p)
        inputs += ['-i', wl(p)]
        chain.append('[%s][%d:v]overlay=0:%d:enable=\'gte(t,%.3f)*lt(t,%.3f)\'[v%d]' % (last, i + 1, HDR + VH - 44, t0, t1, i))
        last = 'v%d' % i
        print('%7.1f-%7.1f  %s' % (t0, t1, text))
    fs = os.path.join(tmp, 'filter.txt')
    open(fs, 'w').write(';\n'.join(chain + ['[%s]format=yuv420p[out]' % last]))
    cmd = [FFMPEG, '-y', '-hide_banner', '-loglevel', 'error', '-i', wl(a.src)] + inputs + [
        '-filter_complex_script', wl(fs), '-map', '[out]', '-c:v', 'libx264', '-preset', 'medium', '-crf', '19',
        '-movflags', '+faststart', wl(a.out)]
    subprocess.run(cmd, check=True)
    print('video ->', a.out)


if __name__ == '__main__':
    main()
