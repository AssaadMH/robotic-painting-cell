#!/usr/bin/env python3
"""Assemble the demo video from the Gazebo render (WSL: PIL + Windows ffmpeg).

Input : ~/gz_render/frames/%05d.jpg + timeline.jsonl (from gz_play, 2 frames per simulated second),
        results/{paths,plan,verify}.json
Output: video/demo.mp4 (1280x720, 24 fps, captions only, no audio), the render framed as the Gazebo Sim window

The Gazebo frames are 0.5 s of simulation apart. They are laid out at 10 fps (time-lapse x5 for the job) and
motion-interpolated to 24 fps by ffmpeg. Title and result cards are drawn with PIL.
Then tools/recaption.py redraws the captions after the interpolation (make_video_gz.py --out video/demo_interp.mp4).
usage: make_video_gz.py [--name "..."] [--college "..."] [--repo URL] [--drive URL] [--render DIR] [--no-interp]
"""
import argparse
import json
import os
import shutil
import subprocess

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
VID = os.path.join(ROOT, 'video')
OUTF = os.path.join(VID, 'gz_composed')
FFMPEG = '/mnt/c/Users/HP/Tools/ffmpeg-8.1.2-essentials_build/bin/ffmpeg.exe'
W, H = 1280, 720
FPS_IN = 10                    # 10 render frames (0.5 s of simulation each) per video second = x5 time-lapse; keeps the video over 3 min
T_INTRO = 72.0
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'           # Gazebo GUI text
FONT_B = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
TXT = '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf'   # cards and captions
TXT_B = '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf'
SURF = {'hood': 'hood', 'roof': 'roof', 'side': 'side panel'}
SIDE = {'R': 'right', 'L': 'left'}


def font(sz, bold=False):
    return ImageFont.truetype(FONT_B if bold else FONT, sz)


def tfont(sz, bold=False):
    return ImageFont.truetype(TXT_B if bold else TXT, sz)


def wl(p):
    return p.replace('/mnt/c/', 'C:/')


# ------------------------------------------------------------------ cards
def card(lines):
    """Plain black card; lines = [(text, size, bold)], left aligned and vertically centred."""
    im = Image.new('RGB', (W, H), (0, 0, 0))
    d = ImageDraw.Draw(im)
    gap = [int(sz * 1.6) for _, sz, _ in lines]
    y = (H - sum(gap)) // 2
    for (text, sz, bold), g in zip(lines, gap):
        d.text((110, y), text, font=tfont(sz, bold), fill=(255, 255, 255) if bold else (215, 215, 215))
        y += g
    return im


# ------------------------------------------------------------------ Gazebo window
# Light Gazebo Sim GUI (colours and toolbar icons taken from a screenshot of the real window, video/gz_gui_shot.png).
# The 3D view shows the camera-sensor frame of that step; sim time, iterations and RTF are those of the render.
HDR, BAR, PANEL = 44, 34, 300
VW, VH = W - PANEL, H - HDR - BAR
GUI_BG, GUI_ROW, GUI_ROW2, GUI_HEAD, GUI_TXT = (243, 243, 243), (245, 245, 245), (250, 250, 250), (238, 238, 238), (40, 40, 44)
ENTITIES = ['sun', 'lamp0', 'lamp1', 'booth', 'car', 'painting_cell', 'spray', 'cine']
_toolbar = []


def toolbar():
    if not _toolbar:
        shot = Image.open(os.path.join(VID, 'gz_gui_shot.png')).convert('RGB')
        for box in ((6, 75, 456, 125), (6, 125, 406, 175)):        # shapes / lights row, transform / tools row
            t = shot.crop(box)
            _toolbar.append(t.resize((int(t.width * 0.7), int(t.height * 0.7)), Image.LANCZOS))
    return _toolbar


def clock(t):
    return '%02d:%02d:%06.3f' % (int(t // 3600), int(t % 3600 // 60), t % 60)


def gz_window(frame, r, rtf, caption, speed=''):
    im = Image.new('RGB', (W, H), GUI_BG)
    s = VH / frame.height                                          # fill the 3D view height, crop the sides
    f = frame.resize((int(frame.width * s), VH), Image.LANCZOS)
    x = (f.width - VW) // 2
    im.paste(f.crop((x, 0, x + VW, VH)), (0, HDR))
    d = ImageDraw.Draw(im, 'RGBA')
    # title bar
    d.rectangle([0, 0, W, HDR - 1], fill=GUI_BG)
    for i in range(3):
        d.rectangle([16, 14 + 7 * i, 36, 15 + 7 * i], fill=(0, 0, 0))
    d.text((60, 10), 'Gazebo Sim', font=font(20), fill=GUI_TXT)
    for i in range(3):
        d.ellipse([W - 22, 13 + 7 * i, W - 18, 17 + 7 * i], fill=(0, 0, 0))
    d.line([(0, HDR - 1), (W, HDR - 1)], fill=(215, 215, 215))
    # toolbars over the 3D view
    y = HDR
    for t in toolbar():
        im.paste(t, (4, y))
        y += t.height
    # right panel: component inspector + entity tree
    px = VW
    d.rectangle([px, HDR, W, H - BAR], fill=GUI_ROW2)
    d.line([(px, HDR), (px, H - BAR)], fill=(215, 215, 215))
    d.rectangle([px, HDR, W, HDR + 40], fill=GUI_HEAD)
    d.rectangle([px + 10, HDR + 10, px + 30, HDR + 30], fill=(40, 150, 230))
    d.text((px + 40, HDR + 10), 'painting_cell', font=font(17), fill=GUI_TXT)
    d.text((W - 80, HDR + 10), 'Model', font=font(17), fill=GUI_TXT)
    tcp = r.get('tcp', [0, 0, 0])
    rows = [('Name', 'painting_cell'), ('Static', 'false'), ('Joints', '8 (rail, slide, A1-A6)'),
            ('Controller', 'joint_trajectory'), ('Spray TCP x', '%.3f m' % tcp[0]), ('Spray TCP y', '%.3f m' % tcp[1]),
            ('Spray TCP z', '%.3f m' % tcp[2]), ('Car yaw', '%.0f deg' % r.get('yaw', 0)),
            ('Paint strips', '%d / 1396' % r.get('painted_chunks', 0))]
    y = HDR + 42
    for i, (k, v) in enumerate(rows):
        d.rectangle([px + 1, y, W, y + 26], fill=GUI_ROW if i % 2 == 0 else GUI_ROW2)
        d.text((px + 14, y + 4), k, font=font(15), fill=(70, 70, 76))
        tw = d.textlength(v, font=font(15))
        d.text((W - tw - 12, y + 4), v, font=font(15), fill=(20, 20, 24))
        y += 26
    y += 8
    d.rectangle([px + 1, y, W, y + 36], fill=GUI_HEAD)
    d.text((px + 8, y + 8), 'Entity Tree', font=font(17), fill=GUI_TXT)
    d.text((W - 28, y + 2), '+', font=font(26), fill=(120, 120, 120))
    y += 42
    for e in ENTITIES:
        sel = e == 'painting_cell'
        if sel:
            d.rectangle([px + 1, y - 2, W, y + 22], fill=(210, 228, 246))
        d.text((px + 14, y), '>', font=font(14), fill=(120, 120, 120))
        d.rectangle([px + 32, y + 4, px + 44, y + 16], fill=(250, 200, 60) if e in ('sun', 'lamp0', 'lamp1') else (40, 150, 230))
        d.text((px + 54, y), e, font=font(15), fill=(20, 20, 24))
        y += 26
    # bottom bar: world control + world stats
    d.rectangle([0, H - BAR, W, H], fill=GUI_BG)
    d.line([(0, H - BAR), (W, H - BAR)], fill=(215, 215, 215))
    d.rectangle([14, H - 24, 18, H - 10], fill=(60, 60, 60))     # pause icon: the world is running
    d.rectangle([23, H - 24, 27, H - 10], fill=(60, 60, 60))
    d.polygon([(42, H - 24), (52, H - 17), (42, H - 10)], fill=(60, 60, 60))
    d.rectangle([53, H - 24, 56, H - 10], fill=(60, 60, 60))
    stats = 'RTF %.2f      Sim time %s      Real time %s      Iterations %d' % (
        rtf, clock(r.get('stamp', r['t'])), clock(r.get('wall', 0.0)), r['k'] + 1)
    tw = d.textlength(stats, font=font(15))
    d.text((W - tw - 16, H - 26), stats, font=font(15), fill=GUI_TXT)
    # caption inside the 3D view
    yb = HDR + VH
    d.rectangle([0, yb - 44, VW, yb], fill=(0, 0, 0, 150))
    d.text((18, yb - 35), caption, font=tfont(24), fill=(255, 255, 255))
    if speed:
        tw = d.textlength(speed, font=tfont(20))
        d.text((VW - tw - 18, yb - 32), speed, font=tfont(20), fill=(220, 220, 220))
    return im


def surface_label(s):
    if not s or '_' not in s:
        return ''
    base, side = s.rsplit('_', 1)
    return '%s (%s)' % (SURF.get(base, base), SIDE.get(side, side))


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', default='Lassaad Mahmoudi')
    ap.add_argument('--college', default='ENIG - Ecole Nationale d\'Ingenieurs de Gabes')
    ap.add_argument('--repo', default='github.com/AssaadMH/robotic-painting-cell')
    ap.add_argument('--drive', default='(Google Drive link)')
    ap.add_argument('--render', default=os.path.expanduser('~/gz_render'))
    ap.add_argument('--no-interp', action='store_true')
    ap.add_argument('--out', default=os.path.join(VID, 'demo.mp4'))
    a = ap.parse_args()

    tl = [json.loads(l) for l in open(os.path.join(a.render, 'timeline.jsonl'))]
    paths = json.load(open(os.path.join(ROOT, 'results', 'paths.json')))
    plan = json.load(open(os.path.join(ROOT, 'results', 'plan.json')))
    ver = json.load(open(os.path.join(ROOT, 'results', 'verify.json')))
    m = plan['metrics']
    n_str = len(m['strokes'])
    ok = sum(1 for s in m['strokes'] if s['ok'])
    segs = [s for s in plan['segments'] if s['times']]
    cycle = sum(s['times'][-1] for s in segs)
    path_m = sum(c['path_m'] for c in paths['coverage'])
    cov = {c['surface']: c['coverage'] for c in paths['coverage']}
    uni = {u['surface']: u['coverage'] for u in paths['union_coverage']}

    if os.path.exists(OUTF):
        shutil.rmtree(OUTF)
    os.makedirs(OUTF)
    n_out = [0]

    def emit(im, seconds=None):
        reps = 1 if seconds is None else max(1, int(round(seconds * FPS_IN)))
        for _ in range(reps):
            im.save(os.path.join(OUTF, '%06d.jpg' % n_out[0]), quality=92)
            n_out[0] += 1

    emit(card([('Robotic car painting cell', 54, True),
                ('Simulation in Gazebo Harmonic, ROS 2 Jazzy and MoveIt 2', 28, False),
                ('', 20, False),
                (a.name, 30, True),
                (a.college, 24, False),
                ('Yuktii AI Labs, Robotics Simulation Intern assessment', 24, False)]), 5.0)
    frames = os.path.join(a.render, 'frames')
    by_k = {r['k']: r for r in tl}
    ks = sorted(int(f[:-4]) for f in os.listdir(frames) if f.endswith('.jpg'))
    rtf, prev = 0.0, None
    for k in ks:
        r = by_k.get(k)
        if r is None:
            continue
        frame = Image.open(os.path.join(frames, '%05d.jpg' % k)).convert('RGB')
        t = r['t']
        if prev is not None and 'wall' in r and r['k'] == prev['k'] + 1 and r['wall'] > prev['wall']:
            x = (r['stamp'] - prev['stamp']) / (r['wall'] - prev['wall'])
            rtf = x if rtf == 0.0 else 0.9 * rtf + 0.1 * x
        prev = r
        if r['kind'] == 'intro':
            cap = ('Paint booth with the car on a turntable' if t < 24 else
                   'KUKA KR 210 L150 on a rail and cross slide' if t < 48 else 'Car body in primer, 4.8 x 1.9 x 1.6 m')
            im = gz_window(frame, r, rtf, cap)
        elif r['kind'] == 'outro':
            im = gz_window(frame, r, rtf, 'Done: %d of %d strokes painted' % (ok, n_str))
        else:
            if r['kind'] == 'rotate':
                cap = 'Turning the car 180 degrees'
            elif r['kind'] == 'paint':
                cap = 'Painting the %s, stroke %d of %d' % (surface_label(r['surface']), r['strokes_done'] + 1, n_str)
            else:
                cap = 'Moving to the next stroke'
            im = gz_window(frame, r, rtf, cap, '%gx speed' % (FPS_IN / 2.0))
        emit(im)
    emit(card([('Results', 44, True),
               ('', 16, False),
               ('Strokes planned and painted: %d of %d' % (ok, n_str), 28, False),
               ('Spray path: %.1f m' % path_m, 28, False),
               ('Coverage: hood %.1f %%, roof %.1f %%, sides %.1f %%' % (100 * uni.get('hood', 0), 100 * uni.get('roof', 0), 100 * cov.get('side_R', 0)), 28, False),
               ('Collision check: %d robot states, %d collisions' % (ver['collision_checked'], ver['collision_invalid']), 28, False),
               ('Cycle time: %.1f min for one car' % (cycle / 60), 28, False)]), 8.0)
    emit(card([(a.name, 34, True),
               (a.repo, 26, False),
               ('', 16, False),
               ('Thank you', 30, False)]), 5.0)
    print('composed %d frames (%.1f s at %d fps)' % (n_out[0], n_out[0] / FPS_IN, FPS_IN))
    out = a.out
    vf = 'minterpolate=fps=24:mi_mode=mci:mc_mode=obmc:me=epzs:vsbmc=0,format=yuv420p' if not a.no_interp else 'fps=24,format=yuv420p'
    cmd = [FFMPEG, '-y', '-hide_banner', '-loglevel', 'error', '-framerate', str(FPS_IN), '-i', wl(os.path.join(OUTF, '%06d.jpg')),
           '-vf', vf, '-c:v', 'libx264', '-preset', 'medium', '-crf', '19', '-movflags', '+faststart', wl(out)]
    subprocess.run(cmd, check=True)
    print('video ->', out)


if __name__ == '__main__':
    main()
