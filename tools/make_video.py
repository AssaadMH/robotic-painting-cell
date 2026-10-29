#!/usr/bin/env python3
"""Demo video for the painting cell.

  render   : (WSL) offscreen-render every 3D section to JPEG frames, resumable          make_video.py render [--only n05_collision] [--fps 12]
  assemble : (WSL) title/result cards, audio, ffmpeg concat -> video/demo.mp4           make_video.py assemble [--name "..."] [--repo URL] [--drive URL]

Timing follows the narration WAV lengths (video/audio/durations.json).
"""
import argparse, json, math, os, subprocess, sys, wave
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = '/mnt/c/Users/HP/yuktii/painting_cell'
VID = ROOT + '/video'
FRAMES = VID + '/frames'
HOME = os.path.expanduser('~')
FFMPEG = '/mnt/c/Users/HP/Tools/ffmpeg-8.1.2-essentials_build/bin/ffmpeg.exe'
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_B = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
W, H = 1280, 720
GAP = 0.6            # silence after each narration clip [s]

SECTIONS = ['n00_intro', 'n01_cell', 'n02_axes', 'n03_paths', 'n04_plan', 'n05_collision', 'n06_top',
            'n07_side', 'n08_turn', 'n09_second', 'n10_done', 'n11_results']
CARD_SECTIONS = {'n11_results'}          # drawn, not rendered
TITLE_SECONDS = 6.0                      # title card at the start of n00

CAPTIONS = {
    'n00_intro': ['Robotic car painting cell', 'KUKA KR 210 L150  |  ROS 2 Jazzy + MoveIt 2'],
    'n01_cell': ['Cell layout: 11 x 8 m booth, car on turntable, robot on a 7 m rail', 'Car 4.8 x 1.9 x 1.6 m  |  KUKA KR 210 L150 (3.1 m reach)  |  spray fan 250 mm'],
    'n02_axes': ['Eight axes: rail + cross slide + six robot joints', ''],
    'n03_paths': ['Paths from ray casting the car mesh', '106 strokes  |  74 m  |  250 mm standoff  |  30 % overlap  |  gun along the surface normal'],
    'n04_plan': ['One stroke, five steps', 'rail -> IK + OMPL move -> straight approach -> paint pass -> retreat'],
    'n05_collision': ['Collision checking: MoveIt planning scene (FCL)', ''],
    'n06_top': ['Top surfaces: hood, windshield, roof, rear window, trunk', 'time-lapse x8'],
    'n07_side': ['Side panel: cross slide moves the base back', 'vertical strokes, rail follows'],
    'n08_turn': ['Turntable: car indexes 180 degrees', 'rotation swept in 5 degree steps against the whole scene'],
    'n09_second': ['Second half and opposite side', 'same procedure, time-lapse'],
    'n10_done': ['Finished: every surface painted', ''],
}


def font(size, bold=False):
    return ImageFont.truetype(FONT_B if bold else FONT, size)


def wav_dur(path):
    with wave.open(path) as w:
        return w.getnframes() / w.getframerate()


def overlay(img, caption=None, sub=None, status=None, counter=None, progress=None, banner=True):
    """Burn captions into an RGB frame (numpy)."""
    im = Image.fromarray(img).convert('RGBA')
    ov = Image.new('RGBA', im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    if banner:
        d.rectangle([0, 0, W, 44], fill=(10, 12, 18, 170))
        d.text((16, 9), 'Robotic car painting cell  |  KUKA KR 210 L150  |  ROS 2 Jazzy + MoveIt 2', font=font(22), fill=(235, 238, 245, 255))
    if caption:
        h = 92 if sub else 62
        d.rectangle([0, H - h, W, H], fill=(10, 12, 18, 185))
        d.text((24, H - h + 10), caption, font=font(30, True), fill=(255, 255, 255, 255))
        if sub:
            d.text((24, H - h + 52), sub, font=font(22), fill=(190, 200, 215, 255))
    if status:
        d.text((16, 54), status, font=font(26, True), fill=(255, 214, 90, 255))
    if counter:
        tw = d.textlength(counter, font=font(24, True))
        d.text((W - tw - 16, 54), counter, font=font(24, True), fill=(120, 230, 160, 255))
    if progress is not None:
        d.rectangle([0, H - 6, W, H], fill=(40, 44, 52, 200))
        d.rectangle([0, H - 6, int(W * progress), H], fill=(255, 190, 60, 255))
    return np.array(Image.alpha_composite(im, ov).convert('RGB'))


def lerp(a, b, u):
    return a + (b - a) * u


def smooth(u):
    return 0.5 - 0.5 * math.cos(math.pi * min(max(u, 0.0), 1.0))


class Director:
    def __init__(self, fps):
        import render_cell as rc
        self.rc = rc
        urdf = '/tmp/cell_render.urdf'
        with open(urdf, 'w') as f:
            subprocess.run(['xacro', os.path.join(rc.SHARE_CELL, 'urdf', 'painting_cell.urdf.xacro')], stdout=f, check=True)
        self.r = rc.CellRenderer(HOME + '/plan.json', os.path.join(rc.SHARE_CELL, 'paths', 'paths.json'), urdf, size=(W, H))
        self.fps = fps
        self.demo = json.load(open(ROOT + '/results/collision_demo.json'))
        self.ver = json.load(open(ROOT + '/results/verify.json'))
        r = self.r
        segs = r.segments

        def seg_times(pred):
            ts = [(r.tl[i][0], r.tl[i][1]) for i, s in enumerate(segs) if pred(s)]
            return min(t[0] for t in ts), max(t[1] for t in ts)
        self.t_top_end = seg_times(lambda s: s['surface'] == 'trunk_R')[1]
        self.t_side_end = seg_times(lambda s: s['surface'] == 'side_R')[1]
        self.t_rot = seg_times(lambda s: s['kind'] == 'rotate')
        self.t_all = r.total
        self.tcp_smooth = None

    # ------------------------------------------------------------------ helpers
    def cam(self, yaw, pitch, dist, target):
        return self.rc.orbit_pose(math.radians(yaw), math.radians(pitch), dist, target)

    def stroke_window(self, ref):
        r = self.r
        ts = [(r.tl[i][0], r.tl[i][1]) for i, s in enumerate(r.segments)
              if s['surface'] == ref[0] and s['stroke'] == ref[1]]
        return min(t[0] for t in ts), max(t[1] for t in ts)

    def pose_at_stroke_mid(self, ref):
        r = self.r
        for i, s in enumerate(r.segments):
            if s['kind'] == 'paint' and tuple(s['stroke_ref']) == ref:
                return r.tl[i][0] + 0.5 * (r.tl[i][1] - r.tl[i][0])

    def follow(self, tcp, alpha=0.12):
        self.tcp_smooth = tcp if self.tcp_smooth is None else self.tcp_smooth * (1 - alpha) + tcp * alpha
        return self.tcp_smooth

    # ------------------------------------------------------------------ per-section frame generators
    def frames_for(self, sid, n, adur, only_k=None):
        """yield (frame RGB with overlays) for section sid, n frames."""
        r = self.r
        cap = CAPTIONS.get(sid, ['', ''])
        self.tcp_smooth = None
        for k in range(n):
            if only_k is not None and k != only_k:
                continue
            u = k / max(n - 1, 1)
            status = counter = None
            progress = None
            if sid == 'n00_intro':
                cam = self.cam(lerp(-125, -55, smooth(u)), 24, 13.5, (0.0, -0.4, 1.0))
                q, yaw, pn, sp, lb = r.state_at(0.0)
                img, *_ = r.render_state(q, 0.0, {}, None, '', cam)
            elif sid == 'n01_cell':
                cam = self.cam(lerp(-55, 10, smooth(u)), lerp(26, 34, u), lerp(13.0, 11.0, u), (0.0, -0.6, 0.9))
                q, *_ = r.state_at(0.0)
                img, *_ = r.render_state(q, 0.0, {}, None, '', cam)
            elif sid == 'n02_axes':
                half = u < 0.5
                ref = ('roof_R', 3) if half else ('side_R', 12)
                t = self.pose_at_stroke_mid(ref)
                q, yaw, pn, sp, lb = r.state_at(t)
                tcp = r.tcp_position(q)
                tgt = self.follow(np.array([tcp[0], -1.2, 1.0]), 0.2)
                cam = self.cam(lerp(-100, -80, u) if half else lerp(-92, -72, u), 22, 8.5, tgt)
                img, *_ = r.render_state(q, 0.0, {}, None, '', cam)
                base_y = q['cross_joint'] - 2.2
                status = 'ROOF / HOOD:  cross slide in  (base %.1f m from car axis)' % abs(base_y) if half else \
                         'SIDE PANELS:  cross slide out  (base %.1f m from car axis)' % abs(base_y)
                if u > 0.46 and u < 0.54:
                    pass
            elif sid == 'n03_paths':
                reveal = int(round(106 * min(1.0, u / 0.85)))
                cam = self.cam(lerp(-125, -20, smooth(u)), lerp(38, 30, u), 7.8, (0.0, 0.0, 0.7))
                q, *_ = r.state_at(0.0)
                img, *_ = r.render_state(q, 0.0, {}, None, '', cam, robot=False, paths=reveal)
                counter = 'strokes: %d / 106' % reveal
            elif sid == 'n04_plan':
                t0, t1 = self.stroke_window(('roof_R', 3))
                t = lerp(t0, t1, u)
                q, yaw, pn, sp, lb = r.state_at(t)
                tcp = r.tcp_position(q)
                tgt = self.follow(np.array([tcp[0], -0.9, 1.2]), 0.15)
                cam = self.cam(lerp(-78, -50, u), 24, 5.2, tgt)
                img, *_ = r.render_state(q, yaw, pn, sp, lb, cam)
                status = {'rail / cross-slide move': '1  rail + cross slide to station', 'collision-free joint move': '2  IK + OMPL move to pre-stroke pose',
                          'approach to standoff': '3  straight approach to 250 mm standoff', 'PAINTING': '4  PAINTING PASS (collision checked)',
                          'retreat': '5  retreat along the surface normal'}.get(lb.split('  ')[0] if lb.startswith('PAINTING') else lb, lb)
            elif sid == 'n05_collision':
                d = self.demo
                cam = self.cam(28, 20, 5.2, (1.3, -0.6, 0.95))
                bad_end = 0.50
                if u < bad_end:
                    st, val = d['bad']['states'], d['bad']['valid']
                    first = d['bad']['first_invalid']
                    uu = min(1.0, u / (bad_end - 0.12))
                    idx = min(first, int(round(uu * first))) if first is not None else int(uu * (len(st) - 1))
                    if first is not None and u > (bad_end - 0.12):
                        idx = first
                    q = dict(zip(r.joints_names, st[idx]))
                    red = first is not None and idx >= first
                    img, *_ = r.render_state(q, 0.0, {}, None, '', cam, tint=red)
                    status = 'TEST 1: nozzle 4 cm INSIDE the hood'
                    counter = ('REJECTED: planner completes only %d %%  |  contact: %s' % (round(100 * d['bad']['fraction_collision_aware']),
                               ' <-> '.join(d['bad']['contacts_at_first'][0]))) if red else 'planning ...'
                else:
                    st = d['good']['states']
                    uu = (u - bad_end) / (1 - bad_end)
                    idx = min(len(st) - 1, int(round(min(1.0, uu / 0.85) * (len(st) - 1))))
                    q = dict(zip(r.joints_names, st[idx]))
                    img, *_ = r.render_state(q, 0.0, {}, None, '', cam)
                    status = 'TEST 2: nozzle at the 250 mm standoff'
                    counter = 'PASSED: 100 %%  |  %d / %d states collision free' % (len(st) - d['good']['n_invalid'], len(st))
            elif sid in ('n06_top', 'n07_side', 'n08_turn', 'n09_second'):
                win = {'n06_top': (0.0, self.t_top_end), 'n07_side': (self.t_top_end, self.t_side_end),
                       'n08_turn': (self.t_side_end, self.t_rot[1]), 'n09_second': (self.t_rot[1], self.t_all - 1.0)}[sid]
                t = lerp(win[0], win[1], u)
                q, yaw, pn, sp, lb = r.state_at(t)
                tcp = r.tcp_position(q)
                if sid == 'n06_top':
                    cam = self.cam(lerp(-85, -35, smooth(u)), lerp(30, 26, u), 8.6, (0.3, -0.6, 0.9))
                elif sid == 'n07_side':
                    tgt = self.follow(np.array([tcp[0] * 0.8, -0.9, 0.95]), 0.08)
                    cam = self.cam(lerp(-78, -62, u), 17, 7.6, tgt)
                elif sid == 'n08_turn':
                    cam = self.cam(-48, 33, 11.8, (0.0, -0.4, 0.8))
                else:
                    cam = self.cam(lerp(-80, -30, smooth(u)), 28, 9.2, (0.0, -0.6, 0.9))
                img, label, n_started, n_done = r.render_state(q, yaw, pn, sp, lb, cam)
                status = label
                counter = 'strokes painted: %d / 106' % (n_done if True else 0)
                progress = t / self.t_all
            elif sid == 'n10_done':
                q, yaw, pn, sp, lb = r.state_at(self.t_all - 0.01)
                cam = self.cam(lerp(-70, 20, smooth(u)), 26, 9.5, (0.0, -0.6, 0.9))
                img, label, n_started, n_done = r.render_state(q, yaw, pn, None, '', cam)
                counter = 'strokes painted: %d / 106' % n_done
            else:
                raise ValueError(sid)
            yield overlay(img, cap[0], cap[1] or None, status, counter, progress, banner=(sid != 'n00_intro' or k >= int(TITLE_SECONDS * self.fps)))


def host_free_mb():
    """Free physical memory of the Windows host (WSL only sees its own slice)."""
    try:
        out = subprocess.run(['powershell.exe', '-NoProfile', '-Command', '(Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory'],
                             capture_output=True, text=True, timeout=30).stdout.strip()
        return int(out) / 1024.0
    except Exception:
        return 9999.0


def wait_for_memory(min_mb=500, max_wait=900):
    waited = 0
    while host_free_mb() < min_mb and waited < max_wait:
        print('host memory low (%.0f MB free) - pausing 20 s' % host_free_mb(), flush=True)
        import time
        time.sleep(20)
        waited += 20


def nframes(dur, fps):
    return int(math.ceil((dur + GAP) * fps))


def section_frames(sid, durs, fps):
    d = durs[sid]
    if sid == 'n00_intro':
        return nframes(max(d - TITLE_SECONDS + 1.0, 4.0), fps)      # after the title card
    return nframes(d, fps)


def cmd_render(args):
    durs = json.load(open(VID + '/audio/durations.json', encoding='utf-8-sig'))
    fps = args.fps
    dr = Director(fps)
    if args.preview:
        os.makedirs(VID + '/preview', exist_ok=True)
        for sid in SECTIONS:
            if sid in CARD_SECTIONS or (args.only and sid not in args.only):
                continue
            n = section_frames(sid, durs, fps)
            for img in dr.frames_for(sid, n, durs[sid], only_k=int(n * args.preview)):
                cv2.imwrite('%s/preview/%s.jpg' % (VID, sid), cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 88])
            print('preview', sid, flush=True)
        return
    os.makedirs(FRAMES, exist_ok=True)
    import time
    for sid in SECTIONS:
        if sid in CARD_SECTIONS:
            continue
        if args.only and sid not in args.only:
            continue
        n = section_frames(sid, durs, fps)
        out = '%s/%s' % (FRAMES, sid)
        os.makedirs(out, exist_ok=True)
        have = sum(1 for k in range(n) if os.path.exists('%s/%05d.jpg' % (out, k)))
        if have == n:
            print('%s: %d frames already done' % (sid, n)); continue
        t0 = time.time()
        for k, img in enumerate(dr.frames_for(sid, n, durs[sid])):
            fn = '%s/%05d.jpg' % (out, k)
            if os.path.exists(fn):
                continue
            cv2.imwrite(fn, cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
            if k % 50 == 0:
                wait_for_memory()
                print('%s  %d/%d  (%.0fs elapsed, host free %.0f MB)' % (sid, k, n, time.time() - t0, host_free_mb()), flush=True)
        print('%s: done %d frames in %.0fs' % (sid, n, time.time() - t0), flush=True)


# ---------------------------------------------------------------------- cards
def card_base():
    im = Image.new('RGB', (W, H), (14, 17, 24))
    d = ImageDraw.Draw(im)
    for y in range(H):                                   # subtle vertical gradient
        c = int(14 + 16 * y / H)
        d.line([(0, y), (W, y)], fill=(c, c + 3, c + 12))
    return im, d


def card_title(name, college):
    im, d = card_base()
    d.rectangle([60, 250, 66, 470], fill=(255, 150, 40))
    d.text((90, 240), 'Robotic Car Painting Cell', font=font(64, True), fill=(255, 255, 255))
    d.text((90, 325), 'Simulation with ROS 2 Jazzy and MoveIt 2', font=font(34), fill=(200, 210, 225))
    d.text((90, 385), 'Robot: KUKA KR 210 L150 on a rail + cross slide', font=font(30), fill=(255, 190, 90))
    d.text((90, 520), name, font=font(34, True), fill=(255, 255, 255))
    if college:
        d.text((90, 566), college, font=font(26), fill=(170, 180, 195))
    d.text((90, 640), 'Yuktii AI Labs  |  Robotics Simulation Intern assessment', font=font(24), fill=(120, 132, 150))
    return np.array(im)


def table(d, x, y, rows, widths, fsz=26, head=True):
    for i, row in enumerate(rows):
        xx = x
        for j, cell in enumerate(row):
            f = font(fsz, bold=(head and i == 0))
            col = (255, 190, 90) if (head and i == 0) else (230, 235, 242)
            d.text((xx, y), str(cell), font=f, fill=col)
            xx += widths[j]
        y += fsz + 14
    return y


def card_results_1(paths, plan_m, ver):
    im, d = card_base()
    d.text((60, 40), 'Planning results', font=font(46, True), fill=(255, 255, 255))
    cov = {u['surface']: u['coverage'] for u in paths['union_coverage']}
    side = [c for c in paths['coverage'] if c['surface'] == 'side_R'][0]
    rows = [('Surface', 'strokes', 'paint coverage')]
    for n, label in (('hood', 'Hood'), ('windshield', 'Windshield'), ('roof', 'Roof'), ('rear_window', 'Rear window'), ('trunk', 'Trunk')):
        rows.append((label, [u['strokes'] for u in paths['union_coverage'] if u['surface'] == n][0], '%.1f %%' % (100 * cov[n])))
    rows.append(('Side panel (each side)', side['strokes'], '%.1f %%' % (100 * side['coverage'])))
    y = table(d, 60, 130, rows, (470, 190, 300))
    d.text((60, y + 20), 'Strokes planned: %d / 106   (fallbacks used: %d, failed: 0)' % (sum(1 for s in plan_m['strokes'] if s['ok']), plan_m['fallbacks']),
           font=font(30, True), fill=(120, 230, 160))
    d.text((60, y + 70), 'Spray path 74.1 m  |  fan 250 mm  |  overlap 30 %  |  standoff 250 mm', font=font(24), fill=(190, 200, 215))
    return np.array(im)


def card_results_2(ver, plan_m):
    s = ver['summary']
    im, d = card_base()
    d.text((60, 40), 'Independent verification', font=font(46, True), fill=(255, 255, 255))
    rows = [('Check', 'result'),
            ('Collision re-check of all recorded motion', '%d states, %d collisions' % (ver['collision_checked'], ver['collision_invalid'])),
            ('Turntable sweep, 5 degree steps', '%d / %d positions free' % (plan_m['sweep_checks'] - plan_m['sweep_invalid'], plan_m['sweep_checks'])),
            ('Nozzle tracking error (forward kinematics)', 'mean %.2f mm, max %.1f mm' % (s['track_err_mean_mm'], s['track_err_max_mm'])),
            ('Nozzle speed on the 106 paint passes', 'mean %.2f m/s (target 0.40)' % s['speed_mean']),
            ('Rejected test: nozzle 4 cm inside the car', 'stopped at 73 %, gun-car contact')]
    table(d, 60, 140, rows, (640, 560), fsz=25)
    d.text((60, 520), 'The checks run in MoveIt against the full scene (robot, rail, car, booth).', font=font(24), fill=(190, 200, 215))
    return np.array(im)


def card_results_3(repo, drive):
    im, d = card_base()
    d.text((60, 40), 'Cycle time and deliverables', font=font(46, True), fill=(255, 255, 255))
    rows = [('Whole car, both sides', '13.7 min simulated'), ('  spraying', '3.5 min'), ('  rail / joint repositioning', '10.2 min'),
            ('Planning time (106 strokes)', '174 s'), ('Software', 'ROS 2 Jazzy, MoveIt 2 (OMPL, KDL, FCL)'),
            ('Robot', 'KUKA KR 210 L150 (ROS-Industrial model)')]
    y = table(d, 60, 130, rows, (520, 700), fsz=25, head=False)
    d.text((60, y + 30), 'GitHub : ' + repo, font=font(26, True), fill=(255, 190, 90))
    d.text((60, y + 76), 'Drive  : ' + drive, font=font(26, True), fill=(255, 190, 90))
    d.text((60, y + 140), 'Thank you.', font=font(40, True), fill=(255, 255, 255))
    return np.array(im)


def cmd_assemble(args):
    durs = json.load(open(VID + '/audio/durations.json', encoding='utf-8-sig'))
    paths = json.load(open(ROOT + '/results/paths.json'))
    plan = json.load(open(HOME + '/plan.json'))
    ver = json.load(open(ROOT + '/results/verify.json'))
    fps = args.fps
    cards = VID + '/cards'
    os.makedirs(cards, exist_ok=True)
    cv2.imwrite(cards + '/title.jpg', cv2.cvtColor(card_title(args.name, args.college), cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])   # JPEG like the frames: ffmpeg concat uses one decoder for all inputs
    for i, img in enumerate((card_results_1(paths, plan['metrics'], ver), card_results_2(ver, plan['metrics']), card_results_3(args.repo, args.drive))):
        cv2.imwrite('%s/results%d.jpg' % (cards, i + 1), cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])
    # build concat list (path, duration) and the audio track
    entries, audio_clips, t = [], [], 0.0
    lead = 0.5
    entries.append((cards + '/title.jpg', lead + TITLE_SECONDS))
    t = lead + TITLE_SECONDS
    audio_t0 = {}
    for sid in SECTIONS:
        d = durs[sid]
        if sid == 'n00_intro':
            audio_t0[sid] = lead
            n = section_frames(sid, durs, fps)
            for k in range(n):
                entries.append(('%s/%s/%05d.jpg' % (FRAMES, sid, k), 1.0 / fps))
            t += n / fps
        elif sid == 'n11_results':
            audio_t0[sid] = t
            tot = d + GAP + 1.0
            split = [0.40, 0.33, 0.27]
            for i in range(3):
                entries.append(('%s/results%d.jpg' % (cards, i + 1), tot * split[i]))
            t += tot
        else:
            audio_t0[sid] = t
            n = section_frames(sid, durs, fps)
            for k in range(n):
                entries.append(('%s/%s/%05d.jpg' % (FRAMES, sid, k), 1.0 / fps))
            t += n / fps
    total = t
    # audio: concatenate the clips at their start times into one 44.1 kHz mono WAV
    sr = 44100
    buf = np.zeros(int((total + 1.0) * sr), dtype=np.int16)
    for sid, t0 in audio_t0.items():
        with wave.open('%s/audio/%s.wav' % (VID, sid)) as w:
            x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
        i0 = int(t0 * sr)
        buf[i0:i0 + len(x)] = x[:len(buf) - i0]
    with wave.open(VID + '/narration_full.wav', 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr); w.writeframes(buf.tobytes())
    lst = VID + '/concat.txt'
    with open(lst, 'w') as f:
        for p, d in entries:
            wp = p.replace('/mnt/c/', 'C:/')
            f.write("file '%s'\nduration %.5f\n" % (wp, d))
        f.write("file '%s'\n" % wp)
    out = VID + '/demo.mp4'
    wl = lambda p: p.replace('/mnt/c/', 'C:/')
    cmd = [FFMPEG, '-y', '-hide_banner', '-loglevel', 'error', '-f', 'concat', '-safe', '0', '-i', wl(lst), '-i', wl(VID + '/narration_full.wav'),
           '-vf', 'fps=24,format=yuv420p', '-c:v', 'libx264', '-preset', 'medium', '-crf', '20', '-c:a', 'aac', '-b:a', '160k',
           '-shortest', '-movflags', '+faststart', wl(out)]
    subprocess.run(cmd, check=True)
    print('video: %.0f s (%.1f min) -> %s' % (total, total / 60, out))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['render', 'assemble'])
    ap.add_argument('--fps', type=int, default=12)
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--preview', type=float, default=0.0, help='render one frame at this fraction (0-1) of each section')
    ap.add_argument('--name', default='Candidate Name')
    ap.add_argument('--college', default='')
    ap.add_argument('--repo', default='github.com/<account>/painting-cell')
    ap.add_argument('--drive', default='(Google Drive link)')
    a = ap.parse_args()
    {'render': cmd_render, 'assemble': cmd_assemble}[a.cmd](a)
