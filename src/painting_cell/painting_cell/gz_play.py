"""Execute the recorded MoveIt plan in Gazebo Harmonic and film it.

The whole plan is sent as one FollowJointTrajectory goal to the joint_trajectory_controller (gz_ros2_control). The
JTC rejects a trajectory stamped far in the future, so it is sent unstamped ("start now") on the frame where the
job begins, while the world is paused. The world is stepped one camera frame at a time (1/FPS s of simulation time); before
each step the player
  * moves the cinema camera along a smooth path,
  * turns the car on the turntable (static model, set_pose),
  * puts the spray cone on the paint TCP while a paint pass is running (forward kinematics of the plan at the
    arm's measured phase, so the cone stays on the gun),
  * recolours the paint strips the gun has passed (visual_config),
and after the step it saves the camera image stamped at that simulation time. The joint states published by
Gazebo are logged against the plan (tracking error).

  ros2 run painting_cell gz_play --plan ~/plan_togg.json --out ~/gz_render [--max-frames N]
Outputs: <out>/frames/%05d.jpg, <out>/timeline.jsonl (one line per frame), <out>/summary.json
"""
import argparse
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.visual_pb2 import Visual
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node as GzNode

from painting_cell import cell_common as cc

WORLD = 'booth'
STEPS_PER_FRAME = 10           # physics steps (0.05 s) = ros2_control periods per camera frame (see booth.sdf)
CONTROL_DT = 0.05              # the position loop reaches each command one control period later
PHASE_SEARCH = [0.05 * i for i in range(-20, 21)]  # phase search window around the expected plan time [s]
JOB_LEAD = 2                   # frames between sending the job trajectory and T_INTRO (start delay + position lag)
T_INTRO = 72.0                 # robot waits in READY while the camera tours the cell [s]
T_ROT = 40.0                   # turntable 180 deg
T_OUTRO = 60.0                 # final tour of the painted car
CAR_CENTER = np.array([0.0, 0.0, 0.75])


# ------------------------------------------------------------------ kinematics
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, a):
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


class Chain:
    """Forward kinematics world -> tip from the URDF (fixed, revolute and prismatic joints)."""

    def __init__(self, urdf, tip):
        root = ET.fromstring(urdf)
        by_child = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            ax = j.find('axis')
            axis = [float(v) for v in (ax.get('xyz') if ax is not None else '1 0 0').split()]
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = rpy_matrix(*rpy), xyz
            by_child[j.find('child').get('link')] = (j.get('name'), j.get('type'), T, np.array(axis), j.find('parent').get('link'))
        self.chain = []
        link = tip
        while link in by_child:
            name, typ, T, axis, parent = by_child[link]
            self.chain.insert(0, (name, typ, T, axis))
            link = parent

    def fk(self, q):
        M = np.eye(4)
        for name, typ, T, axis in self.chain:
            M = M @ T
            if typ in ('revolute', 'continuous'):
                R = np.eye(4)
                R[:3, :3] = axis_angle(axis, q[name])
                M = M @ R
            elif typ == 'prismatic':
                P = np.eye(4)
                P[:3, 3] = axis * q[name]
                M = M @ P
        return M


def quat(R):
    return cc.quat_from_matrix(R)           # (x, y, z, w)


def look_at(eye, target):
    """Camera pose for gz: the camera looks along its +x axis, z up."""
    x = np.asarray(target, float) - eye
    x /= np.linalg.norm(x)
    y = np.cross([0, 0, 1.0], x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    return quat(np.column_stack([x, y, z]))


def smoothstep(a):
    a = min(max(a, 0.0), 1.0)
    return a * a * (3 - 2 * a)


# ------------------------------------------------------------------ show timeline
class Show:
    """Plan segments laid out on one clock (show time 0 = first rendered frame)."""

    def __init__(self, plan):
        self.joints = plan['meta']['joints']
        self.items = []                # (t0, t1, segment)
        t = T_INTRO
        for seg in plan['segments']:
            if seg['kind'] == 'rotate':
                self.items.append((t, t + T_ROT, seg))
                t += T_ROT
                continue
            if not seg['times']:
                continue
            d = seg['times'][-1]
            self.items.append((t, t + d, seg))
            t += d
        self.t_job_end = t
        self.t_end = t + T_OUTRO
        self.q0 = dict(zip(self.joints, plan['segments'][0]['positions'][0]))

    def trajectory(self):
        """One JointTrajectory over the whole job (times relative to T_INTRO)."""
        jt = JointTrajectory()
        jt.joint_names = list(self.joints)
        last_t, last_q = -1.0, None
        for t0, t1, seg in self.items:
            if seg['kind'] == 'rotate':
                pts = [(t0, last_q), (t1, last_q)]               # hold while the car turns
            else:
                pts = [(t0 + ts, q) for ts, q in zip(seg['times'], seg['positions'])]
            for t, q in pts:
                if t <= last_t + 1e-4:
                    continue
                p = JointTrajectoryPoint()
                p.positions = [float(v) for v in q]
                rel = t - T_INTRO
                p.time_from_start = Duration(sec=int(rel), nanosec=int(round((rel - int(rel)) * 1e9)) % 1000000000)
                jt.points.append(p)
                last_t, last_q = t, q
        return jt

    def at(self, t):
        """(segment or None, fraction, planned joint dict, turntable yaw)."""
        yaw = 0.0
        q = dict(self.q0)
        cur, frac = None, 0.0
        for t0, t1, seg in self.items:
            if seg['kind'] == 'rotate':
                if t >= t1:
                    yaw = seg['yaw']
                elif t >= t0:
                    yaw = seg['yaw'] - 180.0 + 180.0 * smoothstep((t - t0) / (t1 - t0))
                    cur, frac = seg, (t - t0) / (t1 - t0)
                continue
            if t < t0:
                break
            ts = np.array(seg['times'])
            P = np.array(seg['positions'])
            if t >= t1:
                q = dict(zip(self.joints, P[-1]))
                continue
            u = t - t0
            q = {j: float(np.interp(u, ts, P[:, i])) for i, j in enumerate(self.joints)}
            cur, frac = seg, u / max(t1 - t0, 1e-9)
            break
        return cur, frac, q, yaw


# ------------------------------------------------------------------ camera
class Camera:
    """Smoothed camera: a desired (eye, target) per show time, low-pass filtered."""

    def __init__(self, show):
        self.show = show
        self.eye = None
        self.tgt = None
        self.tcp_lp = None

    def desired(self, t, seg, tcp):
        s = self.show
        if t < T_INTRO:                                        # intro: sweep from the front to the robot side
            a = smoothstep(t / T_INTRO)
            ang = math.radians(40 + a * (-95))                 # 40 deg -> -55 deg around the car
            eye = np.array([4.9 * math.cos(ang), 3.5 * math.sin(ang), 3.1 - 0.8 * a])
            return eye, CAR_CENTER + [0.3 * (1 - a), 0, 0]
        if t > s.t_job_end:                                    # outro: tour of the painted car (far side)
            a = smoothstep((t - s.t_job_end) / T_OUTRO)
            ang = math.radians(-20 + a * 200)
            eye = np.array([4.9 * math.cos(ang), 3.5 * math.sin(ang), 2.0 + 1.0 * math.sin(a * math.pi)])
            return eye, CAR_CENTER
        if seg is not None and seg['kind'] == 'rotate':
            return np.array([4.6, 3.4, 3.35]), np.array([0, 0, 0.5])
        base = (seg or {}).get('surface', '')
        x = float(np.clip(tcp[0], -2.6, 2.6))
        if base.startswith('side'):                            # look along the near side from the end of the car
            end = 4.9 if x < 0 else -4.9
            return np.array([end, -2.3, 2.6]), np.array([x * 0.6, -0.9, 0.85])
        # hood / roof: high three-quarter view from the robot side, tracking the gun along x
        return np.array([x + (1.8 if x < 1 else -1.8), -3.55, 3.3]), np.array([x * 0.7, -0.1, 1.1])

    def update(self, t, seg, tcp, dt):
        self.tcp_lp = tcp if self.tcp_lp is None else self.tcp_lp + (tcp - self.tcp_lp) * (1 - math.exp(-dt / 3.0))
        eye, tgt = self.desired(t, seg, self.tcp_lp)
        tau = 0.6 if (t < T_INTRO or t > self.show.t_job_end) else 6.0
        if self.eye is None:
            self.eye, self.tgt = eye, tgt
        k = 1 - math.exp(-dt / tau)
        self.eye = self.eye + (eye - self.eye) * k
        self.tgt = self.tgt + (tgt - self.tgt) * k
        self.eye = np.array([np.clip(self.eye[0], -5.2, 5.2), np.clip(self.eye[1], -3.75, 3.75), np.clip(self.eye[2], 0.4, 3.45)])
        return self.eye, self.tgt


# ------------------------------------------------------------------ player
class Player(Node):
    def __init__(self, args):
        super().__init__('gz_play')
        self.args = args
        self.pub = self.create_publisher(JointTrajectory, '/painting_controller/joint_trajectory', 10)
        # the ~0.5 MB job trajectory is not delivered over the topic; the action interface carries it
        self.job = ActionClient(self, FollowJointTrajectory, '/painting_controller/follow_joint_trajectory')
        self.js = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.gz = GzNode()
        self.img = {}
        self.img_lock = threading.Lock()
        share = get_package_share_directory('painting_cell')
        urdf = subprocess.run(['xacro', os.path.join(share, 'urdf', 'painting_cell.urdf.xacro')], check=True,
                              capture_output=True, text=True).stdout
        self.chain = Chain(urdf, cc.TCP_LINK)
        self.paint = json.load(open(os.path.join(share, 'gz', 'paint_index.json')))
        self.fps = self.paint['fps']
        self.chunks = {}
        for c in self.paint['chunks']:
            self.chunks.setdefault((c['surface'], c['stroke']), []).append(c)
        self.painted = set()
        self.tau = None                                              # plan time of the arm (track_phase)

    # -- callbacks
    def on_js(self, msg):
        self.js = msg

    def on_img(self, msg):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
        with self.img_lock:
            self.img = {'a': a, 't': msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9}

    # -- gz requests
    def req(self, service, msg, rtype=Boolean, timeout=5000):
        ok, rep = self.gz.request(service, msg, type(msg), rtype, timeout)
        return ok

    def control(self, pause=True, steps=0):
        wc = WorldControl()
        wc.pause = pause
        if steps:
            wc.multi_step = steps
        return self.req('/world/%s/control' % WORLD, wc)

    def set_pose(self, name, xyz, q):
        p = Pose()
        p.name = name
        p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
        return self.req('/world/%s/set_pose' % WORLD, p)

    def recolor(self, visual, rgb, rough, metal):
        v = Visual()
        v.name = visual
        v.parent_name = self.paint['link']
        for c in (v.material.diffuse, v.material.ambient):
            c.r, c.g, c.b, c.a = rgb[0], rgb[1], rgb[2], 1.0
        v.material.specular.r = v.material.specular.g = v.material.specular.b = 0.7
        v.material.specular.a = 1.0
        v.material.pbr.roughness = rough
        v.material.pbr.metalness = metal
        return self.req('/world/%s/visual_config' % WORLD, v)

    # -- main
    def run(self):
        a = self.args
        plan = json.load(open(os.path.expanduser(a.plan)))
        show = Show(plan)
        os.makedirs(os.path.join(a.out, 'frames'), exist_ok=True)
        self.get_logger().info('show: %.0f s simulated (job %.0f s), %d frames at %d fps' %
                               (show.t_end, show.t_job_end - T_INTRO, int(show.t_end * self.fps), self.fps))
        # 1. controllers up? (the world must run for the broadcaster to publish)
        self.control(pause=False)
        t0w = time.time()
        while self.js is None and time.time() - t0w < 180:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.js is None:
            self.get_logger().error('no /joint_states - controllers not running')
            return 1
        while self.pub.get_subscription_count() == 0 and time.time() - t0w < 200:
            rclpy.spin_once(self, timeout_sec=0.2)
        # 2. pause; the first camera image after one frame of stepping fixes the clock. The camera renders
        #    once per STEPS_PER_FRAME steps but at its own phase, so frame k is "the next new image after
        #    the k-th step", stamped t_ref + (k + 1) / fps.
        self.control(pause=True)
        self.set_pose('spray', (0, 0, -20), (0, 0, 0, 1))
        self.set_pose('car', (0, 0, cc.CAR_Z), (0, 0, 0, 1))
        self.gz.subscribe(Image, '/cine', self.on_img)
        time.sleep(1.0)
        dt = 1.0 / self.fps
        self.control(pause=True, steps=STEPS_PER_FRAME)
        if self.wait_new_image(-1.0) is None:
            self.get_logger().error('the camera does not render')
            return 1
        t_ref = self.img['t']
        self.last_stamp = t_ref
        t_zero = t_ref + dt                                          # sim time of show time 0 (frame 0)
        # 3. robot to the first plan pose during the intro; the job trajectory is sent on frame k_job (below)
        hold = JointTrajectory()
        hold.joint_names = list(show.joints)
        hold.points = [JointTrajectoryPoint(positions=[float(show.q0[j]) for j in show.joints], time_from_start=Duration(sec=4))]
        self.pub.publish(hold)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        jt = show.trajectory()
        # the JTC starts an unstamped trajectory at its first update after receipt (the next step) and the position
        # loop lags one control period, so it is sent JOB_LEAD frames before T_INTRO
        k_job = int(round((T_INTRO - JOB_LEAD * CONTROL_DT) * self.fps))
        self.get_logger().info('camera reference %.3f s; trajectory: %d points, sent on frame %d' % (t_ref, len(jt.points), k_job))
        cam = Camera(show)
        saver = queue.Queue(maxsize=8)
        threading.Thread(target=self.save_worker, args=(saver,), daemon=True).start()
        log = open(os.path.join(a.out, 'timeline.jsonl'), 'w')
        n_frames = int(show.t_end * self.fps) if a.max_frames is None else min(a.max_frames, int(show.t_end * self.fps))
        start = a.start_frame
        if start:                                                    # fast-forward: no camera subscriber -> no rendering
            self.gz.unsubscribe('/cine')
            self.get_logger().info('fast-forward to frame %d (physics only)' % start)
        err_max, err_sum, err_n = 0.0, 0.0, 0
        track = []                                                   # (show time, segment kind, measured joints)
        yaw_cur = 0.0
        w0 = time.time()
        prof = {'scene': 0.0, 'step': 0.0, 'image': 0.0, 'post': 0.0}
        for k in range(n_frames):
            p0 = time.time()
            t = k * dt                                               # show time of this frame
            _, _, _, yaw = show.at(t)
            # the arm does not run exactly on the show clock (controller start, position loop, missed updates): the
            # spray cone, the paint and the camera follow the plan at the arm's own phase tau (measured after every
            # step, see track_phase), advanced by one frame = the arm as it is drawn in this frame
            tr = t if self.tau is None else self.tau + dt
            seg, frac, q, _ = show.at(tr)
            M = self.chain.fk(q)
            tcp = M[:3, 3].copy()
            # turntable
            if abs(yaw - yaw_cur) > 1e-6:
                h = math.radians(yaw)
                self.set_pose('car', (0, 0, cc.CAR_Z), (0, 0, math.sin(h / 2), math.cos(h / 2)))
                yaw_cur = yaw
            # spray cone + paint
            spraying = seg is not None and seg['kind'] == 'paint'
            ff = k < start                                           # fast-forward: no camera / spray poses
            if spraying:
                if not ff:
                    self.set_pose('spray', tcp, quat(M[:3, :3]))
                name, idx = seg['stroke_ref']
                for c in self.chunks.get((name, idx), []):
                    if c['end_frac'] <= frac + 0.5 / max(len(self.chunks[(name, idx)]), 1) and c['visual'] not in self.painted:
                        self.recolor(c['visual'], self.paint['paint_rgb'], 0.22, 0.35)
                        self.painted.add(c['visual'])
            else:
                if getattr(self, '_spray_on', True) and not ff:
                    self.set_pose('spray', (0, 0, -20), (0, 0, 0, 1))
                # finish strokes whose paint pass just ended
                for t0_, t1_, sg in show.items:
                    if sg['kind'] == 'paint' and t1_ <= tr:
                        for c in self.chunks.get(tuple(sg['stroke_ref']), []):
                            if c['visual'] not in self.painted:
                                self.recolor(c['visual'], self.paint['paint_rgb'], 0.22, 0.35)
                                self.painted.add(c['visual'])
            self._spray_on = spraying and not ff
            eye, tgt = cam.update(tr, seg, tcp, dt)
            if not ff:
                self.set_pose('cine', eye, look_at(eye, tgt))
            if k == k_job:
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = jt
                fut = self.job.send_goal_async(goal)
                tg = time.time()
                while not fut.done() and time.time() - tg < 60:
                    rclpy.spin_once(self, timeout_sec=0.05)
                if not (fut.done() and fut.result().accepted):
                    self.get_logger().error('job trajectory rejected by the painting_controller')
                    return 1
            # step one frame and wait for its image
            p1 = time.time()
            img = None
            if k < start:                                            # fast-forward: physics only, camera unsubscribed
                it = self.iterations()
                self.control(pause=True, steps=STEPS_PER_FRAME)
                while self.iterations() < it + STEPS_PER_FRAME:
                    time.sleep(0.05)
                p2 = p3 = time.time()
            else:
                if k == start and start:
                    self.gz.subscribe(Image, '/cine', self.on_img)
                    time.sleep(1.0)
                    self.last_stamp = t_zero + (k - 1) * dt + 1e-3
                self.control(pause=True, steps=STEPS_PER_FRAME)
                p2 = time.time()
                img = self.wait_new_image(self.last_stamp, timeout=600.0 if k == start else 120.0)
                if img is not None:
                    self.last_stamp = self.img['t']
                p3 = time.time()
            rclpy.spin_once(self, timeout_sec=0.0)
            for _ in range(20):                                      # drain joint states
                rclpy.spin_once(self, timeout_sec=0.0)
            err, act_q = None, []
            if self.js is not None:
                act = dict(zip(self.js.name, self.js.position))
                if all(j in act for j in show.joints):
                    act_q = [act[j] for j in show.joints]
                    if t >= T_INTRO:
                        self.track_phase(show, act, tr)
                    err = max(abs(act[j] - q[j]) for j in show.joints)
                    if T_INTRO <= t <= show.t_job_end:
                        err_max, err_sum, err_n = max(err_max, err), err_sum + err, err_n + 1
                        track.append((t, seg['kind'] if seg else '-', dict(act)))
            if img is not None and k >= start:
                saver.put((os.path.join(a.out, 'frames', '%05d.jpg' % k), img))
            done = sum(1 for t0_, t1_, sg in show.items if sg['kind'] == 'paint' and t1_ <= tr)
            log.write(json.dumps({'k': k, 't': round(t, 4), 'stamp': round(self.last_stamp, 4), 'kind': seg['kind'] if seg else ('intro' if t < T_INTRO else 'outro'),
                                  'surface': seg.get('surface', '') if seg else '', 'stroke': seg.get('stroke', -1) if seg else -1,
                                  'frac': round(frac, 3), 'yaw': yaw, 'strokes_done': done, 'painted_chunks': len(self.painted),
                                  'joint_err': err, 'phase': round(t - tr, 3), 'q_act': [round(v, 4) for v in act_q], 'q_plan': [round(q[j], 4) for j in show.joints], 'tcp': [round(v, 4) for v in tcp], 'wall': round(time.time() - w0, 3)}) + '\n')
            p4 = time.time()
            for key, v in (('scene', p1 - p0), ('step', p2 - p1), ('image', p3 - p2), ('post', p4 - p3)):
                prof[key] += v
            if k % 30 == 0 or k < 4:
                self.get_logger().info('profile per frame: ' + ', '.join('%s %.2f s' % (key, v / (k + 1)) for key, v in prof.items()))
                el = time.time() - w0
                self.get_logger().info('frame %d/%d  t=%.1f s  %s  strokes %d  err %.4f  (%.2f s/frame, ETA %.0f min)' % (
                    k, n_frames, t, seg['kind'] if seg else '-', done, err or 0, el / (k + 1), el / (k + 1) * (n_frames - k) / 60))
                log.flush()
        saver.put(None)
        saver.join() if hasattr(saver, 'join') else None
        time.sleep(2)
        log.close()
        summary = {'frames': n_frames, 'fps_sim': self.fps, 'show_s': show.t_end, 'job_s': show.t_job_end - T_INTRO,
                   'joint_err_max_rad': err_max, 'joint_err_mean_rad': err_sum / max(err_n, 1),
                   'painted_chunks': len(self.painted), 'chunks_total': len(self.paint['chunks']),
                   'wall_s': time.time() - w0}
        summary.update(self.path_stats(show, track))
        json.dump(summary, open(os.path.join(a.out, 'summary.json'), 'w'), indent=1)
        self.get_logger().info('done: %s' % summary)
        return 0

    def path_stats(self, show, track):
        """How well Gazebo followed the planned path, independent of timing: for every job frame, the distance from
        the measured nozzle position to the nearest planned nozzle position within +-3 s, and that plan time minus
        the show time (the arm's lead)."""
        if not track:
            return {}
        ts, qs = [], []                                              # the plan's knots on the show clock
        for t0, t1, seg in show.items:
            if seg['kind'] != 'rotate':
                ts += [t0 + s for s in seg['times']]
                qs += seg['positions']
        ts, qs = np.array(ts), np.array(qs)
        keep = np.r_[True, np.diff(ts) > 1e-6]
        ts, qs = ts[keep], qs[keep]
        g = np.arange(T_INTRO, show.t_job_end, 0.05)
        Q = np.stack([np.interp(g, ts, qs[:, i]) for i in range(len(show.joints))], 1)
        P = np.array([self.chain.fk(dict(zip(show.joints, q)))[:3, 3] for q in Q])
        dev, dev_paint, lead = [], [], []
        for t, kind, act in track:
            p = self.chain.fk(act)[:3, 3]
            i0, i1 = np.searchsorted(g, t - 3.0), np.searchsorted(g, t + 3.0)
            if i1 <= i0:
                continue
            e = np.linalg.norm(P[i0:i1] - p, axis=1)
            i = int(e.argmin())
            dev.append(e[i] * 1000)
            lead.append(g[i0 + i] - t)
            if kind == 'paint':
                dev_paint.append(e[i] * 1000)
        if not dev:
            return {}
        pc = lambda v, q: float(np.percentile(v, q)) if v else 0.0
        return {'nozzle_dev_mm_mean': float(np.mean(dev)), 'nozzle_dev_mm_p99': pc(dev, 99), 'nozzle_dev_mm_max': float(max(dev)),
                'nozzle_dev_paint_mm_mean': float(np.mean(dev_paint)) if dev_paint else 0.0, 'nozzle_dev_paint_mm_p99': pc(dev_paint, 99),
                'nozzle_dev_paint_mm_max': float(max(dev_paint)) if dev_paint else 0.0, 'paint_frames': len(dev_paint), 'job_frames': len(dev),
                'lead_s_mean': float(np.mean(lead)), 'lead_s_min': float(min(lead)), 'lead_s_max': float(max(lead)),
                'paint_frames_over_10mm': int(sum(1 for v in dev_paint if v > 10.0))}

    def track_phase(self, show, act, guess):
        """Plan time tau whose pose matches the measured arm, searched in +-1 s around the expected value; a small
        pull towards the guess keeps tau advancing while the arm holds still (turntable, stops)."""
        best, tau = None, guess
        for d in PHASE_SEARCH:
            q = show.at(guess + d)[2]
            e = max(abs(act[j] - q[j]) for j in show.joints) + 0.01 * abs(d)
            if best is None or e < best:
                best, tau = e, guess + d
        self.tau = tau

    def iterations(self):
        """Current world iteration count (from the low-rate /stats topic)."""
        box = {}
        ev = threading.Event()

        def cb(msg):
            box['it'] = msg.iterations
            ev.set()
        self.gz.subscribe(WorldStatistics, '/stats', cb)
        ev.wait(10.0)
        self.gz.unsubscribe('/stats')
        return box.get('it', 0)

    def wait_new_image(self, after, timeout=120.0):
        """The first image stamped later than `after` (sim seconds)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.img_lock:
                if self.img and self.img['t'] > after + 1e-4:
                    return self.img['a']
            time.sleep(0.003)
        self.get_logger().warn('no new image after t=%.3f' % after)
        return None

    @staticmethod
    def save_worker(q):
        from PIL import Image as PImage
        while True:
            item = q.get()
            if item is None:
                q.task_done()
                return
            path, a = item
            PImage.fromarray(a).save(path, quality=92)
            q.task_done()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plan', default='~/plan_togg.json')
    ap.add_argument('--out', default=os.path.expanduser('~/gz_render'))
    ap.add_argument('--max-frames', type=int)
    ap.add_argument('--start-frame', type=int, default=0)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Player(args)
    rc = node.run()
    node.destroy_node()
    os._exit(rc or 0)
