#!/usr/bin/env python3
"""Plan the whole painting job with MoveIt 2 and record every motion to results/plan.json.

For every stroke on the car:
  1. rail to the stroke's station (collision-checked)            -- group 'rail'
  2. IK + OMPL move to a pre-stroke pose 200 mm off the surface    -- group 'kr210'
  3. Cartesian approach down to the 250 mm spray standoff
  4. Cartesian painting pass along the surface-following polyline (collision checked)
  5. Cartesian retreat along the surface normal
If a stroke cannot be planned it is retried from other rail stations and with the gun tilted a little.
Between the two turntable positions the arm is parked, the rail goes to its end stop and the car
rotation is swept in 5 degree steps against the full scene (rail, robot, walls).
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             PlanningOptions, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from painting_cell import cell_common as cc

SPRAY_SPEED = 0.40          # m/s along the surface while painting
APPROACH = 0.20             # pre-stroke distance beyond the standoff [m]
MIN_FRACTION = 0.995
READY = {'joint_a1': 0.0, 'joint_a2': 0.35, 'joint_a3': -1.10, 'joint_a4': 0.0, 'joint_a5': 1.00, 'joint_a6': 0.0}


class CellPlanner(Node):
    def __init__(self):
        super().__init__('cell_planner')
        self.cart = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.ik_srv = self.create_client(GetPositionIK, '/compute_ik')
        self.valid_srv = self.create_client(GetStateValidity, '/check_state_validity')
        self.scene_srv = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.mg = ActionClient(self, MoveGroup, '/move_action')
        for c in (self.cart, self.ik_srv, self.valid_srv, self.scene_srv):
            while not c.wait_for_service(timeout_sec=2.0):
                self.get_logger().info('waiting for %s ...' % c.srv_name)
        while not self.mg.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('waiting for /move_action ...')
        self.state = dict(READY)
        self.state[cc.RAIL_JOINT] = 0.0
        self.state[cc.CROSS_JOINT] = 0.0
        self.yaw = 0.0
        self.mesh = cc.load_stl_mesh(cc.share('meshes', 'car_full.stl'))
        self.segments = []
        self.metrics = {'strokes': [], 'collision_rejections': 0, 'plans': 0, 'fallbacks': 0,
                        'failed': 0, 'sweep_checks': 0, 'sweep_invalid': 0, 'ik_fail': 0, 'ompl_fail': 0}

    # ---------------------------------------------------------------- low level
    def call(self, client, req, timeout=30.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def robot_state(self, state=None):
        s = state or self.state
        rs = RobotState()
        js = JointState()
        js.name = list(cc.ALL_JOINTS)
        js.position = [float(s[j]) for j in cc.ALL_JOINTS]
        rs.joint_state = js
        return rs

    def apply_scene(self, yaw, first=False):
        objs = cc.environment_objects() if first else []
        objs.append(cc.car_object(self.mesh, yaw))
        req = ApplyPlanningScene.Request()
        req.scene = cc.scene_diff(objs, cc.environment_colors() if first else [])
        self.call(self.scene_srv, req)
        self.yaw = yaw

    def state_valid(self, state=None):
        req = GetStateValidity.Request()
        req.robot_state = self.robot_state(state)
        req.group_name = 'kr210_rail'
        res = self.call(self.valid_srv, req)
        return res.valid, res.contacts

    def ik(self, pose, seed=None, avoid=True):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = 'kr210'
        r.robot_state = self.robot_state(seed)
        r.avoid_collisions = avoid
        r.ik_link_name = cc.TCP_LINK
        ps = PoseStamped()
        ps.header.frame_id = cc.FRAME
        ps.pose = pose
        r.pose_stamped = ps
        r.timeout.sec = 0
        r.timeout.nanosec = 200_000_000
        res = self.call(self.ik_srv, req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        new = dict(self.state if seed is None else seed)
        for n, p in zip(res.solution.joint_state.name, res.solution.joint_state.position):
            if n in new:
                new[n] = p
        return new

    def ik_multi(self, pose, seed):
        """IK from the current posture first, then from several elbow-up postures (KDL is a local solver)."""
        sol = self.ik(pose, seed=seed)
        if sol is not None:
            return sol
        for a1, a2, a3, a5 in ((0.0, 0.35, -1.10, 1.0), (0.3, 0.5, -1.3, 1.2), (-0.3, 0.5, -1.3, 1.2),
                               (0.0, 0.9, -1.8, 0.9), (0.0, 0.0, -0.8, 0.8), (0.0, 1.2, -2.0, 1.2)):
            sd = dict(seed)
            sd.update({'joint_a1': a1, 'joint_a2': a2, 'joint_a3': a3, 'joint_a4': 0.0, 'joint_a5': a5, 'joint_a6': 0.0})
            sol = self.ik(pose, seed=sd)
            if sol is not None:
                return sol
        return None

    def plan_to(self, goal, group, seed=None):
        """OMPL joint-space plan from `seed` (or current) state to `goal` (dict of group joints)."""
        names = {'kr210': cc.ARM_JOINTS, 'rail': cc.RAIL_JOINTS, 'kr210_rail': cc.ALL_JOINTS}[group]
        mg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group
        req.start_state = self.robot_state(seed)
        req.planner_id = 'RRTConnectkConfigDefault'
        req.num_planning_attempts = 6
        req.allowed_planning_time = 4.0
        req.max_velocity_scaling_factor = 0.6
        req.max_acceleration_scaling_factor = 0.4
        c = Constraints()
        for n in names:
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(goal[n])
            jc.tolerance_above = jc.tolerance_below = 1e-3
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        mg.request = req
        mg.planning_options = PlanningOptions()
        mg.planning_options.plan_only = True
        fut = self.mg.send_goal_async(mg)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=60)
        res = rf.result().result
        self.metrics['plans'] += 1
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.metrics['ompl_fail'] += 1
            return None
        return res.planned_trajectory.joint_trajectory

    def cartesian(self, poses, seed=None, avoid=True, max_step=0.02):
        req = GetCartesianPath.Request()
        req.header.frame_id = cc.FRAME
        req.start_state = self.robot_state(seed)
        req.group_name = 'kr210'
        req.link_name = cc.TCP_LINK
        req.waypoints = poses
        req.max_step = max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 0.5
        try:                                   # cap the TCP speed (field exists in MoveIt 2 Jazzy)
            req.max_cartesian_speed = SPRAY_SPEED
            req.cartesian_speed_limited_link = cc.TCP_LINK
        except AttributeError:
            pass
        res = self.call(self.cart, req, 60)
        return res.fraction, res.solution.joint_trajectory

    @staticmethod
    def stretch(jt, factor):
        """Slow a timed trajectory down by `factor` (>1): positions unchanged, times scaled."""
        if factor <= 1.0 + 1e-6:
            return jt
        for pt in jt.points:
            t = (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) * factor
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
            pt.velocities, pt.accelerations = [], []
        return jt

    # ---------------------------------------------------------------- bookkeeping
    @staticmethod
    def last_state(jt, base):
        s = dict(base)
        for n, p in zip(jt.joint_names, jt.points[-1].positions):
            s[n] = p
        return s

    def make_segment(self, kind, jt, base, surface='', stroke=-1, yaw=None):
        """Normalise a JointTrajectory into a segment over ALL_JOINTS."""
        cols = {n: i for i, n in enumerate(jt.joint_names)}
        times, pos = [], []
        for pt in jt.points:
            times.append(pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9)
            pos.append([float(pt.positions[cols[j]]) if j in cols else float(base[j]) for j in cc.ALL_JOINTS])
        return {'kind': kind, 'surface': surface, 'stroke': stroke,
                'yaw': self.yaw if yaw is None else yaw, 'times': times, 'positions': pos}

    # ---------------------------------------------------------------- stroke planning
    def try_stroke(self, name, idx, P, N, station, cross, tilt_deg):
        """Plan one stroke transactionally. Returns (segments, end_state) or (None, reason)."""
        segs, st = [], dict(self.state)
        d = np.gradient(P, axis=0)
        d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
        Nt = N.copy()
        if abs(tilt_deg) > 1e-6:                      # lean the gun sideways about the stroke direction
            t = math.radians(tilt_deg)
            for i in range(len(Nt)):
                a = np.cross(d[i], Nt[i])
                Nt[i] = Nt[i] * math.cos(t) + a * math.sin(t)
        standoff = self.standoff
        tcp = P + N * standoff                          # nozzle position (tilt keeps the nozzle point)
        poses = [cc.pose_from(tcp[i], cc.tool_frame(Nt[i], d[i])) for i in range(len(P))]
        pre = tcp[0] + Nt[0] * APPROACH
        pre_pose = cc.pose_from(pre, cc.tool_frame(Nt[0], d[0]))
        post = tcp[-1] + Nt[-1] * 0.25
        post_pose = cc.pose_from(post, cc.tool_frame(Nt[-1], d[-1]))

        # 1. rail to station
        if abs(st[cc.RAIL_JOINT] - station) > 1e-3 or abs(st[cc.CROSS_JOINT] - cross) > 1e-3:
            goal = {cc.RAIL_JOINT: station, cc.CROSS_JOINT: cross}
            jt = self.plan_to(goal, 'rail', seed=st)
            if jt is None:
                # fold the arm to 'ready' first, then slide
                j1 = self.plan_to(dict(READY), 'kr210', seed=st)
                if j1 is None:
                    return None, 'rail_blocked'
                segs.append(self.make_segment('move', j1, st, name, idx))
                st = self.last_state(j1, st)
                jt = self.plan_to(goal, 'rail', seed=st)
                if jt is None:
                    return None, 'rail_blocked'
            segs.append(self.make_segment('rail', jt, st, name, idx))
            st = self.last_state(jt, st)
        # 2. IK + plan to pre pose
        sol = self.ik_multi(pre_pose, st)
        if sol is None:
            self.metrics['ik_fail'] += 1
            return None, 'ik_pre'
        jt = self.plan_to({j: sol[j] for j in cc.ARM_JOINTS}, 'kr210', seed=st)
        if jt is None:
            return None, 'ompl_pre'
        segs.append(self.make_segment('move', jt, st, name, idx))
        st = self.last_state(jt, st)
        # 3. approach (straight in)
        frac, jt = self.cartesian([poses[0]], seed=st)
        if frac < MIN_FRACTION:
            fr2, _ = self.cartesian([poses[0]], seed=st, avoid=False)
            if fr2 >= MIN_FRACTION:
                self.metrics['collision_rejections'] += 1
            return None, 'approach'
        segs.append(self.make_segment('approach', jt, st, name, idx))
        st = self.last_state(jt, st)
        # 4. paint pass
        frac, jt = self.cartesian(poses[1:], seed=st) if len(poses) > 1 else (1.0, None)
        if frac < MIN_FRACTION or jt is None:
            fr2, _ = self.cartesian(poses[1:], seed=st, avoid=False)
            if fr2 >= MIN_FRACTION:
                self.metrics['collision_rejections'] += 1
            return None, 'paint(%.2f)' % frac
        path_len = float(np.sum(np.linalg.norm(np.diff(tcp[1:], axis=0), axis=1))) if len(tcp) > 2 else 0.0
        T = jt.points[-1].time_from_start.sec + jt.points[-1].time_from_start.nanosec * 1e-9
        if path_len > 0 and T > 0:
            jt = self.stretch(jt, (path_len / SPRAY_SPEED) / T)
        seg = self.make_segment('paint', jt, st, name, idx)
        seg['tcp_path_m'] = path_len
        seg['stroke_ref'] = [name, idx]
        segs.append(seg)
        st = self.last_state(jt, st)
        # 5. retreat
        frac, jt = self.cartesian([post_pose], seed=st)
        if frac < 0.98:
            return None, 'retreat'
        segs.append(self.make_segment('retreat', jt, st, name, idx))
        st = self.last_state(jt, st)
        return segs, st

    def run_stroke(self, name, idx, stroke, yaw):
        Rz = cc.rot_z(yaw)
        P = (Rz @ np.array(stroke['points']).T).T + np.array([0, 0, cc.CAR_Z])
        N = (Rz @ np.array(stroke['normals']).T).T
        base_station = stroke['rail_station']
        is_side = name.startswith('side')
        cross0 = cc.CROSS_SIDE if is_side else cc.CROSS_TOP
        # fallbacks: shift along the rail, move the cross slide, lean the gun
        crosses = [cross0, cross0 - 0.1, cross0 + 0.1] if is_side else [cross0, cross0 - 0.15, cross0 - 0.3]
        tries = [(0.0, crosses[0], 0.0), (0.0, crosses[0], 12.0), (0.0, crosses[0], -12.0),
                 (0.0, crosses[1], 0.0), (0.0, crosses[2], 0.0),
                 (0.35, crosses[0], 0.0), (-0.35, crosses[0], 0.0),
                 (0.7, crosses[0], 0.0), (-0.7, crosses[0], 0.0),
                 (0.35, crosses[1], 12.0), (-0.35, crosses[1], -12.0),
                 (1.05, crosses[0], 0.0), (-1.05, crosses[0], 0.0), (1.4, crosses[0], 0.0), (-1.4, crosses[0], 0.0),
                 (1.05, crosses[1], 12.0), (-1.05, crosses[1], -12.0)]
        reasons = []
        for k, (off, cross, tilt) in enumerate(tries):
            station = float(np.clip(base_station + off, -cc.RAIL_PARK, cc.RAIL_PARK))
            cross = float(np.clip(cross, -0.9, 0.0))
            segs, res = self.try_stroke(name, idx, P, N, station, cross, tilt)
            if segs is not None:
                for sg in segs:
                    self.segments.append(sg)
                self.state = res
                if k > 0:
                    self.metrics['fallbacks'] += 1
                    self.metrics.setdefault('fallback_reasons', []).append({'stroke': '%s[%d]' % (name, idx), 'before_success': reasons})
                self.metrics['strokes'].append({'surface': name, 'stroke': idx, 'ok': True, 'attempt': k,
                                                'station': station, 'cross': cross, 'tilt': tilt,
                                                'length': stroke['length']})
                return True
            reasons.append('%s@%.2f,%.2f/%+.0f' % (res, station, cross, tilt))
        self.metrics['failed'] += 1
        self.metrics['strokes'].append({'surface': name, 'stroke': idx, 'ok': False, 'reasons': reasons,
                                        'length': stroke['length']})
        self.get_logger().warn('stroke %s[%d] FAILED: %s' % (name, idx, '; '.join(reasons[:4])))
        return False

    # ---------------------------------------------------------------- turntable
    def go_ready_and_park(self, park_x):
        jt = self.plan_to(dict(READY), 'kr210')
        if jt is not None:
            self.segments.append(self.make_segment('move', jt, self.state, 'park'))
            self.state = self.last_state(jt, self.state)
        jt = self.plan_to({cc.RAIL_JOINT: park_x, cc.CROSS_JOINT: 0.0}, 'rail')
        if jt is None:
            self.get_logger().error('cannot park the rail')
            return False
        self.segments.append(self.make_segment('rail', jt, self.state, 'park'))
        self.state = self.last_state(jt, self.state)
        return True

    def rotate_car(self, new_yaw):
        """Sweep the car from the current yaw to new_yaw in 5 deg steps, checking the full scene."""
        step = 5.0 if new_yaw > self.yaw else -5.0
        ang = self.yaw
        angles = [ang]
        ok_all = True
        while abs(new_yaw - ang) > 1e-6:
            ang += step
            self.apply_scene(ang)
            valid, contacts = self.state_valid()
            self.metrics['sweep_checks'] += 1
            if not valid:
                ok_all = False
                self.metrics['sweep_invalid'] += 1
                self.get_logger().warn('turntable sweep: collision at %.0f deg: %s' %
                                       (ang, [(c.contact_body_1, c.contact_body_2) for c in contacts[:3]]))
            angles.append(ang)
        self.segments.append({'kind': 'rotate', 'surface': 'turntable', 'stroke': -1, 'yaw': new_yaw,
                              'angles': angles, 'valid': ok_all, 'times': [], 'positions': []})
        return ok_all

    # ---------------------------------------------------------------- driver
    def run(self, only_surfaces=None, max_strokes=None, out=None):
        data = cc.load_paths()
        self.standoff = data['params']['standoff']
        self.apply_scene(0.0, first=True)
        valid, contacts = self.state_valid()
        self.get_logger().info('start state valid: %s' % valid)
        n_done = 0
        t0 = time.time()
        for ph in data['phases']:
            yaw = float(ph['turntable_yaw_deg'])
            if abs(yaw - self.yaw) > 1e-6:
                if not self.go_ready_and_park(cc.RAIL_PARK):
                    break
                self.rotate_car(yaw)
            for sf in ph['surfaces']:
                if only_surfaces and sf['name'] not in only_surfaces:
                    continue
                for i, stroke in enumerate(sf['strokes']):
                    if max_strokes is not None and n_done >= max_strokes:
                        break
                    ok = self.run_stroke(sf['name'], i, stroke, yaw)
                    n_done += 1
                    self.get_logger().info('%s[%d] %s  (%d done, %.0fs)' % (sf['name'], i, 'ok' if ok else 'FAILED',
                                                                           n_done, time.time() - t0))
        jt = self.plan_to(dict(READY), 'kr210')
        if jt is not None:
            self.segments.append(self.make_segment('move', jt, self.state, 'park'))
            self.state = self.last_state(jt, self.state)
        out = out or os.path.join(os.path.expanduser('~'), 'plan.json')
        meta = {'standoff': self.standoff, 'spray_speed': SPRAY_SPEED, 'params': data['params'],
                'joints': cc.ALL_JOINTS, 'planning_wall_time_s': time.time() - t0}
        with open(out, 'w') as f:
            json.dump({'meta': meta, 'metrics': self.metrics, 'segments': self.segments}, f)
        self.get_logger().info('wrote %s' % out)
        return self.metrics


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--max', type=int)
    ap.add_argument('--out')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = CellPlanner()
    m = node.run(only_surfaces=args.only, max_strokes=args.max, out=args.out)
    ok = sum(1 for s in m['strokes'] if s['ok'])
    print('strokes ok %d / %d   fallbacks %d   collision rejections %d   sweep invalid %d/%d' %
          (ok, len(m['strokes']), m['fallbacks'], m['collision_rejections'], m['sweep_invalid'], m['sweep_checks']))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
