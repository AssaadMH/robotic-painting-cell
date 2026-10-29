#!/usr/bin/env python3
"""Replay results/plan.json in RViz: joint states, turntable (car pose), paint ribbons, spray cone, status text."""
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point
from moveit_msgs.msg import CollisionObject
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from painting_cell import cell_common as cc

SURFACE_COLORS = {
    'hood': (0.90, 0.10, 0.10), 'windshield': (0.95, 0.55, 0.10), 'roof': (0.15, 0.45, 0.90),
    'rear_window': (0.60, 0.25, 0.85), 'trunk': (0.10, 0.75, 0.40), 'side': (0.95, 0.85, 0.15),
}


def surface_color(name):
    base = name.rsplit('_', 1)[0]
    r, g, b = SURFACE_COLORS.get(base, (0.8, 0.8, 0.8))
    if name.endswith('_L'):                 # slightly lighter on the second turntable position
        r, g, b = [min(1.0, v * 0.75 + 0.25) for v in (r, g, b)]
    return r, g, b


class Player(Node):
    def __init__(self):
        super().__init__('paint_player')
        self.declare_parameter('plan', os.path.join(cc.share('..', '..', '..', '..'), 'plan.json'))
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('paths', cc.share('paths', 'paths.json'))
        self.declare_parameter('loop', False)
        plan_file = self.get_parameter('plan').value
        self.speed = float(self.get_parameter('speed').value)
        with open(plan_file) as f:
            self.plan = json.load(f)
        with open(self.get_parameter('paths').value) as f:
            self.paths = json.load(f)
        self.fan = self.paths['params']['fan_width']
        self.standoff = self.paths['params']['standoff']
        self.mesh = cc.load_stl_mesh(cc.share('meshes', 'car_full.stl'))
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.ps_pub = self.create_publisher(__import__('moveit_msgs.msg', fromlist=['PlanningScene']).PlanningScene,
                                            '/planning_scene', 10)
        self.mk_pub = self.create_publisher(MarkerArray, '/paint_markers', 10)
        self.tf_br = TransformBroadcaster(self)            # world -> car: paint ribbons live in the car frame and turn with the turntable
        self.state = [0.0] * 7
        self.state_map = None
        # strokes lookup in the car frame
        self.stroke = {}
        for ph in self.paths['phases']:
            for sf in ph['surfaces']:
                for i, s in enumerate(sf['strokes']):
                    self.stroke[(sf['name'], i)] = (np.array(s['points']), np.array(s['normals']))
        self.painted = {}          # (surface, idx) -> fraction painted (0..1)
        self.painted_yaw = {}
        self.cur = {'text': 'ready', 'cone': None}
        self.yaw = 0.0
        # play list
        self.segments = self.plan['segments']
        self.si = 0
        self.t_seg = 0.0
        self.rot_t = 0.0
        self.last_tick = time.time()
        self.hold = 1.5
        self.done = False
        self.create_timer(0.02, self.tick)
        self.create_timer(0.1, self.publish_markers)
        self.scene_sends = 0
        self.scene_timer = self.create_timer(1.0, self.publish_scene_startup)
        self.marker_m = {}         # ref -> number of ribbon points last published
        self.first = True
        self.get_logger().info('playing %d segments, speed x%.1f' % (len(self.segments), self.speed))

    # ------------------------------------------------------------ helpers
    def set_yaw(self, yaw):
        self.yaw = yaw
        self.publish_scene(env=False)

    def publish_scene_startup(self):
        """Send floor/walls/car a few times at start so a late-starting MoveIt/RViz still receives them."""
        self.publish_scene(env=True)
        self.scene_sends += 1
        if self.scene_sends >= 8:
            self.scene_timer.cancel()

    def publish_scene(self, env=True):
        """Floor, walls and the car at its current yaw. Re-sent every second so a late subscriber still gets it."""
        objs = (cc.environment_objects() if env else []) + [cc.car_object(self.mesh, self.yaw)]
        self.ps_pub.publish(cc.scene_diff(objs, cc.environment_colors() if env else [cc.object_color('car', (0.80, 0.80, 0.82, 1.0))]))

    def interp(self, seg, t):
        times, pos = seg['times'], seg['positions']
        if t >= times[-1]:
            return pos[-1]
        k = int(np.searchsorted(times, t, side='right')) - 1
        k = max(0, min(k, len(times) - 2))
        u = (t - times[k]) / max(times[k + 1] - times[k], 1e-9)
        a, b = np.array(pos[k]), np.array(pos[k + 1])
        return list(a + (b - a) * u)

    def publish_js(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = cc.ALL_JOINTS
        msg.position = [float(v) for v in q]
        self.js_pub.publish(msg)
        self.state = list(q)

    # ------------------------------------------------------------ main loop
    def publish_car_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = cc.FRAME
        t.child_frame_id = 'car'
        t.transform.translation.z = cc.CAR_Z
        t.transform.rotation.z = math.sin(math.radians(self.yaw) / 2)
        t.transform.rotation.w = math.cos(math.radians(self.yaw) / 2)
        self.tf_br.sendTransform(t)

    def tick(self):
        self.publish_car_tf()
        now = time.time()
        dt = (now - self.last_tick) * self.speed
        self.last_tick = now
        if self.first:
            self.first = False
            self.set_yaw(0.0)
        if self.done or self.si >= len(self.segments):
            if not self.done:
                self.done = True
                self.cur['text'] = 'PAINTING COMPLETE'
                self.cur['cone'] = None
                self.get_logger().info('playback finished')
            self.publish_js(self.state)
            return
        seg = self.segments[self.si]
        kind = seg['kind']
        if kind == 'rotate':
            self.rot_t += dt
            dur = 6.0
            a0, a1 = seg['angles'][0], seg['angles'][-1]
            u = min(1.0, self.rot_t / dur)
            u_s = 0.5 - 0.5 * math.cos(math.pi * u)
            self.set_yaw(a0 + (a1 - a0) * u_s)
            self.cur['text'] = 'turntable: car rotating to %d deg' % round(a1)
            self.cur['cone'] = None
            self.publish_js(self.state)
            if u >= 1.0:
                self.rot_t = 0.0
                self.si += 1
            return
        self.t_seg += dt
        T = seg['times'][-1] if seg['times'] else 0.0
        q = self.interp(seg, self.t_seg) if seg['times'] else self.state
        self.publish_js(q)
        label = seg['surface']
        if kind == 'paint':
            ref = tuple(seg['stroke_ref'])
            frac = min(1.0, self.t_seg / max(T, 1e-6))
            self.painted[ref] = frac
            self.painted_yaw[ref] = seg['yaw']
            self.cur['text'] = 'PAINTING  %s  stroke %d' % (label, seg['stroke'] + 1)
            self.cur['cone'] = (ref, frac, seg['yaw'])
        else:
            self.cur['text'] = '%s  %s' % (kind, label)
            self.cur['cone'] = None
        if self.t_seg >= T:
            if kind == 'paint':
                self.painted[tuple(seg['stroke_ref'])] = 1.0     # stroke finished: ribbon complete
            self.t_seg = 0.0
            self.si += 1

    # ------------------------------------------------------------ markers
    def world_stroke(self, ref, yaw):
        P, N = self.stroke[ref]
        Rz = cc.rot_z(yaw)
        return (Rz @ P.T).T + np.array([0, 0, cc.CAR_Z]), (Rz @ N.T).T

    def ribbon(self, ref, frac, n_id):
        P, N = self.stroke[ref]                            # car frame
        m = max(2, int(round(frac * (len(P) - 1))) + 1)
        P, N = P[:m], N[:m]
        d = np.gradient(P, axis=0)
        d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
        b = np.cross(N, d)
        b /= np.maximum(np.linalg.norm(b, axis=1, keepdims=True), 1e-9)
        h = self.fan / 2
        L, R = P - b * h + N * 0.004, P + b * h + N * 0.004          # left / right edge of the ribbon
        tri = []
        for i in range(len(P) - 1):
            tri += [L[i], R[i], L[i + 1], R[i], R[i + 1], L[i + 1],            # front face
                    L[i + 1], R[i], L[i], L[i + 1], R[i + 1], R[i]]            # back face
        r, g, bl = surface_color(ref[0])
        mk = Marker()
        mk.header.frame_id = 'car'
        mk.ns = 'paint'
        mk.id = n_id
        mk.type = Marker.TRIANGLE_LIST
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 1.0
        mk.color = ColorRGBA(r=r, g=g, b=bl, a=1.0)
        mk.points = [Point(x=float(q[0]), y=float(q[1]), z=float(q[2])) for q in tri]
        return mk, m

    def publish_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for n, (ref, frac) in enumerate(list(self.painted.items())):
            m_now = max(2, int(round(frac * (len(self.stroke[ref][0]) - 1))) + 1)
            if self.marker_m.get(ref) == m_now:
                continue                       # unchanged: RViz keeps the marker, do not resend
            mk, m = self.ribbon(ref, frac, n)
            mk.header.stamp = stamp
            self.marker_m[ref] = m
            arr.markers.append(mk)
        # spray cone
        cone = Marker()
        cone.header.frame_id = 'car'
        cone.header.stamp = stamp
        cone.ns = 'spray'
        cone.id = 0
        cone.type = Marker.TRIANGLE_LIST
        if self.cur['cone'] is not None:
            ref, frac, yaw = self.cur['cone']
            P, N = self.stroke[ref]                        # car frame
            k = min(len(P) - 1, int(round(frac * (len(P) - 1))))
            tip = P[k] + N[k] * self.standoff
            d = np.gradient(P, axis=0)[k]
            d /= max(np.linalg.norm(d), 1e-9)
            b = np.cross(N[k], d)
            b /= max(np.linalg.norm(b), 1e-9)
            h = self.fan / 2
            base = [P[k] - b * h, P[k] + b * h, P[k] + d * 0.05, P[k] - d * 0.05]
            cone.action = Marker.ADD
            cone.pose.orientation.w = 1.0
            cone.scale.x = cone.scale.y = cone.scale.z = 1.0
            r, g, bl = surface_color(ref[0])
            cone.color = ColorRGBA(r=r, g=g, b=bl, a=0.35)
            for i in range(4):
                for q in (tip, base[i], base[(i + 1) % 4]):
                    cone.points.append(Point(x=float(q[0]), y=float(q[1]), z=float(q[2])))
        else:
            cone.action = Marker.DELETE
        arr.markers.append(cone)
        txt = Marker()
        txt.header.frame_id = cc.FRAME
        txt.header.stamp = stamp
        txt.ns = 'status'
        txt.id = 0
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = 0.0, -3.0, 3.3
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.35
        txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        done = sum(1 for v in self.painted.values() if v >= 0.999)
        txt.text = '%s\nstrokes painted: %d / %d' % (self.cur['text'], done, len(self.stroke))
        arr.markers.append(txt)
        self.mk_pub.publish(arr)


def main():
    rclpy.init()
    node = Player()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
