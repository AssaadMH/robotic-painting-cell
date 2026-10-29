"""Shared helpers for the painting-cell planner and player (ROS 2 Jazzy, MoveIt 2)."""
import json
import math
import os
import struct

import numpy as np
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

ARM_JOINTS = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
RAIL_JOINT = 'rail_joint'
CROSS_JOINT = 'cross_joint'
RAIL_JOINTS = [RAIL_JOINT, CROSS_JOINT]
ALL_JOINTS = RAIL_JOINTS + ARM_JOINTS
CROSS_TOP = 0.0       # cross slide position for roof/hood/trunk work (robot close to the car)
CROSS_SIDE = -0.80    # ... for the side panels (robot 0.8 m further out)
TCP_LINK = 'paint_tcp'
FRAME = 'world'

CAR_Z = 0.01          # car wheels stand on the floor (turntable is flush with it)
RAIL_PARK = 3.4       # carriage parking position while the car turns [m]
RAIL_Y = -2.2


def share(*parts):
    return os.path.join(get_package_share_directory('painting_cell'), *parts)


def load_stl_mesh(path):
    """Binary STL -> shape_msgs/Mesh (vertices deduplicated)."""
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        rec = np.frombuffer(f.read(n * 50), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    tris = rec['v'].reshape(-1, 3)
    uniq, inv = np.unique(np.round(tris, 6), axis=0, return_inverse=True)
    mesh = Mesh()
    mesh.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in uniq]
    inv = inv.reshape(-1, 3)
    mesh.triangles = [MeshTriangle(vertex_indices=[int(a), int(b), int(c)]) for a, b, c in inv]
    return mesh


def quat_from_matrix(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def tool_frame(normal, direction):
    """Rotation of the paint TCP: z = into the surface (-normal), x = stroke direction."""
    z = -np.asarray(normal, float)
    z /= np.linalg.norm(z)
    x = np.asarray(direction, float)
    x = x - z * float(np.dot(x, z))
    nx = np.linalg.norm(x)
    if nx < 1e-9:
        x = np.cross(z, [0, 0, 1.0])
        nx = np.linalg.norm(x)
    x /= nx
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def pose_from(p, R):
    q = quat_from_matrix(R)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = [float(v) for v in q]
    return pose


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def car_pose(yaw_deg):
    pose = Pose()
    pose.position.z = CAR_Z
    pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
    pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
    return pose


def box_object(oid, size, xyz, op=CollisionObject.ADD):
    o = CollisionObject()
    o.header.frame_id = FRAME
    o.id = oid
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(v) for v in size]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [float(v) for v in xyz]
    pose.orientation.w = 1.0
    o.primitives = [prim]
    o.primitive_poses = [pose]
    o.operation = op
    return o


def environment_objects():
    """Floor slab and booth walls (collision objects)."""
    objs = [box_object('floor', (11.0, 8.0, 0.10), (0, 0, -0.05))]
    t, h = 0.15, 3.6
    objs += [
        box_object('wall_back', (11.0, t, h), (0, 3.9, h / 2)),
        box_object('wall_front', (11.0, t, h), (0, -3.9, h / 2)),
        box_object('wall_left', (t, 8.0, h), (-5.4, 0, h / 2)),
        box_object('wall_right', (t, 8.0, h), (5.4, 0, h / 2)),
    ]
    return objs


def object_color(oid, rgba):
    c = ObjectColor()
    c.id = oid
    c.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    return c


def environment_colors():
    cols = [object_color('floor', (0.55, 0.57, 0.60, 1.0)),
            object_color('car', (0.80, 0.80, 0.82, 1.0))]
    for w in ('wall_back', 'wall_front', 'wall_left', 'wall_right'):
        cols.append(object_color(w, (0.60, 0.75, 0.90, 0.12)))
    return cols


def car_object(mesh, yaw_deg, op=CollisionObject.ADD):
    o = CollisionObject()
    o.header.frame_id = FRAME
    o.id = 'car'
    o.meshes = [mesh]
    o.mesh_poses = [car_pose(yaw_deg)]
    o.operation = op
    return o


def scene_diff(objects, colors=()):
    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects = list(objects)
    ps.object_colors = list(colors)
    return ps


def load_paths():
    with open(share('paths', 'paths.json')) as f:
        return json.load(f)
