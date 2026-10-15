"""X500 lumberjack self-return geometry, expressed in Gazebo base_link (metres).

Static dimensions are derived from x500_base/model.sdf and
x500_lumberjack/model.sdf supplied on 2026-10-14.
Moving-arm kinematics match uav_lumberjack_control/arm_kinematics.cpp:
logical q2/q3/q4 angles are NOT Gazebo's raw joint positions.
All masks operate on arrays (N, 3). No ROS or Open3D dependency here.
"""

import math
import numpy as np


EXPECTED_SENSOR_FRAME = 'x500_lumberjack/mid360_link/mid360_gpu_lidar'


def rotation_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def rotation_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def rpy_matrix(roll, pitch, yaw):
    """SDF/ROS conventional Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)


# SDF: base_link -> mid360_mount_link -> mid360_link -> mid360_gpu_lidar.
# The sensor's own pose relative_to=mid360_link is identity.
MOUNT_ROTATION = rpy_matrix(0., 0.349066, 0.)
BASE_SENSOR_ROTATION = MOUNT_ROTATION @ rpy_matrix(0., 0., 3.141593)
BASE_SENSOR_TRANSLATION = np.array([0.099, 0.0, 0.010]) + (
    MOUNT_ROTATION @ np.array([0.0, 0.0, 0.033])
)


def sensor_to_base(points_sensor):
    points_sensor = np.asarray(points_sensor, dtype=np.float64)
    return points_sensor @ BASE_SENSOR_ROTATION.T + BASE_SENSOR_TRANSLATION


def quaternion_xyzw_to_rotation(q):
    """Return body->world rotation, taking quaternion as (x,y,z,w)."""
    x, y, z, w = [float(v) for v in q]
    d = x*x + y*y + z*z + w*w
    if d < 1e-12:
        raise ValueError('Invalid odometry quaternion')
    k = 2.0 / d
    return np.array([
        [1.-k*(y*y+z*z), k*(x*y-z*w), k*(x*z+y*w)],
        [k*(x*y+z*w), 1.-k*(x*x+z*z), k*(y*z-x*w)],
        [k*(x*z-y*w), k*(y*z+x*w), 1.-k*(x*x+y*y)],
    ])


def mask_box(points, center, size, margin=0.0, rotation=None):
    rel = points - np.asarray(center, dtype=np.float64)
    if rotation is not None:
        rel = rel @ np.asarray(rotation, dtype=np.float64)
    half = np.asarray(size, dtype=np.float64) * 0.5 + margin
    return np.all(np.abs(rel) <= half, axis=1)


def mask_vertical_cylinder(points, center, radius, height, margin=0.0):
    rel = points - np.asarray(center, dtype=np.float64)
    radial_sq = rel[:, 0] ** 2 + rel[:, 1] ** 2
    return ((radial_sq <= (radius + margin) ** 2) &
            (np.abs(rel[:, 2]) <= height * 0.5 + margin))


def mask_capsule(points, p0, p1, radius, margin=0.0):
    p0 = np.asarray(p0, dtype=np.float64)
    d = np.asarray(p1, dtype=np.float64) - p0
    dd = float(d @ d)
    if dd < 1e-12:
        rel = points - p0
        return np.einsum('ij,ij->i', rel, rel) <= (radius + margin)**2
    t = np.clip((points - p0) @ d / dd, 0., 1.)
    diff = points - (p0 + t[:, None] * d)
    return np.einsum('ij,ij->i', diff, diff) <= (radius + margin)**2


def static_self_mask(points_base, margin=0.008):
    """Chassis, skids, rotors/motor hubs, camera and fixed arm mounts."""
    p = np.asarray(points_base, dtype=np.float64)
    mask = np.zeros(len(p), dtype=bool)

    # PX4 x500_base collision box and skid primitives.
    mask |= mask_box(p, [0., 0., .007], [.3535534, .3535534, .050], margin)
    for side in (-1., 1.):
        # Skid rod: base_link_collision_1/2 rolled +/-0.35 rad around X.
        tilt = side * .35
        center = np.array([0., side*.098, -.123])
        axis = rotation_x(tilt) @ np.array([0., 0., 1.])
        mask |= mask_capsule(p, center - axis*.105, center + axis*.105,
                             .011, margin)
        mask |= mask_capsule(p, [-.125, side*.132, -.2195],
                             [.125, side*.132, -.2195], .011, margin)

    # Four disks: rotor_0...3; collision blade extends 0.27923 m.
    # A swept disk is conservative and independent of instantaneous blade angle.
    for x in (-.174, .174):
        for y in (-.174, .174):
            mask |= mask_vertical_cylinder(p, [x, y, .060],
                                            .279230769/2, .017, margin)
            mask |= mask_vertical_cylinder(p, [x, y, .032], .030, .060, margin)

    # Fixed mounts: camera (SDF base_link pose), MID360 housing, arm pedestal.
    mask |= mask_box(p, [.120, 0., -.025], [.020, .040, .040],
                     margin, rotation_y(.436332))
    mask |= mask_box(p, BASE_SENSOR_TRANSLATION, [.073, .065, .060],
                     margin, BASE_SENSOR_ROTATION)
    mask |= mask_vertical_cylinder(p, [0., 0., -.045], .055, .040, margin)
    mask |= mask_vertical_cylinder(p, [0., 0., -.080], .038, .070, margin)
    return mask


def arm_landmarks(logical_joint_deg):
    """Actual J2/J3/J4 geometry, copied from the project's FK convention.

    Returns link pivot positions in base_link, plus saw orientation.
    """
    q2, q3, q4 = np.deg2rad(np.asarray(logical_joint_deg, dtype=float))
    upper = rotation_y(-q2)
    fore = rotation_y(-(q2 + q3))
    wrist_rot = fore @ rotation_x(q4)
    saw_rot = fore @ rotation_x(q4 + math.pi/2)
    j2 = np.array([0., 0., -.110])
    j3 = j2 + upper @ np.array([.180, 0., 0.])
    j4 = j3 + fore @ np.array([.170, 0., 0.])
    wrist = j4 + fore @ np.array([.025, 0., 0.])
    saw = wrist + wrist_rot @ np.array([.060, .030, 0.])
    return j2, j3, j4, wrist, saw, saw_rot


def moving_arm_mask(points_base, logical_joint_deg, margin=.008):
    """Capsule/OBB union around actual moving arm and chain-saw cutting bar."""
    p = np.asarray(points_base, dtype=np.float64)
    j2, j3, j4, wrist, saw, saw_rot = arm_landmarks(logical_joint_deg)
    mask = mask_capsule(p, j2, j3, .029, margin)
    mask |= mask_capsule(p, j3, j4, .027, margin)
    mask |= mask_capsule(p, j4, wrist, .030, margin)
    for pivot, r in ((j2, .038), (j3, .032), (j4, .030)):
        mask |= mask_capsule(p, pivot, pivot, r, margin)
    # Chainsaw's 0.10 m long bar + radius 0.035 end rollers = 0.17 m length.
    # Saw drive sprocket and cutting-zone collision are enclosed by the OBB.
    mask |= mask_box(p, saw, [.175, .082, .028], margin, saw_rot)
    return mask


def self_filter_mask(points_base, joint_deg, margin=.008):
    mask = static_self_mask(points_base, margin)
    if joint_deg is not None:
        mask |= moving_arm_mask(points_base, joint_deg, margin)
    return mask
