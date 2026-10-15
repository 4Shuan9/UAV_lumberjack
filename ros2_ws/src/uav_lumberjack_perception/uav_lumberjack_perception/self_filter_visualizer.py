"""Visualize exact Stage 2 self-filter primitives in Gazebo world coordinates.

Diagnostic-only ROS 2 node; does not touch the actual LiDAR point filtering,
perception topics, flight control, or Stage 2 ground estimation.
"""

import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from .environment_geometry import (
    BASE_SENSOR_ROTATION,
    BASE_SENSOR_TRANSLATION,
    arm_landmarks,
    quaternion_xyzw_to_rotation,
    rotation_x,
    rotation_y,
)


def rotation_to_quaternion(rot):
    """3x3 proper rotation matrix -> ROS quaternion (x, y, z, w)."""
    m = np.asarray(rot, dtype=np.float64)
    tr = float(np.trace(m))
    if tr > 0.:
        s = math.sqrt(tr + 1.) * 2.
        q = [(m[2, 1] - m[1, 2]) / s,
             (m[0, 2] - m[2, 0]) / s,
             (m[1, 0] - m[0, 1]) / s, .25 * s]
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = math.sqrt(max(1e-12, 1.+m[0,0]-m[1,1]-m[2,2]))*2.
            q = [.25*s, (m[0,1]+m[1,0])/s,
                 (m[0,2]+m[2,0])/s, (m[2,1]-m[1,2])/s]
        elif i == 1:
            s = math.sqrt(max(1e-12, 1.+m[1,1]-m[0,0]-m[2,2]))*2.
            q = [(m[0,1]+m[1,0])/s, .25*s,
                 (m[1,2]+m[2,1])/s, (m[0,2]-m[2,0])/s]
        else:
            s = math.sqrt(max(1e-12, 1.+m[2,2]-m[0,0]-m[1,1]))*2.
            q = [(m[0,2]+m[2,0])/s, (m[1,2]+m[2,1])/s,
                 .25*s, (m[1,0]-m[0,1])/s]
    q = np.asarray(q, dtype=np.float64)
    q /= max(1e-12, float(np.linalg.norm(q)))
    return q


def align_z_to_vector(direction):
    """Rotation that aligns local cylinder Z with segment direction."""
    z = np.asarray(direction, dtype=np.float64)
    z /= np.linalg.norm(z)
    tmp = np.array([0., 1., 0.]) if abs(z[1]) < .9 else np.array([1., 0., 0.])
    x = np.cross(tmp, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack((x, y, z), axis=1)


class SelfFilterVisualizer(Node):
    def __init__(self):
        super().__init__('self_filter_visualizer')
        self.declare_parameter('odom_topic', '/model/x500_lumberjack/base_link_odometry')
        self.declare_parameter('joint_states_topic', '/lumberjack_arm/joint_states')
        self.declare_parameter('marker_topic', '/perception/lidar/self_filter_geometry')
        self.declare_parameter('self_filter_margin_m', .008)
        self.declare_parameter('visualization_rate_hz', 15.0)
        self.declare_parameter('max_pose_age_sec', .75)
        self.margin = float(self.get_parameter('self_filter_margin_m').value)
        rate = float(self.get_parameter('visualization_rate_hz').value)
        if not 0. <= self.margin <= .05 or not 0.5 <= rate <= 20.:
            raise ValueError('Invalid margin or visualization rate')
        self.pose_max_age_ns = int(float(self.get_parameter('max_pose_age_sec').value)*1e9)
        self.publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(
            JointState, str(self.get_parameter('joint_states_topic').value),
            self.joints_callback, qos_profile_sensor_data)
        self.base_xyz = None
        self.base_R = None
        self.pose_stamp_ns = None
        self.joint_deg = None
        self.joint_stamp_ns = None
        self.timer = self.create_timer(1./rate, self.publish)
        self.get_logger().info(
            '[SELF FILTER VIZ]\n'
            '  status          : ready\n'
            '  topic           : %s\n'
            '  frame           : world\n'
            '  geometry        : body/rotors/skids/arm/saw\n'
            '  alpha           : 0.75\n'
            '  publish rate    : %.1f Hz' %
            (str(self.get_parameter('marker_topic').value), rate))

    def now_ns(self):
        return int(self.get_clock().now().nanoseconds)

    def odom_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        xyz = np.array([p.x, p.y, p.z], dtype=np.float64)
        if not np.all(np.isfinite(xyz)):
            return
        try:
            rot = quaternion_xyzw_to_rotation([q.x, q.y, q.z, q.w])
        except ValueError:
            return
        self.base_xyz = xyz
        self.base_R = rot
        self.pose_stamp_ns = self.now_ns()

    def joints_callback(self, msg):
        names = {name.split('::')[-1].split('/')[-1]: i for i, name in enumerate(msg.name)}
        if not all(k in names for k in ('j2', 'j3', 'j4')):
            return
        raw = np.array([msg.position[names[k]] for k in ('j2', 'j3', 'j4')], dtype=np.float64)
        if not np.all(np.isfinite(raw)):
            return
        # Matches environment_preprocessor.py and environment_geometry.py.
        self.joint_deg = np.rad2deg(raw) + np.array([-30., -150., -90.])
        self.joint_stamp_ns = self.now_ns()

    def publish(self):
        now = self.now_ns()
        if (self.base_R is None or self.pose_stamp_ns is None or
                now < self.pose_stamp_ns or now-self.pose_stamp_ns > self.pose_max_age_ns):
            return
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mgn = self.margin
        next_id = 0

        def add(kind, center, size, color, rotation=None, ns='self_mask'):
            nonlocal next_id
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = stamp
            marker.ns = ns
            marker.id = next_id
            next_id += 1
            marker.type = kind
            marker.action = Marker.ADD
            c = np.asarray(center, dtype=np.float64)
            R_local = np.eye(3) if rotation is None else np.asarray(rotation, dtype=np.float64)
            p_w = self.base_xyz + self.base_R @ c
            q = rotation_to_quaternion(self.base_R @ R_local)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, p_w)
            marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = map(float, q)
            marker.scale.x, marker.scale.y, marker.scale.z = map(float, size)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            # Fixed alpha requested for easy RViz2 inspection.
            marker.color.a = 0.75
            marker.lifetime = Duration(sec=1, nanosec=0)
            arr.markers.append(marker)

        def box(center, size, color, rotation=None):
            add(Marker.CUBE, center, np.asarray(size)+2*mgn, color, rotation)

        def cylinder(center, radius, height, color, rotation=None):
            add(Marker.CYLINDER, center,
                [2*(radius+mgn), 2*(radius+mgn), height+2*mgn], color, rotation)

        def ball(center, radius, color):
            add(Marker.SPHERE, center, [2*(radius+mgn)]*3, color)

        def capsule(p0, p1, radius, color):
            a, b = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
            v = b-a
            dist = np.linalg.norm(v)
            if dist > 1e-8:
                cylinder((a+b)*.5, radius, dist, color, align_z_to_vector(v))
            ball(a, radius, color)
            ball(b, radius, color)

        magenta = (.95, .10, .65, .22)
        orange = (1.0, .55, .05, .27)
        gold = (1.0, .78, .18, .28)
        green = (.1, .9, .3, .44)
        cyan = (.1, .9, 1.0, .35)
        box([0., 0., .007], [.3535534, .3535534, .050], magenta)
        for side in (-1., 1.):
            axis = rotation_x(side*.35) @ np.array([0., 0., 1.])
            center = np.array([0., side*.098, -.123])
            capsule(center-axis*.105, center+axis*.105, .011, gold)
            capsule([-.125, side*.132, -.2195], [.125, side*.132, -.2195], .011, gold)

        for x in (-.174, .174):
            for y in (-.174, .174):
                cylinder([x,y,.060], .279230769/2, .017, orange)
                cylinder([x,y,.032], .030, .060, gold)
        box([.120, 0., -.025], [.020, .040, .040], cyan, rotation_y(.436332))
        box(BASE_SENSOR_TRANSLATION, [.073, .065, .060], cyan, BASE_SENSOR_ROTATION)
        cylinder([0., 0., -.045], .055, .040, magenta)
        cylinder([0., 0., -.080], .038, .070, magenta)

        # The moving arm is only shown with fresh joint feedback. Otherwise,
        # markers expire after 1s rather than silently depicting the wrong pose.
        if (self.joint_deg is not None and self.joint_stamp_ns is not None and
                0 <= now-self.joint_stamp_ns <= self.pose_max_age_ns):
            j2, j3, j4, wrist, saw, saw_rot = arm_landmarks(self.joint_deg)
            capsule(j2, j3, .029, green)
            capsule(j3, j4, .027, green)
            capsule(j4, wrist, .030, green)
            for pivot, rad in ((j2,.038),(j3,.032),(j4,.030)):
                ball(pivot, rad, green)
            box(saw, [.175, .082, .028], green, saw_rot)

        self.publisher.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SelfFilterVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
