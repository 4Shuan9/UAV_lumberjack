"""Stage 2: SDF-based self filtering + lightweight PMF-style terrain removal.

The original /mid360/points and Agriculture/Offboard perception path are never
modified. Output clouds are transformed directly into Gazebo world frame and
retain the MID360 source timestamp. A fresh Gazebo odometry pose is required
before any world-frame output; no RViz TF lookup is needed for these topics.
"""

import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Header

from .terrain_ground_filter import TerrainConfig, classify_ground

from .environment_geometry import (
    EXPECTED_SENSOR_FRAME,
    quaternion_xyzw_to_rotation,
    self_filter_mask,
    sensor_to_base,
)


class EnvironmentPreprocessor(Node):
    def __init__(self):
        super().__init__('environment_preprocessor')
        self.declare_parameter('points_topic', '/mid360/points')
        self.declare_parameter('odom_topic', '/model/x500_lumberjack/base_link_odometry')
        self.declare_parameter('joint_states_topic', '/lumberjack_arm/joint_states')
        self.declare_parameter('sensor_frame', EXPECTED_SENSOR_FRAME)
        self.declare_parameter('self_filter_margin_m', .008)
        self.declare_parameter('require_joint_states', True)
        self.declare_parameter('joint_states_max_age_sec', .50)
        self.declare_parameter('odom_max_age_sec', .40)
        self.declare_parameter('ground_enabled', True)
        # Terrain is not globally planar; estimate local low-height patches.
        self.declare_parameter('ground_cell_size_m', 0.65)
        self.declare_parameter('ground_height_tolerance_m', 0.12)
        self.declare_parameter('ground_max_slope_deg', 35.0)
        self.declare_parameter('ground_max_reference_distance_m', 0.95)
        self.declare_parameter('ground_min_grid_cells', 12)
        self.declare_parameter('ground_min_ground_points', 50)
        self.declare_parameter('ground_opening_slope', 0.22)
        self.declare_parameter('ground_max_seed_rise_m', 0.36)
        self.declare_parameter('ground_seed_link_radius_cells', 2.6)
        self.declare_parameter('ground_seed_max_step_rise_m', 0.42)
        self.declare_parameter('ground_anchor_quantile', 0.15)
        self.declare_parameter('ground_max_world_z_m', 999.0)
        self.declare_parameter('ground_auto_height_quantile', 0.25)
        self.declare_parameter('ground_auto_height_margin_m', 1.10)
        self.declare_parameter('log_period_sec', 20.0)
        # Match MID360 scans; no artificial 3 Hz limit.
        self.declare_parameter('publish_rate_hz', 10.0)
        # Preserve the known, raised launch platform as an obstacle.
        self.declare_parameter('platform_center_xy', [-37.5, 15.0])
        self.declare_parameter('platform_half_size_xy', [2.05, 2.05])

        def param(k):
            return self.get_parameter(k).value

        self.expected_frame = str(param('sensor_frame'))
        self.margin = float(param('self_filter_margin_m'))
        self.require_joints = bool(param('require_joint_states'))
        self.joint_age = float(param('joint_states_max_age_sec'))
        self.odom_age = float(param('odom_max_age_sec'))
        self.ground_enabled = bool(param('ground_enabled'))
        self.ground_cfg = TerrainConfig(
            cell_size_m=float(param('ground_cell_size_m')),
            height_tolerance_m=float(param('ground_height_tolerance_m')),
            max_slope_deg=float(param('ground_max_slope_deg')),
            max_reference_distance_m=float(param('ground_max_reference_distance_m')),
            min_grid_cells=int(param('ground_min_grid_cells')),
            min_ground_points=int(param('ground_min_ground_points')),
            opening_slope=float(param('ground_opening_slope')),
            seed_link_radius_cells=float(param('ground_seed_link_radius_cells')),
            seed_max_step_rise_m=float(param('ground_seed_max_step_rise_m')),
            anchor_quantile=float(param('ground_anchor_quantile')),
            max_ground_world_z_m=float(param('ground_max_world_z_m')),
            max_seed_rise_m=float(param('ground_max_seed_rise_m')),
            auto_height_quantile=float(param('ground_auto_height_quantile')),
            auto_height_margin_m=float(param('ground_auto_height_margin_m')),
        )
        self.log_period_ns = int(float(param('log_period_sec')) * 1e9)
        self.publish_rate_hz = float(param('publish_rate_hz'))
        self.last_processing_ms = 0.0
        self.published_since_log = 0
        self.platform_center = np.array(param('platform_center_xy'), dtype=np.float64)
        self.platform_half_size = np.array(param('platform_half_size_xy'), dtype=np.float64)
        if not (0.1 <= self.publish_rate_hz <= 30.0):
            raise ValueError('publish_rate_hz must be within [0.1, 30]')
        if not (0.15 <= self.ground_cfg.cell_size_m <= 2.0
                and 0.02 <= self.ground_cfg.height_tolerance_m <= 0.40
                and 2.0 <= self.ground_cfg.max_slope_deg <= 60.0
                and self.ground_cfg.max_reference_distance_m > 0.0):
            raise ValueError('Invalid terrain filter geometry')
        if not (1.0 <= self.ground_cfg.seed_link_radius_cells <= 5.0
                and 0.05 <= self.ground_cfg.seed_max_step_rise_m <= 1.5
                and 0.01 <= self.ground_cfg.anchor_quantile <= .4
                and np.isfinite(self.ground_cfg.max_ground_world_z_m)
                and 0.05 <= self.ground_cfg.auto_height_quantile <= 0.50
                and 0.15 <= self.ground_cfg.auto_height_margin_m <= 5.0):
            raise ValueError('Invalid terrain surface anchoring parameters')
        self.last_cloud_process_ns = None
        self.latest_joint_deg = None
        self.joint_rx_ns = None
        self.latest_base_xyz = None
        self.latest_base_rotation = None
        self.odom_rx_ns = None
        self.last_log_ns = None
        self.log_counts = [0, 0, 0, 0]

        if not (0. <= self.margin <= .05):
            raise ValueError('self_filter_margin_m must be within [0, 0.05]')
        if (self.platform_center.shape != (2,)
                or self.platform_half_size.shape != (2,)):
            raise ValueError('Invalid platform footprint')

        self.pub_self = self.create_publisher(
            PointCloud2, '/perception/lidar/points_self_filtered', qos_profile_sensor_data)
        self.pub_ground = self.create_publisher(
            PointCloud2, '/perception/lidar/points_ground', qos_profile_sensor_data)
        self.pub_nonground = self.create_publisher(
            PointCloud2, '/perception/lidar/points_nonground', qos_profile_sensor_data)

        self.create_subscription(JointState, str(param('joint_states_topic')),
                                 self.joints_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(param('odom_topic')),
                                 self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, str(param('points_topic')),
                                 self.cloud_callback, qos_profile_sensor_data)
        self.get_logger().info(
            '[ENVIRONMENT]\n'
            '  status          : ready\n'
            '  backend         : anchored PMF terrain grid\n'
            '  self filter     : SDF + live J2/J3/J4\n'
            '  input           : %s\n'
            '  ground output   : /perception/lidar/points_ground\n'
            '  nonground output: /perception/lidar/points_nonground\n'
            '  frame           : world\n'
            '  max rate        : %.1f Hz\n'
            '  terrain cell    : %.2f m\n'
            '  height candidate: q%.0f + %.2f m\n'
            '  platform guard  : ON' % (
                str(param('points_topic')), self.publish_rate_hz,
                self.ground_cfg.cell_size_m,
                100 * self.ground_cfg.auto_height_quantile,
                self.ground_cfg.auto_height_margin_m))

    def now_ns(self):
        return int(self.get_clock().now().nanoseconds)

    def joints_callback(self, msg):
        names = {name.split('::')[-1].split('/')[-1]: i
                 for i, name in enumerate(msg.name)}
        if not all(k in names for k in ('j2', 'j3', 'j4')):
            self.get_logger().warn('[ENVIRONMENT] JointState missing j2/j3/j4',
                                   throttle_duration_sec=5.0)
            return
        raw = np.array([msg.position[names[k]] for k in ('j2', 'j3', 'j4')],
                       dtype=np.float64)
        if not np.all(np.isfinite(raw)):
            return
        # Gazebo raw zero = baked HOME. Logical = raw degrees + HOME offsets.
        self.latest_joint_deg = np.rad2deg(raw) + np.array([-30., -150., -90.])
        self.joint_rx_ns = self.now_ns()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        xyz = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
        if not np.all(np.isfinite(xyz)):
            return
        try:
            rotation = quaternion_xyzw_to_rotation([q.x, q.y, q.z, q.w])
        except ValueError:
            return
        self.latest_base_xyz = xyz
        self.latest_base_rotation = rotation
        self.odom_rx_ns = self.now_ns()

    @staticmethod
    def pointcloud_xyz(msg):
        """Read arbitrary point_step/row_step organized PointCloud2 XYZ FLOAT32."""
        f = {field.name: field for field in msg.fields}
        if any(k not in f or f[k].datatype != PointField.FLOAT32
               for k in ('x', 'y', 'z')):
            raise ValueError('PointCloud2 must provide FLOAT32 x/y/z')
        if msg.width == 0 or msg.height == 0 or not msg.data:
            return np.empty((0, 3), dtype=np.float32)
        endian = '>' if msg.is_bigendian else '<'
        dt = np.dtype({'names': ['x', 'y', 'z'],
                       'formats': [endian+'f4']*3,
                       'offsets': [f[k].offset for k in ('x', 'y', 'z')],
                       'itemsize': msg.point_step})
        arr = np.ndarray(shape=(msg.height, msg.width), dtype=dt,
                         buffer=memoryview(msg.data),
                         strides=(msg.row_step, msg.point_step))
        xyz = np.column_stack([arr[k].reshape(-1) for k in ('x', 'y', 'z')])
        return xyz[np.isfinite(xyz).all(axis=1)].astype(np.float32, copy=False)

    @staticmethod
    def make_cloud(points, header):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(len(points))
        msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32,
                                 count=1) for i, n in enumerate(('x', 'y', 'z'))]
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12
        msg.row_step = msg.width * 12
        msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
        return msg

    def fresh_world_pose(self, now_ns):
        """Never label LiDAR-frame coordinates as world without a valid pose."""
        return (self.latest_base_rotation is not None
                and self.latest_base_xyz is not None
                and self.odom_rx_ns is not None
                and 0 <= now_ns - self.odom_rx_ns <= self.odom_age * 1e9)

    def _platform_mask(self, pts):
        # Ground filtering must not erase the raised takeoff platform.
        return np.all(np.abs(pts[:, :2] - self.platform_center)
                      <= self.platform_half_size, axis=1)

    def _ground_mask(self, points_world):
        if not self.ground_enabled:
            return np.zeros(len(points_world), dtype=bool), 'disabled'
        protected = self._platform_mask(points_world)
        return classify_ground(points_world, protected, self.ground_cfg)

    def cloud_callback(self, msg):
        if msg.header.frame_id != self.expected_frame:
            self.get_logger().error(
                '[ENVIRONMENT] frame mismatch: ' + msg.header.frame_id +
                ' expected ' + self.expected_frame,
                throttle_duration_sec=5.0)
            return
        started = time.perf_counter()
        now_ns = self.now_ns()
        min_period_ns = int(1e9/self.publish_rate_hz)
        if (self.last_cloud_process_ns is not None and
                0 <= now_ns - self.last_cloud_process_ns < min_period_ns):
            return
        if self.require_joints and (self.latest_joint_deg is None or
                self.joint_rx_ns is None or now_ns - self.joint_rx_ns >
                self.joint_age * 1e9 or now_ns < self.joint_rx_ns):
            self.get_logger().warn(
                '[ENVIRONMENT] waiting for fresh /lumberjack_arm/joint_states; '
                'refusing incomplete self-filtered output',
                throttle_duration_sec=5.0)
            return
        if not self.fresh_world_pose(now_ns):
            self.get_logger().warn(
                '[ENVIRONMENT] waiting for fresh Gazebo world odometry; '
                'refusing invalid world cloud', throttle_duration_sec=5.0)
            return
        try:
            raw = self.pointcloud_xyz(msg)
            base = sensor_to_base(raw)
            remove = self_filter_mask(base, self.latest_joint_deg, self.margin)
            kept_base = base[~remove]
            world = (kept_base @ self.latest_base_rotation.T
                     + self.latest_base_xyz)
            ground, ground_status = self._ground_mask(world)
            # Source LiDAR timestamp is retained, but XYZ and frame_id are
            # consistent: no downstream TF required for these three topics.
            out_header = Header(stamp=msg.header.stamp, frame_id='world')
            self.pub_self.publish(self.make_cloud(world, out_header))
            self.pub_ground.publish(self.make_cloud(world[ground], out_header))
            self.pub_nonground.publish(self.make_cloud(world[~ground], out_header))
            self.last_cloud_process_ns = now_ns
            self.published_since_log += 1
            self.last_processing_ms = 1e3*(time.perf_counter()-started)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error('[ENVIRONMENT] processing error: ' + str(exc),
                                    throttle_duration_sec=3.0)
            return

        if self.last_log_ns is None or now_ns < self.last_log_ns or (
                now_ns - self.last_log_ns >= self.log_period_ns):
            elapsed = (now_ns-self.last_log_ns)/1e9 if self.last_log_ns is not None and now_ns>=self.last_log_ns else 0
            pub_rate = self.published_since_log/elapsed if elapsed>0 else 0.
            self.last_log_ns = now_ns
            self.published_since_log = 0
            self.get_logger().info(
                '[ENVIRONMENT] input=%d self=%d ground=%d nonground=%d '
                '| status=%s | %.1f Hz | %.1f ms' %
                (len(raw), int(remove.sum()), int(ground.sum()),
                 len(kept_base) - int(ground.sum()), ground_status,
                 pub_rate, self.last_processing_ms))


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentPreprocessor()
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
