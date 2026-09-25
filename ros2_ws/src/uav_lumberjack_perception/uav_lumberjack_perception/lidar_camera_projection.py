import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException


class LidarCameraProjection(Node):
    """
    Step13.3.1
    将 MID360 PointCloud2 投影到 Gazebo RGB Camera 图像上。

    注意：
    - 不使用 cv_bridge；
    - 直接使用 NumPy 解析 sensor_msgs/Image 和 PointCloud2；
    - 当前 Gazebo Camera 采用 +X forward, +Y left, +Z up；
      因此像素投影为：
          u = cx - fx * Y / X
          v = cy - fy * Z / X
    """

    def __init__(self):
        super().__init__('lidar_camera_projection')

        self.latest_bgr = None
        self.latest_header = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/mid360/points',
            self.cloud_callback,
            qos_profile_sensor_data
        )

        self.projection_pub = self.create_publisher(
            Image,
            '/perception/lidar_projection',
            10
        )

        self.info_received = False
        self.frame_count = 0

        # 仅用于可视化，限制每帧最多绘制的点数，避免 rqt 太卡。
        self.max_draw_points = 5000

        self.get_logger().info(
            '=============================================='
        )
        self.get_logger().info(
            ' Step13.3.1 LiDAR -> Camera projection started'
        )
        self.get_logger().info(
            ' Input : /camera/image_raw'
        )
        self.get_logger().info(
            ' Input : /camera/camera_info'
        )
        self.get_logger().info(
            ' Input : /mid360/points'
        )
        self.get_logger().info(
            ' Output: /perception/lidar_projection'
        )
        self.get_logger().info(
            ' cv_bridge is NOT used'
        )
        self.get_logger().info(
            '=============================================='
        )

    # ============================================================
    # ROS Image <-> NumPy
    # ============================================================

    def ros_image_to_bgr(self, msg):
        encoding = msg.encoding.lower()

        if encoding in ['rgb8', 'bgr8']:
            channels = 3
        elif encoding in ['rgba8', 'bgra8']:
            channels = 4
        elif encoding == 'mono8':
            channels = 1
        else:
            raise ValueError(
                f'Unsupported image encoding: {msg.encoding}'
            )

        raw = np.frombuffer(
            msg.data,
            dtype=np.uint8
        )

        expected_size = msg.height * msg.step

        if raw.size < expected_size:
            raise ValueError(
                f'Image data too small: '
                f'{raw.size} < {expected_size}'
            )

        rows = raw[:expected_size].reshape(
            msg.height,
            msg.step
        )

        useful_bytes = msg.width * channels
        image = rows[:, :useful_bytes]

        if channels == 1:
            image = image.reshape(
                msg.height,
                msg.width
            )
            return cv2.cvtColor(
                image,
                cv2.COLOR_GRAY2BGR
            )

        image = image.reshape(
            msg.height,
            msg.width,
            channels
        )

        if encoding == 'rgb8':
            return cv2.cvtColor(
                image,
                cv2.COLOR_RGB2BGR
            )

        if encoding == 'bgr8':
            return image.copy()

        if encoding == 'rgba8':
            return cv2.cvtColor(
                image,
                cv2.COLOR_RGBA2BGR
            )

        return cv2.cvtColor(
            image,
            cv2.COLOR_BGRA2BGR
        )

    def numpy_to_ros_bgr8(self, image, header):
        msg = Image()
        msg.header = header
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = image.tobytes()
        return msg

    # ============================================================
    # PointCloud2 parser
    # ============================================================

    def pointcloud_xyz(self, msg):
        fields = {
            field.name: field
            for field in msg.fields
        }

        for name in ('x', 'y', 'z'):
            if name not in fields:
                raise ValueError(
                    f'PointCloud2 has no "{name}" field'
                )

            # sensor_msgs/PointField.FLOAT32 == 7
            if fields[name].datatype != 7:
                raise ValueError(
                    f'PointCloud2 field "{name}" '
                    f'is not FLOAT32'
                )

        point_bytes = msg.width * msg.point_step

        raw = np.frombuffer(
            msg.data,
            dtype=np.uint8
        )

        # 兼容可能存在的 row padding
        if msg.row_step == point_bytes:
            packed = raw.reshape(
                msg.height * msg.width,
                msg.point_step
            )
        else:
            rows = []

            for row in range(msg.height):
                start = row * msg.row_step
                stop = start + point_bytes

                row_data = raw[start:stop].reshape(
                    msg.width,
                    msg.point_step
                )

                rows.append(row_data)

            packed = np.vstack(rows)

        float_dtype = (
            '>f4'
            if msg.is_bigendian
            else '<f4'
        )

        def read_float32(offset):
            values = packed[
                :,
                offset:offset + 4
            ].copy()

            return np.frombuffer(
                values.tobytes(),
                dtype=float_dtype
            )

        x = read_float32(fields['x'].offset)
        y = read_float32(fields['y'].offset)
        z = read_float32(fields['z'].offset)

        points = np.column_stack(
            (x, y, z)
        ).astype(
            np.float64,
            copy=False
        )

        valid = np.isfinite(points).all(axis=1)

        return points[valid]

    # ============================================================
    # TF quaternion -> rotation matrix
    # ============================================================

    @staticmethod
    def quaternion_to_matrix(q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        n = x*x + y*y + z*z + w*w

        if n < 1.0e-12:
            return np.eye(3)

        s = 2.0 / n

        xx = x * x * s
        yy = y * y * s
        zz = z * z * s
        xy = x * y * s
        xz = x * z * s
        yz = y * z * s
        wx = w * x * s
        wy = w * y * s
        wz = w * z * s

        return np.array([
            [1.0 - yy - zz, xy - wz,       xz + wy],
            [xy + wz,       1.0 - xx - zz, yz - wx],
            [xz - wy,       yz + wx,       1.0 - xx - yy]
        ], dtype=np.float64)

    # ============================================================
    # Callbacks
    # ============================================================

    def image_callback(self, msg):
        try:
            self.latest_bgr = self.ros_image_to_bgr(
                msg
            )
            self.latest_header = msg.header

        except Exception as exc:
            self.get_logger().error(
                f'Image conversion failed: {exc}'
            )

    def camera_info_callback(self, msg):
        if self.info_received:
            return

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.camera_frame = msg.header.frame_id

        self.info_received = True

        self.get_logger().info(
            '[OK] CameraInfo: '
            f'fx={self.fx:.3f}, '
            f'fy={self.fy:.3f}, '
            f'cx={self.cx:.3f}, '
            f'cy={self.cy:.3f}, '
            f'frame="{self.camera_frame}"'
        )

    def cloud_callback(self, msg):
        if (
            self.latest_bgr is None
            or not self.info_received
        ):
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                msg.header.frame_id,
                Time()
            )

        except TransformException as exc:
            self.get_logger().warning(
                f'TF not ready: {exc}',
                throttle_duration_sec=2.0
            )
            return

        try:
            points_lidar = self.pointcloud_xyz(
                msg
            )

        except Exception as exc:
            self.get_logger().error(
                f'PointCloud2 parse failed: {exc}'
            )
            return

        if points_lidar.shape[0] == 0:
            return

        t = transform.transform.translation
        q = transform.transform.rotation

        translation = np.array(
            [t.x, t.y, t.z],
            dtype=np.float64
        )

        rotation = self.quaternion_to_matrix(
            q
        )

        # TF 返回的是：
        # target(camera) <- source(lidar)
        #
        # p_camera = R * p_lidar + t
        points_camera = (
            points_lidar @ rotation.T
            + translation
        )

        # Gazebo camera sensor frame:
        #
        # +X forward
        # +Y left
        # +Z up
        #
        # Image:
        # +u right
        # +v down
        #
        # 因此：
        # u = cx - fx * Y/X
        # v = cy - fy * Z/X

        depth = points_camera[:, 0]

        valid_front = depth > 0.10

        points_camera = points_camera[
            valid_front
        ]

        depth = depth[
            valid_front
        ]

        if points_camera.shape[0] == 0:
            return

        u = (
            self.cx
            - self.fx
            * points_camera[:, 1]
            / depth
        )

        v = (
            self.cy
            - self.fy
            * points_camera[:, 2]
            / depth
        )

        height, width = (
            self.latest_bgr.shape[:2]
        )

        inside = (
            (u >= 0)
            & (u < width)
            & (v >= 0)
            & (v < height)
        )

        u = u[inside]
        v = v[inside]
        depth = depth[inside]

        if u.size == 0:
            return

        # 限制调试图上的点数，避免显示卡顿
        stride = max(
            1,
            int(
                math.ceil(
                    u.size
                    / self.max_draw_points
                )
            )
        )

        u_draw = u[::stride].astype(
            np.int32
        )

        v_draw = v[::stride].astype(
            np.int32
        )

        depth_draw = depth[::stride]

        debug = self.latest_bgr.copy()

        # 用深度简单区分近远：
        # 近处偏红，远处偏蓝。
        min_depth = float(
            np.min(depth_draw)
        )

        max_depth = float(
            np.max(depth_draw)
        )

        span = max(
            max_depth - min_depth,
            1.0e-6
        )

        normalized = (
            (depth_draw - min_depth)
            / span
        )

        for px, py, norm_d in zip(
            u_draw,
            v_draw,
            normalized
        ):
            red = int(
                255 * (1.0 - norm_d)
            )

            blue = int(
                255 * norm_d
            )

            cv2.circle(
                debug,
                (int(px), int(py)),
                2,
                (blue, 255, red),
                -1
            )

        self.projection_pub.publish(
            self.numpy_to_ros_bgr8(
                debug,
                self.latest_header
            )
        )

        self.frame_count += 1

        if self.frame_count % 20 == 0:
            self.get_logger().info(
                '[PROJECTION] '
                f'cloud={points_lidar.shape[0]} pts, '
                f'in_image={u.size} pts, '
                f'drawn={u_draw.size} pts'
            )


def main(args=None):
    rclpy.init(args=args)

    node = LidarCameraProjection()

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
