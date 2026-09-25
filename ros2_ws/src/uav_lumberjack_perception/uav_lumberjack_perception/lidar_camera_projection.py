import math
import time

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
    Step13.3.1 - Optimized presentation version

    Main optimization:
      1) Camera callback only stores the newest ROS Image message.
         It does NOT convert 30 FPS images continuously.
      2) Image conversion happens only when a new LiDAR cloud arrives
         (~10 Hz), so camera conversion workload is reduced from ~30 Hz
         to ~10 Hz.
      3) LiDAR -> Camera static transform is cached after the first lookup.
      4) PointCloud2 XYZ is read with a NumPy structured view.
      5) Presentation image is rendered at 0.75 scale (960x540 from
         1280x720) to reduce display / DDS / rqt bandwidth.
      6) Output uses sensor-data QoS to avoid old-frame backlog.

    Gazebo camera convention used here:
        +X forward
        +Y left
        +Z up

    Projection:
        u = cx - fx * Y / X
        v = cy - fy * Z / X

    cv_bridge is NOT used.
    """

    def __init__(self):
        super().__init__('lidar_camera_projection')

        # ========================================================
        # Presentation parameters
        # ========================================================

        # 1280x720 -> 960x540.
        # This affects presentation only, not the underlying 3D geometry.
        self.display_scale = 0.75

        # Fixed depth range gives stable colors between frames.
        self.display_depth_min = 0.5
        self.display_depth_max = 8.0

        # Small clean points for presentation.
        self.point_radius = 1

        # Drawing workload limit only.
        self.max_draw_points = 4000

        # ========================================================
        # Runtime state
        # ========================================================

        # Camera callback stores only the newest ROS message.
        self.latest_image_msg = None

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

        # Cached LiDAR -> Camera transform.
        # Both sensors are rigidly mounted to the UAV, so their relative
        # transform is static.
        self.cached_rotation = None
        self.cached_translation = None
        self.cached_lidar_frame = None

        self.frame_count = 0
        self.smoothed_fps = 0.0
        self.smoothed_process_ms = 0.0
        self.last_publish_time = None
        self.last_tf_warning_time = 0.0

        # ========================================================
        # ROS interfaces
        # ========================================================

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
            qos_profile_sensor_data
        )

        self.get_logger().info(
            '===================================================='
        )
        self.get_logger().info(
            ' Step13.3.1 RGB + LiDAR Projection [OPTIMIZED]'
        )
        self.get_logger().info(
            ' Camera callback: store latest frame only'
        )
        self.get_logger().info(
            f' Display scale : {self.display_scale:.2f}'
        )
        self.get_logger().info(
            ' Output: /perception/lidar_projection'
        )
        self.get_logger().info(
            ' cv_bridge is NOT used'
        )
        self.get_logger().info(
            '===================================================='
        )

    # ============================================================
    # ROS Image -> OpenCV BGR
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
    # Fast PointCloud2 XYZ reader
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

        endian = '>' if msg.is_bigendian else '<'

        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': [
                endian + 'f4',
                endian + 'f4',
                endian + 'f4'
            ],
            'offsets': [
                fields['x'].offset,
                fields['y'].offset,
                fields['z'].offset
            ],
            'itemsize': msg.point_step
        })

        cloud = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=dtype,
            buffer=memoryview(msg.data),
            strides=(msg.row_step, msg.point_step)
        )

        x = cloud['x'].reshape(-1)
        y = cloud['y'].reshape(-1)
        z = cloud['z'].reshape(-1)

        valid = (
            np.isfinite(x)
            & np.isfinite(y)
            & np.isfinite(z)
        )

        points = np.column_stack(
            (
                x[valid],
                y[valid],
                z[valid]
            )
        ).astype(
            np.float32,
            copy=False
        )

        return points

    # ============================================================
    # Quaternion -> rotation matrix
    # ============================================================

    @staticmethod
    def quaternion_to_matrix(q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        n = x*x + y*y + z*z + w*w

        if n < 1.0e-12:
            return np.eye(
                3,
                dtype=np.float32
            )

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
        ], dtype=np.float32)

    # ============================================================
    # Static TF cache
    # ============================================================

    def ensure_transform(self, lidar_frame):
        if (
            self.cached_rotation is not None
            and self.cached_translation is not None
            and self.cached_lidar_frame == lidar_frame
        ):
            return True

        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                lidar_frame,
                Time()
            )

        except TransformException as exc:
            now = time.perf_counter()

            if now - self.last_tf_warning_time > 2.0:
                self.last_tf_warning_time = now

                self.get_logger().warning(
                    f'TF not ready: {exc}'
                )

            return False

        t = transform.transform.translation
        q = transform.transform.rotation

        self.cached_translation = np.array(
            [t.x, t.y, t.z],
            dtype=np.float32
        )

        self.cached_rotation = self.quaternion_to_matrix(
            q
        )

        self.cached_lidar_frame = lidar_frame

        self.get_logger().info(
            '[OK] Cached static LiDAR -> Camera transform'
        )

        return True

    # ============================================================
    # Visualization helpers
    # ============================================================

    def depth_to_colors(self, depth):
        clipped = np.clip(
            depth,
            self.display_depth_min,
            self.display_depth_max
        )

        normalized = (
            clipped - self.display_depth_min
        ) / (
            self.display_depth_max
            - self.display_depth_min
        )

        # Near = warm, far = cool.
        color_index = (
            255.0 * (1.0 - normalized)
        ).astype(
            np.uint8
        )

        color_image = cv2.applyColorMap(
            color_index.reshape(-1, 1),
            cv2.COLORMAP_TURBO
        )

        return color_image.reshape(
            -1,
            3
        )

    def draw_points_vectorized(
        self,
        image,
        u,
        v,
        colors
    ):
        height, width = image.shape[:2]

        u = u.astype(np.int32)
        v = v.astype(np.int32)

        radius = max(
            0,
            int(self.point_radius)
        )

        for dy in range(-radius, radius + 1):
            yy = v + dy

            valid_y = (
                (yy >= 0)
                & (yy < height)
            )

            if not np.any(valid_y):
                continue

            for dx in range(-radius, radius + 1):
                if dx*dx + dy*dy > radius*radius:
                    continue

                xx = u + dx

                valid = (
                    valid_y
                    & (xx >= 0)
                    & (xx < width)
                )

                if not np.any(valid):
                    continue

                image[
                    yy[valid],
                    xx[valid]
                ] = colors[valid]

    def draw_info_panel(
        self,
        image,
        cloud_count,
        projected_count,
        drawn_count
    ):
        overlay = image.copy()

        x1, y1 = 14, 14
        x2, y2 = 355, 126

        cv2.rectangle(
            overlay,
            (x1, y1),
            (x2, y2),
            (20, 20, 20),
            -1
        )

        cv2.addWeighted(
            overlay,
            0.55,
            image,
            0.45,
            0,
            image
        )

        cv2.putText(
            image,
            'RGB + MID360 DEPTH PROJECTION',
            (27, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.53,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            image,
            f'FPS       : {self.smoothed_fps:4.1f}',
            (27, 67),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (235, 235, 235),
            1,
            cv2.LINE_AA
        )

        cv2.putText(
            image,
            f'Process   : {self.smoothed_process_ms:4.1f} ms',
            (27, 89),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (235, 235, 235),
            1,
            cv2.LINE_AA
        )

        cv2.putText(
            image,
            f'Cloud/Image: {cloud_count}/{projected_count}',
            (27, 111),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (235, 235, 235),
            1,
            cv2.LINE_AA
        )

        cv2.circle(
            image,
            (334, 37),
            6,
            (0, 220, 0),
            -1,
            cv2.LINE_AA
        )

    def draw_depth_legend(self, image):
        height, width = image.shape[:2]

        bar_h = min(
            250,
            height - 95
        )

        bar_w = 14

        x1 = width - 44
        y1 = 48
        x2 = x1 + bar_w
        y2 = y1 + bar_h

        indices = np.linspace(
            255,
            0,
            bar_h,
            dtype=np.uint8
        ).reshape(
            bar_h,
            1
        )

        colorbar = cv2.applyColorMap(
            indices,
            cv2.COLORMAP_TURBO
        )

        colorbar = np.repeat(
            colorbar,
            bar_w,
            axis=1
        )

        image[
            y1:y2,
            x1:x2
        ] = colorbar

        cv2.rectangle(
            image,
            (x1 - 1, y1 - 1),
            (x2, y2),
            (255, 255, 255),
            1
        )

        cv2.putText(
            image,
            'DEPTH',
            (width - 70, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

        cv2.putText(
            image,
            f'{self.display_depth_min:.1f}m',
            (width - 82, y1 + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

        cv2.putText(
            image,
            f'{self.display_depth_max:.1f}m',
            (width - 82, y2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

    # ============================================================
    # ROS callbacks
    # ============================================================

    def image_callback(self, msg):
        # IMPORTANT:
        # Do not convert the 30 Hz camera stream here.
        # Just keep the newest message.
        self.latest_image_msg = msg

    def camera_info_callback(self, msg):
        if self.camera_frame is not None:
            return

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.camera_frame = msg.header.frame_id

        self.get_logger().info(
            '[OK] CameraInfo: '
            f'fx={self.fx:.3f}, '
            f'fy={self.fy:.3f}, '
            f'cx={self.cx:.3f}, '
            f'cy={self.cy:.3f}, '
            f'frame="{self.camera_frame}"'
        )

    def cloud_callback(self, msg):
        start_time = time.perf_counter()

        if (
            self.latest_image_msg is None
            or self.camera_frame is None
        ):
            return

        if not self.ensure_transform(
            msg.header.frame_id
        ):
            return

        # --------------------------------------------------------
        # Convert only ONE camera frame per LiDAR frame
        # --------------------------------------------------------

        image_msg = self.latest_image_msg

        try:
            bgr = self.ros_image_to_bgr(
                image_msg
            )

        except Exception as exc:
            self.get_logger().error(
                f'Image conversion failed: {exc}'
            )
            return

        # Presentation resolution only.
        if self.display_scale != 1.0:
            display = cv2.resize(
                bgr,
                None,
                fx=self.display_scale,
                fy=self.display_scale,
                interpolation=cv2.INTER_AREA
            )
        else:
            display = bgr.copy()

        # --------------------------------------------------------
        # PointCloud2 -> XYZ
        # --------------------------------------------------------

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

        # --------------------------------------------------------
        # LiDAR frame -> Camera frame
        # --------------------------------------------------------

        points_camera = (
            points_lidar
            @ self.cached_rotation.T
            + self.cached_translation
        )

        # --------------------------------------------------------
        # Camera 3D -> original 1280x720 image coordinates
        # --------------------------------------------------------

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

        original_height = image_msg.height
        original_width = image_msg.width

        inside = (
            (u >= 0)
            & (u < original_width)
            & (v >= 0)
            & (v < original_height)
        )

        u = u[inside]
        v = v[inside]
        depth = depth[inside]

        projected_count = int(
            u.size
        )

        if projected_count == 0:
            return

        # --------------------------------------------------------
        # Presentation scaling
        # --------------------------------------------------------

        u = u * self.display_scale
        v = v * self.display_scale

        if projected_count > self.max_draw_points:
            stride = int(
                math.ceil(
                    projected_count
                    / self.max_draw_points
                )
            )
        else:
            stride = 1

        u_draw = u[::stride]
        v_draw = v[::stride]
        depth_draw = depth[::stride]

        colors = self.depth_to_colors(
            depth_draw
        )

        self.draw_points_vectorized(
            display,
            u_draw,
            v_draw,
            colors
        )

        # --------------------------------------------------------
        # Performance statistics
        # --------------------------------------------------------

        now = time.perf_counter()

        if self.last_publish_time is not None:
            dt = now - self.last_publish_time

            if dt > 1.0e-6:
                instant_fps = 1.0 / dt

                if self.smoothed_fps <= 0.0:
                    self.smoothed_fps = instant_fps
                else:
                    self.smoothed_fps = (
                        0.80 * self.smoothed_fps
                        + 0.20 * instant_fps
                    )

        self.last_publish_time = now

        process_ms = (
            time.perf_counter()
            - start_time
        ) * 1000.0

        if self.smoothed_process_ms <= 0.0:
            self.smoothed_process_ms = process_ms
        else:
            self.smoothed_process_ms = (
                0.80 * self.smoothed_process_ms
                + 0.20 * process_ms
            )

        # --------------------------------------------------------
        # Presentation overlay
        # --------------------------------------------------------

        self.draw_info_panel(
            display,
            int(points_lidar.shape[0]),
            projected_count,
            int(u_draw.size)
        )

        self.draw_depth_legend(
            display
        )

        height, _ = display.shape[:2]

        cv2.putText(
            display,
            'Camera semantics  +  LiDAR geometry  ->  2D-3D correspondence',
            (18, height - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

        self.projection_pub.publish(
            self.numpy_to_ros_bgr8(
                display,
                image_msg.header
            )
        )

        self.frame_count += 1

        if self.frame_count % 20 == 0:
            self.get_logger().info(
                '[PROJECTION] '
                f'cloud={points_lidar.shape[0]} pts, '
                f'in_image={projected_count} pts, '
                f'drawn={u_draw.size} pts, '
                f'fps={self.smoothed_fps:.1f}, '
                f'process={self.smoothed_process_ms:.1f} ms'
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
