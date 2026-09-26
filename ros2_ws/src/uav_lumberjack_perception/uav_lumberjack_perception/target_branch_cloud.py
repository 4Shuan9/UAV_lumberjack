import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException


class TargetBranchCloud(Node):
    """
    Step13.3.2 + 13.3.3 + 13.3.4

    RGB red Mask + LiDAR projection
        -> candidate 3D points
        -> depth-consistency filtering
        -> TRACKING grace / LOST / REACQUIRING state machine
        -> stable target branch 3D cloud

    Core fusion rule:
        M(v_i, u_i) > 0  -> keep corresponding LiDAR 3D point as candidate

    Output:
        /perception/target_branch_cloud
        frame_id = base_link

    cv_bridge is NOT used.
    """

    TRACKING = 'TRACKING'
    LOST = 'LOST'
    REACQUIRING = 'REACQUIRING'

    def __init__(self):
        super().__init__('target_branch_cloud')

        self.target_frame = 'base_link'
        self.min_camera_depth = 0.10
        self.mask_dilate_px = 0

        # Step13.3.3: depth clustering
        self.depth_cluster_gap = 0.20
        self.min_depth_cluster_points = 5
        self.min_target_depth = 0.20
        self.max_target_depth = 5.00
        self.max_cluster_span = 0.60

        # Step13.3.4: temporal continuity
        self.max_depth_jump = 0.60
        self.max_centroid_jump = 0.75

        # Step13.3.4.1: tracking-loss hysteresis.
        # Do not declare LOST because of only one or two bad frames.
        # MID360 is ~10 Hz, so 3 consecutive misses are about 0.3 s.
        self.tracking_missing_required_frames = 3
        self.tracking_missing_count = 0

        # Lost target must be consistently seen for 3 frames before reuse.
        self.reacquire_required_frames = 3
        self.reacquire_max_depth_jump = 0.60
        self.reacquire_max_centroid_jump = 0.75

        self.state = self.LOST

        self.last_valid_depth = None
        self.last_valid_centroid = None

        self.reacquire_count = 0
        self.reacquire_depth = None
        self.reacquire_centroid = None

        self.latest_mask_msg = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.image_width = None
        self.image_height = None
        self.camera_frame = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cam_R = None
        self.cam_t = None
        self.cam_lidar_frame = None

        self.base_R = None
        self.base_t = None
        self.base_lidar_frame = None

        self.last_tf_warning_time = 0.0

        self.frame_count = 0
        self.smoothed_fps = 0.0
        self.last_publish_time = None

        self.mask_sub = self.create_subscription(
            Image,
            '/perception/red_mask',
            self.mask_callback,
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

        self.target_cloud_pub = self.create_publisher(
            PointCloud2,
            '/perception/target_branch_cloud',
            qos_profile_sensor_data
        )

        self.get_logger().info('====================================================')
        self.get_logger().info(' Step13.3.4 Target Tracking State Machine')
        self.get_logger().info(' States: LOST -> REACQUIRING -> TRACKING')
        self.get_logger().info(
            f' Tracking loss confirmation: '
            f'{self.tracking_missing_required_frames} consecutive missing frames'
        )
        self.get_logger().info(
            f' Reacquire confirmation: '
            f'{self.reacquire_required_frames} consecutive frames'
        )
        self.get_logger().info(
            f' Valid target depth: '
            f'{self.min_target_depth:.2f} ~ {self.max_target_depth:.2f} m'
        )
        self.get_logger().info(' Output: /perception/target_branch_cloud')
        self.get_logger().info('====================================================')

    # ============================================================
    # State helpers
    # ============================================================

    def set_state(self, new_state, reason=''):
        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state

        if reason:
            self.get_logger().info(
                f'[STATE] {old_state} -> {new_state}, reason={reason}'
            )
        else:
            self.get_logger().info(
                f'[STATE] {old_state} -> {new_state}'
            )

    def reset_tracking_missing(self):
        self.tracking_missing_count = 0

    def reset_reacquire(self):
        self.reacquire_count = 0
        self.reacquire_depth = None
        self.reacquire_centroid = None

    def clear_tracking_history(self):
        self.last_valid_depth = None
        self.last_valid_centroid = None

    # ============================================================
    # Mask
    # ============================================================

    def mask_to_numpy(self, msg):
        if msg.encoding.lower() != 'mono8':
            raise ValueError(
                f'Expected mono8 mask, got "{msg.encoding}"'
            )

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = msg.height * msg.step

        if raw.size < expected_size:
            raise ValueError(
                f'Mask data too small: {raw.size} < {expected_size}'
            )

        rows = raw[:expected_size].reshape(
            msg.height,
            msg.step
        )

        mask = rows[:, :msg.width]

        if self.mask_dilate_px > 0:
            radius = int(self.mask_dilate_px)
            size = 2 * radius + 1
            kernel = np.ones(
                (size, size),
                dtype=np.uint8
            )
            mask = cv2.dilate(
                mask,
                kernel,
                iterations=1
            )

        return mask

    # ============================================================
    # PointCloud2 -> XYZ
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

            if fields[name].datatype != PointField.FLOAT32:
                raise ValueError(
                    f'PointCloud2 field "{name}" is not FLOAT32'
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

        finite = (
            np.isfinite(x)
            & np.isfinite(y)
            & np.isfinite(z)
        )

        return np.column_stack(
            (x[finite], y[finite], z[finite])
        ).astype(
            np.float32,
            copy=False
        )

    # ============================================================
    # XYZ -> PointCloud2
    # ============================================================

    def xyz_to_pointcloud2(self, points, stamp, frame_id):
        points = np.asarray(
            points,
            dtype=np.float32
        )

        msg = PointCloud2()

        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.height = 1
        msg.width = int(points.shape[0])

        msg.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
        ]

        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        if points.shape[0] == 0:
            msg.data = b''
        else:
            msg.data = np.ascontiguousarray(points).tobytes()

        return msg

    # ============================================================
    # Quaternion -> rotation matrix
    # ============================================================

    @staticmethod
    def quaternion_to_matrix(q):
        x, y, z, w = q.x, q.y, q.z, q.w

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
    # TF cache
    # ============================================================

    def ensure_transforms(self, lidar_frame):
        need_cam = (
            self.cam_R is None
            or self.cam_t is None
            or self.cam_lidar_frame != lidar_frame
        )

        need_base = (
            self.base_R is None
            or self.base_t is None
            or self.base_lidar_frame != lidar_frame
        )

        try:
            if need_cam:
                tf_cam = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    lidar_frame,
                    Time()
                )

                t = tf_cam.transform.translation
                q = tf_cam.transform.rotation

                self.cam_t = np.array(
                    [t.x, t.y, t.z],
                    dtype=np.float32
                )

                self.cam_R = self.quaternion_to_matrix(q)
                self.cam_lidar_frame = lidar_frame

                self.get_logger().info(
                    '[OK] Cached LiDAR -> Camera transform'
                )

            if need_base:
                tf_base = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    lidar_frame,
                    Time()
                )

                t = tf_base.transform.translation
                q = tf_base.transform.rotation

                self.base_t = np.array(
                    [t.x, t.y, t.z],
                    dtype=np.float32
                )

                self.base_R = self.quaternion_to_matrix(q)
                self.base_lidar_frame = lidar_frame

                self.get_logger().info(
                    '[OK] Cached LiDAR -> base_link transform'
                )

        except TransformException as exc:
            now = time.perf_counter()

            if now - self.last_tf_warning_time > 2.0:
                self.last_tf_warning_time = now
                self.get_logger().warning(
                    f'TF not ready: {exc}'
                )

            return False

        return True

    # ============================================================
    # Cluster construction
    # ============================================================

    def build_valid_clusters(
        self,
        candidate_lidar,
        candidate_depth
    ):
        if candidate_depth.size == 0:
            return []

        order = np.argsort(candidate_depth)

        sorted_depth = candidate_depth[order]
        sorted_lidar = candidate_lidar[order]

        depth_jump = np.diff(sorted_depth)

        split_indices = np.where(
            depth_jump > self.depth_cluster_gap
        )[0] + 1

        depth_clusters = np.split(
            sorted_depth,
            split_indices
        )

        lidar_clusters = np.split(
            sorted_lidar,
            split_indices
        )

        valid_clusters = []

        for lidar_cluster, depth_cluster in zip(
            lidar_clusters,
            depth_clusters
        ):
            if depth_cluster.size < self.min_depth_cluster_points:
                continue

            depth_min = float(np.min(depth_cluster))
            depth_max = float(np.max(depth_cluster))
            depth_median = float(np.median(depth_cluster))
            depth_span = depth_max - depth_min

            if not (
                self.min_target_depth
                <= depth_median
                <= self.max_target_depth
            ):
                continue

            if depth_span > self.max_cluster_span:
                continue

            base_cluster = (
                lidar_cluster @ self.base_R.T + self.base_t
            ).astype(
                np.float32,
                copy=False
            )

            centroid = np.mean(
                base_cluster,
                axis=0
            ).astype(
                np.float32,
                copy=False
            )

            valid_clusters.append({
                'lidar': lidar_cluster,
                'depth': depth_cluster,
                'base': base_cluster,
                'median_depth': depth_median,
                'centroid': centroid,
                'size': int(depth_cluster.size),
            })

        return valid_clusters

    # ============================================================
    # Cluster selection
    # ============================================================

    @staticmethod
    def largest_cluster(clusters):
        if len(clusters) == 0:
            return None

        return max(
            clusters,
            key=lambda c: c['size']
        )

    def select_tracking_cluster(self, clusters):
        if len(clusters) == 0:
            return None, 'no_valid_depth_cluster'

        if (
            self.last_valid_depth is None
            or self.last_valid_centroid is None
        ):
            return self.largest_cluster(clusters), 'tracking_history_empty'

        accepted = []

        for cluster in clusters:
            depth_jump = abs(
                cluster['median_depth']
                - self.last_valid_depth
            )

            centroid_jump = float(
                np.linalg.norm(
                    cluster['centroid']
                    - self.last_valid_centroid
                )
            )

            if (
                depth_jump <= self.max_depth_jump
                and centroid_jump <= self.max_centroid_jump
            ):
                score = depth_jump + centroid_jump
                accepted.append((score, cluster))

        if len(accepted) == 0:
            return None, 'tracking_discontinuity'

        accepted.sort(
            key=lambda item: item[0]
        )

        return accepted[0][1], 'accepted'

    def select_reacquire_cluster(self, clusters):
        if len(clusters) == 0:
            return None, 'no_valid_depth_cluster'

        if (
            self.reacquire_depth is None
            or self.reacquire_centroid is None
        ):
            return self.largest_cluster(clusters), 'new_candidate'

        accepted = []

        for cluster in clusters:
            depth_jump = abs(
                cluster['median_depth']
                - self.reacquire_depth
            )

            centroid_jump = float(
                np.linalg.norm(
                    cluster['centroid']
                    - self.reacquire_centroid
                )
            )

            if (
                depth_jump <= self.reacquire_max_depth_jump
                and centroid_jump <= self.reacquire_max_centroid_jump
            ):
                score = depth_jump + centroid_jump
                accepted.append((score, cluster))

        if len(accepted) == 0:
            return None, 'reacquire_discontinuity'

        accepted.sort(
            key=lambda item: item[0]
        )

        return accepted[0][1], 'accepted'

    # ============================================================
    # Callbacks
    # ============================================================

    def mask_callback(self, msg):
        self.latest_mask_msg = msg

    def camera_info_callback(self, msg):
        if self.camera_frame is not None:
            return

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

        self.image_width = int(msg.width)
        self.image_height = int(msg.height)

        self.camera_frame = msg.header.frame_id

        self.get_logger().info(
            '[OK] CameraInfo: '
            f'{self.image_width}x{self.image_height}, '
            f'fx={self.fx:.3f}, fy={self.fy:.3f}, '
            f'cx={self.cx:.3f}, cy={self.cy:.3f}, '
            f'frame="{self.camera_frame}"'
        )

    def cloud_callback(self, msg):
        if (
            self.latest_mask_msg is None
            or self.camera_frame is None
        ):
            return

        if not self.ensure_transforms(
            msg.header.frame_id
        ):
            return

        try:
            mask = self.mask_to_numpy(
                self.latest_mask_msg
            )
        except Exception as exc:
            self.get_logger().error(
                f'Mask conversion failed: {exc}'
            )
            return

        if (
            mask.shape[1] != self.image_width
            or mask.shape[0] != self.image_height
        ):
            self.get_logger().error(
                'Mask size does not match CameraInfo: '
                f'mask={mask.shape[1]}x{mask.shape[0]}, '
                f'camera={self.image_width}x{self.image_height}'
            )
            return

        try:
            points_lidar = self.pointcloud_xyz(msg)
        except Exception as exc:
            self.get_logger().error(
                f'PointCloud2 parse failed: {exc}'
            )
            return

        if points_lidar.shape[0] == 0:
            self.publish_empty(msg.header.stamp)
            return

        points_camera = (
            points_lidar @ self.cam_R.T + self.cam_t
        )

        depth = points_camera[:, 0]
        valid_front = depth > self.min_camera_depth

        points_camera = points_camera[valid_front]
        points_lidar_front = points_lidar[valid_front]
        depth = depth[valid_front]

        if points_camera.shape[0] == 0:
            self.publish_empty(msg.header.stamp)
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

        inside = (
            (u >= 0.0)
            & (u < self.image_width)
            & (v >= 0.0)
            & (v < self.image_height)
        )

        u = u[inside]
        v = v[inside]
        depth_image = depth[inside]
        points_lidar_image = points_lidar_front[inside]

        if u.size == 0:
            self.publish_empty(msg.header.stamp)
            return

        u_i = np.rint(u).astype(np.int32)
        v_i = np.rint(v).astype(np.int32)

        valid_pixel = (
            (u_i >= 0)
            & (u_i < self.image_width)
            & (v_i >= 0)
            & (v_i < self.image_height)
        )

        u_i = u_i[valid_pixel]
        v_i = v_i[valid_pixel]
        depth_image = depth_image[valid_pixel]
        points_lidar_image = points_lidar_image[valid_pixel]

        target_mask = mask[v_i, u_i] > 0

        candidate_lidar = points_lidar_image[target_mask]
        candidate_depth = depth_image[target_mask]

        clusters = self.build_valid_clusters(
            candidate_lidar,
            candidate_depth
        )

        output_base = np.empty(
            (0, 3),
            dtype=np.float32
        )

        output_cluster = None
        event_reason = ''

        # ========================================================
        # State machine
        # ========================================================

        if self.state == self.TRACKING:
            cluster, reason = self.select_tracking_cluster(
                clusters
            )

            if cluster is not None:
                # Valid target visible again: cancel any short miss streak.
                self.reset_tracking_missing()

                output_cluster = cluster
                output_base = cluster['base']

                self.last_valid_depth = cluster['median_depth']
                self.last_valid_centroid = cluster['centroid'].copy()

                event_reason = 'tracking_ok'

            else:
                # One or two missing frames are treated as a short grace
                # period. We stay in TRACKING, but publish an EMPTY cloud
                # so later PCA/planning never consumes stale geometry.
                self.tracking_missing_count += 1

                if (
                    self.tracking_missing_count
                    < self.tracking_missing_required_frames
                ):
                    event_reason = (
                        f'tracking_missing_'
                        f'{self.tracking_missing_count}/'
                        f'{self.tracking_missing_required_frames}'
                    )

                else:
                    self.reset_tracking_missing()
                    self.clear_tracking_history()
                    self.reset_reacquire()

                    self.set_state(
                        self.LOST,
                        reason=(
                            f'{reason}; '
                            f'missing_confirmed_'
                            f'{self.tracking_missing_required_frames}/'
                            f'{self.tracking_missing_required_frames}'
                        )
                    )

                    event_reason = 'tracking_loss_confirmed'

        elif self.state == self.LOST:
            cluster = self.largest_cluster(
                clusters
            )

            if cluster is not None:
                self.reacquire_count = 1
                self.reacquire_depth = cluster['median_depth']
                self.reacquire_centroid = cluster['centroid'].copy()

                self.set_state(
                    self.REACQUIRING,
                    reason=(
                        f'candidate 1/'
                        f'{self.reacquire_required_frames}'
                    )
                )

                event_reason = 'reacquire_started'
            else:
                event_reason = 'target_not_visible'

        elif self.state == self.REACQUIRING:
            cluster, reason = self.select_reacquire_cluster(
                clusters
            )

            if cluster is None:
                if len(clusters) == 0:
                    self.reset_reacquire()

                    self.set_state(
                        self.LOST,
                        reason=reason
                    )

                    event_reason = reason

                else:
                    cluster = self.largest_cluster(
                        clusters
                    )

                    self.reacquire_count = 1
                    self.reacquire_depth = cluster['median_depth']
                    self.reacquire_centroid = cluster['centroid'].copy()

                    event_reason = (
                        'reacquire_reset_1/'
                        f'{self.reacquire_required_frames}'
                    )

            else:
                self.reacquire_count += 1
                self.reacquire_depth = cluster['median_depth']
                self.reacquire_centroid = cluster['centroid'].copy()

                event_reason = (
                    f'reacquiring_'
                    f'{self.reacquire_count}/'
                    f'{self.reacquire_required_frames}'
                )

                if (
                    self.reacquire_count
                    >= self.reacquire_required_frames
                ):
                    output_cluster = cluster
                    output_base = cluster['base']

                    self.last_valid_depth = cluster['median_depth']
                    self.last_valid_centroid = cluster['centroid'].copy()

                    self.reset_tracking_missing()
                    self.reset_reacquire()

                    self.set_state(
                        self.TRACKING,
                        reason='reacquisition_confirmed'
                    )

                    event_reason = 'reacquisition_confirmed'

        # ========================================================
        # Publish
        # ========================================================

        out_msg = self.xyz_to_pointcloud2(
            output_base,
            msg.header.stamp,
            self.target_frame
        )

        self.target_cloud_pub.publish(out_msg)

        # ========================================================
        # Statistics
        # ========================================================

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
        self.frame_count += 1

        if self.frame_count % 10 == 0:
            self.get_logger().info(
                '[TARGET CLOUD] '
                f'state={self.state}, '
                f'cloud={points_lidar.shape[0]}, '
                f'in_image={points_lidar_image.shape[0]}, '
                f'candidate={candidate_lidar.shape[0]}, '
                f'clusters={len(clusters)}, '
                f'output={output_base.shape[0]}, '
                f'fps={self.smoothed_fps:.1f}, '
                f'event={event_reason}'
            )

            if (
                output_cluster is not None
                and output_base.shape[0] > 0
            ):
                c = output_cluster['centroid']

                self.get_logger().info(
                    '[VALID TARGET] '
                    f'depth=['
                    f'{float(np.min(output_cluster["depth"])):.3f}, '
                    f'{float(np.max(output_cluster["depth"])):.3f}] m, '
                    f'median={output_cluster["median_depth"]:.3f} m, '
                    f'centroid_base=['
                    f'{c[0]:.3f}, '
                    f'{c[1]:.3f}, '
                    f'{c[2]:.3f}]'
                )

            elif (
                self.state == self.TRACKING
                and self.tracking_missing_count > 0
            ):
                self.get_logger().info(
                    '[TRACKING GRACE] '
                    f'missing='
                    f'{self.tracking_missing_count}/'
                    f'{self.tracking_missing_required_frames}, '
                    'target cloud output is empty'
                )

            elif self.state == self.REACQUIRING:
                self.get_logger().info(
                    '[REACQUIRING] '
                    f'confirmation='
                    f'{self.reacquire_count}/'
                    f'{self.reacquire_required_frames}'
                )

            elif self.state == self.LOST:
                self.get_logger().info(
                    '[LOST] target cloud output is empty'
                )

    # ============================================================
    # Empty cloud
    # ============================================================

    def publish_empty(self, stamp):
        empty = self.xyz_to_pointcloud2(
            np.empty(
                (0, 3),
                dtype=np.float32
            ),
            stamp,
            self.target_frame
        )

        self.target_cloud_pub.publish(empty)


def main(args=None):
    rclpy.init(args=args)

    node = TargetBranchCloud()

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
