import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger


class MultiViewFusionNode(Node):
    """
    Step13.4.5 v1.1
    Lightweight robust online multi-view fusion.

    Main idea:
        - keep a stable reference AXIS for branch identity/orientation
        - do NOT permanently lock the branch LENGTH to the first frame
        - axial endpoints may expand only after repeated support from
          several valid frames
        - repeated surfaces are ignored by nearest-surface distance
        - only genuinely new, model-consistent points are fused

    This avoids both failure modes observed in Step13:
        1) dynamic bounds drifting longer and longer
        2) first-frame fixed bounds truncating a partially observed branch

    No ICP / TSDF / heavy PCL pipeline is required.
    """

    def __init__(self):
        super().__init__('multi_view_fusion_node')

        self.input_topic = '/perception/target_branch_cloud_world'
        self.output_topic = '/perception/target_branch_cloud_fused'
        self.world_frame = 'world'

        # ========================================================
        # Parameters
        # ========================================================

        self.declare_parameter('auto_fusion_enabled', True)
        self.declare_parameter('evaluation_interval', 0.75)

        self.declare_parameter('voxel_size', 0.005)
        self.declare_parameter('min_capture_points', 10)

        # Same-branch frame gate.
        self.declare_parameter('max_axis_angle_deg', 25.0)
        self.declare_parameter('max_center_perp_distance', 0.08)
        self.declare_parameter('max_center_axial_shift', 0.25)

        # Cylindrical corridor around the reference axis.
        self.declare_parameter('max_axis_radius', 0.08)

        # Bootstrap range from the first valid frame.
        self.declare_parameter('bootstrap_low_percentile', 2.0)
        self.declare_parameter('bootstrap_high_percentile', 98.0)

        # Each later frame proposes robust axial endpoints.
        self.declare_parameter('frame_low_percentile', 5.0)
        self.declare_parameter('frame_high_percentile', 95.0)

        # Endpoint expansion needs repeated, consistent support.
        self.declare_parameter('endpoint_confirm_frames', 3)
        self.declare_parameter('endpoint_trigger', 0.012)
        self.declare_parameter('endpoint_candidate_tolerance', 0.025)
        self.declare_parameter('endpoint_max_step', 0.040)

        # Small gate margin around already confirmed endpoints.
        self.declare_parameter('axial_margin', 0.010)

        # Surface novelty.
        self.declare_parameter('novelty_distance', 0.015)
        self.declare_parameter('min_novel_points', 5)
        self.declare_parameter('min_novelty_ratio', 0.20)

        self.auto_fusion_enabled = bool(
            self.get_parameter('auto_fusion_enabled').value
        )
        self.evaluation_interval = float(
            self.get_parameter('evaluation_interval').value
        )

        self.voxel_size = float(
            self.get_parameter('voxel_size').value
        )
        self.min_capture_points = int(
            self.get_parameter('min_capture_points').value
        )

        self.max_axis_angle_deg = float(
            self.get_parameter('max_axis_angle_deg').value
        )
        self.max_center_perp_distance = float(
            self.get_parameter('max_center_perp_distance').value
        )
        self.max_center_axial_shift = float(
            self.get_parameter('max_center_axial_shift').value
        )
        self.max_axis_radius = float(
            self.get_parameter('max_axis_radius').value
        )

        self.bootstrap_low_percentile = float(
            self.get_parameter('bootstrap_low_percentile').value
        )
        self.bootstrap_high_percentile = float(
            self.get_parameter('bootstrap_high_percentile').value
        )
        self.frame_low_percentile = float(
            self.get_parameter('frame_low_percentile').value
        )
        self.frame_high_percentile = float(
            self.get_parameter('frame_high_percentile').value
        )

        self.endpoint_confirm_frames = int(
            self.get_parameter('endpoint_confirm_frames').value
        )
        self.endpoint_trigger = float(
            self.get_parameter('endpoint_trigger').value
        )
        self.endpoint_candidate_tolerance = float(
            self.get_parameter('endpoint_candidate_tolerance').value
        )
        self.endpoint_max_step = float(
            self.get_parameter('endpoint_max_step').value
        )
        self.axial_margin = float(
            self.get_parameter('axial_margin').value
        )

        self.novelty_distance = float(
            self.get_parameter('novelty_distance').value
        )
        self.min_novel_points = int(
            self.get_parameter('min_novel_points').value
        )
        self.min_novelty_ratio = float(
            self.get_parameter('min_novelty_ratio').value
        )

        if self.evaluation_interval < 0.0:
            raise ValueError('evaluation_interval must be >= 0')
        if self.voxel_size <= 0.0:
            raise ValueError('voxel_size must be > 0')
        if self.min_capture_points < 3:
            raise ValueError('min_capture_points must be >= 3')
        if self.endpoint_confirm_frames < 1:
            raise ValueError('endpoint_confirm_frames must be >= 1')
        if self.endpoint_trigger < 0.0:
            raise ValueError('endpoint_trigger must be >= 0')
        if self.endpoint_candidate_tolerance <= 0.0:
            raise ValueError(
                'endpoint_candidate_tolerance must be > 0'
            )
        if self.endpoint_max_step <= 0.0:
            raise ValueError('endpoint_max_step must be > 0')
        if self.axial_margin < 0.0:
            raise ValueError('axial_margin must be >= 0')
        if self.novelty_distance <= 0.0:
            raise ValueError('novelty_distance must be > 0')
        if not 0.0 <= self.min_novelty_ratio <= 1.0:
            raise ValueError(
                'min_novelty_ratio must be in [0, 1]'
            )

        self.evaluation_interval_ns = int(
            self.evaluation_interval * 1.0e9
        )

        # ========================================================
        # State
        # ========================================================

        self.latest_points = None
        self.latest_stamp = None

        self.fused_points = np.empty(
            (0, 3),
            dtype=np.float32
        )

        # Stable branch reference axis.
        self.reference_center = None
        self.reference_direction = None

        # Confirmed axial envelope along the reference axis.
        self.confirmed_s_low = None
        self.confirmed_s_high = None

        # Pending endpoint evidence.
        self.pending_low_value = None
        self.pending_low_count = 0

        self.pending_high_value = None
        self.pending_high_count = 0

        self.last_evaluation_time_ns = None

        self.accept_count = 0
        self.ignore_count = 0
        self.reject_count = 0

        self.last_ignore_log_time_ns = None
        self.ignore_log_period_ns = int(2.0e9)

        # ========================================================
        # ROS interfaces
        # ========================================================

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data
        )

        self.fused_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile_sensor_data
        )

        self.capture_srv = self.create_service(
            Trigger,
            '/perception/multiview/capture',
            self.capture_callback
        )

        self.reset_srv = self.create_service(
            Trigger,
            '/perception/multiview/reset',
            self.reset_callback
        )

        self.publish_timer = self.create_timer(
            0.5,
            self.publish_fused_cloud
        )

        self.get_logger().info(
            '[FUSION] v1.1 | '
            'stable axis + confirmed adaptive endpoints'
        )

    # ============================================================
    # PointCloud2 <-> XYZ
    # ============================================================

    @staticmethod
    def pointcloud_xyz(msg):
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

        if msg.width == 0 or len(msg.data) == 0:
            return np.empty(
                (0, 3),
                dtype=np.float32
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

        return np.column_stack((
            x[finite],
            y[finite],
            z[finite]
        )).astype(
            np.float32,
            copy=False
        )

    @staticmethod
    def xyz_to_pointcloud2(
        points,
        stamp,
        frame_id
    ):
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
            )
        ]

        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        msg.data = np.ascontiguousarray(
            points,
            dtype=np.float32
        ).tobytes()

        return msg

    # ============================================================
    # Voxel
    # ============================================================

    def voxel_downsample(self, points):
        if points.shape[0] == 0:
            return points.astype(
                np.float32,
                copy=False
            )

        voxel_index = np.floor(
            points / self.voxel_size
        ).astype(np.int64)

        unique_voxels, inverse = np.unique(
            voxel_index,
            axis=0,
            return_inverse=True
        )

        sums = np.zeros(
            (unique_voxels.shape[0], 3),
            dtype=np.float64
        )

        counts = np.zeros(
            unique_voxels.shape[0],
            dtype=np.int64
        )

        np.add.at(sums, inverse, points)
        np.add.at(counts, inverse, 1)

        return (
            sums / counts[:, None]
        ).astype(np.float32)

    # ============================================================
    # PCA reference axis
    # ============================================================

    @staticmethod
    def pca_axis(points):
        center = np.mean(
            points,
            axis=0
        ).astype(np.float64)

        centered = (
            points.astype(np.float64)
            - center
        )

        covariance = (
            centered.T @ centered
        ) / float(points.shape[0])

        eigenvalues, eigenvectors = np.linalg.eigh(
            covariance
        )

        order = np.argsort(eigenvalues)[::-1]
        direction = eigenvectors[:, order[0]]

        norm = float(np.linalg.norm(direction))
        if norm < 1.0e-9:
            raise ValueError(
                'PCA direction norm is too small'
            )

        direction = direction / norm

        major_component = int(
            np.argmax(np.abs(direction))
        )
        if direction[major_component] < 0.0:
            direction = -direction

        return center, direction

    @staticmethod
    def axis_angle_deg(d1, d2):
        dot = abs(
            float(np.dot(d1, d2))
        )

        dot = float(
            np.clip(dot, 0.0, 1.0)
        )

        return math.degrees(
            math.acos(dot)
        )

    def initialize_reference(self, points):
        center, direction = self.pca_axis(
            points
        )

        rel = (
            points.astype(np.float64)
            - center
        )

        s = rel @ direction

        s_low = float(
            np.percentile(
                s,
                self.bootstrap_low_percentile
            )
        )

        s_high = float(
            np.percentile(
                s,
                self.bootstrap_high_percentile
            )
        )

        if s_high <= s_low:
            raise ValueError(
                'Invalid bootstrap axial range'
            )

        self.reference_center = center
        self.reference_direction = direction

        self.confirmed_s_low = s_low
        self.confirmed_s_high = s_high

        self.pending_low_value = None
        self.pending_low_count = 0
        self.pending_high_value = None
        self.pending_high_count = 0

    # ============================================================
    # Confirmed endpoint expansion
    # ============================================================

    def update_one_endpoint(
        self,
        side,
        proposed_value
    ):
        """
        Repeated-support endpoint update.

        Returns:
            True if a confirmed endpoint was expanded.
        """

        if side == 'low':
            confirmed = self.confirmed_s_low

            if (
                proposed_value
                >= confirmed - self.endpoint_trigger
            ):
                self.pending_low_value = None
                self.pending_low_count = 0
                return False

            if (
                self.pending_low_value is None
                or abs(
                    proposed_value
                    - self.pending_low_value
                ) > self.endpoint_candidate_tolerance
            ):
                self.pending_low_value = proposed_value
                self.pending_low_count = 1
                return False

            self.pending_low_value = 0.5 * (
                self.pending_low_value
                + proposed_value
            )
            self.pending_low_count += 1

            if (
                self.pending_low_count
                < self.endpoint_confirm_frames
            ):
                return False

            # Limit each confirmed expansion step.
            new_value = max(
                self.pending_low_value,
                confirmed - self.endpoint_max_step
            )

            self.confirmed_s_low = min(
                confirmed,
                new_value
            )

            self.pending_low_value = None
            self.pending_low_count = 0
            return True

        confirmed = self.confirmed_s_high

        if (
            proposed_value
            <= confirmed + self.endpoint_trigger
        ):
            self.pending_high_value = None
            self.pending_high_count = 0
            return False

        if (
            self.pending_high_value is None
            or abs(
                proposed_value
                - self.pending_high_value
            ) > self.endpoint_candidate_tolerance
        ):
            self.pending_high_value = proposed_value
            self.pending_high_count = 1
            return False

        self.pending_high_value = 0.5 * (
            self.pending_high_value
            + proposed_value
        )
        self.pending_high_count += 1

        if (
            self.pending_high_count
            < self.endpoint_confirm_frames
        ):
            return False

        new_value = min(
            self.pending_high_value,
            confirmed + self.endpoint_max_step
        )

        self.confirmed_s_high = max(
            confirmed,
            new_value
        )

        self.pending_high_value = None
        self.pending_high_count = 0
        return True

    def update_endpoint_consensus(
        self,
        s_values
    ):
        if s_values.shape[0] < self.min_capture_points:
            return False

        proposed_low = float(
            np.percentile(
                s_values,
                self.frame_low_percentile
            )
        )

        proposed_high = float(
            np.percentile(
                s_values,
                self.frame_high_percentile
            )
        )

        low_changed = self.update_one_endpoint(
            'low',
            proposed_low
        )

        high_changed = self.update_one_endpoint(
            'high',
            proposed_high
        )

        if low_changed or high_changed:
            confirmed_length = (
                self.confirmed_s_high
                - self.confirmed_s_low
            )

            self.get_logger().info(
                '[AXIS RANGE] '
                f'confirmed_L={confirmed_length:.3f}m'
            )

        return (
            low_changed
            or high_changed
        )

    # ============================================================
    # Nearest surface distance
    # ============================================================

    @staticmethod
    def nearest_distances(
        query_points,
        model_points
    ):
        if query_points.shape[0] == 0:
            return np.empty(
                (0,),
                dtype=np.float64
            )

        if model_points.shape[0] == 0:
            return np.full(
                query_points.shape[0],
                np.inf,
                dtype=np.float64
            )

        query = query_points.astype(
            np.float64,
            copy=False
        )

        model = model_points.astype(
            np.float64,
            copy=False
        )

        result = np.empty(
            query.shape[0],
            dtype=np.float64
        )

        chunk_size = 128

        for start in range(
            0,
            query.shape[0],
            chunk_size
        ):
            stop = min(
                query.shape[0],
                start + chunk_size
            )

            diff = (
                query[start:stop, None, :]
                - model[None, :, :]
            )

            dist2 = np.sum(
                diff * diff,
                axis=2
            )

            result[start:stop] = np.sqrt(
                np.min(
                    dist2,
                    axis=1
                )
            )

        return result

    # ============================================================
    # Timing
    # ============================================================

    def evaluation_ready(self):
        now_ns = int(
            self.get_clock().now().nanoseconds
        )

        if self.last_evaluation_time_ns is None:
            self.last_evaluation_time_ns = now_ns
            return True

        if now_ns < self.last_evaluation_time_ns:
            self.last_evaluation_time_ns = now_ns
            return True

        if (
            now_ns
            - self.last_evaluation_time_ns
        ) < self.evaluation_interval_ns:
            return False

        self.last_evaluation_time_ns = now_ns
        return True

    # ============================================================
    # ACCEPT / IGNORE / REJECT
    # ============================================================

    def evaluate_and_integrate(self, points):
        points = self.voxel_downsample(
            points
        )

        input_count = int(
            points.shape[0]
        )

        if input_count < self.min_capture_points:
            self.reject_count += 1
            return (
                'REJECT',
                f'too_few_points n={input_count}'
            )

        # Bootstrap.
        if self.reference_center is None:
            try:
                self.initialize_reference(points)

            except Exception as exc:
                self.reject_count += 1
                return (
                    'REJECT',
                    f'bootstrap_failed "{exc}"'
                )

            self.fused_points = points.copy()
            self.accept_count += 1
            self.publish_fused_cloud()

            ref_length = (
                self.confirmed_s_high
                - self.confirmed_s_low
            )

            return (
                'ACCEPT',
                f'bootstrap model={input_count} '
                f'refL={ref_length:.3f}m'
            )

        # Frame-level same-branch gate.
        try:
            frame_center, frame_direction = self.pca_axis(
                points
            )

        except Exception as exc:
            self.reject_count += 1
            return (
                'REJECT',
                f'pca_failed "{exc}"'
            )

        angle_deg = self.axis_angle_deg(
            self.reference_direction,
            frame_direction
        )

        if angle_deg > self.max_axis_angle_deg:
            self.reject_count += 1
            return (
                'REJECT',
                f'axis_mismatch angle={angle_deg:.1f}deg'
            )

        center_rel = (
            frame_center
            - self.reference_center
        )

        center_axial = float(
            np.dot(
                center_rel,
                self.reference_direction
            )
        )

        center_perp_vec = (
            center_rel
            - center_axial
            * self.reference_direction
        )

        center_perp = float(
            np.linalg.norm(
                center_perp_vec
            )
        )

        if center_perp > self.max_center_perp_distance:
            self.reject_count += 1
            return (
                'REJECT',
                f'center_off_axis perp={center_perp:.3f}m'
            )

        if abs(center_axial) > self.max_center_axial_shift:
            self.reject_count += 1
            return (
                'REJECT',
                f'center_shift axial={center_axial:.3f}m'
            )

        # Project to fixed reference axis and first apply only the
        # radial cylinder corridor.
        rel = (
            points.astype(np.float64)
            - self.reference_center
        )

        s = rel @ self.reference_direction

        radial_vec = (
            rel
            - s[:, None]
            * self.reference_direction[None, :]
        )

        radial_dist = np.linalg.norm(
            radial_vec,
            axis=1
        )

        radial_ok = (
            radial_dist
            <= self.max_axis_radius
        )

        radial_points = points[
            radial_ok
        ]

        radial_s = s[
            radial_ok
        ]

        if radial_points.shape[0] < self.min_capture_points:
            self.reject_count += 1
            return (
                'REJECT',
                f'radial_gate kept='
                f'{radial_points.shape[0]}/{input_count}'
            )

        # Let repeated frame evidence expand the endpoint envelope.
        self.update_endpoint_consensus(
            radial_s
        )

        # Now apply the CONFIRMED axial range.
        axial_ok = (
            (
                radial_s
                >= self.confirmed_s_low
                - self.axial_margin
            )
            &
            (
                radial_s
                <= self.confirmed_s_high
                + self.axial_margin
            )
        )

        gated_points = radial_points[
            axial_ok
        ]

        gated_count = int(
            gated_points.shape[0]
        )

        if gated_count < self.min_capture_points:
            self.reject_count += 1
            return (
                'REJECT',
                f'axial_gate kept={gated_count}/{input_count}'
            )

        # Surface novelty.
        nearest = self.nearest_distances(
            gated_points,
            self.fused_points
        )

        novel_mask = (
            nearest
            >= self.novelty_distance
        )

        novel_points = gated_points[
            novel_mask
        ]

        novel_count = int(
            novel_points.shape[0]
        )

        novelty_ratio = (
            float(novel_count)
            / float(gated_count)
            if gated_count > 0
            else 0.0
        )

        if (
            novel_count < self.min_novel_points
            or novelty_ratio < self.min_novelty_ratio
        ):
            self.ignore_count += 1
            return (
                'IGNORE',
                f'novel={novel_count}/{gated_count} '
                f'({100.0*novelty_ratio:.1f}%) '
                f'model={self.fused_points.shape[0]}'
            )

        old_count = int(
            self.fused_points.shape[0]
        )

        combined = np.concatenate(
            (
                self.fused_points,
                novel_points
            ),
            axis=0
        )

        self.fused_points = self.voxel_downsample(
            combined
        )

        new_count = int(
            self.fused_points.shape[0]
        )

        self.accept_count += 1
        self.publish_fused_cloud()

        return (
            'ACCEPT',
            f'novel={novel_count}/{gated_count} '
            f'({100.0*novelty_ratio:.1f}%) '
            f'model={old_count}->{new_count}'
        )

    # ============================================================
    # Logging
    # ============================================================

    def log_decision(
        self,
        decision,
        message
    ):
        now_ns = int(
            self.get_clock().now().nanoseconds
        )

        if decision == 'ACCEPT':
            self.get_logger().info(
                '[FUSION ACCEPT] '
                + message
            )
            return

        if decision == 'REJECT':
            self.get_logger().warn(
                '[FUSION REJECT] '
                + message
            )
            return

        if (
            self.last_ignore_log_time_ns is None
            or now_ns < self.last_ignore_log_time_ns
            or (
                now_ns
                - self.last_ignore_log_time_ns
            ) >= self.ignore_log_period_ns
        ):
            self.last_ignore_log_time_ns = now_ns

            self.get_logger().info(
                '[FUSION IGNORE] '
                + message
            )

    # ============================================================
    # Subscription / services
    # ============================================================

    def cloud_callback(self, msg):
        if msg.header.frame_id != self.world_frame:
            self.get_logger().warn(
                '[FUSION REJECT] '
                f'wrong_frame="{msg.header.frame_id}"',
                throttle_duration_sec=2.0
            )
            return

        try:
            points = self.pointcloud_xyz(
                msg
            )

        except Exception as exc:
            self.get_logger().error(
                '[FUSION REJECT] '
                f'parse_failed "{exc}"'
            )
            return

        if points.shape[0] < self.min_capture_points:
            self.latest_points = None
            self.latest_stamp = msg.header.stamp
            return

        self.latest_points = points.copy()
        self.latest_stamp = msg.header.stamp

        if not self.auto_fusion_enabled:
            return

        if not self.evaluation_ready():
            return

        decision, message = self.evaluate_and_integrate(
            self.latest_points
        )

        self.log_decision(
            decision,
            message
        )

    def capture_callback(
        self,
        request,
        response
    ):
        del request

        if self.latest_points is None:
            response.success = False
            response.message = (
                'No valid target cloud available.'
            )
            return response

        decision, message = self.evaluate_and_integrate(
            self.latest_points.copy()
        )

        self.log_decision(
            decision,
            message
        )

        response.success = (
            decision == 'ACCEPT'
        )
        response.message = (
            f'{decision}: {message}'
        )

        return response

    def reset_callback(
        self,
        request,
        response
    ):
        del request

        old_points = int(
            self.fused_points.shape[0]
        )

        old_counts = (
            self.accept_count,
            self.ignore_count,
            self.reject_count
        )

        self.fused_points = np.empty(
            (0, 3),
            dtype=np.float32
        )

        self.reference_center = None
        self.reference_direction = None

        self.confirmed_s_low = None
        self.confirmed_s_high = None

        self.pending_low_value = None
        self.pending_low_count = 0
        self.pending_high_value = None
        self.pending_high_count = 0

        self.accept_count = 0
        self.ignore_count = 0
        self.reject_count = 0

        self.last_evaluation_time_ns = None
        self.last_ignore_log_time_ns = None

        response.success = True
        response.message = (
            f'cleared model={old_points} pts, '
            f'A/I/R={old_counts[0]}/'
            f'{old_counts[1]}/'
            f'{old_counts[2]}'
        )

        self.get_logger().info(
            '[FUSION RESET] '
            + response.message
        )

        self.publish_fused_cloud()

        return response

    # ============================================================
    # Publish
    # ============================================================

    def publish_fused_cloud(self):
        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg = self.xyz_to_pointcloud2(
            self.fused_points,
            stamp,
            self.world_frame
        )

        self.fused_pub.publish(
            msg
        )


def main(args=None):
    rclpy.init(args=args)

    node = MultiViewFusionNode()

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
