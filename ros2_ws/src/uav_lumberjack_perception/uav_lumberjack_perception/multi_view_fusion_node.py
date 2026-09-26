import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger


class MultiViewFusionNode(Node):
    """
    Step13.4.4-A
    Manual multi-view target-branch point-cloud fusion.

    Input:
        /perception/target_branch_cloud_world
        frame_id = world

    Services:
        /perception/multiview/capture
        /perception/multiview/reset

    Output:
        /perception/target_branch_cloud_fused
        frame_id = world

    First version:
        - manual keyframe capture
        - world-frame accumulation
        - NumPy voxel downsampling
        - no automatic view selection yet
    """

    def __init__(self):
        super().__init__('multi_view_fusion_node')

        # ========================================================
        # Parameters
        # ========================================================

        self.input_topic = (
            '/perception/target_branch_cloud_world'
        )

        self.output_topic = (
            '/perception/target_branch_cloud_fused'
        )

        self.world_frame = 'world'

        # 5 mm voxel:
        # enough to remove nearly duplicated observations
        # without destroying the ~38 mm-radius branch geometry.
        self.voxel_size = 0.005

        self.min_capture_points = 5

        # ========================================================
        # State
        # ========================================================

        self.latest_points = None
        self.latest_stamp = None

        self.keyframes = []

        self.fused_points = np.empty(
            (0, 3),
            dtype=np.float32
        )

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

        # Republish the current fused model at 2 Hz,
        # so RViz can be opened after capture and still see it.
        self.publish_timer = self.create_timer(
            0.5,
            self.publish_fused_cloud
        )

        self.get_logger().info(
            '===================================================='
        )
        self.get_logger().info(
            ' Step13.4.4-A Manual Multi-View Fusion'
        )
        self.get_logger().info(
            f' Input : {self.input_topic}'
        )
        self.get_logger().info(
            f' Output: {self.output_topic}'
        )
        self.get_logger().info(
            ' Capture service: '
            '/perception/multiview/capture'
        )
        self.get_logger().info(
            ' Reset service  : '
            '/perception/multiview/reset'
        )
        self.get_logger().info(
            f' voxel_size={self.voxel_size:.3f} m'
        )
        self.get_logger().info(
            '===================================================='
        )

    # ============================================================
    # PointCloud2 -> XYZ
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
            strides=(
                msg.row_step,
                msg.point_step
            )
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

    # ============================================================
    # XYZ -> PointCloud2
    # ============================================================

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
        msg.width = int(
            points.shape[0]
        )

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
        msg.row_step = (
            msg.point_step
            * msg.width
        )
        msg.is_dense = True

        msg.data = np.ascontiguousarray(
            points,
            dtype=np.float32
        ).tobytes()

        return msg

    # ============================================================
    # Voxel downsampling
    # ============================================================

    def voxel_downsample(self, points):
        if points.shape[0] == 0:
            return points

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

        np.add.at(
            sums,
            inverse,
            points
        )

        np.add.at(
            counts,
            inverse,
            1
        )

        downsampled = (
            sums
            / counts[:, None]
        )

        return downsampled.astype(
            np.float32
        )

    # ============================================================
    # Latest valid world-frame cloud
    # ============================================================

    def cloud_callback(self, msg):
        if msg.header.frame_id != self.world_frame:
            self.get_logger().warn(
                '[MULTIVIEW] Ignoring cloud because '
                f'frame_id="{msg.header.frame_id}", '
                f'expected "{self.world_frame}"',
                throttle_duration_sec=2.0
            )
            return

        try:
            points = self.pointcloud_xyz(
                msg
            )

        except Exception as exc:
            self.get_logger().error(
                f'PointCloud parse failed: {exc}'
            )
            return

        # Empty clouds are intentionally published by the
        # target state machine during LOST / REACQUIRING.
        if points.shape[0] < self.min_capture_points:
            self.latest_points = None
            self.latest_stamp = msg.header.stamp
            return

        self.latest_points = points.copy()
        self.latest_stamp = msg.header.stamp

    # ============================================================
    # Manual capture
    # ============================================================

    def capture_callback(
        self,
        request,
        response
    ):
        del request

        if self.latest_points is None:
            response.success = False
            response.message = (
                'No valid target cloud available. '
                'Wait until target is TRACKING.'
            )

            self.get_logger().warn(
                '[CAPTURE REJECTED] '
                'No valid current target cloud'
            )

            return response

        current = self.latest_points.copy()

        self.keyframes.append(
            current
        )

        raw_fused = np.concatenate(
            self.keyframes,
            axis=0
        )

        self.fused_points = (
            self.voxel_downsample(
                raw_fused
            )
        )

        keyframe_count = len(
            self.keyframes
        )

        message = (
            f'Captured keyframe {keyframe_count}: '
            f'input={current.shape[0]} pts, '
            f'raw_total={raw_fused.shape[0]} pts, '
            f'voxel_total={self.fused_points.shape[0]} pts'
        )

        response.success = True
        response.message = message

        self.get_logger().info(
            '[CAPTURE] ' + message
        )

        self.publish_fused_cloud()

        return response

    # ============================================================
    # Reset
    # ============================================================

    def reset_callback(
        self,
        request,
        response
    ):
        del request

        old_keyframes = len(
            self.keyframes
        )

        old_points = int(
            self.fused_points.shape[0]
        )

        self.keyframes.clear()

        self.fused_points = np.empty(
            (0, 3),
            dtype=np.float32
        )

        response.success = True
        response.message = (
            f'Cleared {old_keyframes} keyframes '
            f'and {old_points} fused points'
        )

        self.get_logger().info(
            '[RESET] '
            + response.message
        )

        self.publish_fused_cloud()

        return response

    # ============================================================
    # Publish fused model
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
