import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException


class TargetCloudWorld(Node):

    def __init__(self):
        super().__init__('target_cloud_world')

        self.target_frame = 'world'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.sub = self.create_subscription(
            PointCloud2,
            '/perception/target_branch_cloud',
            self.cloud_callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/perception/target_branch_cloud_world',
            qos_profile_sensor_data
        )

        self.frame_count = 0
        self.last_warning_time = 0.0

        self.get_logger().info(
            '===================================================='
        )
        self.get_logger().info(
            ' Step13.4.4-A Target Cloud: base_link -> world'
        )
        self.get_logger().info(
            ' Input : /perception/target_branch_cloud'
        )
        self.get_logger().info(
            ' Output: /perception/target_branch_cloud_world'
        )
        self.get_logger().info(
            ' TF    : world <- base_link'
        )
        self.get_logger().info(
            '===================================================='
        )

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

    def xyz_to_pointcloud2(
        self,
        points,
        stamp,
        frame_id
    ):
        msg = PointCloud2()

        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.height = 1
        msg.width = points.shape[0]

        msg.is_bigendian = False
        msg.is_dense = True

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

        msg.point_step = 12
        msg.row_step = (
            msg.point_step
            * msg.width
        )

        msg.data = np.ascontiguousarray(
            points,
            dtype=np.float32
        ).tobytes()

        return msg

    # ============================================================
    # Quaternion -> Rotation matrix
    # ============================================================

    @staticmethod
    def quaternion_to_matrix(q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        norm = (
            x * x
            + y * y
            + z * z
            + w * w
        )

        if norm < 1.0e-12:
            return np.eye(
                3,
                dtype=np.float64
            )

        s = 2.0 / norm

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
            [
                1.0 - yy - zz,
                xy - wz,
                xz + wy
            ],
            [
                xy + wz,
                1.0 - xx - zz,
                yz - wx
            ],
            [
                xz - wy,
                yz + wx,
                1.0 - xx - yy
            ]
        ], dtype=np.float64)

    # ============================================================
    # Callback
    # ============================================================

    def cloud_callback(self, msg):
        try:
            points_base = self.pointcloud_xyz(
                msg
            )

        except Exception as exc:
            self.get_logger().error(
                f'PointCloud conversion failed: {exc}'
            )
            return

        # Empty input means target invalid / LOST / REACQUIRING.
        if points_base.shape[0] == 0:
            out = self.xyz_to_pointcloud2(
                np.empty(
                    (0, 3),
                    dtype=np.float32
                ),
                msg.header.stamp,
                self.target_frame
            )

            self.pub.publish(out)
            return

        try:
            stamp = Time.from_msg(
                msg.header.stamp
            )

            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                stamp
            )

        except TransformException as exc:
            now = self.get_clock().now()

            if (
                now.nanoseconds
                - self.last_warning_time
                > 1_000_000_000
            ):
                self.last_warning_time = (
                    now.nanoseconds
                )

                self.get_logger().warn(
                    f'TF lookup failed '
                    f'{self.target_frame} <- '
                    f'{msg.header.frame_id}: {exc}'
                )

            return

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        R = self.quaternion_to_matrix(q)

        translation = np.array(
            [t.x, t.y, t.z],
            dtype=np.float64
        )

        points_world = (
            points_base.astype(
                np.float64
            )
            @ R.T
            + translation
        ).astype(
            np.float32
        )

        out = self.xyz_to_pointcloud2(
            points_world,
            msg.header.stamp,
            self.target_frame
        )

        self.pub.publish(out)

        self.frame_count += 1

        if self.frame_count % 10 == 0:
            centroid = np.mean(
                points_world,
                axis=0
            )

            self.get_logger().info(
                '[WORLD CLOUD] '
                f'n={points_world.shape[0]}, '
                f'centroid=['
                f'{centroid[0]:.3f}, '
                f'{centroid[1]:.3f}, '
                f'{centroid[2]:.3f}]'
            )


def main(args=None):
    rclpy.init(args=args)

    node = TargetCloudWorld()

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
