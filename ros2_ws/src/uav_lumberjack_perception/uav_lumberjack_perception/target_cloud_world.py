from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException


class TargetCloudWorld(Node):
    """
    Exact-time base_link -> world transform with a short retry queue.

    If the cloud arrives a few milliseconds before its matching TF,
    queue it briefly and retry. Never substitute a "latest" TF.
    """

    def __init__(self):
        super().__init__('target_cloud_world')

        self.target_frame = 'world'

        self.declare_parameter('tf_retry_period', 0.01)
        self.declare_parameter('tf_max_wait', 0.10)
        self.declare_parameter('max_pending_clouds', 3)

        self.tf_retry_period = float(
            self.get_parameter('tf_retry_period').value
        )
        self.tf_max_wait = float(
            self.get_parameter('tf_max_wait').value
        )
        self.max_pending_clouds = int(
            self.get_parameter('max_pending_clouds').value
        )

        if self.tf_retry_period <= 0.0:
            raise ValueError('tf_retry_period must be > 0')
        if self.tf_max_wait <= 0.0:
            raise ValueError('tf_max_wait must be > 0')
        if self.max_pending_clouds < 1:
            raise ValueError('max_pending_clouds must be >= 1')

        self.tf_max_wait_ns = int(self.tf_max_wait * 1.0e9)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        self.retry_timer = self.create_timer(
            self.tf_retry_period,
            self.retry_pending
        )

        self.pending_clouds = deque()
        self.processed_count = 0
        self.queued_count = 0
        self.dropped_count = 0

        self.last_drop_log_ns = None
        self.drop_log_period_ns = int(2.0e9)

        self.get_logger().info(
            '[TF]\n'
            '  status          : sync ready\n'
            '  mode            : exact-time queue\n'
            f'  retry           : {1000.0*self.tf_retry_period:.0f} ms\n'
            f'  max wait        : {1000.0*self.tf_max_wait:.0f} ms'
        )

    @staticmethod
    def pointcloud_xyz(msg):
        fields = {field.name: field for field in msg.fields}

        for name in ('x', 'y', 'z'):
            if name not in fields:
                raise ValueError(f'PointCloud2 has no "{name}" field')
            if fields[name].datatype != PointField.FLOAT32:
                raise ValueError(
                    f'PointCloud2 field "{name}" is not FLOAT32'
                )

        if msg.width == 0 or len(msg.data) == 0:
            return np.empty((0, 3), dtype=np.float32)

        endian = '>' if msg.is_bigendian else '<'

        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': [endian + 'f4', endian + 'f4', endian + 'f4'],
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

        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

        return np.column_stack(
            (x[finite], y[finite], z[finite])
        ).astype(np.float32, copy=False)

    @staticmethod
    def xyz_to_pointcloud2(points, stamp, frame_id):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.height = 1
        msg.width = int(points.shape[0])
        msg.is_bigendian = False
        msg.is_dense = True

        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1)
        ]

        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = np.ascontiguousarray(
            points, dtype=np.float32
        ).tobytes()

        return msg

    @staticmethod
    def quaternion_to_matrix(q):
        x, y, z, w = q.x, q.y, q.z, q.w

        norm = x*x + y*y + z*z + w*w
        if norm < 1.0e-12:
            return np.eye(3, dtype=np.float64)

        s = 2.0 / norm

        xx, yy, zz = x*x*s, y*y*s, z*z*s
        xy, xz, yz = x*y*s, x*z*s, y*z*s
        wx, wy, wz = w*x*s, w*y*s, w*z*s

        return np.array([
            [1.0-yy-zz, xy-wz,     xz+wy],
            [xy+wz,     1.0-xx-zz, yz-wx],
            [xz-wy,     yz+wx,     1.0-xx-yy]
        ], dtype=np.float64)

    def publish_empty(self, stamp):
        out = self.xyz_to_pointcloud2(
            np.empty((0, 3), dtype=np.float32),
            stamp,
            self.target_frame
        )
        self.pub.publish(out)

    def transform_and_publish(self, msg, points_base):
        try:
            stamp = Time.from_msg(msg.header.stamp)

            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                stamp
            )

        except TransformException as exc:
            return False, str(exc)

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        R = self.quaternion_to_matrix(q)
        translation = np.array([t.x, t.y, t.z], dtype=np.float64)

        points_world = (
            points_base.astype(np.float64) @ R.T
            + translation
        ).astype(np.float32)

        out = self.xyz_to_pointcloud2(
            points_world,
            msg.header.stamp,
            self.target_frame
        )
        self.pub.publish(out)

        self.processed_count += 1
        return True, ''

    def cloud_callback(self, msg):
        try:
            points_base = self.pointcloud_xyz(msg)

        except Exception as exc:
            self.get_logger().error(
                '[TF] cloud conversion failed | '
                f'{exc}'
            )
            return

        # Empty means LOST / REACQUIRING. Clear pending valid clouds
        # so stale target data cannot be published afterwards.
        if points_base.shape[0] == 0:
            self.pending_clouds.clear()
            self.publish_empty(msg.header.stamp)
            return

        success, error_text = self.transform_and_publish(
            msg,
            points_base
        )

        if success:
            return

        now_ns = int(self.get_clock().now().nanoseconds)

        if len(self.pending_clouds) >= self.max_pending_clouds:
            self.pending_clouds.popleft()
            self.dropped_count += 1

        self.pending_clouds.append({
            'msg': msg,
            'points': points_base,
            'enqueue_ns': now_ns,
            'last_error': error_text
        })

        self.queued_count += 1

    def retry_pending(self):
        if not self.pending_clouds:
            return

        now_ns = int(self.get_clock().now().nanoseconds)

        while self.pending_clouds:
            item = self.pending_clouds[0]

            success, error_text = self.transform_and_publish(
                item['msg'],
                item['points']
            )

            if success:
                self.pending_clouds.popleft()
                continue

            item['last_error'] = error_text
            waited_ns = now_ns - item['enqueue_ns']

            if waited_ns < 0:
                self.pending_clouds.clear()
                return

            if waited_ns < self.tf_max_wait_ns:
                break

            self.pending_clouds.popleft()
            self.dropped_count += 1
            self.log_tf_drop(waited_ns, error_text)

    def log_tf_drop(self, waited_ns, error_text):
        now_ns = int(self.get_clock().now().nanoseconds)

        should_log = (
            self.last_drop_log_ns is None
            or now_ns < self.last_drop_log_ns
            or (
                now_ns - self.last_drop_log_ns
            ) >= self.drop_log_period_ns
        )

        if not should_log:
            return

        self.last_drop_log_ns = now_ns

        self.get_logger().warn(
            '[TF]\n'
            '  event           : DROP\n'
            f'  waited          : {waited_ns/1.0e6:.0f} ms\n'
            f'  dropped total   : {self.dropped_count}\n'
            f'  error           : {error_text}'
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
