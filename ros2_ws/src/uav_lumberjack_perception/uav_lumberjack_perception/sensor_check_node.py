import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2


class SensorCheckNode(Node):

    def __init__(self):
        super().__init__('sensor_check_node')

        self.camera_count = 0
        self.camera_info_count = 0
        self.lidar_count = 0

        self.camera_received = False
        self.camera_info_received = False
        self.lidar_received = False

        # RGB Camera
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            qos_profile_sensor_data
        )

        # Camera intrinsic parameters
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        # MID360 point cloud
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/mid360/points',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        # 每 2 秒打印一次状态
        self.timer = self.create_timer(2.0, self.print_status)

        self.get_logger().info('======================================')
        self.get_logger().info(' Step13 Sensor Check Node started')
        self.get_logger().info(' Waiting for Camera + CameraInfo + MID360')
        self.get_logger().info('======================================')


    def camera_callback(self, msg):

        self.camera_count += 1

        if not self.camera_received:
            self.camera_received = True

            self.get_logger().info(
                f'[OK] Camera received: '
                f'{msg.width}x{msg.height}, '
                f'frame="{msg.header.frame_id}"'
            )


    def camera_info_callback(self, msg):

        self.camera_info_count += 1

        if not self.camera_info_received:
            self.camera_info_received = True

            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]

            self.get_logger().info(
                f'[OK] CameraInfo received: '
                f'fx={fx:.2f}, fy={fy:.2f}, '
                f'cx={cx:.2f}, cy={cy:.2f}, '
                f'frame="{msg.header.frame_id}"'
            )


    def lidar_callback(self, msg):

        self.lidar_count += 1

        if not self.lidar_received:
            self.lidar_received = True

            self.get_logger().info(
                f'[OK] MID360 received: '
                f'width={msg.width}, height={msg.height}, '
                f'frame="{msg.header.frame_id}"'
            )


    def print_status(self):

        camera_status = 'OK' if self.camera_received else 'WAIT'
        info_status = 'OK' if self.camera_info_received else 'WAIT'
        lidar_status = 'OK' if self.lidar_received else 'WAIT'

        self.get_logger().info(
            f'Camera [{camera_status}] {self.camera_count} | '
            f'CameraInfo [{info_status}] {self.camera_info_count} | '
            f'MID360 [{lidar_status}] {self.lidar_count}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = SensorCheckNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
