import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image


class RedBranchDetector(Node):

    def __init__(self):
        super().__init__('red_branch_detector')

        # 输入图像
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # 红色二值 Mask
        self.mask_pub = self.create_publisher(
            Image,
            '/perception/red_mask',
            10
        )

        # 调试图像：轮廓 + 外接框 + 中心点
        self.debug_pub = self.create_publisher(
            Image,
            '/perception/red_debug',
            10
        )

        self.frame_count = 0
        self.encoding_printed = False

        # 最小目标面积，过滤零碎红色噪声
        self.min_area = 300.0

        self.get_logger().info('======================================')
        self.get_logger().info(' Step13.2 Red Branch Detector started')
        self.get_logger().info(' Input : /camera/image_raw')
        self.get_logger().info(' Output: /perception/red_mask')
        self.get_logger().info(' Output: /perception/red_debug')
        self.get_logger().info(' cv_bridge is NOT used')
        self.get_logger().info('======================================')

    def ros_image_to_bgr(self, msg):
        """
        不使用 cv_bridge。
        直接将 sensor_msgs/Image.data 转换为 NumPy/OpenCV 图像。
        """

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

        raw = np.frombuffer(msg.data, dtype=np.uint8)

        # msg.step 是每一行实际占用的字节数
        expected_size = msg.height * msg.step

        if raw.size < expected_size:
            raise ValueError(
                f'Image data too small: '
                f'{raw.size} < {expected_size}'
            )

        # 先按照真实行步长排列，避免存在 padding 时出错
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

            bgr = cv2.cvtColor(
                image,
                cv2.COLOR_GRAY2BGR
            )

        else:
            image = image.reshape(
                msg.height,
                msg.width,
                channels
            )

            if encoding == 'rgb8':
                bgr = cv2.cvtColor(
                    image,
                    cv2.COLOR_RGB2BGR
                )

            elif encoding == 'bgr8':
                bgr = image.copy()

            elif encoding == 'rgba8':
                bgr = cv2.cvtColor(
                    image,
                    cv2.COLOR_RGBA2BGR
                )

            elif encoding == 'bgra8':
                bgr = cv2.cvtColor(
                    image,
                    cv2.COLOR_BGRA2BGR
                )

        return bgr

    def numpy_to_ros_image(self, image, header, encoding):
        """
        NumPy/OpenCV 图像 → sensor_msgs/Image
        同样不依赖 cv_bridge。
        """

        msg = Image()

        msg.header = header
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0

        if image.ndim == 2:
            channels = 1
        else:
            channels = image.shape[2]

        msg.step = msg.width * channels
        msg.data = image.tobytes()

        return msg

    def image_callback(self, msg):

        self.frame_count += 1

        if not self.encoding_printed:
            self.encoding_printed = True

            self.get_logger().info(
                f'Camera image: '
                f'{msg.width}x{msg.height}, '
                f'encoding="{msg.encoding}", '
                f'step={msg.step}'
            )

        try:
            bgr = self.ros_image_to_bgr(msg)

        except Exception as e:
            self.get_logger().error(
                f'Image conversion failed: {e}'
            )
            return

        # -------------------------------------------------
        # 1. BGR → HSV
        # -------------------------------------------------

        hsv = cv2.cvtColor(
            bgr,
            cv2.COLOR_BGR2HSV
        )

        # -------------------------------------------------
        # 2. 红色有两个 Hue 区域
        #
        # 保留你原来红球识别使用的基本方案
        # -------------------------------------------------

        lower_red_1 = np.array(
            [0, 100, 100],
            dtype=np.uint8
        )

        upper_red_1 = np.array(
            [10, 255, 255],
            dtype=np.uint8
        )

        lower_red_2 = np.array(
            [160, 100, 100],
            dtype=np.uint8
        )

        upper_red_2 = np.array(
            [180, 255, 255],
            dtype=np.uint8
        )

        mask1 = cv2.inRange(
            hsv,
            lower_red_1,
            upper_red_1
        )

        mask2 = cv2.inRange(
            hsv,
            lower_red_2,
            upper_red_2
        )

        mask = cv2.bitwise_or(
            mask1,
            mask2
        )

        # -------------------------------------------------
        # 3. 简单形态学去噪
        # -------------------------------------------------

        kernel = np.ones(
            (5, 5),
            dtype=np.uint8
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # -------------------------------------------------
        # 4. 提取轮廓
        # -------------------------------------------------

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        debug = bgr.copy()

        detected = False
        target_area = 0.0
        center_x = -1
        center_y = -1

        if contours:

            # 取最大的红色区域
            largest = max(
                contours,
                key=cv2.contourArea
            )

            target_area = cv2.contourArea(
                largest
            )

            if target_area >= self.min_area:

                detected = True

                x, y, w, h = cv2.boundingRect(
                    largest
                )

                moments = cv2.moments(
                    largest
                )

                if moments['m00'] != 0:
                    center_x = int(
                        moments['m10']
                        / moments['m00']
                    )

                    center_y = int(
                        moments['m01']
                        / moments['m00']
                    )
                else:
                    center_x = x + w // 2
                    center_y = y + h // 2

                # 轮廓
                cv2.drawContours(
                    debug,
                    [largest],
                    -1,
                    (0, 255, 0),
                    2
                )

                # 外接框
                cv2.rectangle(
                    debug,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 255),
                    2
                )

                # 中心点
                cv2.circle(
                    debug,
                    (center_x, center_y),
                    6,
                    (255, 0, 0),
                    -1
                )

                cv2.putText(
                    debug,
                    'TARGET BRANCH',
                    (x, max(y - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

        # -------------------------------------------------
        # 5. 每约 30 帧打印一次状态
        # -------------------------------------------------

        if self.frame_count % 30 == 0:

            if detected:
                self.get_logger().info(
                    f'[DETECTED] '
                    f'center=({center_x}, {center_y}), '
                    f'area={target_area:.0f} px'
                )

            else:
                self.get_logger().info(
                    '[NO TARGET] red branch not detected'
                )

        # -------------------------------------------------
        # 6. 发布 Mask
        # -------------------------------------------------

        mask_msg = self.numpy_to_ros_image(
            mask,
            msg.header,
            'mono8'
        )

        self.mask_pub.publish(
            mask_msg
        )

        # -------------------------------------------------
        # 7. 发布调试图
        # -------------------------------------------------

        debug_msg = self.numpy_to_ros_image(
            debug,
            msg.header,
            'bgr8'
        )

        self.debug_pub.publish(
            debug_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = RedBranchDetector()

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
