import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image


class RedBranchDetector(Node):
    """
    Step13.2.1
    Robust red-branch Mask under viewpoint / illumination changes.

    Main improvements over Step13.2:
      1) Narrow HSV red hue to reject brown trunk / branches.
      2) Keep high saturation but allow lower V for shadowed true-red pixels.
      3) Small OPEN + stronger CLOSE morphology:
         remove isolated noise while reconnecting thin broken branch regions.
      4) Publish ONLY the largest valid target component as /perception/red_mask.
      5) Lower minimum contour area so oblique / farther views are not rejected
         too aggressively.

    cv_bridge is NOT used.
    """

    def __init__(self):
        super().__init__('red_branch_detector')

        # ========================================================
        # ROS interfaces
        # ========================================================

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Final target-only binary Mask used by RGB-LiDAR fusion.
        self.mask_pub = self.create_publisher(
            Image,
            '/perception/red_mask',
            10
        )

        # Optional raw candidate Mask for tuning / comparison.
        self.raw_mask_pub = self.create_publisher(
            Image,
            '/perception/red_mask_raw',
            10
        )

        # Debug RGB image.
        self.debug_pub = self.create_publisher(
            Image,
            '/perception/red_debug',
            10
        )

        # ========================================================
        # Detection parameters
        # ========================================================

        self.frame_count = 0
        self.encoding_printed = False

        # Previous Step13.2 used 300 px.
        # Lower this so oblique / farther views remain detectable.
        self.min_area = 80.0

        # HSV red hue ranges.
        # OpenCV H range is [0, 179].
        self.hue_low_1 = 0
        self.hue_high_1 = 8

        self.hue_low_2 = 172
        self.hue_high_2 = 179

        # Relaxed saturation / brightness gate.
        # The old detector used S >= 100 and V >= 100.
        self.min_saturation = 110
        self.min_value = 30

        # Morphology:
        # small OPEN preserves a thin branch;
        # larger CLOSE reconnects small gaps caused by illumination/view angle.
        self.open_kernel_size = 3
        self.close_kernel_size = 7

        self.get_logger().info('====================================================')
        self.get_logger().info(' Step13.2.1 Robust Red Branch Detector')
        self.get_logger().info(' Input : /camera/image_raw')
        self.get_logger().info(' Final : /perception/red_mask')
        self.get_logger().info(' Raw   : /perception/red_mask_raw')
        self.get_logger().info(' Debug : /perception/red_debug')
        self.get_logger().info(
            f' HSV: H=[{self.hue_low_1},{self.hue_high_1}] or '
            f'[{self.hue_low_2},{self.hue_high_2}], '
            f'S>={self.min_saturation}, V>={self.min_value}'
        )
        self.get_logger().info(
            f' min_area={self.min_area:.0f} px, '
            f'OPEN={self.open_kernel_size}, '
            f'CLOSE={self.close_kernel_size}'
        )
        self.get_logger().info(' cv_bridge is NOT used')
        self.get_logger().info('====================================================')

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

        expected_size = (
            msg.height
            * msg.step
        )

        if raw.size < expected_size:
            raise ValueError(
                f'Image data too small: '
                f'{raw.size} < {expected_size}'
            )

        rows = raw[:expected_size].reshape(
            msg.height,
            msg.step
        )

        useful_bytes = (
            msg.width
            * channels
        )

        image = rows[
            :,
            :useful_bytes
        ]

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

    @staticmethod
    def numpy_to_ros_image(
        image,
        header,
        encoding
    ):
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

        msg.step = (
            msg.width
            * channels
        )

        msg.data = np.ascontiguousarray(
            image
        ).tobytes()

        return msg

    # ============================================================
    # Robust red candidate Mask
    # ============================================================

    def build_raw_red_mask(self, bgr):
        """
        Strict HSV-only red candidate mask.

        Design goal:
          - Reject brown trunk / brown branches.
          - Keep true saturated red even when it becomes darker in shadow.

        Therefore:
          - Hue is narrow.
          - Saturation remains high.
          - Value threshold is allowed to be low.
        """

        hsv = cv2.cvtColor(
            bgr,
            cv2.COLOR_BGR2HSV
        )

        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]

        hue_red = (
            (
                (h >= self.hue_low_1)
                & (h <= self.hue_high_1)
            )
            |
            (
                (h >= self.hue_low_2)
                & (h <= self.hue_high_2)
            )
        )

        red_mask = (
            hue_red
            & (s >= self.min_saturation)
            & (v >= self.min_value)
        )

        raw_mask = np.where(
            red_mask,
            255,
            0
        ).astype(
            np.uint8
        )

        return raw_mask

    # ============================================================
    # Morphology + largest valid component
    # ============================================================

    def clean_and_select_target(
        self,
        raw_mask
    ):
        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (
                self.open_kernel_size,
                self.open_kernel_size
            )
        )

        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (
                self.close_kernel_size,
                self.close_kernel_size
            )
        )

        clean = cv2.morphologyEx(
            raw_mask,
            cv2.MORPH_OPEN,
            open_kernel
        )

        clean = cv2.morphologyEx(
            clean,
            cv2.MORPH_CLOSE,
            close_kernel
        )

        contours, _ = cv2.findContours(
            clean,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        target_mask = np.zeros_like(
            clean
        )

        result = {
            'detected': False,
            'contour': None,
            'area': 0.0,
            'bbox': None,
            'center': (-1, -1),
            'target_pixels': 0,
        }

        if not contours:
            return target_mask, result

        largest = max(
            contours,
            key=cv2.contourArea
        )

        area = float(
            cv2.contourArea(
                largest
            )
        )

        if area < self.min_area:
            return target_mask, result

        # IMPORTANT:
        # The final published Mask contains ONLY the chosen target component.
        cv2.drawContours(
            target_mask,
            [largest],
            -1,
            255,
            thickness=cv2.FILLED
        )

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
            center_x = (
                x
                + w // 2
            )

            center_y = (
                y
                + h // 2
            )

        result.update({
            'detected': True,
            'contour': largest,
            'area': area,
            'bbox': (x, y, w, h),
            'center': (
                center_x,
                center_y
            ),
            'target_pixels': int(
                cv2.countNonZero(
                    target_mask
                )
            ),
        })

        return (
            target_mask,
            result
        )

    # ============================================================
    # Callback
    # ============================================================

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
            bgr = self.ros_image_to_bgr(
                msg
            )

        except Exception as exc:
            self.get_logger().error(
                f'Image conversion failed: {exc}'
            )
            return

        # --------------------------------------------------------
        # 1) Robust raw red candidate Mask
        # --------------------------------------------------------

        raw_mask = self.build_raw_red_mask(
            bgr
        )

        raw_pixels = int(
            cv2.countNonZero(
                raw_mask
            )
        )

        # --------------------------------------------------------
        # 2) Clean + largest valid target component
        # --------------------------------------------------------

        (
            target_mask,
            result
        ) = self.clean_and_select_target(
            raw_mask
        )

        # --------------------------------------------------------
        # 3) Debug visualization
        # --------------------------------------------------------

        debug = bgr.copy()

        if result['detected']:
            contour = result['contour']
            area = result['area']
            x, y, w, h = result['bbox']
            center_x, center_y = result['center']
            target_pixels = result['target_pixels']

            cv2.drawContours(
                debug,
                [contour],
                -1,
                (0, 255, 0),
                2
            )

            cv2.rectangle(
                debug,
                (x, y),
                (
                    x + w,
                    y + h
                ),
                (0, 255, 255),
                2
            )

            cv2.circle(
                debug,
                (
                    center_x,
                    center_y
                ),
                6,
                (255, 0, 0),
                -1
            )

            cv2.putText(
                debug,
                'TARGET BRANCH',
                (
                    x,
                    max(
                        y - 32,
                        20
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2
            )

            cv2.putText(
                debug,
                (
                    f'area={area:.0f}px '
                    f'mask={target_pixels}px'
                ),
                (
                    x,
                    max(
                        y - 10,
                        40
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 255, 255),
                1
            )

        else:
            cv2.putText(
                debug,
                'TARGET NOT VISIBLE',
                (30, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        # --------------------------------------------------------
        # 4) Periodic diagnostic log
        # --------------------------------------------------------

        if self.frame_count % 30 == 0:
            if result['detected']:
                center_x, center_y = result[
                    'center'
                ]

                self.get_logger().info(
                    '[MASK OK] '
                    f'center=({center_x},{center_y}), '
                    f'contour_area={result["area"]:.0f}px, '
                    f'raw_pixels={raw_pixels}, '
                    f'target_pixels={result["target_pixels"]}'
                )

            else:
                self.get_logger().info(
                    '[MASK LOST] '
                    f'raw_pixels={raw_pixels}, '
                    f'largest contour < {self.min_area:.0f}px '
                    'or no red component'
                )

        # --------------------------------------------------------
        # 5) Publish
        # --------------------------------------------------------

        self.raw_mask_pub.publish(
            self.numpy_to_ros_image(
                raw_mask,
                msg.header,
                'mono8'
            )
        )

        self.mask_pub.publish(
            self.numpy_to_ros_image(
                target_mask,
                msg.header,
                'mono8'
            )
        )

        self.debug_pub.publish(
            self.numpy_to_ros_image(
                debug,
                msg.header,
                'bgr8'
            )
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
