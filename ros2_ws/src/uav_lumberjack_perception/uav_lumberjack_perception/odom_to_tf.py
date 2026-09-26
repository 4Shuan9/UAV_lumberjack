import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):

    def __init__(self):
        super().__init__('odom_to_tf')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            '/model/x500_lumberjack/base_link_odometry',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Odometry -> TF started: world -> base_link'
        )

    def odom_callback(self, msg):
        tf_msg = TransformStamped()

        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = (
            msg.header.frame_id
            if msg.header.frame_id
            else 'world'
        )

        tf_msg.child_frame_id = (
            msg.child_frame_id
            if msg.child_frame_id
            else 'base_link'
        )

        tf_msg.transform.translation.x = (
            msg.pose.pose.position.x
        )
        tf_msg.transform.translation.y = (
            msg.pose.pose.position.y
        )
        tf_msg.transform.translation.z = (
            msg.pose.pose.position.z
        )

        tf_msg.transform.rotation = (
            msg.pose.pose.orientation
        )

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)

    node = OdomToTf()

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
