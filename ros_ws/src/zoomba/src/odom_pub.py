#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)


    def timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdomPublisher()

    rclpy.spin(odom_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()