#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')
        self.publisher_ = self.create_publisher(String, 'wheels', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.wheel_width = 1
        self.wheel_radius = .5

    def kinematics(self, linear_x, angular_z):
        right_wheel = (linear_x + angular_z * (self.wheel_width/2)) / self.wheel_radius
        left_wheel = (linear_x - angular_z * (self.wheel_width/2)) / self.wheel_radius
        return(left_wheel,right_wheel)
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        msgout = String()
        left, right = self.kinematics(msg.linear.x, msg.angular.z)
        msgout.data = f"{left},{right}"
        self.publisher_.publish(msgout)
        


def main(args=None):
    rclpy.init(args=args)

    diff_drive_controller = DiffDriveController()

    rclpy.spin(diff_drive_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    diff_drive_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()