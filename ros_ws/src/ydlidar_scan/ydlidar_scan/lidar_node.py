import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import ydlidar
import numpy as np


class YDLidarNode(Node):
    def __init__(self):
        super().__init__('ydlidar_node')

        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        ydlidar.os_init()

        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, "/dev/ttyUSB0")
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 5)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, 0.08)

        self.scan = ydlidar.LaserScan()

        if not self.laser.initialize():
            self.get_logger().error("Failed to initialize lidar")
            return

        if not self.laser.turnOn():
            self.get_logger().error("Failed to turn on lidar")
            return

        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info("YDLIDAR ROS node started")

    def update(self):
        if not self.laser.doProcessSimple(self.scan):
            return

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"

        angles = np.array([p.angle for p in self.scan.points])
        ranges = np.array([p.range for p in self.scan.points])

        msg.angle_min = float(np.min(angles))
        msg.angle_max = float(np.max(angles))
        msg.angle_increment = float((msg.angle_max - msg.angle_min) / len(angles))
        msg.range_min = 0.08
        msg.range_max = 16.0

        msg.ranges = ranges.tolist()

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = YDLidarNode()
    rclpy.spin(node)

    node.laser.turnOff()
    node.laser.disconnecting()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
