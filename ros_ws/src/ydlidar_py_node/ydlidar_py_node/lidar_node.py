import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import ydlidar
import numpy as np

class YDLidarNode(Node):
    def __init__(self):
        super().__init__('ydlidar_node')

        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        self.scan = ydlidar.LaserScan()

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
        self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

        if not self.laser.initialize():
            self.get_logger().error("Lidar init failed")
            return

        if not self.laser.turnOn():
            self.get_logger().error("Lidar turnOn failed")
            return

        self.get_logger().info("YDLIDAR started")

        self.timer = self.create_timer(0.05, self.update)  # 20 Hz

    def update(self):
        if not self.laser.doProcessSimple(self.scan):
            return

        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"

        msg.angle_min = -3.14159
        msg.angle_max = 3.14159
        msg.angle_increment = 0.0  # optional (RViz can still plot)
        msg.range_min = 0.08
        msg.range_max = 16.0

        ranges = []

        for p in self.scan.points:
            r = float(p.range)
            if r < 0.08 or r > 16.0:
                r = float('inf')
            ranges.append(r)

        msg.ranges = ranges

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = YDLidarNode()
    rclpy.spin(node)

    node.laser.turnOff()
    node.laser.disconnecting()

    node.destroy_node()
    rclpy.shutdown()