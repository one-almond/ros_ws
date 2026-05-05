import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import threading


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        # =====================
        # SERIAL SETUP
        # =====================
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

        # =====================
        # ROS TOPICS
        # =====================
        self.wheel_sub = self.create_subscription(
            String,
            '/wheels',
            self.wheel_callback,
            10
        )

        # self.state_pub = self.create_publisher(
        #     JointState,
        #     '/joint_states',
        #     10
        # )

        # =====================
        # SERIAL THREAD
        # =====================
        self.running = True
        self.thread = threading.Thread(target=self.read_serial)
        self.thread.start()

        self.get_logger().info("Serial bridge started")

    # =====================
    # ROS → Arduino
    # =====================
    def wheel_callback(self, msg: String):

        l,r = msg.data.split(",")

        cmd = f"DRIVE L{l} R{r} \n"
        #self.get_logger().info(f"Sending: {cmd}")
        self.ser.write(cmd.encode())

    # =====================
    # Arduino → ROS
    # =====================
    def read_serial(self):
        while self.running:
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                self.get_logger().info(f"Arduino: {line}")


def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.running = False
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()