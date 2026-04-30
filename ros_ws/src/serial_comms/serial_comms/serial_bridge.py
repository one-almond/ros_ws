import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
        self.cmd_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self.command_callback,
            10
        )

        self.state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

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
    def command_callback(self, msg: JointState):

        if len(msg.position) < 3:
            return

        b = int(msg.position[0])
        s = int(msg.position[1])
        e = int(msg.position[2])

        cmd = f"MOVE B{b} S{s} E{e}\n"
        self.ser.write(cmd.encode())

    # =====================
    # Arduino → ROS
    # =====================
    def read_serial(self):

        while self.running:
            try:
                line = self.ser.readline().decode().strip()

                if line.startswith("STATE"):
                    parts = line.split()

                    b = int(parts[1][1:])
                    s = int(parts[2][1:])
                    e = int(parts[3][1:])

                    msg = JointState()
                    msg.name = ["base", "shoulder", "elbow"]
                    msg.position = [float(b), float(s), float(e)]

                    self.state_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(str(e))


def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.running = False
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()