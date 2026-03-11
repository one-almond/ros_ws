#!/usr/bin/env python3
"""
HLS-LFCD2 LiDAR driver node for ROS2 Humble.

Reads the 42-byte packets directly from the LiDAR UART and publishes
a sensor_msgs/LaserScan on /scan.

Packet format (42 bytes per packet, 60 packets per 360° scan):
  Byte 0      : 0xFA  sync
  Byte 1      : index 0xA0–0xDB  (packet 0–59)
  Bytes 2–3   : RPM (uint16 LE)
  Bytes 4–39  : 6 measurements × 6 bytes
                  [inten_L, inten_H, dist_L, dist_H, res, res]
  Bytes 40–41 : checksum  (0xFF - (sum(bytes 0–39) & 0xFF))

Angle formula : angle° = (index - 0xA0) * 6 + measurement_offset
"""

import math
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan


# ── Packet constants ──────────────────────────────────────────────────────────
START_BYTE      = 0xFA
FIRST_INDEX     = 0xA0
LAST_INDEX      = 0xDB
PACKET_LEN      = 42
MEAS_PER_PACKET = 6
TOTAL_ANGLES    = 360          # 60 packets × 6 measurements
MAX_RANGE_M     = 5.0          # sensor spec
MIN_RANGE_M     = 0.12         # sensor spec


class LfcdNode(Node):

    def __init__(self):
        super().__init__('lfcd_driver')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('port',       '/dev/ttyAMA0')
        self.declare_parameter('baud_rate',  230400)
        self.declare_parameter('frame_id',   'laser')
        self.declare_parameter('scan_topic', '/scan')

        port      = self.get_parameter('port').value
        baud      = self.get_parameter('baud_rate').value
        frame_id  = self.get_parameter('frame_id').value
        topic     = self.get_parameter('scan_topic').value

        # ── Publisher (sensor QoS — best effort, keep last 1) ─────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(LaserScan, topic, qos)

        # ── Serial port ───────────────────────────────────────────────────────
        self.get_logger().info(f'Opening {port} at {baud} baud...')
        try:
            self.serial = serial.Serial(port, baud, timeout=1.0)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port: {e}')
            raise SystemExit(1)

        # ── Scan buffer ───────────────────────────────────────────────────────
        self.distances  = [0.0] * TOTAL_ANGLES   # metres
        self.intensities = [0.0] * TOTAL_ANGLES
        self.frame_id   = frame_id

        # ── Receive state machine ─────────────────────────────────────────────
        self.buf    = bytearray(PACKET_LEN)
        self.bufidx = 0
        self.synced = False
        self.error_count = 0
        self.scan_count  = 0

        # ── Timer — read serial in the ROS2 event loop ────────────────────────
        self.create_timer(0.001, self.read_serial)   # 1 ms poll

        self.get_logger().info(
            f'HLS-LFCD2 driver started → publishing on {topic}'
        )

    # ── Serial reader / state machine ─────────────────────────────────────────
    def read_serial(self):
        if not self.serial.is_open:
            return

        waiting = self.serial.in_waiting
        if waiting == 0:
            return

        data = self.serial.read(min(waiting, 256))

        for byte in data:
            if not self.synced:
                if byte == START_BYTE:
                    self.buf[0]  = byte
                    self.bufidx  = 1
                    self.synced  = True
                continue

            self.buf[self.bufidx] = byte
            self.bufidx += 1

            if self.bufidx == PACKET_LEN:
                if (self.buf[0] == START_BYTE and
                        FIRST_INDEX <= self.buf[1] <= LAST_INDEX):
                    self._decode_packet()
                else:
                    self.error_count += 1
                self.bufidx = 0
                self.synced = False

    # ── Packet decoder ─────────────────────────────────────────────────────────
    def _decode_packet(self):
        if not self._checksum_ok():
            self.error_count += 1
            return

        pkt_index = self.buf[1] - FIRST_INDEX            # 0–59
        # rpm = self.buf[2] | (self.buf[3] << 8)         # available if needed

        for i in range(MEAS_PER_PACKET):
            base  = 4 + i * 6
            inten = self.buf[base]     | (self.buf[base + 1] << 8)
            dist  = self.buf[base + 2] | (self.buf[base + 3] << 8)

            angle_idx = pkt_index * MEAS_PER_PACKET + i  # 0–359

            dist_m = dist / 1000.0
            if dist > 0 and MIN_RANGE_M <= dist_m <= MAX_RANGE_M:
                self.distances[angle_idx]   = dist_m
                self.intensities[angle_idx] = float(inten)
            else:
                self.distances[angle_idx]   = 0.0
                self.intensities[angle_idx] = 0.0

        # Publish once we have a complete 360° scan
        if pkt_index == (LAST_INDEX - FIRST_INDEX):
            self._publish_scan()

    # ── Checksum ───────────────────────────────────────────────────────────────
    def _checksum_ok(self):
        cs = 0xFF - (sum(self.buf[:40]) & 0xFF)
        return self.buf[40] == cs or self.buf[41] == cs

    # ── Build and publish LaserScan ────────────────────────────────────────────
    def _publish_scan(self):
        self.scan_count += 1
        now = self.get_clock().now().to_msg()

        msg = LaserScan()
        msg.header.stamp    = now
        msg.header.frame_id = self.frame_id

        # The HLS-LFCD2 scans counter-clockwise; index 0 = 0°
        msg.angle_min       =  0.0
        msg.angle_max       =  2.0 * math.pi
        msg.angle_increment =  2.0 * math.pi / TOTAL_ANGLES   # 1° in radians
        msg.time_increment  =  0.0
        msg.scan_time       =  1.0 / 5.0   # ~5 Hz nominal
        msg.range_min       =  MIN_RANGE_M
        msg.range_max       =  MAX_RANGE_M

        msg.ranges      = list(self.distances)
        msg.intensities = list(self.intensities)

        self.pub.publish(msg)

        if self.scan_count % 50 == 0:
            valid = sum(1 for r in self.distances if r > 0)
            self.get_logger().info(
                f'Scan #{self.scan_count}  valid={valid}/360  '
                f'crc_errors={self.error_count}'
            )

    # ── Cleanup ────────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self.serial.is_open:
            self.serial.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()


# ── Entry point ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = LfcdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
