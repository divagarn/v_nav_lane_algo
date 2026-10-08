#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from ground_segmentation_msgs.msg import SteerSpeed

# Constants
START_BYTE = 0x28  # '(' in hex
END_BYTE = 0x29    # ')' in hex
PACKET_SIZE = 6    # total bytes per packet
OFFSET = -30
RESOLUTION = 0.01
DEGREE_RANGE = (-20, 20)


class UartBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # Open serial port
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as e:
            self.get_logger().fatal(f"UART open failed: {e}")
            raise SystemExit

        self.get_logger().info(f"UART bridge: {port} @ {baud} baud")
        self.buf = bytearray()  # rolling binary buffer

        # Publisher
        self.pub = self.create_publisher(SteerSpeed, '/wheel_uart', 50)

        # Timer to poll serial at ~100 Hz (fast read), publish as packets arrive
        self.timer = self.create_timer(0.01, self.timer_callback)

    def parse_packet(self, data):
        """
        Parse a 6-byte UART packet and extract steering (degrees) and speed.
        """
        if len(data) != PACKET_SIZE:
            self.get_logger().warn(
                f"[UART] Invalid packet size: {len(data)} bytes (expected {PACKET_SIZE})")
            return None

        # Check start and end bytes
        if data[0] != START_BYTE or data[5] != END_BYTE:
            self.get_logger().warn(
                f"[UART] Invalid packet markers: start={hex(data[0])} end={hex(data[5])} "
                f"(expected start={hex(START_BYTE)}, end={hex(END_BYTE)})")
            return None

        # Debug raw packet
        hex_data = ' '.join([f"{b:02x}" for b in data])
        self.get_logger().debug(f"[UART] Raw packet: {hex_data}")

        # Decode steering: bytes 1-2 are 16-bit signed integer (big-endian)
        steering_raw = int.from_bytes(data[1:3], byteorder='big', signed=True)
        steering_deg = steering_raw * RESOLUTION + OFFSET
        steering_deg = max(min(steering_deg, DEGREE_RANGE[1]), DEGREE_RANGE[0])
        steering_rad = steering_deg

        # Decode speed: byte 3
        speed = data[3]

        self.get_logger().info(
            f"[UART] decoded: raw={steering_raw}, angle={steering_deg:.2f}°, "
            f"rad={steering_rad:.4f}, speed={speed}")
        return steering_rad, speed

    @staticmethod
    def find_packet_start(data):
        """Find the index of the first start byte in the buffer."""
        for i in range(len(data)):
            if data[i] == START_BYTE:
                return i
        return -1

    def timer_callback(self):
        """Timer callback to read serial data and publish parsed packets."""
        # Read whatever arrived this cycle
        try:
            new_data = self.ser.read(128)
            if new_data:
                hex_data = ' '.join([f"{b:02x}" for b in new_data])
                self.get_logger().info(f"Receiving UART data: {hex_data}")
                self.buf.extend(new_data)
            else:
                self.get_logger().debug("No new UART data received this cycle.")
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")
            self.buf = bytearray()
            return

        # Process all complete packets in buffer
        while len(self.buf) >= PACKET_SIZE:
            # Find start byte
            start_idx = self.find_packet_start(self.buf)

            # If no start byte found, clear buffer
            if start_idx == -1:
                self.buf = bytearray()
                break

            # Discard data before start byte
            if start_idx > 0:
                self.buf = self.buf[start_idx:]
                continue

            # Check if we have a complete packet
            if len(self.buf) >= PACKET_SIZE:
                packet = bytes(self.buf[:PACKET_SIZE])
                result = self.parse_packet(packet)

                if result:
                    steering, speed = result
                    msg = SteerSpeed()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.steering = float(steering)
                    msg.speed = float(speed)
                    self.pub.publish(msg)
                    self.get_logger().debug(
                        f"Published: steering={steering:.4f} rad, speed={speed:.2f}")

                # Remove processed packet from buffer
                self.buf = self.buf[PACKET_SIZE:]
            else:
                break

    def shutdown(self):
        """Clean shutdown."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)
    node = UartBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
