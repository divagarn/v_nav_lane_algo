#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

from buggy_uart.uart_interface import UartInterface


class UartSender(Node):
    def __init__(self):
        super().__init__('uart_sender')
        
        # Declare and get parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('loopback', False)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        loopback = self.get_parameter('loopback').value
        
        # Initialize variables
        self.yellow_detected = False
        self.red_detected = True
        self.degree = 0.0
        # DMS alert level (0-4, critical=3)
        self.dms_alert_level = 0.0
        
        # Initialize UART interface (pass ROS logger)
        self.uart = UartInterface(port, baudrate, loopback, ros_logger=self.get_logger())
        
        # Initialize subscribers
        self.create_subscription(Float32, '/steering_angle', self.steering_callback, 1)
        self.create_subscription(Bool, '/final_yellow_detected', self.yellow_callback, 1)
        self.create_subscription(Bool, '/final_red_detected', self.red_callback, 1)
        self.create_subscription(Float32, '/dms/alert_level', self.alert_level_callback, 1)

        # Track last received times
        self.last_steering_time = self.get_clock().now()
        self.last_yellow_time = self.get_clock().now()
        self.last_red_time = self.get_clock().now()
        
        # Send message timer (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("UART Sender node initialized")
        self.get_logger().info(f"Port: {port}, Baudrate: {baudrate}, Loopback: {loopback}")
    
    def _secs_since(self, stamp):
        """Helper to get seconds elapsed since a timestamp"""
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def steering_callback(self, msg):
        """Callback for steering angle updates"""
        self.degree = msg.data
        self.last_steering_time = self.get_clock().now()

    def yellow_callback(self, msg):
        """Callback for yellow detection updates"""
        self.yellow_detected = msg.data
        self.last_yellow_time = self.get_clock().now()

    def red_callback(self, msg):
        """Callback for red detection updates"""
        self.red_detected = msg.data
        self.last_red_time = self.get_clock().now()

    def alert_level_callback(self, msg):
        """Callback for DMS alert level updates (std_msgs/Float32)"""
        try:
            self.dms_alert_level = float(msg.data)
            self.get_logger().debug(f"[UART] Received DMS alert_level: {self.dms_alert_level}")
        except Exception as e:
            self.get_logger().warn(f"[UART] Invalid alert_level message: {e}")

    def timer_callback(self):
        """Timer callback to send UART message at regular intervals"""
        # Timeout logic: if no data for >1s, force red and steer 0
        steer_timeout = self._secs_since(self.last_steering_time) > 1.0
        yellow_timeout = self._secs_since(self.last_yellow_time) > 1.0
        red_timeout = self._secs_since(self.last_red_time) > 1.0
        #self.get_logger().warn(f"[UART] {self.degree}, {self.yellow_detected}, {self.red_detected}")

        # if steer_timeout or yellow_timeout or red_timeout:
        #     self.red_detected = True
        #     self.degree = 0.0
        #     self.get_logger().warn("[UART] No data from topic(s), forcing STOP: red=True, steering=0.0")
        self.get_logger().info(f"Sending UART message - yellow: {self.yellow_detected}, red: {self.red_detected}, angle: {self.degree}")

        self.uart.send(self.yellow_detected, self.red_detected, self.degree)
    
    def shutdown(self):
        """Clean shutdown of the node - sends one final STOP command
        (yellow=0, red=1, degree=0) before closing the serial port, so
        Ctrl+C (or any other shutdown path - both go through here, see
        main()'s try/finally) leaves the real vehicle explicitly commanded
        to stop rather than whichever command happened to be in flight
        last. Without this, killing this node just stops sending anything
        further - the vehicle keeps acting on its last-received command
        until/unless the firmware has its own communication-timeout
        failsafe, which this does not assume."""
        try:
            self.get_logger().warn("Sending final STOP command (red=1) before shutdown")
            self.uart.send(False, True, 0.0)
        except Exception as e:
            self.get_logger().error(f"Failed to send final STOP command: {e}")
        self.uart.close()
        self.get_logger().info("UART Sender node shutting down")


def main(args=None):
    rclpy.init(args=args)
    node = UartSender()
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