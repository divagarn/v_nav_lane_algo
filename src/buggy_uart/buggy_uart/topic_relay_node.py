#!/usr/bin/env python3
"""
Topic relay node to bridge sensor outputs to UART control topics.

Maps:
  - /steering_angle → /final_steering_angle (from steering_calculator)
  - /zone_obstacle_status.red_zone → /final_red_detected (from obstacle_detector)
  - /zone_obstacle_status.yellow_zone → /final_yellow_detected (from obstacle_detector)
  - NOT(red OR yellow) → /final_green_detected (synthesized signal)

This ensures uart_sender_node receives the correct signal names from the
perceptual system (steering calculator and obstacle detector).
Green signal is published when neither red nor yellow obstacles are detected.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from ground_segmentation_msgs.msg import ZoneObstacleStatus


class TopicRelayNode(Node):
    def __init__(self):
        super().__init__('topic_relay_node')
        
        # Publishers for UART control signals
        self.final_steering_pub = self.create_publisher(Float32, '/final_steering_angle', 1)
        self.final_red_pub = self.create_publisher(Bool, '/final_red_detected', 1)
        self.final_yellow_pub = self.create_publisher(Bool, '/final_yellow_detected', 1)
        self.final_green_pub = self.create_publisher(Bool, '/final_green_detected', 1)
        
        # Subscribers to sensor outputs
        self.steering_sub = self.create_subscription(
            Float32,
            '/steering_angle',
            self.steering_callback,
            1
        )
        
        self.zone_status_sub = self.create_subscription(
            ZoneObstacleStatus,
            '/zone_obstacle_status',
            self.zone_status_callback,
            1
        )
        
        self.get_logger().info('Topic relay node initialized')
        self.get_logger().info('Bridging:')
        self.get_logger().info('  /steering_angle → /final_steering_angle')
        self.get_logger().info('  /zone_obstacle_status.red_zone → /final_red_detected')
        self.get_logger().info('  /zone_obstacle_status.yellow_zone → /final_yellow_detected')
        self.get_logger().info('  NOT(red OR yellow) → /final_green_detected')
    
    def steering_callback(self, msg):
        """Relay steering angle to final topic"""
        self.final_steering_pub.publish(msg)
    
    def zone_status_callback(self, msg):
        """Extract and relay red/yellow zone status from obstacle detector"""
        # Publish red zone detection
        red_msg = Bool()
        red_msg.data = msg.red_zone
        self.final_red_pub.publish(red_msg)
        
        # Publish yellow zone detection
        yellow_msg = Bool()
        yellow_msg.data = msg.yellow_zone
        self.final_yellow_pub.publish(yellow_msg)
        
        # Publish green signal: true if neither red nor yellow obstacles detected
        green_msg = Bool()
        green_msg.data = not (msg.red_zone or msg.yellow_zone)
        self.final_green_pub.publish(green_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TopicRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
