#!/usr/bin/env python3
"""Ported from cart_navigation_ws's cart_perception/steering_calculator_node.py
- pure-pursuit + PID steering calculator consuming freespace_grid_node's
/target_lateral_position, publishing /steering_angle + /vehicle_speed.
Only change from the original: SteerSpeed comes from this workspace's own
ground_segmentation_msgs (same fields/convention cart_msgs.SteerSpeed used
there), matching what buggy_uart's uart_bridge already publishes on
/wheel_uart here.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion

# NumPy compatibility for older transforms3d used by tf_transformations.
if not hasattr(np, "float"):
    np.float = float

from tf_transformations import quaternion_from_euler
from ground_segmentation_msgs.msg import SteerSpeed

class SteeringCalculator(Node):
    def __init__(self):
        super().__init__('steering_calculator')

        # Vehicle parameters
        self.declare_parameter('wheelbase', 1.6)
        self.declare_parameter('max_steering_angle', 20.0)
        # 1.0 = command the pure-pursuit-computed angle exactly, no damping.
        # Was 0.7 (under-steering by 30% on every turn) - that's a tuning
        # gain, not a physical limit, and is what was producing wide,
        # not-quite-committed turns instead of tracking the actual planned
        # curvature. Raise above 1.0 to deliberately over-steer (cut
        # corners tighter than the plan asks); lower it again only if 1.0
        # turns out to overshoot/oscillate on this vehicle.
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('lookahead_distance', 4.0)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('wheel_visualization_size', 0.5)
        self.declare_parameter('steering_kp', 0.6)
        self.declare_parameter('steering_ki', 0.1)
        self.declare_parameter('steering_kd', 0.2)
        self.declare_parameter('small_error_threshold', 5.0)
        self.declare_parameter('fine_tuning_factor', 1.2)
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('speed_reduction_factor', 0.5)
        # True (default): always drive at fixed_speed_kmh regardless of
        # steering angle - skips speed_reduction_factor entirely. False
        # restores the old behavior (slows down proportionally to how
        # sharply it's turning, between min_speed/max_speed).
        self.declare_parameter('use_fixed_speed', True)
        self.declare_parameter('fixed_speed_kmh', 4.0)
        self.declare_parameter('calculation_rate', 10.0)
        # Recovery override (freespace_grid_node's stuck/recovery state
        # machine) - while /recovery_active is true (and fresh), this node
        # ignores /target_lateral_position and the pure-pursuit/PID logic
        # entirely, and just relays whatever /recovery_speed_kmh (signed -
        # negative reverses) and /recovery_steering_deg it was last told,
        # straight to /steering_angle and /vehicle_speed. Same
        # timeout-then-default pattern as the wheel feedback staleness
        # check below - if freespace_grid_node stops publishing (crashed,
        # or simply never ran), recovery mode can't get stuck on forever.
        self.declare_parameter('recovery_timeout_sec', 1.0)
        # Master "may the vehicle actually move" gate - see
        # navigation_enabled_callback's own comment. Fails safe: no
        # message ever received on /navigation_enabled means False, same
        # as an explicit False - the vehicle does not drive just because
        # this node started. manual_pose_aligner republishes True every
        # 0.2s while enabled (not just once) specifically so this timeout
        # can require a LIVE confirming tool, not a one-time flag that
        # stays stuck true forever if that tool crashes or its terminal
        # gets closed mid-drive.
        self.declare_parameter('navigation_enabled_timeout_sec', 1.0)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.steering_gain = self.get_parameter('steering_gain').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.wheel_visualization_size = self.get_parameter('wheel_visualization_size').value
        self.steering_kp = self.get_parameter('steering_kp').value
        self.steering_ki = self.get_parameter('steering_ki').value
        self.steering_kd = self.get_parameter('steering_kd').value
        self.small_error_threshold = self.get_parameter('small_error_threshold').value
        self.fine_tuning_factor = self.get_parameter('fine_tuning_factor').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.speed_reduction_factor = self.get_parameter('speed_reduction_factor').value
        self.use_fixed_speed = self.get_parameter('use_fixed_speed').value
        self.fixed_speed_kmh = self.get_parameter('fixed_speed_kmh').value
        self.timer_rate = self.get_parameter('calculation_rate').value
        self.recovery_timeout_sec = self.get_parameter('recovery_timeout_sec').value

        # Recovery override state - see recovery_timeout_sec's own comment.
        self.recovery_active = False
        self.recovery_speed_kmh = 0.0
        self.recovery_steering_deg = 0.0
        self.last_recovery_time = self.get_clock().now()

        # Master navigation gate - see navigation_enabled_callback.
        self.navigation_enabled = False
        self.last_navigation_enabled_time = self.get_clock().now()
        self.navigation_enabled_timeout_sec = self.get_parameter('navigation_enabled_timeout_sec').value

        # Published by costmap_lane_planner when start_stop_origin is true
        # and the vehicle has driven a full lap back to wherever 'g' was
        # pressed - see lap_complete_callback. No timeout/staleness check
        # like navigation_enabled has: this is a one-shot latch (costmap_
        # lane_planner never un-latches it once true), and defaults safely
        # to False (not held) if start_stop_origin is false or no message
        # has arrived yet.
        self.lap_complete = False

        # Published by path_obstacle_monitor_node when a moving obstacle
        # (LiDAR and/or camera fusion) is currently crossing
        # costmap_lane_planner's live path - see moving_obstacle_blocking_
        # callback. No staleness/freshness check needed here unlike
        # navigation_enabled: a stale "true" is already the SAFE default
        # (stopped), and if the feature is disabled or the monitor never
        # ran at all, this simply stays False (never set), so normal
        # driving is unaffected either way.
        self.moving_obstacle_blocking = False

        # Input target position
        self.target_y = 0.0
        self.last_target_time = self.get_clock().now()

        # Current vehicle state from wheel feedback
        self.current_steering = 0.0
        self.current_speed = 0.0
        self.last_feedback_time = self.get_clock().now()
        self.feedback_steering_rad = 0.0

        # PID state
        self.steering_error_sum = 0.0
        self.prev_steering_error = 0.0
        self.max_error_sum = 10.0

        # Publishers
        self.steering_pub = self.create_publisher(Float32, '/steering_angle', 1)
        self.speed_pub = self.create_publisher(Float32, '/vehicle_speed', 1)
        # Real hardware's UART packet has no speed field at all - yellow is
        # the entire speed contract (yellow=1 -> 2km/h preset, yellow=0 ->
        # 4km/h preset; see steering_uart_bridge.py's docstring for the
        # hardware-confirmed thresholds). /vehicle_speed only drives sim's
        # ackermann_steering_controller, so without this, fixed_speed_kmh
        # has no effect on the real vehicle at all.
        self.yellow_pub = self.create_publisher(Bool, '/final_yellow_detected', 1)
        self.visualization_pub = self.create_publisher(MarkerArray, '/steering_visualization', 1)
        self.actual_vs_target_pub = self.create_publisher(MarkerArray, '/steering_actual_vs_target', 1)

        # Subscribers
        self.create_subscription(Float32, '/target_lateral_position', self.target_position_callback, 1)
        self.create_subscription(Bool, '/enable_steering_visualization', self.visualization_toggle_callback, 1)
        self.create_subscription(SteerSpeed, '/wheel_uart', self.wheel_feedback_callback, 1)
        self.create_subscription(Bool, '/recovery_active', self.recovery_active_callback, 1)
        self.create_subscription(Bool, '/navigation_enabled', self.navigation_enabled_callback, 1)
        self.create_subscription(Bool, '/lap_complete', self.lap_complete_callback, 1)
        self.create_subscription(
            Bool, '/moving_obstacle_blocking', self.moving_obstacle_blocking_callback, 1)
        self.create_subscription(Float32, '/recovery_speed_kmh', self.recovery_speed_callback, 1)
        self.create_subscription(Float32, '/recovery_steering_deg', self.recovery_steering_callback, 1)

        # Timer for periodic steering calculation
        self.timer = self.create_timer(1.0 / self.timer_rate, self.calculate_steering)

        self.get_logger().info(f"Steering Calculator initialized with wheelbase: {self.wheelbase}m, "
                     f"max angle: {self.max_steering_angle}°, lookahead: {self.lookahead_distance}m")

    def target_position_callback(self, msg):
        """Callback for receiving target lateral position"""
        self.target_y = msg.data
        self.last_target_time = self.get_clock().now()

    def visualization_toggle_callback(self, msg):
        """Callback for toggling visualization on/off"""
        self.enable_visualization = msg.data
        self.get_logger().info(f"Steering visualization {'enabled' if self.enable_visualization else 'disabled'}")

    def wheel_feedback_callback(self, msg):
        """Callback for receiving wheel steering angle and speed feedback"""
        self.current_steering = msg.steering
        self.current_speed = msg.speed
        self.last_feedback_time = self.get_clock().now()
        self.get_logger().debug(f"Received wheel feedback - Steering: {self.current_steering:.2f}° | Speed: {self.current_speed:.2f} km/h")

    def navigation_enabled_callback(self, msg):
        """Master gate - published by manual_pose_aligner's 'g' (go) / 'x'
        (hold) keys once you've confirmed localization looks right in
        RViz. Held False (the default, before any message arrives) means
        speed is forced to 0 in calculate_steering regardless of anything
        else - recovery included - so the vehicle physically cannot start
        moving on its own just because this stack finished launching."""
        was_enabled = self.navigation_enabled
        self.navigation_enabled = msg.data
        self.last_navigation_enabled_time = self.get_clock().now()
        if self.navigation_enabled and not was_enabled:
            self.get_logger().info('Navigation ENABLED - vehicle may now drive.')
        elif was_enabled and not self.navigation_enabled:
            self.get_logger().warn('Navigation HELD - forcing speed to 0.')

    def lap_complete_callback(self, msg):
        was_complete = self.lap_complete
        self.lap_complete = msg.data
        if self.lap_complete and not was_complete:
            self.get_logger().warn('Lap complete (start_stop_origin) - holding at start point.')

    def moving_obstacle_blocking_callback(self, msg):
        was_blocking = self.moving_obstacle_blocking
        self.moving_obstacle_blocking = msg.data
        if self.moving_obstacle_blocking and not was_blocking:
            self.get_logger().warn(
                'Moving obstacle in path - holding (speed=0) until it clears.')
        elif was_blocking and not self.moving_obstacle_blocking:
            self.get_logger().info(
                'Moving obstacle cleared - resuming (same path, no replan).')

    def recovery_active_callback(self, msg):
        self.recovery_active = msg.data
        self.last_recovery_time = self.get_clock().now()

    def recovery_speed_callback(self, msg):
        self.recovery_speed_kmh = msg.data

    def recovery_steering_callback(self, msg):
        self.recovery_steering_deg = msg.data

    def _secs_since(self, stamp):
        """Helper to get seconds elapsed since a timestamp"""
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _publish_yellow_for(self, speed_kmh):
        """Real-hardware speed preset selector - see yellow_pub's own
        comment. Threshold matches steering_uart_bridge.py's exactly."""
        self.yellow_pub.publish(Bool(data=abs(speed_kmh) <= 3.0))

    def _publish_recovery_command(self):
        """Bypass pure-pursuit/PID entirely and relay the commanded
        recovery speed/steering straight through - see recovery_timeout_sec's
        own comment for why this exists and how it degrades safely."""
        steering_msg = Float32()
        steering_msg.data = self.recovery_steering_deg
        self.steering_pub.publish(steering_msg)
        speed_msg = Float32()
        speed_msg.data = self.recovery_speed_kmh
        self.speed_pub.publish(speed_msg)
        self._publish_yellow_for(self.recovery_speed_kmh)
        self.get_logger().info(
            f'RECOVERY override: steering={self.recovery_steering_deg:.1f}deg, '
            f'speed={self.recovery_speed_kmh:.2f}km/h', throttle_duration_sec=1.0)

    def calculate_steering(self):
        """Calculate the steering angle based on the target lateral position and current feedback"""
        # Master gate - checked BEFORE recovery too, so a not-yet-confirmed
        # alignment also blocks the reverse-and-retry recovery maneuver,
        # not just ordinary driving. Also requires a FRESH confirmation
        # (see navigation_enabled_timeout_sec) - a crashed/closed
        # manual_pose_aligner must stop the vehicle, not leave it driving
        # on a stale "enabled" flag forever.
        navigation_fresh = (
            self._secs_since(self.last_navigation_enabled_time) <= self.navigation_enabled_timeout_sec)
        if not (self.navigation_enabled and navigation_fresh) or self.lap_complete \
                or self.moving_obstacle_blocking:
            steering_msg = Float32()
            steering_msg.data = 0.0
            self.steering_pub.publish(steering_msg)
            speed_msg = Float32()
            speed_msg.data = 0.0
            self.speed_pub.publish(speed_msg)
            # Stopped (speed=0) here is red's job, not yellow's - yellow
            # still reports the preset that will apply once moving again,
            # so it's not resetting into "slow" just because the vehicle
            # has 0 commanded speed this cycle.
            self._publish_yellow_for(self.fixed_speed_kmh if self.use_fixed_speed else self.max_speed)
            if self.moving_obstacle_blocking:
                self.get_logger().info(
                    'Moving obstacle in path - holding (speed=0).', throttle_duration_sec=5.0)
            elif self.lap_complete:
                self.get_logger().info(
                    'Lap complete - holding at start point (speed=0). Press \'x\' then '
                    '\'g\' in manual_pose_aligner to start a new lap.', throttle_duration_sec=5.0)
            else:
                self.get_logger().info(
                    'Navigation not yet enabled - holding (speed=0). Confirm alignment and '
                    'press \'g\' in manual_pose_aligner to start.', throttle_duration_sec=5.0)
            return

        if self.recovery_active and self._secs_since(self.last_recovery_time) <= self.recovery_timeout_sec:
            self._publish_recovery_command()
            return

        have_recent_feedback = self._secs_since(self.last_feedback_time) <= 1.0

        if self._secs_since(self.last_target_time) > 1.0:
            if have_recent_feedback:
                # No recent target but we have feedback - maintain current steering
                steering_angle = self.current_steering
                target_steering = self.current_steering  # For visualization
                reason = "Maintaining current steering from feedback"
            else:
                # No recent target or feedback - default to straight
                steering_angle = 0.0
                target_steering = 0.0  # For visualization
                reason = "No recent target or feedback data"
        else:
            # Calculate steering angle using Pure Pursuit algorithm
            # For a vehicle with Ackermann steering, the steering angle can be calculated as:
            # angle = arctan(2 * wheelbase * lateral_offset / lookahead_distance^2)

            # Get target lateral position at lookahead distance
            lateral_offset = self.target_y

            # Calculate the steering angle in radians
            steering_radians = np.arctan2(2.0 * self.wheelbase * lateral_offset,
                                         self.lookahead_distance**2)

            # Convert to degrees and apply steering gain
            target_steering = np.degrees(steering_radians) * self.steering_gain

            # Apply steering limits
            target_steering = max(min(target_steering, self.max_steering_angle), -self.max_steering_angle)

            # Flip the sign based on the vehicle's convention (positive is right)
            target_steering = -target_steering  # Flip sign: positive becomes right turn

            # If we have recent feedback, we can apply PID control for smoother steering
            if have_recent_feedback:
                # Calculate error (difference between target and actual steering)
                steering_error = target_steering - self.current_steering

                # For golf cart with slow response, use a more aggressive approach
                # when error is small for fine tuning
                if abs(steering_error) < self.small_error_threshold:
                    # Use higher gain for small errors to improve precision
                    steering_error = steering_error * self.fine_tuning_factor
                    self.get_logger().debug(f"Fine-tuning steering: error {steering_error:.2f}° with factor {self.fine_tuning_factor:.2f}")

                # Calculate integral term with anti-windup
                self.steering_error_sum += steering_error
                self.steering_error_sum = max(min(self.steering_error_sum, self.max_error_sum), -self.max_error_sum)

                # Calculate derivative term
                steering_error_derivative = steering_error - self.prev_steering_error
                self.prev_steering_error = steering_error

                # Apply PID formula
                steering_adjustment = (
                    self.steering_kp * steering_error +
                    self.steering_ki * self.steering_error_sum +
                    self.steering_kd * steering_error_derivative
                )

                # Calculate final steering command
                steering_angle = self.current_steering + steering_adjustment

                # Apply steering limits again after adjustment
                steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)

                reason = f"Target position: {lateral_offset:.2f}m, PID adjusted from current: {self.current_steering:.2f}°"

                # Debug logging for PID components
                self.get_logger().debug(f"PID components - P: {self.steering_kp * steering_error:.2f}, "
                              f"I: {self.steering_ki * self.steering_error_sum:.2f}, "
                              f"D: {self.steering_kd * steering_error_derivative:.2f}")
            else:
                # No feedback, use calculated value directly
                steering_angle = target_steering
                reason = f"Target position: {lateral_offset:.2f}m at {self.lookahead_distance}m ahead (no feedback)"

        if self.use_fixed_speed:
            speed = self.fixed_speed_kmh
        else:
            # Reduce speed when turning sharply
            steering_factor = 1.0 - (abs(steering_angle) / self.max_steering_angle) * self.speed_reduction_factor
            speed = self.max_speed * steering_factor
            # Ensure speed stays within limits
            speed = max(min(speed, self.max_speed), self.min_speed)

        # Publish the steering angle and speed
        steering_msg = Float32()
        steering_msg.data = steering_angle
        self.steering_pub.publish(steering_msg)
        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)
        self._publish_yellow_for(speed)

        # Log the decision with feedback information if available
        feedback_info = ""
        if have_recent_feedback:
            feedback_info = f" | Feedback: {self.current_steering:.2f}° @ {self.current_speed:.1f} km/h"
        self.get_logger().info(f"Steering angle: {steering_angle:.2f}° | Speed: {speed:.1f} km/h{feedback_info} | Reason: {reason}")

        # Visualize if enabled
        if self.enable_visualization:
            self.visualize_steering(steering_angle, speed, target_steering)

    def visualize_steering(self, steering_angle, speed=0.0, target_steering=None):
        """Create visualization markers for the steering angle and speed"""
        marker_array = MarkerArray()

        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header.frame_id = "base_link"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = "steering"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        self.visualization_pub.publish(marker_array)

        if not self.enable_visualization:
            return

        # Create new markers
        marker_array = MarkerArray()

        # Create steering wheel marker
        wheel = Marker()
        wheel.header.frame_id = "base_link"
        wheel.header.stamp = self.get_clock().now().to_msg()
        wheel.ns = "steering"
        wheel.id = 0
        wheel.type = Marker.CYLINDER
        wheel.action = Marker.ADD

        # Position at driver's position (approximate)
        wheel.pose.position.x = 0.5  # 0.5m in front of base_link origin
        wheel.pose.position.y = -0.4  # Slightly to the right (driver's position)
        wheel.pose.position.z = 1.0  # At typical steering wheel height

        # Set orientation - rotated around Z based on steering angle
        # Steering wheel rotates opposite to actual wheels
        q = quaternion_from_euler(0, np.radians(90), np.radians(steering_angle))
        wheel.pose.orientation.x = q[0]
        wheel.pose.orientation.y = q[1]
        wheel.pose.orientation.z = q[2]
        wheel.pose.orientation.w = q[3]

        # Set scale for a steering wheel
        wheel.scale.x = self.wheel_visualization_size
        wheel.scale.y = self.wheel_visualization_size
        wheel.scale.z = 0.1  # Thickness

        # Black color for steering wheel
        wheel.color.r = 0.2
        wheel.color.g = 0.2
        wheel.color.b = 0.2
        wheel.color.a = 0.8

        marker_array.markers.append(wheel)

        # Create front wheels markers showing the steering angle
        left_wheel = Marker()
        left_wheel.header.frame_id = "base_link"
        left_wheel.header.stamp = self.get_clock().now().to_msg()
        left_wheel.ns = "steering"
        left_wheel.id = 1
        left_wheel.type = Marker.CYLINDER
        left_wheel.action = Marker.ADD

        # Position at front left wheel
        left_wheel.pose.position.x = self.wheelbase  # At front axle
        left_wheel.pose.position.y = 0.8  # Left side (assuming 1.6m track width)
        left_wheel.pose.position.z = 0.3  # Wheel radius

        # Set orientation based on steering angle
        q = quaternion_from_euler(0, np.radians(90), np.radians(steering_angle))
        left_wheel.pose.orientation.x = q[0]
        left_wheel.pose.orientation.y = q[1]
        left_wheel.pose.orientation.z = q[2]
        left_wheel.pose.orientation.w = q[3]

        # Set scale for a wheel
        left_wheel.scale.x = 0.6  # Wheel diameter
        left_wheel.scale.y = 0.6  # Wheel diameter
        left_wheel.scale.z = 0.2  # Wheel width

        # Black color for wheel
        left_wheel.color.r = 0.1
        left_wheel.color.g = 0.1
        left_wheel.color.b = 0.1
        left_wheel.color.a = 0.8

        marker_array.markers.append(left_wheel)

        # Right wheel (copy of left wheel with different position)
        right_wheel = Marker()
        right_wheel.header.frame_id = "base_link"
        right_wheel.header.stamp = self.get_clock().now().to_msg()
        right_wheel.ns = "steering"
        right_wheel.id = 2
        right_wheel.type = Marker.CYLINDER
        right_wheel.action = Marker.ADD

        # Position at front right wheel
        right_wheel.pose.position.x = self.wheelbase  # At front axle
        right_wheel.pose.position.y = -0.8  # Right side
        right_wheel.pose.position.z = 0.3  # Wheel radius

        # Set orientation based on steering angle
        right_wheel.pose.orientation = left_wheel.pose.orientation

        # Set scale for a wheel
        right_wheel.scale = left_wheel.scale

        # Black color for wheel
        right_wheel.color = left_wheel.color

        marker_array.markers.append(right_wheel)

        # Add text showing the steering angle
        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "steering"
        text.id = 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        # Position above vehicle
        text.pose.position.x = 1.0  # Slightly ahead of driver
        text.pose.position.y = 0.0  # Center
        text.pose.position.z = 1.5  # Above vehicle

        # Add current feedback information to display
        have_recent_feedback = self._secs_since(self.last_feedback_time) <= 1.0
        if have_recent_feedback:
            feedback_text = f" | Feedback: {self.current_steering:.1f}°, {self.current_speed:.1f} km/h"
        else:
            feedback_text = " | No feedback"

        text.text = f"Command: {steering_angle:.1f}° | Speed: {speed:.1f} km/h{feedback_text}"
        text.scale.z = 0.3  # Text size

        # White text
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0

        marker_array.markers.append(text)

        # Add direction indicator arrow
        arrow = Marker()
        arrow.header.frame_id = "base_link"
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = "steering"
        arrow.id = 4
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        # Position at front of vehicle
        arrow.pose.position.x = self.wheelbase + 0.5  # In front of wheels
        arrow.pose.position.y = 0.0
        arrow.pose.position.z = 0.5

        # Set orientation based on steering angle
        q = quaternion_from_euler(0, 0, np.radians(steering_angle))
        arrow.pose.orientation.x = q[0]
        arrow.pose.orientation.y = q[1]
        arrow.pose.orientation.z = q[2]
        arrow.pose.orientation.w = q[3]

        # Set scale for arrow
        arrow.scale.x = 1.5  # Arrow length
        arrow.scale.y = 0.3  # Arrow width
        arrow.scale.z = 0.2  # Arrow height

        # Green arrow
        arrow.color.r = 0.0
        arrow.color.g = 0.8
        arrow.color.b = 0.2
        arrow.color.a = 0.8

        marker_array.markers.append(arrow)

        # Publish steering visualization
        self.visualization_pub.publish(marker_array)

        # Create a separate visualization for comparing target and actual steering
        if target_steering is not None and have_recent_feedback:
            self.visualize_steering_comparison(target_steering, self.current_steering)

    def visualize_steering_comparison(self, target_steering, actual_steering):
        """Create visualization markers showing the difference between target and actual steering"""
        marker_array = MarkerArray()

        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header.frame_id = "base_link"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = "steering_comparison"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        self.actual_vs_target_pub.publish(marker_array)

        # Create new marker array
        marker_array = MarkerArray()

        # Target steering arrow (blue)
        target_arrow = Marker()
        target_arrow.header.frame_id = "base_link"
        target_arrow.header.stamp = self.get_clock().now().to_msg()
        target_arrow.ns = "steering_comparison"
        target_arrow.id = 0
        target_arrow.type = Marker.ARROW
        target_arrow.action = Marker.ADD

        # Position above the vehicle
        target_arrow.pose.position.x = 0.0  # Center of vehicle
        target_arrow.pose.position.y = 0.0
        target_arrow.pose.position.z = 1.8  # Above the vehicle

        # Set orientation based on target steering angle
        q = quaternion_from_euler(0, np.radians(-90), np.radians(target_steering))
        target_arrow.pose.orientation.x = q[0]
        target_arrow.pose.orientation.y = q[1]
        target_arrow.pose.orientation.z = q[2]
        target_arrow.pose.orientation.w = q[3]

        # Set scale for arrow
        target_arrow.scale.x = 1.0  # Arrow length
        target_arrow.scale.y = 0.1  # Arrow width
        target_arrow.scale.z = 0.1  # Arrow height

        # Blue color for target
        target_arrow.color.r = 0.0
        target_arrow.color.g = 0.0
        target_arrow.color.b = 1.0
        target_arrow.color.a = 0.8

        marker_array.markers.append(target_arrow)

        # Actual steering arrow (red)
        actual_arrow = Marker()
        actual_arrow.header.frame_id = "base_link"
        actual_arrow.header.stamp = self.get_clock().now().to_msg()
        actual_arrow.ns = "steering_comparison"
        actual_arrow.id = 1
        actual_arrow.type = Marker.ARROW
        actual_arrow.action = Marker.ADD

        # Position above the vehicle, same as target but slightly offset
        actual_arrow.pose.position.x = 0.0  # Center of vehicle
        actual_arrow.pose.position.y = 0.0
        actual_arrow.pose.position.z = 1.7  # Slightly below the target arrow

        # Set orientation based on actual steering angle
        q = quaternion_from_euler(0, np.radians(-90), np.radians(actual_steering))
        actual_arrow.pose.orientation.x = q[0]
        actual_arrow.pose.orientation.y = q[1]
        actual_arrow.pose.orientation.z = q[2]
        actual_arrow.pose.orientation.w = q[3]

        # Set scale for arrow (same as target)
        actual_arrow.scale = target_arrow.scale

        # Red color for actual
        actual_arrow.color.r = 1.0
        actual_arrow.color.g = 0.0
        actual_arrow.color.b = 0.0
        actual_arrow.color.a = 0.8

        marker_array.markers.append(actual_arrow)

        # Text showing the steering angle difference
        diff_text = Marker()
        diff_text.header.frame_id = "base_link"
        diff_text.header.stamp = self.get_clock().now().to_msg()
        diff_text.ns = "steering_comparison"
        diff_text.id = 2
        diff_text.type = Marker.TEXT_VIEW_FACING
        diff_text.action = Marker.ADD

        # Position above the arrows
        diff_text.pose.position.x = 0.0
        diff_text.pose.position.y = 0.0
        diff_text.pose.position.z = 2.0

        # Calculate the difference and format the text
        steering_diff = actual_steering - target_steering
        diff_text.text = f"Target:{target_steering:.1f}°|Actual:{actual_steering:.1f}°|Diff: {steering_diff:.1f}°"
        diff_text.scale.z = 0.2  # Text size

        # White text
        diff_text.color.r = 1.0
        diff_text.color.g = 1.0
        diff_text.color.b = 1.0
        diff_text.color.a = 1.0

        marker_array.markers.append(diff_text)

        # Publish the comparison visualization
        self.actual_vs_target_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
