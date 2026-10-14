"""Real-hardware costmap navigation with the ahead-zone stop gate.

Run AFTER buggy_mapping's real_localize.launch.py (sensors, odometry,
slam_toolbox localization) and BEFORE manual_pose_aligner.

Brings up:
  - costmap_lane_planner        path selection, fed by /static_obstacle_points
  - dynamic_obstacle_detector   moving vs static LiDAR points
  - path_obstacle_monitor       ahead-zone stop gate -> /final_red_detected
  - steering_calculator_node    /steering_angle from the lane target
  - uart_sender_node            writes steering + red/yellow to the MCU

SAFETY: loopback defaults to true, so nothing is written to the serial port.
The sender's own red default is ON, so the vehicle stays stopped until the
gate is confirmed. Set loopback:=false only after checking /steering_angle
and /final_red_detected on the bench, with the wheels off the ground.

Stop logic: /final_red_detected = (ahead zone blocked) OR (navigation not
confirmed by 'g' within navigation_enabled_timeout_sec).

Speed: the real UART packet has NO numeric speed field - only a yellow
preset-select bit (see steering_calculator_node's yellow_pub comment):
yellow=1 -> 2 km/h, yellow=0 -> 4 km/h (hardware-confirmed thresholds,
there is no third preset). speed_kmh here just has to land on the right
side of the 3.0 km/h cutoff; it is not a literal commanded speed.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('buggy_nav_costmap')
    costmap_params = os.path.join(pkg_share, 'params', 'costmap_lane_planner_params.yaml')
    detector_params = os.path.join(pkg_share, 'params', 'dynamic_obstacle_detector_params.yaml')
    monitor_params = os.path.join(pkg_share, 'params', 'path_obstacle_monitor_params.yaml')

    sender_port_arg = DeclareLaunchArgument(
        'sender_port', default_value='/dev/ttyUSB0',
        description='Serial port the MCU is wired to (uart_sender_node).')
    loopback_arg = DeclareLaunchArgument(
        'loopback', default_value='true',
        description='true (default): do not write to the serial port. Set false only after '
                    'checking the topics on the bench.')
    speed_kmh_arg = DeclareLaunchArgument(
        'speed_kmh', default_value='4.0',
        description='Real hardware only has two speed presets, selected by the 3.0 km/h '
                    'cutoff: <=3.0 -> 2 km/h preset, >3.0 -> 4 km/h preset.')
    lane_lookahead_arg = DeclareLaunchArgument(
        'lane_lookahead', default_value='12.0',
        description='How far ahead (m) each candidate arc is checked for clearance and how '
                    'far /suggested_lane extends. Mainly helps on long straight stretches - '
                    "near a turn, a candidate's fixed curvature usually hits a wall well "
                    'before this distance regardless.')
    steering_lookahead_distance_arg = DeclareLaunchArgument(
        'steering_lookahead_distance', default_value='4.0',
        description='How far along the chosen lane the steering target point sits. Higher '
                    'smooths wide turns but cuts sharp corners more; lower tracks tighter but '
                    'can be jerkier.')
    max_obstacle_height_m_arg = DeclareLaunchArgument(
        'max_obstacle_height_m', default_value='5.0',
        description='Ignore LiDAR points taller than this many meters above the GROUND as '
                    'obstacles (overhead signage, branches, etc.) - direct meters, no feet '
                    'conversion. Converted by the node itself into its own sensor-frame '
                    'value.')
    ahead_zone_length_m_arg = DeclareLaunchArgument(
        'ahead_zone_length_m', default_value='5.0',
        description="Length (m) of path_obstacle_monitor_node's ahead-zone stop gate (the "
                    'box shown green when clear, red when an obstacle is inside it). Must '
                    'exceed braking distance at the commanded speed plus sensor/actuation '
                    'latency.')

    costmap_lane_planner = Node(
        package='buggy_nav_costmap',
        executable='costmap_lane_planner_node',
        name='costmap_lane_planner',
        output='screen',
        parameters=[
            costmap_params,
            {
                'use_sim_time': False,
                'obstacle_topic': '/static_obstacle_points',
                'lane_lookahead': ParameterValue(
                    LaunchConfiguration('lane_lookahead'), value_type=float),
                'steering_lookahead_distance': ParameterValue(
                    LaunchConfiguration('steering_lookahead_distance'), value_type=float),
                'max_obstacle_height_m': ParameterValue(
                    LaunchConfiguration('max_obstacle_height_m'), value_type=float),
            },
        ],
    )

    dynamic_obstacle_detector = Node(
        package='buggy_nav_costmap',
        executable='dynamic_obstacle_detector_node',
        name='dynamic_obstacle_detector',
        output='screen',
        parameters=[detector_params, {'use_sim_time': False}],
    )

    path_obstacle_monitor = Node(
        package='buggy_nav_costmap',
        executable='path_obstacle_monitor_node',
        name='path_obstacle_monitor',
        output='screen',
        parameters=[
            monitor_params,
            {
                'use_sim_time': False,
                'ahead_zone_length_m': ParameterValue(
                    LaunchConfiguration('ahead_zone_length_m'), value_type=float),
            },
        ],
    )

    steering_calculator = Node(
        package='buggy_perception',
        executable='steering_calculator_node',
        name='steering_calculator',
        output='screen',
        parameters=[{
            'use_fixed_speed': True,
            'fixed_speed_kmh': ParameterValue(LaunchConfiguration('speed_kmh'), value_type=float),
            'use_sim_time': False,
        }],
    )

    uart_sender = Node(
        package='buggy_uart',
        executable='uart_sender_node',
        name='uart_sender_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('sender_port'),
            'baudrate': 115200,
            'loopback': ParameterValue(LaunchConfiguration('loopback'), value_type=bool),
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        sender_port_arg,
        loopback_arg,
        speed_kmh_arg,
        lane_lookahead_arg,
        steering_lookahead_distance_arg,
        max_obstacle_height_m_arg,
        ahead_zone_length_m_arg,
        costmap_lane_planner,
        dynamic_obstacle_detector,
        path_obstacle_monitor,
        steering_calculator,
        uart_sender,
    ])
