"""Radar static/moving object detection with velocity - ported from a colleague's
closed_loop_nav workspace's sensor_fusion stack (same IMM-Kalman radar tracker,
renamed radar_fusion), trimmed to radar-only for this workspace: no camera/
object_detection runs here, see radar_fusion_params.yaml's own camera.enabled
comment.

Run AFTER whatever publishes the radar driver's own /smart_radar/can_targets_0
(umrr_ros2_driver's radar.launch.py on real hardware, buggy_sensors'
radar_sim.launch.py in Gazebo - see real_sensors_bringup.launch.py's own radar
arg for the real-hardware toggle this project already uses) and this
workspace's existing /odom (wheel_odometry) + /steering_angle
(steering_calculator_node).

use_sim_time also picks sim-vs-real overrides automatically (same pattern
costmap_navigate.launch.py's own costmap_obstacle_topic PythonExpression
uses), since radar_fusion_params.yaml's own defaults are tuned for REAL
hardware and three of them are wrong/absent in this workspace's sim:
  - odom_topic: real default /odom (wheel_odometry) has no sim equivalent -
    nothing in sim publishes plain /odom (tf_odom_relay only relays a TF,
    never a nav_msgs/Odometry topic) - sim uses Gazebo's own
    /ackermann_steering_controller/odometry instead. Confirmed live:
    pointing odom_topic at a topic nothing publishes meant path.
    curvature_source: odom silently read nothing, and the "auto" fallback
    (before this was even set to odom explicitly) landed on /steering_angle
    instead - whose sign convention is inverted for this exact purpose (see
    radar_fusion_params.yaml's own path.curvature_source comment) - that
    turned out to be the actual cause of the predicted path corridor
    curving backwards relative to /suggested_lane.
  - radar.use_doppler / ego_motion.speed_source: real default assumes
    genuine Doppler - radar_sim.py's radial_speed is always 0 (no
    ground-truth velocity model in sim), so both are turned off for sim.
  - ego_motion.radar_mount_x: real default (2.2m, front bumper from
    rear-axle base_link) doesn't match radar_sim.py's own base_link->umrr
    TF mount (1.4m, base_link mid-wheelbase) - only affects the path
    corridor's own internal frame math (object positions themselves use
    the live TF, not this config), but kept consistent for sim.
These four exactly match the friend's own closed_loop_nav --gazebo preset
(run_fusion.sh's own --gazebo flag) - see radar_fusion_params.yaml's own
output_frame comment for the one other thing kept identical to his default
(map, not odom).

Publishes (see radar_fusion_node's own header comment for full field list):
  /radar_fusion/objects   perception_msgs/FusedObjectArray  confirmed tracks,
                          each with position, velocity (over ground), motion_state
                          (stationary/moving/turning), relative_motion_state
  /radar_fusion/tracks    perception_msgs/FusedObjectArray  all confirmed tracks
                          incl. ghosts (debug - only computed if subscribed)

enable_visualization:=true (default) also brings up fusion_visualization_node,
publishing RViz MarkerArrays on /fusion_visualization/markers[_relevant] -
class-sized 3D boxes in output_frame, no camera overlay (overlay.enabled:
false in its own config - no camera running here to overlay onto).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    radar_fusion_share = get_package_share_directory('radar_fusion')
    default_params = os.path.join(radar_fusion_share, 'config', 'radar_fusion_params.yaml')
    fusion_viz_share = get_package_share_directory('fusion_visualization')
    default_viz_params = os.path.join(fusion_viz_share, 'config', 'fusion_visualization.yaml')

    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization', default_value='true',
        description='true (default): also launch fusion_visualization_node for RViz markers '
                    '(/fusion_visualization/markers[_relevant]). false: radar_fusion_node only.')

    odom_topic = PythonExpression([
        "'/ackermann_steering_controller/odometry' if '",
        LaunchConfiguration('use_sim_time'), "' == 'true' else '/odom'"])
    use_doppler = PythonExpression([
        "'false' if '", LaunchConfiguration('use_sim_time'), "' == 'true' else 'true'"])
    ego_speed_source = PythonExpression([
        "'odom' if '", LaunchConfiguration('use_sim_time'), "' == 'true' else 'radar_doppler'"])
    radar_mount_x = PythonExpression([
        "1.4 if '", LaunchConfiguration('use_sim_time'), "' == 'true' else 2.2"])

    radar_fusion_node = Node(
        package='radar_fusion',
        executable='radar_fusion_node',
        name='radar_fusion',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'odom_topic': odom_topic,
                'radar.use_doppler': ParameterValue(use_doppler, value_type=bool),
                'ego_motion.speed_source': ego_speed_source,
                'ego_motion.radar_mount_x': ParameterValue(radar_mount_x, value_type=float),
            },
        ],
    )

    fusion_visualization_node = Node(
        package='fusion_visualization',
        executable='fusion_visualization_node',
        name='fusion_visualization',
        output='screen',
        parameters=[
            default_viz_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(LaunchConfiguration('enable_visualization')),
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        enable_visualization_arg,
        radar_fusion_node,
        fusion_visualization_node,
    ])
