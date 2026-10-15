"""Real-hardware counterpart of localize.launch.py: real Velodyne + real
wheel feedback (buggy_sensors' real_sensors_bringup.launch.py) instead of
Gazebo, otherwise the same perception -> 2D scan -> slam_toolbox
(LOCALIZATION mode, loading a map saved by real_mapping_2d.launch.py or
mapping_2d.launch.py) pipeline as localize.launch.py.

"sync with the real world" here means physically placing the vehicle at
(or very near) wherever real_mapping_2d.launch.py / mapping_2d.launch.py
was actually started from when the loaded map was built - map_start_pose
in slam_toolbox_localization.yaml is [0,0,0], so the map's own origin IS
wherever the odom frame started during that mapping run (see that yaml's
own comment). There's no spawn_x/y/yaw to set here like the sim version -
you physically position the vehicle instead.

No controller_server/planner_server/carrot_path_publisher here at all -
pure perception + localization, nothing drives on its own, and no
actuation node either (no steering_uart_bridge/uart_sender) - this launch
cannot move the vehicle even if something tried to.

executable MUST be localization_slam_toolbox_node, not
async_slam_toolbox_node - same confirmed behavior difference as the sim
version (see that file's docstring) - this only matters for the manual
relocalization below, not ordinary scan-matching.

MANUAL POSE CORRECTION: same workflow as localize.launch.py - if the live
scan doesn't line up with the map in RViz, use RViz's "2D Pose Estimate"
tool. initialpose_to_slam_toolbox bridges that click into slam_toolbox's
real LOCALIZE_AT_POSE mechanism - the TF (map->odom) jumps immediately,
the vehicle does not need to move for the pose itself to take effect.

SAFETY: verify feedback_port matches the actual physical wiring before
running.

enable_kiss_icp:=true replaces wheel_odometry's odom->base_link TF with
kiss_icp_node (LiDAR scan-registration odometry, immune to wheel slip) -
see real_sensors_bringup.launch.py's own description of this arg. Use
this if the map keeps drifting even after alignment is confirmed correct
via the workflow above - that's an ongoing odometry-quality problem, not
a one-time alignment problem, and this fixes the former.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_buggy_sensors = get_package_share_directory('buggy_sensors')
    pkg_buggy_mapping = get_package_share_directory('buggy_mapping')

    slam_params = os.path.join(pkg_buggy_mapping, 'config', 'slam_toolbox_localization.yaml')
    p2l_params = os.path.join(pkg_buggy_mapping, 'config', 'pointcloud_to_laserscan.yaml')
    rviz_config = os.path.join(pkg_buggy_mapping, 'rviz', 'closed_loop_navigate.rviz')
    maps_dir = os.path.join(pkg_buggy_mapping, 'maps')

    feedback_port_arg = DeclareLaunchArgument('feedback_port', default_value='/dev/ttyUSB0')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    ground_filter_arg = DeclareLaunchArgument(
        'ground_filter', default_value='true',
        description='false: skip segment_ground - self_hit_filter runs on raw '
                    '/velodyne_points instead. Recommended true for reliable localization.')
    ground_filter_device_arg = DeclareLaunchArgument(
        'ground_filter_device', default_value='gpu',
        description="ground_filter:=true only - 'gpu' (default) or 'cpu'. See "
                    "real_sensors_bringup.launch.py's own description of this arg.")
    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='my_map2',
        description='Basename (no extension) under buggy_mapping/maps/ saved via '
                    'real_mapping_2d.launch.py + /slam_toolbox/serialize_map - loads '
                    '<map_name>.data/.posegraph.')
    enable_kiss_icp_arg = DeclareLaunchArgument(
        'enable_kiss_icp', default_value='false',
        description='false (default): odom->base_link comes from wheel_odometry '
                    '(dead-reckoning, no slip correction). true: use kiss_icp_node '
                    '(LiDAR scan-registration odometry) instead - see '
                    "real_sensors_bringup.launch.py's own description of this arg. "
                    'Addresses map drift that persists even after a correct manual '
                    'alignment, by giving slam_toolbox a much better odometry prior to '
                    'correct from between its own scan-match updates.')
    camera_arg = DeclareLaunchArgument(
        'camera', default_value='false',
        description='true: additionally launch camera_processing/camera_raw_node - see '
                    "real_sensors_bringup.launch.py's own description of this arg.")
    camera_device_arg = DeclareLaunchArgument('camera_device', default_value='/dev/video0')
    radar_arg = DeclareLaunchArgument(
        'radar', default_value='false',
        description='true: additionally launch umrr_ros2_driver/launch/radar.launch.py '
                    "(smart_radar) - see real_sensors_bringup.launch.py's own description "
                    'of this arg.')
    camera_lidar_fusion_arg = DeclareLaunchArgument(
        'camera_lidar_fusion', default_value='false',
        description='true: additionally launch camera_static_tf + camera_lidar_fusion_node '
                    "(distance-based safety-stop signals) - see real_sensors_bringup."
                    "launch.py's own description of this arg.")
    radar_fusion_arg = DeclareLaunchArgument(
        'radar_fusion', default_value='false',
        description='true: additionally launch radar_fusion/radar_fusion.launch.py (the '
                    'IMM-Kalman radar static/moving object tracker) - see '
                    "real_sensors_bringup.launch.py's own description of this arg. Needs "
                    'radar:=true as well to actually get data.')

    map_file_name = PathJoinSubstitution([maps_dir, LaunchConfiguration('map_name')])

    real_sensors_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buggy_sensors, 'launch', 'real_sensors_bringup.launch.py')
        ),
        launch_arguments={
            'feedback_port': LaunchConfiguration('feedback_port'),
            'ground_filter': LaunchConfiguration('ground_filter'),
            'ground_filter_device': LaunchConfiguration('ground_filter_device'),
            'enable_kiss_icp': LaunchConfiguration('enable_kiss_icp'),
            'camera': LaunchConfiguration('camera'),
            'camera_device': LaunchConfiguration('camera_device'),
            'radar': LaunchConfiguration('radar'),
            'camera_lidar_fusion': LaunchConfiguration('camera_lidar_fusion'),
            'radar_fusion': LaunchConfiguration('radar_fusion'),
        }.items(),
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[p2l_params, {'use_sim_time': False}],
        remappings=[
            ('cloud_in', '/non_ground_points_filtered'),
            ('scan', '/scan'),
        ],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {
            'use_sim_time': False,
            'map_file_name': map_file_name,
            # Same reasoning/values as localize.launch.py's own override
            # (see that file's comment for the confirmed-live finding,
            # from cart_navigation_ws, that a narrower window still wasn't
            # enough) - scoped here only, not edited in the shared yaml.
            # Real wheel odometry likely drifts at least as much as sim's.
            'minimum_travel_distance': 0.1,
            'minimum_travel_heading': 0.1,
            'correlation_search_space_dimension': 2.0,
            # Same tf_buffer_duration widening as localize.launch.py's own
            # override - see that file's comment (slam_toolbox's message
            # filter dropping scans whose timestamp aged out of the shared
            # yaml's 30s TF history buffer under processing backlog).
            # Applies equally here if the perception pipeline ever falls
            # behind real time on this hardware.
            'tf_buffer_duration': 90.0,
        }],
    )

    initialpose_to_slam_toolbox = Node(
        package='buggy_perception',
        executable='initialpose_to_slam_toolbox',
        name='initialpose_to_slam_toolbox',
        output='screen',
        parameters=[{
            'map_file_name': map_file_name,
            'use_sim_time': False,
        }],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        feedback_port_arg,
        rviz_arg,
        ground_filter_arg,
        ground_filter_device_arg,
        map_name_arg,
        enable_kiss_icp_arg,
        camera_arg,
        camera_device_arg,
        radar_arg,
        camera_lidar_fusion_arg,
        radar_fusion_arg,
        real_sensors_bringup,
        pointcloud_to_laserscan,
        slam_toolbox,
        initialpose_to_slam_toolbox,
        rviz2,
    ])
