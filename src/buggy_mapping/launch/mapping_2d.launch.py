"""2D SLAM mapping: buggy_sensors' sensors_bringup (Gazebo + Velodyne +
ground_filter-toggleable segment_ground/self_hit_filter + tf_odom_relay) ->
pointcloud_to_laserscan (flattens /non_ground_points_filtered into a 2D
/scan, since slam_toolbox only consumes 2D scans) -> slam_toolbox (async,
MAPPING mode - builds a new map from scratch as you drive, as opposed to
localization mode which loads a previously-saved one).

No controller_server/costmap/planner_server here at all - this is pure
perception + SLAM, nothing plans or drives on its own. Drive it yourself,
in a SEPARATE terminal (teleop_twist_keyboard needs a real TTY for
keypresses - it can't usefully run as a background launch process):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r cmd_vel:=/ackermann_steering_controller/reference_unstamped

Drive the full loop (cover every corridor you want in the map later -
anything you don't drive past stays unknown/blank), then save via
slam_toolbox's OWN serialization service (see maps/README.md for the full
command and why this isn't the same as nav2_map_server's map_saver_cli):

    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \\
        "{filename: '<path-to-buggy_mapping>/maps/<name>'}"

RViz shows the live map building up (Map display on /map), the flattened
scan feeding it (/scan), and the ground-filtered cloud it came from
(/non_ground_points_filtered) for comparison.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_buggy_sensors = get_package_share_directory('buggy_sensors')
    pkg_buggy_mapping = get_package_share_directory('buggy_mapping')

    slam_params = os.path.join(pkg_buggy_mapping, 'config', 'slam_toolbox_mapping.yaml')
    p2l_params = os.path.join(pkg_buggy_mapping, 'config', 'pointcloud_to_laserscan.yaml')
    rviz_config = os.path.join(pkg_buggy_mapping, 'rviz', 'mapping.rviz')

    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='rect_loop_track.world',
        description='World file under buggy_description/worlds/ to map.')
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='-9.5')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    ground_filter_arg = DeclareLaunchArgument(
        'ground_filter', default_value='true',
        description='false: skip segment_ground - self_hit_filter runs on raw '
                    '/velodyne_points instead. See sensors_bringup.launch.py\'s docstring '
                    'for the trade-off. Recommended true for a clean map.')

    sensors_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buggy_sensors, 'launch', 'sensors_bringup.launch.py')
        ),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'gui': LaunchConfiguration('gui'),
            'ground_filter': LaunchConfiguration('ground_filter'),
        }.items(),
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[p2l_params, {'use_sim_time': True}],
        remappings=[
            ('cloud_in', '/non_ground_points_filtered'),
            ('scan', '/scan'),
        ],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        world_file_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        gui_arg,
        rviz_arg,
        ground_filter_arg,
        sensors_bringup,
        pointcloud_to_laserscan,
        slam_toolbox,
        rviz2,
    ])
