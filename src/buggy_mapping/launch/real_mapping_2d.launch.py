"""Real-hardware counterpart of mapping_2d.launch.py: real Velodyne + real
wheel feedback (buggy_sensors' real_sensors_bringup.launch.py) instead of
Gazebo, otherwise the same perception -> 2D scan -> slam_toolbox (mapping
mode) pipeline. No controller_server/planner_server here - pure
perception + SLAM, nothing plans or drives on its own, and this launch has
no actuation node at all (no steering_uart_bridge/uart_sender) - you must
drive it yourself, physically or via RC, while it builds the map.

Save via slam_toolbox's own serialization service once you've covered the
full loop (see buggy_mapping/maps/README.md for the full command and why
this isn't nav2_map_server's map_saver_cli):

    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \\
        "{filename: '<path-to-buggy_mapping>/maps/<name>'}"

SAFETY: verify feedback_port matches the actual physical wiring before
running.
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

    feedback_port_arg = DeclareLaunchArgument('feedback_port', default_value='/dev/ttyUSB0')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    ground_filter_arg = DeclareLaunchArgument(
        'ground_filter', default_value='true',
        description='false: skip segment_ground - self_hit_filter runs on raw '
                    '/velodyne_points instead. Recommended true for a clean map.')

    real_sensors_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buggy_sensors, 'launch', 'real_sensors_bringup.launch.py')
        ),
        launch_arguments={
            'feedback_port': LaunchConfiguration('feedback_port'),
            'ground_filter': LaunchConfiguration('ground_filter'),
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
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False}],
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
        real_sensors_bringup,
        pointcloud_to_laserscan,
        slam_toolbox,
        rviz2,
    ])
