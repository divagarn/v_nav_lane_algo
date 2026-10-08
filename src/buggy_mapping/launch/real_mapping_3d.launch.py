"""Real-hardware counterpart of mapping_3d.launch.py: real Velodyne + real
wheel feedback (buggy_sensors' real_sensors_bringup.launch.py) instead of
Gazebo, feeding octomap_server directly (no 2D flattening needed).

frame_id is 'odom' - wheel_odometry (buggy_perception) is what publishes
the odom->base_link transform here (dead-reckoning from wheel feedback,
see that node's own docstring for why this isn't a substitute for real
localization). Same KNOWN LIMITATION as the sim version (see buggy_mapping/
maps/README.md): octomap_server has no pose correction of its own - it
only accumulates points using whatever TF existed at each cloud's
timestamp, so drift compounds over a long/large loop with no loop closure
to correct it, same as the underlying wheel odometry it's built on.

Save via buggy_mapping_tools' save_octomap_node (NOT octomap_server's own
octomap_saver_node - confirmed broken on this system, see that package's
source for the full story):

    ros2 run buggy_mapping_tools save_octomap_node <path>.bt

SAFETY: verify feedback_port matches the actual physical wiring before
running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_buggy_sensors = get_package_share_directory('buggy_sensors')

    feedback_port_arg = DeclareLaunchArgument('feedback_port', default_value='/dev/ttyUSB0')
    ground_filter_arg = DeclareLaunchArgument(
        'ground_filter', default_value='true',
        description='false: skip segment_ground - self_hit_filter runs on raw '
                    '/velodyne_points instead. Recommended true for a clean map.')
    resolution_arg = DeclareLaunchArgument(
        'resolution', default_value='0.1',
        description='Octree voxel size (m) - matches the 2D map\'s resolution.')
    max_range_arg = DeclareLaunchArgument(
        'max_range', default_value='15.0',
        description='Max sensor range (m) integrated per cloud.')

    real_sensors_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buggy_sensors, 'launch', 'real_sensors_bringup.launch.py')
        ),
        launch_arguments={
            'feedback_port': LaunchConfiguration('feedback_port'),
            'ground_filter': LaunchConfiguration('ground_filter'),
        }.items(),
    )

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': 'odom',
            'sensor_model.max_range': LaunchConfiguration('max_range'),
            'use_sim_time': False,
        }],
        remappings=[
            ('cloud_in', '/non_ground_points_filtered'),
        ],
    )

    return LaunchDescription([
        feedback_port_arg,
        ground_filter_arg,
        resolution_arg,
        max_range_arg,
        real_sensors_bringup,
        octomap_server,
    ])
