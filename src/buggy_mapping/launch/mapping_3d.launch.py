"""3D occupancy mapping: buggy_sensors' sensors_bringup (Gazebo + Velodyne +
ground_filter-toggleable segment_ground/self_hit_filter + tf_odom_relay) ->
octomap_server (builds a 3D occupancy octree directly from the
ground-filtered 3D point cloud - no 2D flattening needed, unlike the 2D
slam_toolbox path).

frame_id is 'odom' (NOT 'map') - this launch does NOT run slam_toolbox, so
there is no scan-matched/loop-closed 'map' frame available, only the raw
odom->base_link->lidar_mount_link chain tf_odom_relay + the URDF provide.
KNOWN LIMITATION (see maps/README.md): octomap_server has no pose
correction of its own (no scan matching, no loop closure) - it purely
accumulates points using whatever TF existed at each cloud's timestamp,
so this 3D map will drift more over a long/large loop than the 2D
slam_toolbox map does. Good for a first 3D map of a driven area, not a
substitute for the 2D map's pose graph.

Run mapping_2d.launch.py separately (a different terminal, different
Gazebo instance) if you want the corrected 2D map too - the two aren't
combined into one launch here, matching the "separate files" request
this was built from.

Drive it yourself (separate terminal):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r cmd_vel:=/ackermann_steering_controller/reference_unstamped

Save the map: octomap_server's own octomap_saver_node CLI tool was found
to reject every filename tried (live-tested, "Invalid file name or
extension" regardless of a valid .bt/.ot path, positional arg or ROS
param, via ros2 run or the installed binary directly - root cause not
pinned down). Use buggy_mapping_tools' save_octomap_node instead, which
subscribes to /octomap_binary and writes a correctly-formatted .bt file
via octomap's own AbstractOccupancyOcTree::writeBinary() - round-trip
verified live (saved, then successfully reloaded via
octomap_server_static_node with no errors):

    ros2 run buggy_mapping_tools save_octomap_node <path>.bt

See that package's save_octomap_node.cpp for why the obvious-looking
alternative (writing /octomap_binary's raw message `data` field straight
to a file, or calling the base class's write() instead of writeBinary())
both produce a file that LOOKS plausible but fails to load - confirmed
live both ways before landing on this one.
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

    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='rect_loop_track.world',
        description='World file under buggy_description/worlds/ to map.')
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='-9.5')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    ground_filter_arg = DeclareLaunchArgument(
        'ground_filter', default_value='true',
        description='false: skip segment_ground - self_hit_filter runs on raw '
                    '/velodyne_points instead. Recommended true for a clean map.')
    resolution_arg = DeclareLaunchArgument(
        'resolution', default_value='0.1',
        description='Octree voxel size (m) - matches the 2D map\'s resolution.')
    max_range_arg = DeclareLaunchArgument(
        'max_range', default_value='15.0',
        description='Max sensor range (m) integrated per cloud - speed/memory '
                    'trade-off, same reasoning as this project\'s other configs\' '
                    'obstacle_max_range.')

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

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': 'odom',
            'sensor_model.max_range': LaunchConfiguration('max_range'),
            'use_sim_time': True,
        }],
        remappings=[
            ('cloud_in', '/non_ground_points_filtered'),
        ],
    )

    return LaunchDescription([
        world_file_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        gui_arg,
        ground_filter_arg,
        resolution_arg,
        max_range_arg,
        sensors_bringup,
        octomap_server,
    ])
