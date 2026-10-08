"""Full sensor + perception bringup: Gazebo spawn (buggy_description) ->
Velodyne (URDF/Gazebo plugin, always on) -> segment_ground (torch-based
ground removal, TOGGLEABLE via ground_filter) -> self_hit_filter (always
on - excludes the vehicle's own chassis returns regardless of ground_filter)
-> radar_sim (UMRR-A4 stand-in, fed from self_hit_filter's output) ->
tf_odom_relay (ackermann_steering_controller's odom onto standard /tf).

ground_filter:=true (default): segment_ground runs, self_hit_filter reads
its /non_ground_points output - matches the old workspace's normal mode.
ground_filter:=false: segment_ground is skipped entirely (it's the
slowest node in this pipeline - torch RANSAC), self_hit_filter reads raw
/velodyne_points instead - faster, but ground plane returns show up as
spurious obstacles/radar targets. Same trade-off as the old workspace's
various ground_filter args (see buggy_mapping's real_closed_loop_navigate_
slam.launch.py docstring in the old workspace for the fullest writeup of
this trade-off).

radar_sim's output topic name (/non_ground_points_filtered) never
changes regardless of ground_filter - only self_hit_filter's INPUT
remaps between the two upstream sources.

enable_kiss_icp:=true swaps tf_odom_relay out for kiss_icp_node (LiDAR
scan-registration odometry) as the odom->base_link source - see that arg's
own description for why.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_buggy_description = get_package_share_directory('buggy_description')

    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='rect_loop_track.world',
        description='World file under buggy_description/worlds/.')
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='5.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='5.0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    ground_filter_arg = DeclareLaunchArgument(
        'ground_filter', default_value='true',
        description='false: skip segment_ground (the slow torch-based node) entirely - '
                    'self_hit_filter runs on raw /velodyne_points instead. See this '
                    "file's docstring for the trade-off.")
    ground_filter_device_arg = DeclareLaunchArgument(
        'ground_filter_device', default_value='gpu',
        description="ground_filter:=true only - 'gpu' (default), 'cpu', or 'segment'. "
                    "Selects segment_ground's backend (SEGMENT_GROUND_METHOD) - see "
                    "real_sensors_bringup.launch.py's own description of this arg.")
    enable_kiss_icp_arg = DeclareLaunchArgument(
        'enable_kiss_icp', default_value='false',
        description='false (default): odom->base_link comes from tf_odom_relay '
                    "(Gazebo's ackermann_steering_controller), as before. true: "
                    'tf_odom_relay is skipped and kiss_icp_node (LiDAR scan-registration '
                    'odometry, vendored from lane_nav_ws) supplies odom->base_link instead - '
                    'immune to wheel slip, unlike wheel-based odometry. See '
                    'buggy_mapping/launch/localize.launch.py for why this exists (map '
                    'drift after correct manual alignment).')

    non_ground_points_source = PythonExpression([
        "'/non_ground_points' if '", LaunchConfiguration('ground_filter'),
        "' == 'true' else '/velodyne_points'"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buggy_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': PathJoinSubstitution(
                [pkg_buggy_description, 'worlds', LaunchConfiguration('world_file')]),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    segment_ground = Node(
        package='buggy_perception',
        executable='segment_ground',
        name='segment_ground',
        output='screen',
        parameters=[{'use_sim_time': True}],
        additional_env={'SEGMENT_GROUND_METHOD': LaunchConfiguration('ground_filter_device')},
        condition=IfCondition(LaunchConfiguration('ground_filter')),
    )

    self_hit_filter = Node(
        package='buggy_perception',
        executable='self_hit_filter',
        name='self_hit_filter',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/non_ground_points', non_ground_points_source),
        ],
    )

    tf_odom_relay = Node(
        package='buggy_perception',
        executable='tf_odom_relay',
        name='tf_odom_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(LaunchConfiguration('enable_kiss_icp')),
    )

    # lidar_odom_frame set to 'odom' (not the package default 'odom_lidar')
    # and invert_odom_tf:=false so the TF it broadcasts IS odom->base_link
    # (parent odom, child base_link) - the exact transform slam_toolbox
    # reads via TF lookup (odom_frame/base_frame in slam_toolbox_
    # localization.yaml), same slot tf_odom_relay normally fills. base_frame
    # is explicitly 'base_link' (differs from the cloud's own frame_id,
    # lidar_mount_link in sim) so kiss_icp re-expresses its LiDAR-frame pose
    # in base_link via the existing URDF-provided static TF, instead of
    # running in egocentric (LiDAR-frame) mode.
    kiss_icp_node = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        output='screen',
        remappings=[
            ('pointcloud_topic', '/velodyne_points'),
        ],
        parameters=[{
            'base_frame': 'base_link',
            'lidar_odom_frame': 'odom',
            'publish_odom_tf': True,
            'invert_odom_tf': False,
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('enable_kiss_icp')),
    )

    radar_sim = Node(
        package='buggy_sensors',
        executable='radar_sim',
        name='radar_sim',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        world_file_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        gui_arg,
        ground_filter_arg,
        ground_filter_device_arg,
        enable_kiss_icp_arg,
        gazebo,
        segment_ground,
        self_hit_filter,
        tf_odom_relay,
        kiss_icp_node,
        radar_sim,
    ])
