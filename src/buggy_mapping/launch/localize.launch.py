"""Localize the robot against a previously-saved 2D map (mapping_2d.launch.py
+ /slam_toolbox/serialize_map) - "sync with the real world" means the
spawn pose here must match wherever the map was actually built, since
slam_toolbox's LOCALIZATION mode assumes the vehicle re-enters the map at
its own origin (map_start_pose: [0,0,0] in slam_toolbox_localization.yaml -
see that file's own comment). For my_map2 (built on office_track.world),
that means spawn_x:=0.0 spawn_y:=-3.0 spawn_yaw:=0.0 - the same spawn used
throughout this workspace's testing when that map was saved. Get this
wrong and the live scan won't line up with the loaded map in RViz; that's
exactly what the pause/pose-correction workflow below is for.

No controller_server/planner_server/carrot_path_publisher here at all -
this is pure perception + localization, nothing plans or drives on its
own. Bring buggy_sensors' sensors_bringup (Gazebo + Velodyne + ground-
filter-toggleable segment_ground/self_hit_filter + tf_odom_relay), flatten
the ground-filtered cloud into a 2D /scan, then run slam_toolbox in
LOCALIZATION mode against the saved map.

executable MUST be localization_slam_toolbox_node, not
async_slam_toolbox_node (even with mode: localization set in the yaml) -
confirmed in the old workspace: calling /slam_toolbox/deserialize_map
with match_type: LOCALIZE_AT_POSE against a running async_slam_toolbox_node
logs "Requested a localization deserialization in non-localization mode"
and silently does not update map->odom, while the identical call against
localization_slam_toolbox_node updates it immediately. This only matters
for the manual relocalization below, not ordinary scan-matching.

MANUAL POSE CORRECTION: if the live scan doesn't line up with the map in
RViz (spawn didn't exactly match the mapping run, or localization needs a
nudge), use RViz's "2D Pose Estimate" tool to click-and-drag the vehicle's
actual pose on the map. That tool always publishes to /initialpose, but
this slam_toolbox build's LOCALIZATION mode does NOT subscribe to
/initialpose at all (confirmed via `ros2 node info /slam_toolbox`) -
initialpose_to_slam_toolbox bridges the two: it translates each
/initialpose message into a call to /slam_toolbox/deserialize_map with
match_type: LOCALIZE_AT_POSE, the same mechanism slam_toolbox itself uses
to load the map at startup, just invoked again with your clicked pose.
That resets the localization anchor directly, so the TF (map->odom) jumps
immediately - the vehicle does NOT need to move for the pose itself to
take effect.

enable_kiss_icp:=true replaces the wheel/Gazebo-based odom->base_link
source with kiss_icp_node (LiDAR scan-registration odometry) - see
sensors_bringup.launch.py's own description of this arg. Use this if the
map keeps drifting even after alignment is confirmed correct via the
workflow above - that's an ongoing odometry-quality problem, not a
one-time alignment problem, and this fixes the former.
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

    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='office_track.world',
        description='Must match whatever world mapping_2d.launch.py actually mapped.')
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value='0.0',
        description='Must match the spawn pose used when the map was built (see this '
                    "file's docstring) - my_map2's was (0.0, -3.0, 0.0) on office_track.world.")
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='-3.0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    ground_filter_arg = DeclareLaunchArgument(
        'ground_filter', default_value='true',
        description='false: skip segment_ground - self_hit_filter runs on raw '
                    '/velodyne_points instead. Recommended true for reliable localization.')
    ground_filter_device_arg = DeclareLaunchArgument(
        'ground_filter_device', default_value='gpu',
        description="ground_filter:=true only - 'gpu' (default), 'cpu', or 'segment'. See "
                    "real_sensors_bringup.launch.py's own description of this arg.")
    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='my_map2',
        description='Basename (no extension) under buggy_mapping/maps/ saved via '
                    'mapping_2d.launch.py + /slam_toolbox/serialize_map - loads '
                    '<map_name>.data/.posegraph.')
    enable_kiss_icp_arg = DeclareLaunchArgument(
        'enable_kiss_icp', default_value='false',
        description='false (default): odom->base_link comes from tf_odom_relay, as '
                    'before. true: use kiss_icp_node (LiDAR scan-registration odometry) '
                    'instead - see sensors_bringup.launch.py\'s own description of this '
                    'arg. Addresses map drift that persists even after a correct manual '
                    'alignment, by giving slam_toolbox a much better odometry prior to '
                    'correct from between its own scan-match updates.')

    map_file_name = PathJoinSubstitution([maps_dir, LaunchConfiguration('map_name')])

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
            'ground_filter_device': LaunchConfiguration('ground_filter_device'),
            'enable_kiss_icp': LaunchConfiguration('enable_kiss_icp'),
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
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {
            'use_sim_time': True,
            'map_file_name': map_file_name,
            # Overrides scoped to THIS launch file only, not the shared
            # slam_toolbox_localization.yaml - confirmed live in
            # cart_navigation_ws (a different workspace, same drift
            # symptom) that the yaml's defaults (minimum_travel_distance/
            # heading 0.3m/0.3rad, correlation_search_space_dimension
            # 0.5m i.e. +-0.25m search) only re-correct map->odom every
            # 0.3m of travel, searching a narrow +-0.25m window each time.
            # If real odometry error accumulates faster than that between
            # updates, the scan matcher either fails to find a match
            # (error just sits there uncorrected) or locks onto a nearby
            # wrong peak inside that small window (a bad correction, not
            # just a missing one) - both read as "the map drifting" the
            # longer the vehicle drives. Checking almost continuously and
            # searching a much wider +-1.0m window trades noticeably more
            # CPU for headroom.
            'minimum_travel_distance': 0.1,
            'minimum_travel_heading': 0.1,
            'correlation_search_space_dimension': 2.0,
            # Separate, DIFFERENT drift mechanism confirmed live in
            # cart_navigation_ws: slam_toolbox's message filter logs
            # "frame '<sensor_frame>' at time X ... earlier than all the
            # data in the transform cache" and silently DROPS that scan -
            # not a bad correction, no correction attempt at all. That
            # happens when a scan's original timestamp has aged out of
            # the default 30s TF history buffer by the time this node
            # actually gets to process it - i.e. the perception pipeline
            # fell behind real time badly enough that a queued scan is
            # already >30s stale on arrival, which happens under CPU
            # contention (Gazebo real-time factor well below 1.0).
            # Widening the buffer doesn't fix the backlog itself, it just
            # stops a temporarily-delayed scan from being thrown away
            # outright instead of still being used once it does arrive.
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
            'use_sim_time': True,
        }],
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
        ground_filter_device_arg,
        map_name_arg,
        enable_kiss_icp_arg,
        sensors_bringup,
        pointcloud_to_laserscan,
        slam_toolbox,
        initialpose_to_slam_toolbox,
        rviz2,
    ])
