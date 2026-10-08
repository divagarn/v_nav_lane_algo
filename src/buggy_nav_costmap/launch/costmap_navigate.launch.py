"""C++ lane planner - drop-in alternative to buggy_nav's
freespace_navigate.launch.py (which launches the Python freespace_grid_node,
own log-odds voxel grid). This one launches costmap_lane_planner_node
instead: same arc-based lane planning logic, occupancy read directly from
the latest LiDAR point cloud each cycle (a Nav2 Costmap2DROS was tried and
removed - its internal message filter kept dropping every observation
regardless of fix attempted), and a strong (not absolute)
pose-graph-direction bias.

Does NOT touch freespace_navigate.launch.py or any buggy_nav/buggy_
perception file - this is a separate, additive package. Run this INSTEAD
OF freespace_navigate.launch.py, not alongside it (both publish
/target_lateral_position). Launch buggy_mapping's localize.launch.py FIRST
in its own terminal (Gazebo + Velodyne + segment_ground + slam_toolbox
LOCALIZATION + the saved pose graph), same two-terminal split every other
navigation launch in this workspace uses, THEN this file, THEN
steering_calculator_node/sim_ackermann_bridge exactly as
freespace_navigate.launch.py already does (unchanged, this launch doesn't
duplicate them - see that file for the reference commands).

enable_moving_obstacle_stop:=true (default false) adds the "stop for a
moving obstacle in my path, then resume the SAME path once it clears -
never reroute around it" behavior:
  - costmap_lane_planner's obstacle_topic is switched from the raw
    obstacle cloud to dynamic_obstacle_detector_node's own
    /static_obstacle_points (moving points excluded) - so path SELECTION
    becomes physically blind to moving objects, and the chosen path can
    never itself change because of one.
  - dynamic_obstacle_detector_node is brought up to produce that split
    (and /moving_obstacle_points) - see that node's own header comment
    for the classification mechanism.
  - path_obstacle_monitor_node watches /moving_obstacle_points (+
    /camera_red_detected, if camera_lidar_fusion is also running) against
    costmap_lane_planner's live /suggested_lane path and publishes
    /moving_obstacle_blocking - steering_calculator_node gates on this
    the same way it already gates on navigation_enabled/lap_complete,
    holding a hard stop while true. Since the path itself was never
    disturbed, releasing this gate simply resumes whatever
    /target_lateral_position costmap_lane_planner has been continuously
    publishing the whole time - no replanning step exists to trigger.
Designed to be extended with more obstacle sources later (a third sensor
just needs its own subscription + one more OR'd bool inside
path_obstacle_monitor_node - see that node's own header comment).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('buggy_nav_costmap')
    default_params = os.path.join(pkg_share, 'params', 'costmap_lane_planner_params.yaml')
    default_dynamic_obstacle_params = os.path.join(
        pkg_share, 'params', 'dynamic_obstacle_detector_params.yaml')
    default_path_monitor_params = os.path.join(
        pkg_share, 'params', 'path_obstacle_monitor_params.yaml')

    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    enable_moving_obstacle_stop_arg = DeclareLaunchArgument(
        'enable_moving_obstacle_stop', default_value='false',
        description='true: stop for a moving obstacle crossing the current path and resume '
                    'the SAME path once clear (no reroute) - see this file\'s own docstring '
                    'for the full mechanism. false (default): moving obstacles are treated '
                    'exactly like static ones by costmap_lane_planner, same as before this '
                    'feature existed.')
    dynamic_obstacle_detector_params_arg = DeclareLaunchArgument(
        'dynamic_obstacle_detector_params_file', default_value=default_dynamic_obstacle_params)
    path_obstacle_monitor_params_arg = DeclareLaunchArgument(
        'path_obstacle_monitor_params_file', default_value=default_path_monitor_params)

    # When the feature is off this still resolves to the yaml's own
    # default (/non_ground_points_filtered) - a harmless no-op override,
    # not a conditioned parameter, since ROS2 launch parameter lists are
    # built unconditionally at Node-construction time.
    costmap_obstacle_topic = PythonExpression([
        "'/static_obstacle_points' if '",
        LaunchConfiguration('enable_moving_obstacle_stop'),
        "' == 'true' else '/non_ground_points_filtered'"])

    costmap_lane_planner = Node(
        package='buggy_nav_costmap',
        executable='costmap_lane_planner_node',
        name='costmap_lane_planner',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'obstacle_topic': costmap_obstacle_topic,
            },
        ],
    )

    dynamic_obstacle_detector = Node(
        package='buggy_nav_costmap',
        executable='dynamic_obstacle_detector_node',
        name='dynamic_obstacle_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('dynamic_obstacle_detector_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(LaunchConfiguration('enable_moving_obstacle_stop')),
    )

    path_obstacle_monitor = Node(
        package='buggy_nav_costmap',
        executable='path_obstacle_monitor_node',
        name='path_obstacle_monitor',
        output='screen',
        parameters=[
            LaunchConfiguration('path_obstacle_monitor_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(LaunchConfiguration('enable_moving_obstacle_stop')),
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        enable_moving_obstacle_stop_arg,
        dynamic_obstacle_detector_params_arg,
        path_obstacle_monitor_params_arg,
        costmap_lane_planner,
        dynamic_obstacle_detector,
        path_obstacle_monitor,
    ])
