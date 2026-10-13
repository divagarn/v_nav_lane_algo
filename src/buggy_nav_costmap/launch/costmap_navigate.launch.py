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
from launch_ros.parameter_descriptions import ParameterValue


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
    max_obstacle_height_ft_arg = DeclareLaunchArgument(
        'max_obstacle_height_ft', default_value='5.0',
        description='Ignore LiDAR points taller than this many feet above the GROUND as '
                    'obstacles (overhead signage, branches, etc.) - converted here into '
                    "max_obstacle_height's own sensor-frame meters (ft*0.3048 - 1.0, the "
                    'LiDAR mounting height).')
    ahead_zone_length_m_arg = DeclareLaunchArgument(
        'ahead_zone_length_m', default_value='5.0',
        description="Length (m) of path_obstacle_monitor_node's ahead-zone stop gate (the "
                    'box shown green when clear, red when an obstacle is inside it). Must '
                    'exceed braking distance at the commanded speed plus sensor/actuation '
                    'latency.')

    # When the feature is off this still resolves to the yaml's own
    # default (/non_ground_points_filtered) - a harmless no-op override,
    # not a conditioned parameter, since ROS2 launch parameter lists are
    # built unconditionally at Node-construction time.
    costmap_obstacle_topic = PythonExpression([
        "'/static_obstacle_points' if '",
        LaunchConfiguration('enable_moving_obstacle_stop'),
        "' == 'true' else '/non_ground_points_filtered'"])
    max_obstacle_height_m = PythonExpression([
        "float('", LaunchConfiguration('max_obstacle_height_ft'), "') * 0.3048 - 1.0"])

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
                'lane_lookahead': ParameterValue(
                    LaunchConfiguration('lane_lookahead'), value_type=float),
                'steering_lookahead_distance': ParameterValue(
                    LaunchConfiguration('steering_lookahead_distance'), value_type=float),
                'max_obstacle_height': ParameterValue(max_obstacle_height_m, value_type=float),
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
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'ahead_zone_length_m': ParameterValue(
                    LaunchConfiguration('ahead_zone_length_m'), value_type=float),
            },
        ],
        condition=IfCondition(LaunchConfiguration('enable_moving_obstacle_stop')),
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        enable_moving_obstacle_stop_arg,
        dynamic_obstacle_detector_params_arg,
        path_obstacle_monitor_params_arg,
        lane_lookahead_arg,
        steering_lookahead_distance_arg,
        max_obstacle_height_ft_arg,
        ahead_zone_length_m_arg,
        costmap_lane_planner,
        dynamic_obstacle_detector,
        path_obstacle_monitor,
    ])
