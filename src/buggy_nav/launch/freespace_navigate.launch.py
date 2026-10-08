"""Reactive navigation, ported exactly from cart_navigation_ws's own
freespace_demo_localized.launch.py pipeline - replaces carrot_path_
publisher/controller_server/TEB/RRT* entirely for driving. Launch
buggy_mapping's localize.launch.py FIRST (its own terminal - Gazebo +
Velodyne + segment_ground + slam_toolbox LOCALIZATION + the saved pose
graph), THEN this file separately, same two-terminal split every other
launch in this package uses.

Nodes launched here:
  1. freespace_grid_node (buggy_nav) - its own live, from-scratch log-odds
     occupancy grid built directly from segment_ground's /non_ground_points
     + /ground_points (NOT Nav2's costmap), arc-based _plan_lane
     candidate evaluation, stuck/recovery state machine, publishes
     /target_lateral_position (the pure-pursuit target for steering_
     calculator_node) and /freespace_grid_markers (RViz visualization:
     the grid itself, the suggested_lane, the vehicle's real footprint,
     the safety-region inflation halo). Pose-graph turn-direction
     awareness (/turn_direction, /turn_distance_m, /route_preview_path -
     the green line, the pose-graph vertex-alignment markers) is now
     MERGED INTO this node (enable_pose_graph_bias, default true) rather
     than a separate pose_graph_turn_direction_node - confirmed live that
     running it as two independently-timed nodes meant each did its own
     separate TF lookup for the vehicle's current pose, and on weaker
     real-hardware compute the gap between those two reads could grow
     enough that the lane planner and the direction bias were reasoning
     about the robot being in two subtly different places in the same
     instant. Merged, both use the exact same TF read every cycle - see
     freespace_grid_node.py's own enable_pose_graph_bias comment. The
     standalone pose_graph_turn_direction_node (buggy_perception) still
     exists and works if something else ever needs /turn_direction
     independent of this node, but is not launched here.
  2. steering_calculator_node (buggy_perception) - pure-pursuit + PID
     against real/simulated wheel feedback, consumes /target_lateral_
     position, publishes /steering_angle (degrees) + /vehicle_speed
     (km/h). Gated on /navigation_enabled (manual_pose_aligner's 'g'/'x'
     keys) - the vehicle will not move just because this launch started.
  3. sim_ackermann_bridge (buggy_nav) - converts /steering_angle +
     /vehicle_speed into the Twist Gazebo's ackermann_steering_controller
     actually consumes (same /ackermann_steering_controller/reference_
     unstamped topic buggy_nav's own speed_governor.py already uses, so
     no Gazebo/robot-description changes needed).

Does NOT launch Gazebo, segment_ground, slam_toolbox, or the pose graph
itself - localize.launch.py's job. Does NOT launch controller_server,
planner_server, TEB, RRT*, or carrot_path_publisher - this pipeline
replaces all of that for reactive driving.

Confirm alignment in RViz (already open from localize.launch.py), then run
manual_pose_aligner (own terminal) and press 'g' once satisfied - nothing
here drives the vehicle until that gate is set.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_buggy_mapping = get_package_share_directory('buggy_mapping')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='localize.launch.py already opens RViz by default - leave this '
                    'false to avoid a second window, true only if that one was '
                    'started with rviz:=false.')
    avoid_ground_arg = DeclareLaunchArgument(
        'avoid_ground', default_value='false',
        description='true: freespace_grid_node ignores /ground_points entirely, only '
                    'marks the grid from obstacle rays.')
    max_scan_process_rate_hz_arg = DeclareLaunchArgument(
        'max_scan_process_rate_hz', default_value='0.0',
        description='0 (default): unlimited, process every incoming scan - correct for '
                    'sim, where /non_ground_points and /ground_points have been arriving '
                    'much slower than real hardware\'s native rate. Set to a real '
                    'hardware-appropriate rate (e.g. 10.0) there instead - _mark_rays is '
                    'still a plain Python per-point ray-tracing loop, and processing a '
                    'backlogged scan against a pose that\'s no longer current is worse '
                    'than dropping it outright. See freespace_grid_node.py\'s own '
                    'max_scan_process_rate_hz comment.')
    # --- pose-graph turn direction (now merged into freespace_grid_node) ---
    enable_pose_graph_bias_arg = DeclareLaunchArgument(
        'enable_pose_graph_bias', default_value='true',
        description='true (default): freespace_grid_node reads slam_toolbox\'s pose '
                    'graph and biases candidate arcs toward the recorded route '
                    '(direction_bias_gain/reference_path_weight). false: purely '
                    'reactive, same as if no map/localization were running at all.')
    turn_lookahead_m_arg = DeclareLaunchArgument('turn_lookahead_m', default_value='9.0')
    route_preview_distance_m_arg = DeclareLaunchArgument('route_preview_distance_m', default_value='40.0')
    nearest_vertex_max_dist_arg = DeclareLaunchArgument('nearest_vertex_max_dist', default_value='5.0')
    snap_deg_arg = DeclareLaunchArgument('snap_deg', default_value='25.0')
    direction_disambiguation_vertices_arg = DeclareLaunchArgument(
        'direction_disambiguation_vertices', default_value='5')
    pose_graph_stabilize_count_arg = DeclareLaunchArgument(
        'pose_graph_stabilize_count', default_value='2',
        description='Consecutive graph_visualization callbacks the vertex count must '
                    'stay unchanged before freezing it - see freespace_grid_node.py\'s '
                    '_pose_graph_callback for why freezing on the very first message '
                    'was fragile.')
    pose_graph_history_depth_arg = DeclareLaunchArgument(
        'pose_graph_history_depth', default_value='3',
        description='How many recent ACCEPTED pose-graph walk start points to keep '
                    '(per walk - turn-direction and route-preview tracked separately). '
                    'A fresh reading is only accepted if it\'s within route_preview_'
                    'max_jump_m of the MEDIAN of this history, not just the single '
                    'most recent entry.')
    pose_graph_reset_streak_threshold_arg = DeclareLaunchArgument(
        'pose_graph_reset_streak_threshold', default_value='3',
        description='If this many consecutive REJECTED readings all agree with EACH '
                    'OTHER, that\'s treated as evidence the existing history was a bad '
                    'seed (not that the new readings are noise) - history resets to '
                    'trust the new cluster instead of entrenching a wrong value '
                    'forever. Closes a confirmed self-poisoning failure mode: without '
                    'this, a single bad first reading could permanently reject every '
                    'correct subsequent one.')
    route_preview_max_jump_m_arg = DeclareLaunchArgument(
        'route_preview_max_jump_m', default_value='3.0',
        description='A real vehicle cannot move further than this (meters) between '
                    'consecutive cycles - a fresh walk whose start point jumps further '
                    'than this from the recent-history median is rejected as an '
                    'implausible nearest-vertex pick (e.g. snapping back toward the '
                    'start of the recorded route) rather than trusted.')
    route_preview_cache_timeout_sec_arg = DeclareLaunchArgument(
        'route_preview_cache_timeout_sec', default_value='999999.0',
        description='How long to keep reusing the last accepted pose-graph walk when a '
                    'cycle fails to produce a fresh one or the fresh one gets rejected '
                    'as an implausible jump - effectively unbounded by default (keep '
                    'the last known-good path until a genuinely new one replaces it).')

    # --- freespace_grid_node ---
    resolution_arg = DeclareLaunchArgument('resolution', default_value='0.3')
    grid_half_extent_arg = DeclareLaunchArgument('grid_half_extent', default_value='60.0')
    max_ray_length_arg = DeclareLaunchArgument('max_ray_length', default_value='50.0')
    vehicle_half_width_arg = DeclareLaunchArgument('vehicle_half_width', default_value='1.0')
    vehicle_half_length_arg = DeclareLaunchArgument('vehicle_half_length', default_value='1.8')
    vehicle_footprint_margin_arg = DeclareLaunchArgument('vehicle_footprint_margin', default_value='0.3')
    show_safety_region_arg = DeclareLaunchArgument(
        'show_safety_region', default_value='true',
        description='Costmap-style orange inflation halo around every occupied cell '
                    'in /freespace_grid_markers, at exactly the radius (vehicle_half_'
                    'width + vehicle_footprint_margin) the arc planner already '
                    'enforces - makes the safety margin actually visible in RViz.')
    safety_region_alpha_arg = DeclareLaunchArgument('safety_region_alpha', default_value='0.35')
    wheelbase_arg = DeclareLaunchArgument('wheelbase', default_value='1.6')
    max_steering_deg_arg = DeclareLaunchArgument('max_steering_deg', default_value='20.0')
    lane_lookahead_arg = DeclareLaunchArgument('lane_lookahead', default_value='12.0')
    lane_num_candidates_arg = DeclareLaunchArgument('lane_num_candidates', default_value='15')
    direction_bias_gain_arg = DeclareLaunchArgument('direction_bias_gain', default_value='1.5')
    reference_path_weight_arg = DeclareLaunchArgument('reference_path_weight', default_value='1.5')
    decay_half_life_sec_arg = DeclareLaunchArgument(
        'decay_half_life_sec', default_value='0.0',
        description='0 (default): disabled, indefinite memory within the recentering '
                    'window. Confirmed live in cart_navigation_ws that enabling this '
                    'made the grid stop matching the recorded map.')

    # --- steering_calculator_node ---
    lookahead_distance_arg = DeclareLaunchArgument('lookahead_distance', default_value='4.0')
    target_speed_kmph_arg = DeclareLaunchArgument('target_speed_kmph', default_value='4.0')

    freespace_grid = Node(
        package='buggy_nav',
        executable='freespace_grid_node',
        name='freespace_grid_node',
        output='screen',
        parameters=[{
            'fixed_frame': 'odom',
            'base_frame': 'base_link',
            'avoid_ground': LaunchConfiguration('avoid_ground'),
            'max_scan_process_rate_hz': ParameterValue(
                LaunchConfiguration('max_scan_process_rate_hz'), value_type=float),
            # self_hit_filter's output, not segment_ground's raw
            # /non_ground_points - same filtered stream localize.launch.py's
            # pointcloud_to_laserscan already consumes, so the vehicle's own
            # chassis/mount returns never get marked as obstacles here either.
            'non_ground_topic': '/non_ground_points_filtered',
            'ground_topic': '/ground_points',
            'resolution': ParameterValue(LaunchConfiguration('resolution'), value_type=float),
            'grid_half_extent': ParameterValue(LaunchConfiguration('grid_half_extent'), value_type=float),
            'max_ray_length': ParameterValue(LaunchConfiguration('max_ray_length'), value_type=float),
            'vehicle_half_width': ParameterValue(LaunchConfiguration('vehicle_half_width'), value_type=float),
            'vehicle_half_length': ParameterValue(LaunchConfiguration('vehicle_half_length'), value_type=float),
            'vehicle_footprint_margin': ParameterValue(
                LaunchConfiguration('vehicle_footprint_margin'), value_type=float),
            'show_safety_region': LaunchConfiguration('show_safety_region'),
            'safety_region_alpha': ParameterValue(
                LaunchConfiguration('safety_region_alpha'), value_type=float),
            'wheelbase': ParameterValue(LaunchConfiguration('wheelbase'), value_type=float),
            'max_steering_deg': ParameterValue(LaunchConfiguration('max_steering_deg'), value_type=float),
            'lane_lookahead': ParameterValue(LaunchConfiguration('lane_lookahead'), value_type=float),
            'lane_num_candidates': ParameterValue(
                LaunchConfiguration('lane_num_candidates'), value_type=int),
            'direction_bias_gain': ParameterValue(
                LaunchConfiguration('direction_bias_gain'), value_type=float),
            'reference_path_weight': ParameterValue(
                LaunchConfiguration('reference_path_weight'), value_type=float),
            'decay_half_life_sec': ParameterValue(
                LaunchConfiguration('decay_half_life_sec'), value_type=float),
            'steering_lookahead_distance': ParameterValue(
                LaunchConfiguration('lookahead_distance'), value_type=float),
            'enable_pose_graph_bias': LaunchConfiguration('enable_pose_graph_bias'),
            'map_frame': 'map',
            'turn_lookahead_m': ParameterValue(LaunchConfiguration('turn_lookahead_m'), value_type=float),
            'route_preview_distance_m': ParameterValue(
                LaunchConfiguration('route_preview_distance_m'), value_type=float),
            'nearest_vertex_max_dist': ParameterValue(
                LaunchConfiguration('nearest_vertex_max_dist'), value_type=float),
            'snap_deg': ParameterValue(LaunchConfiguration('snap_deg'), value_type=float),
            'direction_disambiguation_vertices': ParameterValue(
                LaunchConfiguration('direction_disambiguation_vertices'), value_type=int),
            'pose_graph_stabilize_count': ParameterValue(
                LaunchConfiguration('pose_graph_stabilize_count'), value_type=int),
            'pose_graph_history_depth': ParameterValue(
                LaunchConfiguration('pose_graph_history_depth'), value_type=int),
            'pose_graph_reset_streak_threshold': ParameterValue(
                LaunchConfiguration('pose_graph_reset_streak_threshold'), value_type=int),
            'route_preview_max_jump_m': ParameterValue(
                LaunchConfiguration('route_preview_max_jump_m'), value_type=float),
            'route_preview_cache_timeout_sec': ParameterValue(
                LaunchConfiguration('route_preview_cache_timeout_sec'), value_type=float),
            'use_sim_time': True,
        }],
    )

    steering_calculator = Node(
        package='buggy_perception',
        executable='steering_calculator_node',
        name='steering_calculator',
        output='screen',
        parameters=[{
            'wheelbase': ParameterValue(LaunchConfiguration('wheelbase'), value_type=float),
            'max_steering_angle': ParameterValue(LaunchConfiguration('max_steering_deg'), value_type=float),
            'lookahead_distance': ParameterValue(LaunchConfiguration('lookahead_distance'), value_type=float),
            'use_fixed_speed': True,
            'fixed_speed_kmh': ParameterValue(LaunchConfiguration('target_speed_kmph'), value_type=float),
            'use_sim_time': True,
        }],
    )

    sim_ackermann_bridge = Node(
        package='buggy_nav',
        executable='sim_ackermann_bridge',
        name='sim_ackermann_bridge',
        output='screen',
        parameters=[{
            'wheelbase': ParameterValue(LaunchConfiguration('wheelbase'), value_type=float),
            'use_sim_time': True,
        }],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_buggy_mapping, 'rviz', 'closed_loop_navigate.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        rviz_arg,
        avoid_ground_arg,
        max_scan_process_rate_hz_arg,
        enable_pose_graph_bias_arg,
        turn_lookahead_m_arg,
        route_preview_distance_m_arg,
        nearest_vertex_max_dist_arg,
        snap_deg_arg,
        direction_disambiguation_vertices_arg,
        pose_graph_stabilize_count_arg,
        pose_graph_history_depth_arg,
        pose_graph_reset_streak_threshold_arg,
        route_preview_max_jump_m_arg,
        route_preview_cache_timeout_sec_arg,
        resolution_arg,
        grid_half_extent_arg,
        max_ray_length_arg,
        vehicle_half_width_arg,
        vehicle_half_length_arg,
        vehicle_footprint_margin_arg,
        show_safety_region_arg,
        safety_region_alpha_arg,
        wheelbase_arg,
        max_steering_deg_arg,
        lane_lookahead_arg,
        lane_num_candidates_arg,
        direction_bias_gain_arg,
        reference_path_weight_arg,
        decay_half_life_sec_arg,
        lookahead_distance_arg,
        target_speed_kmph_arg,
        freespace_grid,
        steering_calculator,
        sim_ackermann_bridge,
        rviz2,
    ])
