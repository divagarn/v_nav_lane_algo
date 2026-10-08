"""RRT* counterpart of navigate_algo_theta_star.launch.py - same split
workflow: launch buggy_mapping's localize.launch.py FIRST (its own
terminal), verify/correct pose in RViz, THEN launch this file separately.

Adds controller_server (TEB, unchanged) + planner_server (kept running,
but ONLY for the /global_costmap/costmap it builds/publishes from
global_costmap_mapped.yaml - its own compute_path_to_pose action is never
targeted, see rrt_star_planner_node.py's own docstring for exactly why it
has to stay running anyway rather than being replaced outright) +
rrt_star_planner_node (this file's actual planner - a standalone action
server, NOT a planner_server plugin) + carrot_path_publisher.

Both nodes' action name is set via a real ROS PARAMETER
(rrt_star_planner's action_name, carrot_path_publisher's
planner_action_name), NOT a launch-time `remappings=` entry - confirmed
live that plain Node remaps do not rename an rclpy ActionServer OR
ActionClient (ros2 node info kept showing the un-remapped default name on
both ends after trying that first). Parametrizing the actual name on both
nodes is what actually separates this from planner_server's own
ALSO-running action of the same default name.

auto_return_to_start:=false (default here, same as the Theta* version) -
manual "2D Goal Pose" clicks, one leg at a time.

costmap_topic defaults to /local_costmap/costmap (controller_server's
rolling 30x30m window, live sensor data only - NO static map layer, see
local_costmap.yaml's own comment), not planner_server's /global_costmap/
costmap - both rrt_star_planner_node AND carrot_path_publisher's own
heading-picker read from this SAME topic, so what the planner searches
against and what the picker's raycasts see always agree. Trades away
awareness of walls outside current sensor range (no static layer to fall
back on) for costmap updates as fast as the sensor pipeline itself runs
(15Hz) instead of only at replan time - paired with carrot_path_publisher's
path-safety monitor (cancels + replans immediately, not just at the next
1Hz cycle, the moment the currently-executing path crosses an inflated/
lethal cell in a fresh costmap) and the pose-graph bias below, which
pulls candidate selection back toward where the vehicle was actually
driven during mapping - compensating for the lost static-layer awareness
by preferring known-good ground over unexplored-by-driving open space
instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_buggy_nav = get_package_share_directory('buggy_nav')

    costmap_params = os.path.join(pkg_buggy_nav, 'config', 'local_costmap.yaml')
    teb_params = os.path.join(pkg_buggy_nav, 'config', 'teb_controller.yaml')
    # Reused purely to give planner_server a valid GridBased plugin to load -
    # its action is never called in this launch (see module docstring).
    planner_server_params = os.path.join(pkg_buggy_nav, 'config', 'theta_star_planner_server.yaml')
    global_costmap_mapped_params = os.path.join(pkg_buggy_nav, 'config', 'global_costmap_mapped.yaml')
    rrt_star_params = os.path.join(pkg_buggy_nav, 'config', 'rrt_star_planner.yaml')

    carrot_distance_arg = DeclareLaunchArgument('carrot_distance', default_value='13.0')
    scan_distance_arg = DeclareLaunchArgument('scan_distance', default_value='13.0')
    resend_period_sec_arg = DeclareLaunchArgument(
        'resend_period_sec', default_value='0.5',
        description='How often request_plan re-evaluates and re-dispatches a path (seconds) '
                    '- 0.5 (default here) = 2Hz. Was hardcoded at 1.0s (1Hz); now tunable. '
                    'Raising the rate (lower value) costs more compute per second, more so '
                    'with use_reactive_chain:=true (that mode already does ~scan_distance / '
                    'waypoint_step_m heading-picks per cycle) - and makes goal_heading_alpha/'
                    'goal_heading_snap_deg converge to a new heading faster in wall-clock '
                    'time, since those are defined per-cycle, not per-second.')
    use_global_planner_arg = DeclareLaunchArgument(
        'use_global_planner', default_value='true',
        description='true (default here): RRT* plans against the full saved map (via its '
                    'own action, remapped in for carrot_path_publisher - see this file\'s '
                    'docstring). false: fall back to the straight-segment technique instead.')
    num_trials_arg = DeclareLaunchArgument(
        'num_trials', default_value='1',
        description='RRT* only: 1 (default) = single search per goal request, same as '
                    'always. >1: run that many independent trials for the same start/goal '
                    'and keep the lowest-cost result - mitigates RRT*\'s randomness '
                    'producing a worse path through a tight turn by chance. '
                    'planning_timeout_sec (rrt_star_planner.yaml) is divided across trials, '
                    'not multiplied, so total worst-case latency per goal request stays '
                    'about the same regardless of this value.')
    centering_weight_arg = DeclareLaunchArgument('centering_weight', default_value='2.0')
    corridor_check_distance_arg = DeclareLaunchArgument('corridor_check_distance', default_value='3.0')
    goal_safety_margin_arg = DeclareLaunchArgument(
        'goal_safety_margin', default_value='1.0',
        description='Placed goals stop this many meters short of the first unsafe '
                    '(lethal/inflated/unknown) costmap cell, instead of right at its edge - '
                    'see carrot_path_publisher.py\'s own comment for why this was needed '
                    '(confirmed live via /carrot_marker sitting inside the inflated zone).')
    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic', default_value='/local_costmap/costmap',
        description='Costmap both rrt_star_planner_node and carrot_path_publisher\'s '
                    'heading-picker read from. Default is the LOCAL costmap (live sensor '
                    'data, rolling window, no static map layer) - see this file\'s own '
                    'docstring for the trade-off. Set to /global_costmap/costmap to go '
                    'back to full-map-aware planning.')
    path_safety_check_arg = DeclareLaunchArgument(
        'path_safety_check', default_value='true',
        description='true (default): carrot_path_publisher cancels the current FollowPath '
                    'goal and replans immediately (not just at the next 1Hz cycle) the '
                    'moment a fresh costmap shows the executing path crossing an inflated/ '
                    'lethal cell.')
    path_safety_replan_cooldown_sec_arg = DeclareLaunchArgument(
        'path_safety_replan_cooldown_sec', default_value='0.5',
        description='Caps how often the immediate cancel-and-replan-on-unsafe-path '
                    'trigger above can actually fire (seconds) - confirmed live that '
                    'without this, a borderline path (checked at the costmap\'s own '
                    '15Hz update rate) could trigger a runaway cancel-replan loop that '
                    'never let the controller produce a stable command, looking like '
                    'the vehicle had simply stopped. Default matches resend_period_sec\'s '
                    'own cadence - a genuinely new obstacle is still caught, just not '
                    're-triggered faster than the controller can realistically respond.')
    pose_graph_weight_arg = DeclareLaunchArgument(
        'pose_graph_weight', default_value='1.5',
        description='Soft bias toward slam_toolbox\'s recorded pose graph (where the '
                    'vehicle was actually driven during mapping) - same weighted-bonus '
                    'pattern as centering_weight, applied in both reactive and goal-seeking '
                    'modes. 0 disables it entirely. Compensates for costmap_topic defaulting '
                    'to the local costmap (no static layer, no map-wide wall awareness).')
    pose_graph_max_influence_arg = DeclareLaunchArgument(
        'pose_graph_max_influence', default_value='5.0',
        description='Beyond this distance (m) from the nearest pose-graph vertex, the bias '
                    'contributes nothing (capped at 0, never a penalty - a soft preference, '
                    'not a hard exclusion).')
    pose_graph_stabilize_count_arg = DeclareLaunchArgument(
        'pose_graph_stabilize_count', default_value='2',
        description='Number of consecutive /slam_toolbox/graph_visualization callbacks '
                    'the vertex count must stay UNCHANGED before freezing it (see '
                    'pose_graph_callback\'s own "FIX #2" comment) - confirmed live this '
                    'was needed: freezing on the very first message was fragile if that '
                    'first message arrived before slam_toolbox finished publishing the '
                    'full loaded graph, permanently truncating it to a tiny count and '
                    'making the green line point back toward the start of the route for '
                    'the rest of the run.')
    waypoint_step_m_arg = DeclareLaunchArgument('waypoint_step_m', default_value='1.5')
    max_waypoint_steps_arg = DeclareLaunchArgument('max_waypoint_steps', default_value='50')
    goal_xy_tolerance_arg = DeclareLaunchArgument('goal_xy_tolerance', default_value='0.5')
    use_lane_following_arg = DeclareLaunchArgument('use_lane_following', default_value='false')
    lane_simplify_epsilon_arg = DeclareLaunchArgument('lane_simplify_epsilon', default_value='0.3')
    use_reactive_chain_arg = DeclareLaunchArgument(
        'use_reactive_chain', default_value='false',
        description='use_global_planner:=false, no /goal_pose set: false (default) picks '
                    'one single reactive carrot per cycle, same as always. true: build a '
                    'short multi-waypoint chain instead (same waypoint_step_m-spaced '
                    'repeated-heading-pick technique used when a real goal is set), '
                    'published on /sub_goals, reaching scan_distance ahead.')
    auto_return_to_start_arg = DeclareLaunchArgument(
        'auto_return_to_start', default_value='false',
        description='false (default here): plain single-goal behavior, one "2D Goal '
                    'Pose" click per leg. true: first goal reached auto-chains a '
                    'return-to-start leg (full closed loop).')
    target_speed_kmph_arg = DeclareLaunchArgument(
        'target_speed_kmph', default_value='4.0',
        description='Fixed speed the real buggy actually drives at (2 or 4 km/h are the '
                    'two real presets) - speed_governor forces the sim vehicle to this '
                    'exact speed whenever moving, since the real UART protocol has no '
                    'speed command at all (steering angle + stop/go flags only). Also '
                    'pinned into controller_server\'s own max_vel_x below (previously left '
                    'at teb_controller.yaml\'s 2.5 m/s default) - without this, TEB plans '
                    'trajectories assuming it can freely speed up/slow down, including '
                    'slowing to near-zero for a bend, which is exactly what made '
                    'speed_governor\'s atan2-recovered steering angle spike on ordinary '
                    'turns (small commanded speed in the denominator).')
    max_steering_rate_deg_s_arg = DeclareLaunchArgument(
        'max_steering_rate_deg_s', default_value='30.0',
        description='Slew-rate limit on speed_governor\'s output steering angle - caps how '
                    'many degrees/sec it is allowed to change by, so even a legitimate sharp '
                    'curvature change (new carrot point, replan) ramps in instead of jumping '
                    'instantly. A real steering actuator cannot snap either.')
    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance', default_value='4.0',
        description='How far ahead along /plan speed_governor looks for its pure-pursuit '
                    'steering target. Shorter = tighter path tracking but jerkier, longer = '
                    'smoother but cuts corners more - independent of carrot_distance (how far '
                    'ahead the PATH ITSELF is planned to).')
    turn_lookahead_m_arg = DeclareLaunchArgument(
        'turn_lookahead_m', default_value='9.0',
        description='How far ahead (meters) along slam_toolbox\'s recorded pose graph '
                    'carrot_path_publisher\'s /turn_direction and /turn_direction_marker '
                    'look to report the upcoming turn (LEFT/RIGHT/STRAIGHT) - see that '
                    'node\'s _pose_graph_lookahead_point for the full method.')
    route_preview_distance_m_arg = DeclareLaunchArgument(
        'route_preview_distance_m', default_value='40.0',
        description='How far ahead (meters) along slam_toolbox\'s recorded pose graph '
                    'carrot_path_publisher\'s green /route_preview_path line is drawn - '
                    'same walk as turn_lookahead_m above (shared '
                    '_pose_graph_lookahead_path), just a longer look further down the '
                    'road instead of only flagging the next single turn.')
    use_route_preview_reference_arg = DeclareLaunchArgument(
        'use_route_preview_reference', default_value='false',
        description='false (default): candidate_score biases toward the pose graph via '
                    'the EXISTING pose_graph_weight/pose_graph_max_influence mechanism '
                    '(nearest vertex ANYWHERE in the whole recorded graph, no direction '
                    'awareness). true: use the directional green /route_preview_path '
                    'line instead (reference_path_weight below) - the same walked, '
                    'direction-disambiguated path drawn in RViz, not just the nearest '
                    'point in the raw graph. Applies to both candidate_score branches '
                    '(reactive/no-goal and goal-seeking).')
    reference_path_weight_arg = DeclareLaunchArgument(
        'reference_path_weight', default_value='1.5',
        description='Weight for the route-preview-line bias above - only used when '
                    'use_route_preview_reference:=true. Same units/shape as '
                    'pose_graph_weight (capped by pose_graph_max_influence).')
    # --- Arc-based lane planner (only active when use_route_preview_reference
    # :=true AND no manual /goal_pose is set) - ported directly from
    # cart_navigation_ws's freespace_grid_node, see _plan_lane_arc's own
    # docstring. Defaults match that node's own tuned values.
    lane_wheelbase_arg = DeclareLaunchArgument('lane_wheelbase', default_value='1.6')
    lane_max_steering_deg_arg = DeclareLaunchArgument('lane_max_steering_deg', default_value='20.0')
    lane_lookahead_arg = DeclareLaunchArgument(
        'lane_lookahead', default_value='12.0',
        description='How far ahead (meters) each candidate arc is walked/checked for '
                    'clearance - independent of scan_distance/carrot_distance above, '
                    'which only apply to the straight-ray picker.')
    lane_num_candidates_arg = DeclareLaunchArgument('lane_num_candidates', default_value='15')
    corridor_scan_limit_arg = DeclareLaunchArgument(
        'corridor_scan_limit', default_value='8.0',
        description='How far left/right each arc sample scans to judge centering - '
                    'separate param from corridor_check_distance above, which only '
                    'applies to the straight-ray picker.')
    lane_straight_preference_ratio_arg = DeclareLaunchArgument(
        'lane_straight_preference_ratio', default_value='0.85',
        description='NO LONGER USED as the safety filter (see lane_min_safety_distance '
                    'below) - kept only so existing launch commands that still pass it '
                    'don\'t error out. See _plan_lane_arc\'s own docstring for why the '
                    'ratio-based filter was replaced: it excluded the genuinely correct '
                    'direction outright whenever it was a tighter/shorter-clearance '
                    'curving arc than an unrelated wide-open straight opening elsewhere.')
    lane_min_safety_distance_arg = DeclareLaunchArgument(
        'lane_min_safety_distance', default_value='2.0',
        description='ABSOLUTE minimum clear distance (meters) a candidate arc must '
                    'reach to be considered at all - the actual safety filter now. '
                    'Among everything that clears this floor, the candidate CLOSEST to '
                    'the route-preview line(s) wins (see reference_path_weight/'
                    'reference_path_mid_weight), with centering as a tiebreaker.')
    lane_candidate_inner_fraction_arg = DeclareLaunchArgument(
        'lane_candidate_inner_fraction', default_value='0.4',
        description='5 discrete candidates (forward, near/far left, near/far right - '
                    'see /lane_candidates in RViz) instead of a lane_num_candidates-way '
                    'continuous sweep. This is the near-left/near-right curvature, as a '
                    'fraction of lane_max_curvature.')
    lane_candidate_outer_fraction_arg = DeclareLaunchArgument(
        'lane_candidate_outer_fraction', default_value='1.0',
        description='Far-left/far-right candidate curvature, as a fraction of '
                    'lane_max_curvature (1.0 = the vehicle\'s actual max steering angle).')
    lane_smoothing_alpha_arg = DeclareLaunchArgument(
        'lane_smoothing_alpha', default_value='0.3',
        description='Exponential smoothing (0=frozen, 1=no smoothing) on the chosen '
                    'curvature between cycles, so the golden lane eases between arcs '
                    'instead of snapping.')
    lane_vehicle_half_width_arg = DeclareLaunchArgument('lane_vehicle_half_width', default_value='1.0')
    lane_vehicle_footprint_margin_arg = DeclareLaunchArgument(
        'lane_vehicle_footprint_margin', default_value='0.3')
    direction_bias_gain_arg = DeclareLaunchArgument(
        'direction_bias_gain', default_value='1.5',
        description='How strongly the arc planner leans toward the pose-graph-derived '
                    'turn direction (reference_yaw) when re-ranking candidates the '
                    'safety filter already judged comparably clear.')
    reference_yaw_cache_timeout_sec_arg = DeclareLaunchArgument(
        'reference_yaw_cache_timeout_sec', default_value='999999.0',
        description='How long (seconds) to keep using the last confidently-computed '
                    'reference_yaw (pose-graph bearing) when a cycle fails to produce a '
                    'fresh one (TF hiccup, every pose-graph vertex filtered as non-finite '
                    'this cycle, etc.) before giving up and falling back to no bias.')
    route_preview_cache_timeout_sec_arg = DeclareLaunchArgument(
        'route_preview_cache_timeout_sec', default_value='999999.0',
        description='Same idea as reference_yaw_cache_timeout_sec, one level up: how long '
                    'to keep reusing the last ACCEPTED green-line polyline (both the long '
                    'and mid lines) when a cycle fails to produce a fresh one or the fresh '
                    'one gets rejected as an implausible jump (see route_preview_max_jump_m).')
    route_preview_max_jump_m_arg = DeclareLaunchArgument(
        'route_preview_max_jump_m', default_value='3.0',
        description='A real vehicle cannot move further than this (meters) between two '
                    'consecutive ~resend_period_sec cycles - a fresh pose-graph walk whose '
                    'start point (nearest vertex found this cycle) jumps further than this '
                    'from the last accepted start point is rejected as a bad nearest-vertex '
                    'pick (e.g. snapping back toward the start of the recorded route) '
                    'rather than trusted, and the cached path is reused instead.')
    route_preview_distance_mid_m_arg = DeclareLaunchArgument(
        'route_preview_distance_mid_m', default_value='20.0',
        description='Second, shorter green reference line (/route_preview_path_mid) - '
                    'between turn_lookahead_m\'s single 9m bearing and route_preview_'
                    'distance_m\'s full 40m line, giving the arc planner a third, more '
                    'locally-reliable direction signal so one bad walk jumping onto the '
                    'wrong branch at a junction has less influence on the final choice.')
    reference_path_mid_weight_arg = DeclareLaunchArgument(
        'reference_path_mid_weight', default_value='1.0',
        description='Weight for the mid-length reference line bias above - only used '
                    'when use_route_preview_reference:=true.')

    carrot_distance = ParameterValue(LaunchConfiguration('carrot_distance'), value_type=float)
    scan_distance = ParameterValue(LaunchConfiguration('scan_distance'), value_type=float)
    resend_period_sec = ParameterValue(LaunchConfiguration('resend_period_sec'), value_type=float)
    use_global_planner = ParameterValue(LaunchConfiguration('use_global_planner'), value_type=bool)
    centering_weight = ParameterValue(LaunchConfiguration('centering_weight'), value_type=float)
    corridor_check_distance = ParameterValue(LaunchConfiguration('corridor_check_distance'), value_type=float)
    goal_safety_margin = ParameterValue(LaunchConfiguration('goal_safety_margin'), value_type=float)
    pose_graph_weight = ParameterValue(LaunchConfiguration('pose_graph_weight'), value_type=float)
    pose_graph_max_influence = ParameterValue(
        LaunchConfiguration('pose_graph_max_influence'), value_type=float)
    waypoint_step_m = ParameterValue(LaunchConfiguration('waypoint_step_m'), value_type=float)
    max_waypoint_steps = ParameterValue(LaunchConfiguration('max_waypoint_steps'), value_type=int)
    goal_xy_tolerance = ParameterValue(LaunchConfiguration('goal_xy_tolerance'), value_type=float)
    use_lane_following = ParameterValue(LaunchConfiguration('use_lane_following'), value_type=bool)
    lane_simplify_epsilon = ParameterValue(LaunchConfiguration('lane_simplify_epsilon'), value_type=float)
    use_reactive_chain = ParameterValue(LaunchConfiguration('use_reactive_chain'), value_type=bool)
    auto_return_to_start = ParameterValue(LaunchConfiguration('auto_return_to_start'), value_type=bool)
    target_speed_ms = PythonExpression([LaunchConfiguration('target_speed_kmph'), ' / 3.6'])

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[costmap_params, teb_params, {
            # Pinned to the same fixed speed speed_governor forces
            # downstream - see target_speed_kmph_arg's own comment above
            # for why (stops TEB from planning a freely-variable speed
            # profile that has no real-hardware equivalent).
            'max_vel_x': ParameterValue(target_speed_ms, value_type=float),
            'use_sim_time': True,
        }],
        remappings=[
            # NOT the final Gazebo topic anymore - speed_governor sits in
            # between and republishes the actual driven command onto
            # /ackermann_steering_controller/reference_unstamped. See that
            # node's own docstring for why (TEB's freely-varying speed has
            # no real-hardware equivalent - the real buggy only accepts a
            # steering angle, speed is fixed in the vehicle itself).
            ('cmd_vel', '/cmd_vel_raw'),
        ],
    )

    speed_governor = Node(
        package='buggy_nav',
        executable='speed_governor',
        name='speed_governor',
        output='screen',
        parameters=[{
            'target_speed_kmph': LaunchConfiguration('target_speed_kmph'),
            'wheelbase': 1.6,
            'max_steering_deg': 20.0,
            'max_steering_rate_deg_s': LaunchConfiguration('max_steering_rate_deg_s'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'use_sim_time': True,
        }],
        remappings=[
            ('cmd_vel_in', '/cmd_vel_raw'),
        ],
    )

    # Kept alive purely for its /global_costmap/costmap - see module
    # docstring. Its own compute_path_to_pose action is never targeted.
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[global_costmap_mapped_params, planner_server_params, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_global_planner')),
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_local_nav',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['planner_server', 'controller_server'],
        }],
        condition=IfCondition(LaunchConfiguration('use_global_planner')),
    )
    lifecycle_manager_no_planner = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_local_nav',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server'],
        }],
        condition=UnlessCondition(LaunchConfiguration('use_global_planner')),
    )

    rrt_star_planner = Node(
        package='buggy_nav',
        executable='rrt_star_planner_node',
        name='rrt_star_planner',
        output='screen',
        parameters=[rrt_star_params, {
            # A plain Node-level `remappings=` entry does NOT rename this
            # node's ActionServer - confirmed live (`ros2 node info` still
            # showed the plain /compute_path_to_pose after trying that,
            # unlike a topic/service remap on the same node, which DOES
            # take effect for rclpy). Setting the actual action name via
            # parameter instead (see rrt_star_planner_node.py's own
            # action_name parameter) is what actually separates this from
            # planner_server's own ALSO-running action of the same
            # default name (see module docstring for why planner_server
            # has to stay running anyway).
            'action_name': 'rrt_star_planner/compute_path_to_pose',
            'costmap_topic': LaunchConfiguration('costmap_topic'),
            'num_trials': ParameterValue(LaunchConfiguration('num_trials'), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration('use_global_planner')),
    )

    carrot_path_publisher = Node(
        package='buggy_nav',
        executable='carrot_path_publisher',
        name='carrot_path_publisher',
        output='screen',
        parameters=[{
            'carrot_distance': carrot_distance,
            'scan_distance': scan_distance,
            'resend_period_sec': resend_period_sec,
            'controller_id': 'FollowPath',
            'planner_id': 'GridBased',
            'global_planner_name': 'RRTStar',
            'planner_action_name': 'rrt_star_planner/compute_path_to_pose',
            'use_global_planner': use_global_planner,
            'costmap_topic': LaunchConfiguration('costmap_topic'),
            'path_safety_check': LaunchConfiguration('path_safety_check'),
            'path_safety_replan_cooldown_sec': ParameterValue(
                LaunchConfiguration('path_safety_replan_cooldown_sec'), value_type=float),
            'centering_weight': centering_weight,
            'corridor_check_distance': corridor_check_distance,
            'goal_safety_margin': goal_safety_margin,
            'pose_graph_weight': pose_graph_weight,
            'pose_graph_max_influence': pose_graph_max_influence,
            'pose_graph_stabilize_count': ParameterValue(
                LaunchConfiguration('pose_graph_stabilize_count'), value_type=int),
            'waypoint_step_m': waypoint_step_m,
            'max_waypoint_steps': max_waypoint_steps,
            'goal_xy_tolerance': goal_xy_tolerance,
            'use_lane_following': use_lane_following,
            'lane_simplify_epsilon': lane_simplify_epsilon,
            'use_reactive_chain': use_reactive_chain,
            'auto_return_to_start': auto_return_to_start,
            'turn_lookahead_m': ParameterValue(
                LaunchConfiguration('turn_lookahead_m'), value_type=float),
            'route_preview_distance_m': ParameterValue(
                LaunchConfiguration('route_preview_distance_m'), value_type=float),
            'use_route_preview_reference': ParameterValue(
                LaunchConfiguration('use_route_preview_reference'), value_type=bool),
            'reference_path_weight': ParameterValue(
                LaunchConfiguration('reference_path_weight'), value_type=float),
            'lane_wheelbase': ParameterValue(LaunchConfiguration('lane_wheelbase'), value_type=float),
            'lane_max_steering_deg': ParameterValue(
                LaunchConfiguration('lane_max_steering_deg'), value_type=float),
            'lane_lookahead': ParameterValue(LaunchConfiguration('lane_lookahead'), value_type=float),
            'lane_num_candidates': ParameterValue(
                LaunchConfiguration('lane_num_candidates'), value_type=int),
            'corridor_scan_limit': ParameterValue(
                LaunchConfiguration('corridor_scan_limit'), value_type=float),
            'lane_straight_preference_ratio': ParameterValue(
                LaunchConfiguration('lane_straight_preference_ratio'), value_type=float),
            'lane_min_safety_distance': ParameterValue(
                LaunchConfiguration('lane_min_safety_distance'), value_type=float),
            'lane_candidate_inner_fraction': ParameterValue(
                LaunchConfiguration('lane_candidate_inner_fraction'), value_type=float),
            'lane_candidate_outer_fraction': ParameterValue(
                LaunchConfiguration('lane_candidate_outer_fraction'), value_type=float),
            'lane_smoothing_alpha': ParameterValue(
                LaunchConfiguration('lane_smoothing_alpha'), value_type=float),
            'lane_vehicle_half_width': ParameterValue(
                LaunchConfiguration('lane_vehicle_half_width'), value_type=float),
            'lane_vehicle_footprint_margin': ParameterValue(
                LaunchConfiguration('lane_vehicle_footprint_margin'), value_type=float),
            'direction_bias_gain': ParameterValue(
                LaunchConfiguration('direction_bias_gain'), value_type=float),
            'reference_yaw_cache_timeout_sec': ParameterValue(
                LaunchConfiguration('reference_yaw_cache_timeout_sec'), value_type=float),
            'route_preview_cache_timeout_sec': ParameterValue(
                LaunchConfiguration('route_preview_cache_timeout_sec'), value_type=float),
            'route_preview_max_jump_m': ParameterValue(
                LaunchConfiguration('route_preview_max_jump_m'), value_type=float),
            'route_preview_distance_mid_m': ParameterValue(
                LaunchConfiguration('route_preview_distance_mid_m'), value_type=float),
            'reference_path_mid_weight': ParameterValue(
                LaunchConfiguration('reference_path_mid_weight'), value_type=float),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        carrot_distance_arg,
        scan_distance_arg,
        resend_period_sec_arg,
        use_global_planner_arg,
        num_trials_arg,
        centering_weight_arg,
        corridor_check_distance_arg,
        goal_safety_margin_arg,
        costmap_topic_arg,
        path_safety_check_arg,
        path_safety_replan_cooldown_sec_arg,
        pose_graph_weight_arg,
        pose_graph_max_influence_arg,
        pose_graph_stabilize_count_arg,
        waypoint_step_m_arg,
        max_waypoint_steps_arg,
        goal_xy_tolerance_arg,
        use_lane_following_arg,
        lane_simplify_epsilon_arg,
        use_reactive_chain_arg,
        auto_return_to_start_arg,
        target_speed_kmph_arg,
        max_steering_rate_deg_s_arg,
        lookahead_distance_arg,
        turn_lookahead_m_arg,
        route_preview_distance_m_arg,
        use_route_preview_reference_arg,
        reference_path_weight_arg,
        lane_wheelbase_arg,
        lane_max_steering_deg_arg,
        lane_lookahead_arg,
        lane_num_candidates_arg,
        corridor_scan_limit_arg,
        lane_straight_preference_ratio_arg,
        lane_min_safety_distance_arg,
        lane_candidate_inner_fraction_arg,
        lane_candidate_outer_fraction_arg,
        lane_smoothing_alpha_arg,
        lane_vehicle_half_width_arg,
        lane_vehicle_footprint_margin_arg,
        direction_bias_gain_arg,
        reference_yaw_cache_timeout_sec_arg,
        route_preview_cache_timeout_sec_arg,
        route_preview_max_jump_m_arg,
        route_preview_distance_mid_m_arg,
        reference_path_mid_weight_arg,
        controller_server,
        speed_governor,
        planner_server,
        lifecycle_manager,
        lifecycle_manager_no_planner,
        rrt_star_planner,
        carrot_path_publisher,
    ])
