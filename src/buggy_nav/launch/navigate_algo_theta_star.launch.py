"""Navigation-algo layer: launch buggy_mapping's localize.launch.py FIRST
(in its own terminal), verify/correct the vehicle's pose in RViz against
the loaded map, THEN launch this file separately. This file has no spawn,
no perception, no localization of its own - it only assumes
/non_ground_points_filtered, /smart_radar/can_targets_0, /map, and a live
map->odom transform are already being published by localize.launch.py,
and adds controller_server (TEB) + planner_server (ThetaStarPlanner, see
buggy_nav/config/theta_star_planner_server.yaml) + carrot_path_publisher
on top.

auto_return_to_start:=false (default here, UNLIKE the old workspace's
equivalent launch) - "I will give the goal manually as of now": set a
goal yourself via RViz's "2D Goal Pose" each time, single-leg point-to-
point, rather than auto-chaining a return-to-start leg. Flip to true
later once manual goal-setting is confirmed working, for the full
drive-a-lap-and-back behavior.

carrot_path_publisher is what actually produces the CENTERED path: with a
goal set, it scores candidate headings by distance-to-goal plus a
centering term (candidate_centering, weighted by centering_weight) that
checks clearance to both sides of each candidate's prospective endpoint -
this is what pulls the path away from hugging either wall. See that
node's own module docstring for the full rationale of every technique it
supports (centerline_lateral_bias for straight sections, use_lane_following
for the medial-axis-skeleton alternative) - none of that changes here,
only which planner_server plugin is behind planner_id: 'GridBased'.

cmd_vel goes to /ackermann_steering_controller/reference_unstamped (sim's
Gazebo ros2_control topic, plain Twist not TwistStamped) - the same
topic buggy_description's own gazebo.launch.py docstring documents for
manual teleop, so driving via nav goals and manual teleop never conflict
on topic naming.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_buggy_nav = get_package_share_directory('buggy_nav')

    costmap_params = os.path.join(pkg_buggy_nav, 'config', 'local_costmap.yaml')
    teb_params = os.path.join(pkg_buggy_nav, 'config', 'teb_controller.yaml')
    planner_server_params = os.path.join(pkg_buggy_nav, 'config', 'theta_star_planner_server.yaml')
    global_costmap_mapped_params = os.path.join(pkg_buggy_nav, 'config', 'global_costmap_mapped.yaml')

    carrot_distance_arg = DeclareLaunchArgument('carrot_distance', default_value='13.0')
    scan_distance_arg = DeclareLaunchArgument('scan_distance', default_value='13.0')
    use_global_planner_arg = DeclareLaunchArgument(
        'use_global_planner', default_value='true',
        description='true (default here): ThetaStarPlanner plans against the full saved '
                    'map. false: fall back to the straight-segment technique instead.')
    centering_weight_arg = DeclareLaunchArgument('centering_weight', default_value='2.0')
    corridor_check_distance_arg = DeclareLaunchArgument('corridor_check_distance', default_value='3.0')
    goal_safety_margin_arg = DeclareLaunchArgument(
        'goal_safety_margin', default_value='1.0',
        description='Placed goals stop this many meters short of the first unsafe '
                    '(lethal/inflated/unknown) costmap cell, instead of right at its edge - '
                    'see carrot_path_publisher.py\'s own comment for why this was needed.')
    waypoint_step_m_arg = DeclareLaunchArgument('waypoint_step_m', default_value='1.5')
    max_waypoint_steps_arg = DeclareLaunchArgument('max_waypoint_steps', default_value='50')
    goal_xy_tolerance_arg = DeclareLaunchArgument('goal_xy_tolerance', default_value='0.5')
    use_lane_following_arg = DeclareLaunchArgument('use_lane_following', default_value='false')
    lane_simplify_epsilon_arg = DeclareLaunchArgument('lane_simplify_epsilon', default_value='0.3')
    auto_return_to_start_arg = DeclareLaunchArgument(
        'auto_return_to_start', default_value='false',
        description='false (default here): plain single-goal behavior, one "2D Goal '
                    'Pose" click per leg - manual goal-setting workflow. true: first '
                    'goal reached auto-chains a return-to-start leg (full closed loop).')

    carrot_distance = ParameterValue(LaunchConfiguration('carrot_distance'), value_type=float)
    scan_distance = ParameterValue(LaunchConfiguration('scan_distance'), value_type=float)
    use_global_planner = ParameterValue(LaunchConfiguration('use_global_planner'), value_type=bool)
    centering_weight = ParameterValue(LaunchConfiguration('centering_weight'), value_type=float)
    corridor_check_distance = ParameterValue(LaunchConfiguration('corridor_check_distance'), value_type=float)
    goal_safety_margin = ParameterValue(LaunchConfiguration('goal_safety_margin'), value_type=float)
    waypoint_step_m = ParameterValue(LaunchConfiguration('waypoint_step_m'), value_type=float)
    max_waypoint_steps = ParameterValue(LaunchConfiguration('max_waypoint_steps'), value_type=int)
    goal_xy_tolerance = ParameterValue(LaunchConfiguration('goal_xy_tolerance'), value_type=float)
    use_lane_following = ParameterValue(LaunchConfiguration('use_lane_following'), value_type=bool)
    lane_simplify_epsilon = ParameterValue(LaunchConfiguration('lane_simplify_epsilon'), value_type=float)
    auto_return_to_start = ParameterValue(LaunchConfiguration('auto_return_to_start'), value_type=bool)

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[costmap_params, teb_params, {'use_sim_time': True}],
        remappings=[
            ('cmd_vel', '/ackermann_steering_controller/reference_unstamped'),
        ],
    )

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

    carrot_path_publisher = Node(
        package='buggy_nav',
        executable='carrot_path_publisher',
        name='carrot_path_publisher',
        output='screen',
        parameters=[{
            'carrot_distance': carrot_distance,
            'scan_distance': scan_distance,
            'resend_period_sec': 1.0,
            'controller_id': 'FollowPath',
            'planner_id': 'GridBased',
            'global_planner_name': 'ThetaStarPlanner',
            'use_global_planner': use_global_planner,
            'centering_weight': centering_weight,
            'goal_safety_margin': goal_safety_margin,
            'corridor_check_distance': corridor_check_distance,
            'waypoint_step_m': waypoint_step_m,
            'max_waypoint_steps': max_waypoint_steps,
            'goal_xy_tolerance': goal_xy_tolerance,
            'use_lane_following': use_lane_following,
            'lane_simplify_epsilon': lane_simplify_epsilon,
            'auto_return_to_start': auto_return_to_start,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        carrot_distance_arg,
        scan_distance_arg,
        use_global_planner_arg,
        centering_weight_arg,
        corridor_check_distance_arg,
        goal_safety_margin_arg,
        waypoint_step_m_arg,
        max_waypoint_steps_arg,
        goal_xy_tolerance_arg,
        use_lane_following_arg,
        lane_simplify_epsilon_arg,
        auto_return_to_start_arg,
        controller_server,
        planner_server,
        lifecycle_manager,
        lifecycle_manager_no_planner,
        carrot_path_publisher,
    ])
