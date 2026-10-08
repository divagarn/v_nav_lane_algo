"""2D SLAM mapping from a recorded rosbag, instead of live sensors (see
mapping_2d.launch.py for the Gazebo version, real_mapping_2d.launch.py for
live real hardware). Same downstream pipeline as both of those: segment_ground
(toggleable) -> self_hit_filter -> pointcloud_to_laserscan -> slam_toolbox
(async, MAPPING mode) - only the SOURCE of /velodyne_points and the
odom->base_link TF changes here.

REQUIRED BAG TOPICS
-------------------
  /velodyne_points   (sensor_msgs/PointCloud2, frame_id 'velodyne')
      Raw 3D LiDAR cloud - fed into segment_ground/self_hit_filter exactly
      like it would be live. Always required.

  /wheel_uart   (ground_segmentation_msgs/SteerSpeed)
      Steering (deg, +right) + speed (km/h) feedback. Required ONLY if
      use_wheel_odometry:=true (the default) - this launch runs the same
      wheel_odometry node real hardware uses, live, to integrate a FRESH
      odom->base_link TF from it every run (so retuning wheelbase, or a
      wheel_odometry bug fix, can be replayed against the same recording
      without re-driving). If you pass use_wheel_odometry:=false instead,
      this topic is not used at all - but then the bag itself MUST already
      contain a valid, complete odom->base_link /tf stream recorded live
      during that same run, since nothing in this launch will produce one.

NOT REQUIRED (this launch supplies them itself, live)
------------------------------------------------------
  /tf_static (base_link->velodyne) - (re)published by velodyne_static_tf
      on every run regardless of what the bag has (idempotent, harmless to
      publish twice) - see publish_static_tf:=false if you're certain the
      bag already has it and want to skip the duplicate.
  /clock - published by `ros2 bag play --clock` itself; every node in this
      launch runs with use_sim_time:=true so the whole pipeline (segment_
      ground, self_hit_filter, slam_toolbox, TF) ticks off the bag's own
      recorded timeline, not wall-clock - this is what makes --rate and
      --start-paused actually control the whole pipeline's speed, not just
      the point cloud.

Check what a given bag actually contains before running:
    ros2 bag info <bag_path>

NOTE: with use_sim_time:=true, any node's first log lines can appear to
"hang" for a couple seconds until bag playback actually starts publishing
/clock - this is normal, not a crash. bag_start_delay below exists so the
perception/SLAM nodes are already up and subscribed BEFORE that first
/clock tick and the first /velodyne_points message arrive, so nothing is
silently dropped while nodes are still discovering each other over DDS.

Save exactly like the other two mapping launches, via slam_toolbox's own
serialization service (see buggy_mapping/maps/README.md for why this isn't
nav2_map_server's map_saver_cli):

    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \\
        "{filename: '<path-to-buggy_mapping>/maps/<name>'}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_buggy_mapping = get_package_share_directory('buggy_mapping')

    slam_params = os.path.join(pkg_buggy_mapping, 'config', 'slam_toolbox_mapping.yaml')
    p2l_params = os.path.join(pkg_buggy_mapping, 'config', 'pointcloud_to_laserscan.yaml')
    rviz_config = os.path.join(pkg_buggy_mapping, 'rviz', 'mapping.rviz')

    # No default - there's no sensible one, and requiring it explicitly
    # (launch fails fast with a clear "required argument not provided"
    # instead of silently mapping nothing) is safer than guessing a path.
    bag_path_arg = DeclareLaunchArgument(
        'bag_path', description='Path to the rosbag2 directory (the one containing '
                                 'metadata.yaml) to play - REQUIRED.')
    rate_arg = DeclareLaunchArgument(
        'rate', default_value='1.0',
        description='Playback speed multiplier passed to ros2 bag play - slam_toolbox\'s '
                    'own timing params (minimum_time_interval etc.) are tuned for '
                    'real-time, so values far from 1.0 may affect scan-matching quality.')
    loop_arg = DeclareLaunchArgument(
        'loop', default_value='false',
        description='Loop bag playback - useful for iterating on ground_filter/wheelbase '
                    'params against the same recording without relaunching everything.')
    start_paused_arg = DeclareLaunchArgument(
        'start_paused', default_value='false',
        description='Start ros2 bag play paused (space to resume in its own terminal, or '
                    "'ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume "
                    '{}\' from another) - useful to confirm RViz/TF look right before any '
                    'data actually starts feeding SLAM.')
    bag_start_delay_arg = DeclareLaunchArgument(
        'bag_start_delay', default_value='3.0',
        description='Seconds to wait after launching the perception/SLAM nodes before '
                    'starting bag playback - gives them time to finish discovering each '
                    "other over DDS so the bag's first messages aren't silently missed.")
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    ground_filter_arg = DeclareLaunchArgument(
        'ground_filter', default_value='true',
        description='false: skip segment_ground - self_hit_filter runs on the bag\'s raw '
                    '/velodyne_points instead. Recommended true for a clean map.')
    publish_static_tf_arg = DeclareLaunchArgument(
        'publish_static_tf', default_value='true',
        description='(Re)publish base_link->velodyne via velodyne_static_tf regardless of '
                    "whether the bag itself has it - idempotent, safe to leave true. Set "
                    'false only if you specifically want to rely on the bag\'s own recorded '
                    '/tf_static instead.')
    use_wheel_odometry_arg = DeclareLaunchArgument(
        'use_wheel_odometry', default_value='true',
        description='true (default): run wheel_odometry live off the bag\'s /wheel_uart to '
                    'produce a fresh odom->base_link TF every run. false: trust the TF '
                    "already recorded in the bag instead - the bag MUST contain a complete "
                    "odom->base_link stream in that case, and /wheel_uart is ignored.")
    enable_kiss_icp_arg = DeclareLaunchArgument(
        'enable_kiss_icp', default_value='false',
        description='true: build odom->base_link from LiDAR scan registration (kiss_icp_node '
                    'on the bag\'s /velodyne_points) instead of wheel dead-reckoning. This '
                    'reduces map drift caused by wheel slip. Disables wheel_odometry, and '
                    'plays ONLY /velodyne_points from the bag so any recorded /tf does not '
                    'conflict with the live TF. slam_toolbox loop closure still corrects '
                    'drift when the loop is revisited.')
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase', default_value='1.6',
        description='Passed to wheel_odometry when use_wheel_odometry:=true - must match '
                    'the vehicle\'s real wheelbase (meters), same param it uses live.')

    non_ground_points_source = PythonExpression([
        "'/non_ground_points' if '", LaunchConfiguration('ground_filter'),
        "' == 'true' else '/velodyne_points'"])
    # Builds the optional trailing flags for ros2 bag play as one string so
    # they can be appended unconditionally in a single ExecuteProcess -
    # avoids needing a separate action per true/false combination.
    extra_play_flags = PythonExpression([
        "('--loop ' if '", LaunchConfiguration('loop'), "' == 'true' else '') + ",
        "('--start-paused' if '", LaunchConfiguration('start_paused'), "' == 'true' else '')"])

    velodyne_static_tf = Node(
        package='buggy_perception',
        executable='velodyne_static_tf',
        name='velodyne_static_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('publish_static_tf')),
    )

    wheel_odometry_enabled = PythonExpression([
        "'true' if '", LaunchConfiguration('use_wheel_odometry'),
        "' == 'true' and '", LaunchConfiguration('enable_kiss_icp'), "' != 'true' else 'false'"])

    wheel_odometry = Node(
        package='buggy_perception',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen',
        parameters=[{'wheelbase': LaunchConfiguration('wheelbase'), 'use_sim_time': True}],
        condition=IfCondition(wheel_odometry_enabled),
    )

    # Same frame setup as real_sensors_bringup.launch.py / sim: base_link
    # is the parent of the velodyne frame (velodyne_static_tf), and the
    # TF kiss publishes is odom->base_link, the slot slam_toolbox reads.
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

    segment_ground = Node(
        package='buggy_perception',
        executable='segment_ground',
        name='segment_ground',
        output='screen',
        parameters=[{'use_sim_time': True}],
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
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
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

    # Delayed relative to everything above (see bag_start_delay's own
    # description) rather than started in parallel with it.
    # With kiss_icp, play only /velodyne_points so the bag's recorded /tf
    # (its own odom->base_link from the original run) cannot fight the
    # live kiss TF. Without it, behavior is unchanged (whole bag).
    topic_play_flags = PythonExpression([
        "'--topics /velodyne_points' if '", LaunchConfiguration('enable_kiss_icp'),
        "' == 'true' else ''"])

    bag_play = TimerAction(
        period=LaunchConfiguration('bag_start_delay'),
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', [
                    'ros2 bag play "', LaunchConfiguration('bag_path'), '" --clock --rate ',
                    LaunchConfiguration('rate'), ' ', extra_play_flags, ' ', topic_play_flags,
                ]],
                name='bag_play',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        bag_path_arg,
        rate_arg,
        loop_arg,
        start_paused_arg,
        bag_start_delay_arg,
        rviz_arg,
        ground_filter_arg,
        publish_static_tf_arg,
        use_wheel_odometry_arg,
        wheelbase_arg,
        enable_kiss_icp_arg,
        velodyne_static_tf,
        wheel_odometry,
        kiss_icp_node,
        segment_ground,
        self_hit_filter,
        pointcloud_to_laserscan,
        slam_toolbox,
        rviz2,
        bag_play,
    ])
