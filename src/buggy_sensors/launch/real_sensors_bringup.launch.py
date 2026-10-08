"""Real-hardware counterpart of sensors_bringup.launch.py: real Velodyne
(velodyne_driver + velodyne_transform, real UDP packets over the network,
not a Gazebo plugin) + velodyne_static_tf (base_link->velodyne static TF -
real hardware has no URDF joint tree to supply this automatically, unlike
sim) + segment_ground (torch-based ground removal, TOGGLEABLE via
ground_filter, same trade-off as the sim launch) + self_hit_filter (always
on) + uart_bridge (real wheel feedback over serial) + wheel_odometry
(dead-reckoning odom->base_link TF integrated from that feedback).

camera:=true (default false) additionally launches camera_processing's
camera_raw_node (raw UYVY -> GPU scale/convert -> BGR, publishing
camera/image_raw directly, no H265 codec stage - see that package's own
camera_raw.launch.py docstring for why this is preferred over the H265
capture+decode pair when downstream only needs camera/image_raw).
camera_device selects which /dev/videoN it opens; everything else
(resolution, fps, topic name) stays at camera_raw_node's own defaults -
override those by launching camera_processing's own launch files
directly if ever needed.

radar:=true (default false) additionally launches umrr_ros2_driver's
radar.launch.py (the smart_radar node, UMRR-A4 over CAN per that
package's own radar.params.template.yaml) - same toggle/off-by-default
pattern real_navigate_algo_rrt_star.launch.py already uses for this
exact driver. Neither camera nor radar is wired into sim - sim already
has its own radar stand-in (radar_sim.py) and camera is explicitly not
yet modeled there (see gazebo.launch.py's own comment).

camera_lidar_fusion:=true additionally launches camera_static_tf (rough,
uncalibrated base_link->camera_link - see that node's own docstring) and
camera_lidar_fusion_node, ported from delhi_demo_ws's ground_segmentation
package: projects LiDAR points into the camera image and estimates a
real-world distance per camera detection, firing /camera_red_detected /
/camera_yellow_detected below stop_distance/warning_distance. This node
does NOT run object detection itself - it only consumes an existing
Detection2DArray on detection_array_topic (default camera/detections/
array, e.g. camera_processing's yolox_detector_node or unified_perception_
node), which must be launched separately for this to ever produce
anything. Independent of the `camera` toggle above - that only brings up
raw image capture; set both together for the full debug-image + fusion
picture, or just this one if you only need the /camera_red_detected /
/camera_yellow_detected signals and already have your own detector and
image source running.

ground_filter:=true (default): segment_ground runs, self_hit_filter reads
its /non_ground_points output. ground_filter:=false: segment_ground is
skipped (the slowest node in this pipeline - torch RANSAC), self_hit_filter
reads raw /velodyne_points instead - confirmed in this same workspace's
sim testing that this trade-off can starve downstream planning of a clean
costmap; same caveat applies here.

SAFETY: verify sender_port/feedback_port match the actual physical wiring.
This launch only reads from the feedback port (uart_bridge, wheel
odometry) - it has no steering/actuation node at all, that's added
separately by whichever navigate-algo launch you bring up on top (real_
navigate_algo_theta_star.launch.py), same split as the sim workflow
(localize.launch.py has no controller_server/planner_server either).

enable_kiss_icp:=true swaps wheel_odometry's TF broadcast out for
kiss_icp_node (LiDAR scan-registration odometry) as the odom->base_link
source - see that arg's own description for why.
"""

from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler)

from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_velodyne_pointcloud = get_package_share_directory('velodyne_pointcloud')
    pkg_camera_processing = get_package_share_directory('camera_processing')
    pkg_umrr_ros2_driver = get_package_share_directory('umrr_ros2_driver')

    feedback_port_arg = DeclareLaunchArgument('feedback_port', default_value='/dev/ttyUSB0')
    ground_filter_arg = DeclareLaunchArgument(
        'ground_filter', default_value='true',
        description='false: skip segment_ground (the slow torch-based node) entirely - '
                    'self_hit_filter runs on raw /velodyne_points instead. See this '
                    "file's docstring for the trade-off.")
    ground_filter_device_arg = DeclareLaunchArgument(
        'ground_filter_device', default_value='gpu',
        description="ground_filter:=true only - 'gpu' (default), 'cpu', or 'segment'. "
                    "Selects segment_ground's backend (SEGMENT_GROUND_METHOD): 'gpu'/'cpu' "
                    'are the same RANSAC algorithm (GPU via CUDA, falls back to CPU if '
                    "unavailable) - use 'cpu' to rule out GPU/CUDA contention (e.g. with "
                    "RViz on a Jetson) as the cause of a hang. 'segment' is a different, "
                    'faster, lower-core CPU-only algorithm (Himmelsbach-style angular-'
                    'segment line-fit, no torch at all) - see segment_ground.py.')
    enable_kiss_icp_arg = DeclareLaunchArgument(
        'enable_kiss_icp', default_value='false',
        description='false (default): odom->base_link comes from wheel_odometry '
                    '(dead-reckoning from /wheel_uart, no slip correction - see that '
                    "node's own docstring). true: wheel_odometry keeps running and "
                    'publishing /odom (still needed by DWB/controller_server) but stops '
                    'broadcasting the TF, and kiss_icp_node (LiDAR scan-registration '
                    'odometry, vendored from lane_nav_ws) supplies odom->base_link '
                    'instead - immune to real wheel slip. See '
                    'buggy_mapping/launch/real_localize.launch.py for why this exists '
                    '(map drift after correct manual alignment).')
    camera_arg = DeclareLaunchArgument(
        'camera', default_value='false',
        description='true: additionally launch camera_processing/camera_raw_node '
                    '(camera/image_raw). false (default): no camera at all - matches this '
                    "file's existing off-by-default pattern for optional hardware.")
    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='camera:=true only - which video device camera_raw_node opens.')
    radar_arg = DeclareLaunchArgument(
        'radar', default_value='false',
        description='true: additionally launch umrr_ros2_driver/launch/radar.launch.py '
                    '(the smart_radar node, UMRR-A4 over CAN). false (default): no radar - '
                    'same toggle real_navigate_algo_rrt_star.launch.py already uses for '
                    'this exact driver.')
    camera_lidar_fusion_arg = DeclareLaunchArgument(
        'camera_lidar_fusion', default_value='false',
        description='true: additionally launch camera_static_tf + camera_lidar_fusion_node '
                    '(distance-based /camera_red_detected, /camera_yellow_detected - see '
                    "this file's own docstring). false (default): neither runs. Requires "
                    'something else publishing Detection2DArray on detection_array_topic - '
                    'this toggle does not bring up a detector itself.')

    publish_wheel_odom_tf = ParameterValue(
        PythonExpression([
            "'false' if '", LaunchConfiguration('enable_kiss_icp'), "' == 'true' else 'true'"]),
        value_type=bool)

    non_ground_points_source = PythonExpression([
        "'/non_ground_points' if '", LaunchConfiguration('ground_filter'),
        "' == 'true' else '/velodyne_points'"])

    velodyne_calibration = os.path.join(pkg_velodyne_pointcloud, 'params', 'VLP16_hires_db.yaml')
    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='screen',
        parameters=[{
            'device_ip': '',
            'gps_time': False,
            'time_offset': 0.0,
            'enabled': True,
            'read_once': False,
            'read_fast': False,
            'repeat_delay': 0.0,
            'frame_id': 'velodyne',
            'model': 'VLP16',
            'rpm': 1200.0,
            'port': 2368,
            'timestamp_first_packet': False,
            'use_sim_time': False,
        }],
    )
    velodyne_transform_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        output='screen',
        parameters=[{
            'calibration': velodyne_calibration,
            'model': 'VLP16',
            'min_range': 0.4,
            'max_range': 130.0,
            'view_direction': 0.0,
            'fixed_frame': '',
            'target_frame': '',
            'organize_cloud': True,
            'use_sim_time': False,
        }],
    )

    velodyne_static_tf = Node(
        package='buggy_perception',
        executable='velodyne_static_tf',
        name='velodyne_static_tf',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    segment_ground = Node(
        package='buggy_perception',
        executable='segment_ground',
        name='segment_ground',
        output='screen',
        parameters=[{'use_sim_time': False}],
        additional_env={'SEGMENT_GROUND_METHOD': LaunchConfiguration('ground_filter_device')},
        condition=IfCondition(LaunchConfiguration('ground_filter')),
    )

    self_hit_filter = Node(
        package='buggy_perception',
        executable='self_hit_filter',
        name='self_hit_filter',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/non_ground_points', non_ground_points_source),
        ],
    )

    uart_bridge = Node(
        package='buggy_uart',
        executable='uart_bridge',
        name='uart_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('feedback_port'),
            'baud': 115200,
            'use_sim_time': False,
        }],
    )

    wheel_odometry = Node(
        package='buggy_perception',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen',
        parameters=[{'publish_tf': publish_wheel_odom_tf, 'use_sim_time': False}],
    )

    # lidar_odom_frame set to 'odom' (not the package default 'odom_lidar')
    # and invert_odom_tf:=false so the TF it broadcasts IS odom->base_link
    # (parent odom, child base_link) - the exact transform slam_toolbox
    # reads via TF lookup, same slot wheel_odometry's TF normally fills.
    # base_frame is explicitly 'base_link' (differs from the raw cloud's
    # own frame_id, 'velodyne' - see velodyne_driver_node's frame_id param
    # above) so kiss_icp re-expresses its LiDAR-frame pose in base_link via
    # velodyne_static_tf's existing base_link->velodyne TF, instead of
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
            'use_sim_time': False,
        }],
        condition=IfCondition(LaunchConfiguration('enable_kiss_icp')),
    )

    camera_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_camera_processing, 'launch', 'camera_raw.launch.py')
        ),
        launch_arguments={
            'device': LaunchConfiguration('camera_device'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('camera')),
    )

    # Takes no launch arguments of its own - loads param/radar.params.
    # template.yaml as-is (one CAN-based UMRR-A4 sensor). Node name MUST
    # stay 'smart_radar' (fixed inside radar.launch.py itself) since that
    # yaml's top-level key is /smart_radar: - not something to override
    # from here.
    radar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_umrr_ros2_driver, 'launch', 'radar.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('radar')),
    )

    camera_static_tf = Node(
        package='buggy_perception',
        executable='camera_static_tf',
        name='camera_static_tf',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('camera_lidar_fusion')),
    )

    # Defaults (points_topic=/non_ground_points_filtered, detection_array_
    # topic=camera/detections/array, image 960x540) match this workspace's
    # self_hit_filter output and camera_processing's own launch file
    # defaults - see this node's own module docstring for the full
    # mechanism and why points_topic was changed from the original
    # delhi_demo_ws default (/non_ground_points, pre-self-hit-filter).
    camera_lidar_fusion_node = Node(
        package='buggy_perception',
        executable='camera_lidar_fusion_node',
        name='camera_lidar_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('camera_lidar_fusion')),
    )

    # A previous run's velodyne_driver_node/velodyne_transform_node/
    # uart_bridge can survive an imperfect Ctrl+C (orphaned from the launch
    # process group, or force-killed mid-cleanup once ros2 launch's
    # SIGTERM->SIGKILL escalation timeout elapses) and keep holding the UDP
    # socket / serial port open - the symptom is exactly "worked on the
    # first launch, silently no /velodyne_points (or garbled/absent wheel
    # feedback) on the next one" since the real replacement either fails to
    # bind/open or does but the OS is still routing data to the dead
    # process's fd. Force-clearing any stale instance of these three
    # processes before starting fresh nodes makes every launch idempotent
    # regardless of why the previous one didn't exit cleanly.
    #
    # velodyne_driver_node/velodyne_transform_node are real compiled C++
    # binaries, so -x (exact /proc/pid/comm match) works and is used for
    # them, same reasoning as before: -f would also match this cleanup
    # command's own `bash -c "...pkill -f velodyne_driver_node..."`
    # invocation (the pattern string is a substring of bash's own argv),
    # risking the shell killing itself mid-script before later pkills ran.
    #
    # uart_bridge is a Python rclpy node - confirmed live (`ps`) that its
    # real argv[0] is /usr/bin/python3 (the script path is just an
    # argument), so /proc/pid/comm is "python3" for it, NOT "uart_bridge" -
    # -x can't target it at all. Using -f instead reintroduces the self-
    # match risk above, so the pattern searches for the exact
    # `__node:=uart_bridge` ROS remap argument (present in the real node's
    # argv, precise enough not to also match e.g. steering_uart_bridge -
    # "uart_bridge" alone IS a trailing substring of that other node's
    # name, `__node:=uart_bridge` as a whole string is not), written as
    # `[u]art_bridge` - a bracket expression matching the same single
    # character 'u' but making the LITERAL text in this cleanup command's
    # own argv different from the string it actually searches for, so
    # pkill can never match its own invocation no matter what surrounds it.
    cleanup_stale_sensor_nodes = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -x velodyne_driver_node; '
             'pkill -9 -x velodyne_transform_node; '
             "pkill -9 -f '__node:=[u]art_bridge'; "
             'pkill -9 -x kiss_icp_node; '
             # camera_raw_node is a Python rclpy node (comm is "python3",
             # same reasoning as uart_bridge above) holding a video device
             # fd; smart_radar's own executable name
             # (smartmicro_radar_node_exe) would get silently truncated by
             # /proc/pid/comm's 15-char limit for a -x match, so both use
             # the __node:= remap-argument trick instead, which doesn't
             # depend on comm length or language at all.
             "pkill -9 -f '__node:=[c]amera_raw_node'; "
             "pkill -9 -f '__node:=[s]mart_radar'; "
             "pkill -9 -f '__node:=[c]amera_static_tf'; "
             "pkill -9 -f '__node:=[c]amera_lidar_fusion_node'; "
             'sleep 0.5; exit 0'],
        name='cleanup_stale_sensor_nodes',
        output='screen',
    )

    start_sensors = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup_stale_sensor_nodes,
            on_exit=[
                velodyne_driver_node,
                velodyne_transform_node,
                velodyne_static_tf,
                segment_ground,
                self_hit_filter,
                uart_bridge,
                wheel_odometry,
                kiss_icp_node,
                camera_bringup,
                radar_bringup,
                camera_static_tf,
                camera_lidar_fusion_node,
            ],
        )
    )

    return LaunchDescription([
        feedback_port_arg,
        ground_filter_arg,
        ground_filter_device_arg,
        enable_kiss_icp_arg,
        camera_arg,
        camera_device_arg,
        radar_arg,
        camera_lidar_fusion_arg,
        cleanup_stale_sensor_nodes,
        start_sensors,
    ])

    # return LaunchDescription([
    #     feedback_port_arg,
    #     ground_filter_arg,
    #     velodyne_driver_node,
    #     velodyne_transform_node,
    #     velodyne_static_tf,
    #     segment_ground,
    #     self_hit_filter,
    #     uart_bridge,
    #     wheel_odometry,
    # ])
