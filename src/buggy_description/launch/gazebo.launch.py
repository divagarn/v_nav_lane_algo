"""Spawn the buggy URDF into an empty Gazebo Classic world with ros2_control
drivetrain active.

Brings up: Gazebo, robot_state_publisher, the model spawn, then (once the
model is spawned) joint_state_broadcaster and ackermann_steering_controller
via controller_manager. Drive it manually with:

    ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/msg/Twist \
        '{linear: {x: 0.5}, angular: {z: 0.1}}'

(topic is "reference_unstamped", not "reference" - the controller config sets
use_stamped_vel: false, which routes plain geometry_msgs/Twist to the
"_unstamped" topic; "reference" only exists when TwistStamped is used)

No sensor plugins (LiDAR/GPS/IMU/camera) wired up yet - that is next.

XML comments are stripped from the expanded URDF before it's published as
robot_description: gazebo_ros2_control has a known bug (tracked upstream at
ros-controls/gazebo_ros2_control#295, root-caused to launch_ros#214) where a
comment containing a colon breaks its internal YAML-style parsing of the
robot_description string, crashing gzserver with "Couldn't parse parameter
override rule". Our xacro files use colons in comments throughout, so this
isn't optional.
"""

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Building-editor models (e.g. the "t2" closed-loop track) live outside any
# ROS package, saved by Gazebo's own Building Editor tool.
BUILDING_EDITOR_MODELS_DIR = os.path.expanduser('~/building_editor_models')


def _strip_xml_comments(xml_text):
    return re.sub(r'<!--.*?-->', '', xml_text, flags=re.DOTALL)


def launch_setup(context, *args, **kwargs):
    model_path = LaunchConfiguration('model').perform(context)
    lidar_tilt_deg = LaunchConfiguration('lidar_tilt_deg').perform(context)

    xacro_result = subprocess.run(
        ['xacro', model_path, f'lidar_tilt_deg:={lidar_tilt_deg}'],
        capture_output=True, text=True, check=True,
    )
    robot_description_str = _strip_xml_comments(xacro_result.stdout)
    robot_description = ParameterValue(robot_description_str, value_type=str)

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_yaw = LaunchConfiguration('spawn_yaw').perform(context)

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_buggy',
        output='screen',
        arguments=[
            '-topic', 'robot_description', '-entity', 'buggy',
            '-x', spawn_x, '-y', spawn_y, '-z', '0.05', '-Y', spawn_yaw,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen',
    )

    # gazebo_ros2_control's controller_manager only exists once the model has
    # actually been spawned into gzserver, so chain the spawners off that.
    load_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner, ackermann_controller_spawner],
        )
    )

    return [
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        load_controllers_after_spawn,
    ]


def generate_launch_description():
    pkg_description = get_package_share_directory('buggy_description')
    default_xacro_path = os.path.join(pkg_description, 'urdf', 'buggy.urdf.xacro')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_xacro_path,
        description='Absolute path to the buggy xacro file',
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run Gazebo headless (no gzclient GUI)',
    )

    lidar_tilt_arg = DeclareLaunchArgument(
        'lidar_tilt_deg',
        default_value='0.0',
        description='Downward LiDAR tilt in degrees (positive = nose down). '
                     'Reduces the ground blind spot at the cost of upward FOV.',
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world'),
        description='Absolute path to a Gazebo .world file. Use '
                     f'{os.path.join(pkg_description, "worlds", "closed_loop_track.world")} '
                     'for the closed-loop track.',
    )

    # Defaults (5, 5) confirmed clear of walls inside the closed_loop_track
    # world; harmless for the default empty world too (open flat ground).
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='5.0', description='Spawn X position (m)')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='5.0', description='Spawn Y position (m)')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Spawn yaw (radians)')

    # So <uri>model://t2</uri> in closed_loop_track.world resolves - t2 was
    # saved outside any ROS package by Gazebo's own Building Editor tool.
    # Explicitly including /usr/share/gazebo-11/models is required here:
    # setting GAZEBO_MODEL_PATH at all (even while appending the old value)
    # suppresses Gazebo's compiled-in default search path, which is where
    # sun/ground_plane actually live on this system - without this, and
    # with the online model database disabled below, those fail to
    # resolve at all.
    GAZEBO_SYSTEM_MODELS_DIR = '/usr/share/gazebo-11/models'
    # Package-tracked models (e.g. "office_track1") live under
    # buggy_description/models, installed by CMakeLists.txt - unlike
    # BUILDING_EDITOR_MODELS_DIR these are version-controlled with the repo.
    pkg_models_dir = os.path.join(get_package_share_directory('buggy_description'), 'models')
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            pkg_models_dir, os.pathsep,
            BUILDING_EDITOR_MODELS_DIR, os.pathsep,
            GAZEBO_SYSTEM_MODELS_DIR, os.pathsep,
            os.environ.get('GAZEBO_MODEL_PATH', ''),
        ],
    )

    # Gazebo Classic's ModelDatabase tries to fetch the online model catalog
    # (models.gazebosim.org) on startup and hangs indefinitely if there's no
    # internet access - a well-known issue in offline/sandboxed setups.
    # Blanking this skips that network call entirely; local models (sun,
    # ground_plane, t2, ...) resolve fine without it.
    disable_online_model_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value='',
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        lidar_tilt_arg,
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        set_model_path,
        disable_online_model_db,
        OpaqueFunction(function=launch_setup),
    ])
