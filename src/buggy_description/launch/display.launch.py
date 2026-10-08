"""Standalone URDF viewer: robot_state_publisher + joint_state_publisher_gui + RViz2.

No Gazebo/ros2_control involved - this is just for visually checking the
vehicle model and manually exercising the steering/wheel joints.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('buggy_description')
    default_xacro_path = os.path.join(pkg_share, 'urdf', 'buggy.urdf.xacro')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'buggy.rviz')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_xacro_path,
        description='Absolute path to the buggy xacro file',
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config] if os.path.exists(default_rviz_config) else [],
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
