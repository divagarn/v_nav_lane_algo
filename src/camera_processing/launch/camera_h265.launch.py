#!/usr/bin/env python3
"""
Launch file for Camera H265 Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video0',
        description='Video device path'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1920',
        description='Image width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='1080',
        description='Image height'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frames per second'
    )
    
    bitrate_arg = DeclareLaunchArgument(
        'bitrate',
        default_value='15000000',
        description='H265 bitrate in bits/sec'
    )
    
    topic_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='camera/compressed',
        description='ROS2 topic to publish video frames to'
    )
    
    # Create node
    camera_node = Node(
        package='camera_processing',
        executable='camera_h265_node',
        name='camera_h265_node',
        parameters=[
            {
                'device': LaunchConfiguration('device'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'bitrate': LaunchConfiguration('bitrate'),
                'topic_name': LaunchConfiguration('topic_name'),
            }
        ],
        output='screen',
    )
    
    return LaunchDescription([
        device_arg,
        width_arg,
        height_arg,
        fps_arg,
        bitrate_arg,
        topic_arg,
        camera_node,
    ])
