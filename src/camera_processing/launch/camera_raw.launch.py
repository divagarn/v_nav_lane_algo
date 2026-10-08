#!/usr/bin/env python3
"""
Launch file for Camera Raw Node

Direct raw capture (UYVY -> GPU scale/convert -> BGR) publishing straight to
camera/image_raw, with no H265 codec stage. Use this INSTEAD of
camera_h265.launch.py + camera_h265_decoder.launch.py when downstream only
needs camera/image_raw (detection, camera/LiDAR fusion, recording for
playback) - it avoids paying for an encode+decode round trip you don't need.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video0',
        description='Video device path'
    )

    capture_width_arg = DeclareLaunchArgument(
        'capture_width',
        default_value='1920',
        description='Camera capture width (native sensor resolution)'
    )

    capture_height_arg = DeclareLaunchArgument(
        'capture_height',
        default_value='1080',
        description='Camera capture height (native sensor resolution)'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frames per second'
    )

    output_width_arg = DeclareLaunchArgument(
        'output_width',
        default_value='960',
        description='Published image width (GPU-scaled from capture resolution)'
    )

    output_height_arg = DeclareLaunchArgument(
        'output_height',
        default_value='540',
        description='Published image height (GPU-scaled from capture resolution)'
    )

    topic_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='camera/image_raw',
        description='ROS2 topic to publish raw image frames to'
    )

    camera_node = Node(
        package='camera_processing',
        executable='camera_raw_node',
        name='camera_raw_node',
        parameters=[
            {
                'device': LaunchConfiguration('device'),
                'capture_width': LaunchConfiguration('capture_width'),
                'capture_height': LaunchConfiguration('capture_height'),
                'fps': LaunchConfiguration('fps'),
                'output_width': LaunchConfiguration('output_width'),
                'output_height': LaunchConfiguration('output_height'),
                'topic_name': LaunchConfiguration('topic_name'),
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        device_arg,
        capture_width_arg,
        capture_height_arg,
        fps_arg,
        output_width_arg,
        output_height_arg,
        topic_arg,
        camera_node,
    ])
