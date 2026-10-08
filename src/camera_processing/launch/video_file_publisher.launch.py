#!/usr/bin/env python3
"""
Launch file for publishing a video file onto camera/image_raw, e.g. to check
yolox_detector_node's output without a live camera:

  ros2 launch camera_processing video_file_publisher.launch.py video_path:=/path/to/video.mp4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        description='Path to the input video file',
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic', default_value='camera/image_raw',
        description='Topic to publish decoded frames on',
    )

    loop_arg = DeclareLaunchArgument(
        'loop', default_value='true',
        description='Loop the video when it ends',
    )

    fps_arg = DeclareLaunchArgument(
        'fps', default_value='0.0',
        description='Publish rate override in Hz (0.0 = use the video\'s native fps)',
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='camera',
        description='Frame id to stamp on published images',
    )

    publisher_node = Node(
        package='camera_processing',
        executable='video_file_publisher_node',
        name='video_file_publisher_node',
        parameters=[
            {
                'video_path': LaunchConfiguration('video_path'),
                'output_topic': LaunchConfiguration('output_topic'),
                'loop': LaunchConfiguration('loop'),
                'fps': LaunchConfiguration('fps'),
                'frame_id': LaunchConfiguration('frame_id'),
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        video_path_arg,
        output_topic_arg,
        loop_arg,
        fps_arg,
        frame_id_arg,
        publisher_node,
    ])
