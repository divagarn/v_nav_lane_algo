#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='camera/compressed',
        description='Input compressed H265 topic',
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='camera/image_raw',
        description='Output decoded raw image topic',
    )

    fallback_arg = DeclareLaunchArgument(
        'allow_software_fallback',
        default_value='false',
        description='Allow avdec_h265 software fallback when hardware decode is unavailable',
    )

    output_width_arg = DeclareLaunchArgument(
        'output_width',
        default_value='960',
        description='Decoded output width (scaled by VIC)',
    )

    output_height_arg = DeclareLaunchArgument(
        'output_height',
        default_value='540',
        description='Decoded output height (scaled by VIC)',
    )

    decoder_node = Node(
        package='camera_processing',
        executable='camera_h265_decoder_node',
        name='camera_h265_decoder_node',
        parameters=[
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'allow_software_fallback': LaunchConfiguration('allow_software_fallback'),
                'output_width': LaunchConfiguration('output_width'),
                'output_height': LaunchConfiguration('output_height'),
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        fallback_arg,
        output_width_arg,
        output_height_arg,
        decoder_node,
    ])
