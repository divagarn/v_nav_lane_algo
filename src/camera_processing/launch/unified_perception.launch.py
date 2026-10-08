#!/usr/bin/env python3
"""
Launch file for the unified detection + segmentation node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        'input_topic', default_value='camera/image_raw',
        description='Input raw camera image topic',
    )
    detection_topic_arg = DeclareLaunchArgument(
        'detection_topic', default_value='camera/detections/image_raw',
        description='Output annotated detection image topic',
    )
    segmentation_topic_arg = DeclareLaunchArgument(
        'segmentation_topic', default_value='camera/segmentation/image_raw',
        description='Output segmentation overlay image topic (background/lane/road)',
    )
    lane_topic_arg = DeclareLaunchArgument(
        'lane_topic', default_value='camera/lane/image_raw',
        description='Output lane-only overlay image topic',
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        description='Path to the unified detection+segmentation ONNX model '
                     '(required, e.g. model_path:=/path/to/up.onnx)',
    )
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold', default_value='0.3',
        description='Detection confidence threshold',
    )
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold', default_value='0.3',
        description='Per-class NMS IoU threshold',
    )
    seg_alpha_arg = DeclareLaunchArgument(
        'seg_alpha', default_value='0.4',
        description='Overlay blend alpha for segmentation/lane visualization',
    )

    node = Node(
        package='camera_processing',
        executable='unified_perception_node',
        name='unified_perception_node',
        parameters=[
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'detection_topic': LaunchConfiguration('detection_topic'),
                'segmentation_topic': LaunchConfiguration('segmentation_topic'),
                'lane_topic': LaunchConfiguration('lane_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'nms_threshold': LaunchConfiguration('nms_threshold'),
                'seg_alpha': LaunchConfiguration('seg_alpha'),
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        input_topic_arg,
        detection_topic_arg,
        segmentation_topic_arg,
        lane_topic_arg,
        model_path_arg,
        conf_threshold_arg,
        nms_threshold_arg,
        seg_alpha_arg,
        node,
    ])
