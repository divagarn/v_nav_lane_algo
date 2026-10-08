#!/usr/bin/env python3
"""
Launch file for the YOLOX detector node.

model_path's extension selects the backend: point it at a .onnx file for
ONNX Runtime, or a .pth checkpoint for PyTorch (needs yolox_repo_path set to
an edgeai-yolox-simplified checkout, since a raw state_dict carries no
architecture info on its own).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='camera/image_raw',
        description='Input raw camera image topic',
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='camera/detections/image_raw',
        description='Output annotated image topic',
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/divagar/edgeai-yolox-simplified/build_edgeai_custom/'
                       'yolox_s_ti_18class_ti_today_qat.onnx',
        description='Path to the YOLOX model: .onnx for ONNX Runtime, .pth for PyTorch',
    )

    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold', default_value='0.3',
        description='Detection confidence threshold',
    )

    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold', default_value='0.45',
        description='NMS IoU threshold',
    )

    num_classes_arg = DeclareLaunchArgument(
        'num_classes', default_value='18',
        description='Number of classes the model was trained with (.pth backend only)',
    )

    decode_grid_stride_arg = DeclareLaunchArgument(
        'decode_grid_stride', default_value='false',
        description='Set true for exports where grid/stride box decode is NOT baked into '
                     'the graph (e.g. yolox_s_ti_18class_twillight_clean_tda4vh.onnx). '
                     'Leave false for exports where it is (e.g. the _ti_today_qat.onnx one).',
    )

    yolox_repo_path_arg = DeclareLaunchArgument(
        'yolox_repo_path',
        default_value='/home/divagar/edgeai-yolox-simplified/git_dir/edgeai-yolox',
        description='Path to an edgeai-yolox-simplified checkout, needed to build the model '
                     'architecture for the .pth backend only',
    )

    detector_node = Node(
        package='camera_processing',
        executable='yolox_detector_node',
        name='yolox_detector_node',
        parameters=[
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'nms_threshold': LaunchConfiguration('nms_threshold'),
                'num_classes': LaunchConfiguration('num_classes'),
                'decode_grid_stride': LaunchConfiguration('decode_grid_stride'),
                'yolox_repo_path': LaunchConfiguration('yolox_repo_path'),
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        model_path_arg,
        conf_threshold_arg,
        nms_threshold_arg,
        num_classes_arg,
        decode_grid_stride_arg,
        yolox_repo_path_arg,
        detector_node,
    ])
