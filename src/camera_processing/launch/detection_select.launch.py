#!/usr/bin/env python3
"""
Detection backend selector.

unified_perception_node ALWAYS runs, since segmentation_topic and lane_topic
only ever come from its model - those two are not optional. What `backend`
actually picks is which model's detection ends up on detection_topic:

  backend:=unified3  (default) -> unified_perception_node's own 3-class
                        detection (lmv, person, two_wheeler) is published to
                        detection_topic. yolox_detector_node is not launched.

  backend:=yolox18   -> yolox_detector_node's 18-class detection (animal,
                        autorickshaw, car, pedestrian, ...) is published to
                        detection_topic instead. unified_perception_node
                        still runs for segmentation_topic/lane_topic, but its
                        own detection output is disabled (enable_detection
                        set false) so the two nodes don't collide publishing
                        to the same topic.
                        Default model: yolox_s_ti_18class_twillight_clean_tda4vh.onnx
                        (empirically the more accurate of the two 18-class
                        exports tested so far - correctly detected a dog and
                        a traffic sign the older _ti_today_qat.onnx export
                        missed on the same footage). Needs
                        decode_grid_stride:=true for this export (see that
                        node's docstring for why).

Usage:
  ros2 launch camera_processing detection_select.launch.py backend:=yolox18
  ros2 launch camera_processing detection_select.launch.py backend:=unified3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        'backend', default_value='unified3',
        description="Which model's detections land on detection_topic: 'yolox18' or "
                    "'unified3' (default). segmentation_topic/lane_topic always come "
                    "from unified_perception_node regardless of this choice.",
    )
    input_topic_arg = DeclareLaunchArgument(
        'input_topic', default_value='camera/image_raw',
        description='Input raw camera image topic',
    )
    detection_topic_arg = DeclareLaunchArgument(
        'detection_topic', default_value='camera/detections/image_raw',
        description='Output annotated detection image topic (both backends publish here)',
    )
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold', default_value='0.3',
        description='Detection confidence threshold',
    )
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold', default_value='0.3',
        description='NMS IoU threshold',
    )

    # -- yolox18-only args --
    yolox_model_path_arg = DeclareLaunchArgument(
        'yolox_model_path',
        default_value='/home/divagar/Downloads/yolox_s_ti_18class_twillight_clean_tda4vh.onnx',
        description='(yolox18 only) path to the .onnx/.pth YOLOX model',
    )
    yolox_decode_grid_stride_arg = DeclareLaunchArgument(
        'yolox_decode_grid_stride', default_value='true',
        description='(yolox18 only) true if the model export does NOT bake grid/stride '
                     'decode into the graph - true matches the default twilight_clean model',
    )

    # -- unified3-only args --
    unified_model_path_arg = DeclareLaunchArgument(
        'unified_model_path',
        default_value='/home/divagar/Downloads/up_20260903.onnx',
        description='(unified3 only) path to the unified detection+segmentation .onnx model',
    )
    segmentation_topic_arg = DeclareLaunchArgument(
        'segmentation_topic', default_value='camera/segmentation/image_raw',
        description='(unified3 only) output segmentation overlay image topic',
    )
    lane_topic_arg = DeclareLaunchArgument(
        'lane_topic', default_value='camera/lane/image_raw',
        description='(unified3 only) output lane-only overlay image topic',
    )

    is_yolox18 = PythonExpression(["'", LaunchConfiguration('backend'), "' == 'yolox18'"])
    is_unified3 = PythonExpression(["'", LaunchConfiguration('backend'), "' == 'unified3'"])

    yolox_node = Node(
        package='camera_processing',
        executable='yolox_detector_node',
        name='yolox_detector_node',
        condition=IfCondition(is_yolox18),
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('detection_topic'),
            'model_path': LaunchConfiguration('yolox_model_path'),
            'decode_grid_stride': LaunchConfiguration('yolox_decode_grid_stride'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
        }],
        output='screen',
    )

    unified_node = Node(
        package='camera_processing',
        executable='unified_perception_node',
        name='unified_perception_node',
        # always runs - segmentation_topic/lane_topic only ever come from here
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'detection_topic': LaunchConfiguration('detection_topic'),
            'segmentation_topic': LaunchConfiguration('segmentation_topic'),
            'lane_topic': LaunchConfiguration('lane_topic'),
            'model_path': LaunchConfiguration('unified_model_path'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'enable_detection': is_unified3,
        }],
        output='screen',
    )

    return LaunchDescription([
        backend_arg,
        input_topic_arg,
        detection_topic_arg,
        conf_threshold_arg,
        nms_threshold_arg,
        yolox_model_path_arg,
        yolox_decode_grid_stride_arg,
        unified_model_path_arg,
        segmentation_topic_arg,
        lane_topic_arg,
        yolox_node,
        unified_node,
    ])
