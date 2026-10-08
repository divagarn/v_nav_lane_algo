#!/usr/bin/env python3
"""
Launch file for the monocular depth estimation node (MiDaS).

Publishes a depth image and a back-projected 3D point cloud from a single
raw RGB camera topic. See mono_depth_node.py's module docstring for the
important caveat that this is relative (scale-ambiguous) depth, not metric
range, and that fx/fy/cx/cy default to a placeholder pinhole model (from
hfov_deg) until this camera has an actual calibration.
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
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='camera/depth/image_raw',
        description='Output depth image topic (32FC1)',
    )
    points_topic_arg = DeclareLaunchArgument(
        'points_topic', default_value='camera/depth/points',
        description='Output 3D point cloud topic',
    )
    model_type_arg = DeclareLaunchArgument(
        'model_type', default_value='MiDaS_small',
        description='torch.hub intel-isl/MiDaS model name '
                    '(MiDaS_small / DPT_Hybrid / DPT_Large)',
    )
    depth_scale_arg = DeclareLaunchArgument(
        'depth_scale', default_value='5.0',
        description='Arbitrary-unit inverse-depth -> pseudo-metres scale factor',
    )
    max_depth_arg = DeclareLaunchArgument(
        'max_depth', default_value='20.0',
        description='Clamp for the (relative) depth output, in pseudo-metres',
    )
    point_stride_arg = DeclareLaunchArgument(
        'point_stride', default_value='4',
        description='Pixel stride when back-projecting the point cloud (higher = sparser/faster)',
    )
    process_every_n_arg = DeclareLaunchArgument(
        'process_every_n', default_value='1',
        description='Only run depth inference on every Nth incoming frame',
    )
    fx_arg = DeclareLaunchArgument(
        'fx', default_value='-1.0',
        description='Focal length x in pixels; <=0 derives it from hfov_deg (no calibration yet)',
    )
    fy_arg = DeclareLaunchArgument(
        'fy', default_value='-1.0',
        description='Focal length y in pixels; <=0 derives it from hfov_deg (no calibration yet)',
    )
    cx_arg = DeclareLaunchArgument(
        'cx', default_value='-1.0', description='Principal point x; <=0 defaults to width/2',
    )
    cy_arg = DeclareLaunchArgument(
        'cy', default_value='-1.0', description='Principal point y; <=0 defaults to height/2',
    )
    hfov_deg_arg = DeclareLaunchArgument(
        'hfov_deg', default_value='90.0',
        description='Assumed horizontal FOV in degrees, used only when fx/fy are not set '
                    '- tune this to the actual wide-angle lens for a less-wrong point cloud',
    )

    mono_depth_node = Node(
        package='camera_processing',
        executable='mono_depth_node',
        name='mono_depth_node',
        parameters=[
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'points_topic': LaunchConfiguration('points_topic'),
                'model_type': LaunchConfiguration('model_type'),
                'depth_scale': LaunchConfiguration('depth_scale'),
                'max_depth': LaunchConfiguration('max_depth'),
                'point_stride': LaunchConfiguration('point_stride'),
                'process_every_n': LaunchConfiguration('process_every_n'),
                'fx': LaunchConfiguration('fx'),
                'fy': LaunchConfiguration('fy'),
                'cx': LaunchConfiguration('cx'),
                'cy': LaunchConfiguration('cy'),
                'hfov_deg': LaunchConfiguration('hfov_deg'),
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        input_topic_arg,
        depth_topic_arg,
        points_topic_arg,
        model_type_arg,
        depth_scale_arg,
        max_depth_arg,
        point_stride_arg,
        process_every_n_arg,
        fx_arg,
        fy_arg,
        cx_arg,
        cy_arg,
        hfov_deg_arg,
        mono_depth_node,
    ])
