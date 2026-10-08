"""Moving-obstacle classifier - ported from the core idea of Asvadi et al.,
"3D Lidar-based Static and Moving Obstacle Detection in Driving
Environments" (RAS 2016): compares each occupied cell's point density
across the last few merged scans against what just the current scan alone
contributed there. Publishes ONLY the moving points - at full sensor
resolution, colour-tinted (red by default) - as a sensor_msgs/PointCloud2
on /moving_obstacle_points; static points are not published at all. See
dynamic_obstacle_detector.hpp's own header comment for the full mechanism,
and why it needs no per-object clustering/tracking/ID association at all -
unlike an earlier, reverted attempt at this same feature, which hit two
live-confirmed bugs from exactly that kind of per-object state (large-
cluster centroid instability, and Gazebo-RTF-induced track churn).

Purely additive - does not touch costmap_navigate.launch.py, freespace_
navigate.launch.py, or any other file. Perception/visualization only.

The paper aligns scans using GPS/IMU + a per-scan ICP step; this project
has neither, so scans are aligned here using the SAME fixed_frame TF chain
the rest of this package already relies on - kiss_icp when enabled
(enable_kiss_icp:=true in sensors_bringup.launch.py/real_sensors_bringup.
launch.py), wheel_odometry/tf_odom_relay otherwise.

use_sim_time:=false for real hardware (same convention as costmap_
navigate.launch.py's own arg).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('buggy_nav_costmap')
    default_params = os.path.join(pkg_share, 'params', 'dynamic_obstacle_detector_params.yaml')

    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    dynamic_obstacle_detector = Node(
        package='buggy_nav_costmap',
        executable='dynamic_obstacle_detector_node',
        name='dynamic_obstacle_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        dynamic_obstacle_detector,
    ])
