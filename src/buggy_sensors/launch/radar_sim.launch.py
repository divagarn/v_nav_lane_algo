"""Launch the UMRR-A4 radar stand-in alone - assumes buggy_description's
gazebo.launch.py (or any other source of /velodyne_points) is already
running. See radar_sim.py's own docstring for the interim raw-Velodyne-input
caveat (no ground segmentation in this workspace yet).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    radar_sim = Node(
        package='buggy_sensors',
        executable='radar_sim',
        name='radar_sim',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([radar_sim])
