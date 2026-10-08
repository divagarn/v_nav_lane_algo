"""Map-only checking tool: no Gazebo, no sensors, no localization - just
the saved map (as a plain nav2_map_server image, NOT slam_toolbox's
pose-graph format - see maps/README.md for why those are two different
things) plus RViz's own click tools, for picking/verifying start and goal
coordinates before spending time on a real navigation run.

  ros2 launch buggy_mapping map_viewer.launch.py map_yaml:=my_map2_map.yaml

Workflow:
  1. "2D Pose Estimate" - click where you want the start pose. One green
     arrow marker, overwritten each time (only one start point at once).
  2. "2D Goal Pose" - click as many candidate goals as you want. Each
     click adds a new numbered amber marker with its (x, y) printed both
     in RViz and in this launch's own terminal - copy those numbers
     straight into localize.launch.py's spawn_x/spawn_y/spawn_yaw or a
     real navigation goal.
  3. "Publish Point" - clears all accumulated goal markers (start marker
     untouched) if the map gets cluttered.

Fixed Frame is 'map', same frame the saved map's own pixel grid is
already in - coordinates you read here are the SAME map-frame coordinates
localize.launch.py / navigate_algo_*.launch.py use, no conversion needed.

map_server is a nav2 LifecycleNode - lifecycle_manager_map's autostart is
what actually gets it from unconfigured -> active so /map publishes at
all; without it the node exists but never serves anything.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_buggy_mapping = get_package_share_directory('buggy_mapping')
    maps_dir = os.path.join(pkg_buggy_mapping, 'maps')
    rviz_config = os.path.join(pkg_buggy_mapping, 'rviz', 'map_viewer.rviz')

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml', default_value='my_map2_map.yaml',
        description='Filename (under buggy_mapping/maps/) of the plain nav2_map_server '
                    'image map to load - e.g. the my_map2_map.yaml/.pgm pair slam_toolbox\'s '
                    'own use_map_saver:true setting already produced alongside the '
                    '.data/.posegraph files mapping_2d.launch.py saves.')

    map_yaml_path = PathJoinSubstitution([maps_dir, LaunchConfiguration('map_yaml')])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'use_sim_time': False,
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    goal_marker_recorder = Node(
        package='buggy_mapping',
        executable='goal_marker_recorder',
        name='goal_marker_recorder',
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        map_yaml_arg,
        map_server,
        lifecycle_manager,
        goal_marker_recorder,
        rviz2,
    ])
