"""Load a previously-saved pose graph (mapping_2d.launch.py +
/slam_toolbox/serialize_map) back into slam_toolbox's MAPPING mode
(not localization) so it's LIVE and editable, instead of read-only -
slam_toolbox's own "continued mapping" feature: pass map_file_name (+
map_start_pose - see that param's own comment below, REQUIRED or the
deserialize silently fails and mapping just starts fresh instead) to
async_slam_toolbox_node with mode: mapping (slam_toolbox_mapping.yaml,
which already has enable_interactive_mode: true) and it resumes the saved
graph as an editable one rather than starting fresh.

This is the ONLY launch file in this workspace that opens a saved map for
EDITING - mapping_2d.launch.py only ever builds a new one from scratch,
localize.launch.py only ever reads an existing one (its
localization_slam_toolbox_node executable does full SLAM-style matching
against a fixed map, not live graph editing).

bring_up_sim:=false (DEFAULT): pure graph editing only - no Gazebo, no
sensors, no live scan. slam_toolbox owns the map frame directly and can
publish/edit the already-loaded graph with zero sensor input, so none of
that is actually needed just to drag existing vertices around and
re-save. use_sim_time is forced false to match (nothing would publish
/clock without Gazebo running, and a sim-time clock that never advances
breaks everything downstream of it).
bring_up_sim:=true: ALSO launches Gazebo + sensors (same as
mapping_2d.launch.py) so you can drive and add NEW scans to the graph
while editing (genuine continued mapping, not just moving existing
vertices) - use_sim_time follows automatically.

HOW TO ACTUALLY EDIT, once this is up and RViz has opened:
  1. Select RViz's "Interact" tool (mapping.rviz's toolbar). The pose
     graph itself is now drawn (the "Pose Graph" MarkerArray display,
     subscribed to /slam_toolbox/graph_visualization - was NOT in
     mapping.rviz before, since ordinary fresh-mapping never needed to
     look at existing vertices), and each vertex also has a draggable
     interactive marker on it (the "Pose Graph (Interactive)" display,
     subscribed to /slam_toolbox/update).
  2. Drag a vertex to where it should actually be.
  3. Commit that drag as a loop closure constraint:
         ros2 service call /slam_toolbox/manual_loop_closure \\
             slam_toolbox/srv/LoopClosure "{}"
     (empty request - it acts on whatever vertex you just moved). The
     underlying pose-graph optimizer then re-solves using that new
     constraint, which can shift OTHER nearby vertices too - that's
     expected, not a bug.
  4. Made a mistake / want to back out before committing? -
         ros2 service call /slam_toolbox/clear_changes std_srvs/srv/Empty "{}"
  5. Once it looks right, save - to a NEW name first is strongly
     recommended so the original is never at risk of being overwritten by
     a bad edit:
         ros2 service call /slam_toolbox/serialize_map \\
             slam_toolbox/srv/SerializePoseGraph \\
             "{filename: '<path-to-buggy_mapping>/maps/<new_name>'}"
     Only point localize.launch.py's map_name at the new file once you've
     actually driven/looked at it and confirmed the edit did what you
     wanted - don't overwrite map_name's existing consumers on a hope.

world_file/spawn_x/y/yaw only matter when bring_up_sim:=true - defaults
match my_map2/office_track.world (this workspace's actual working map).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_buggy_sensors = get_package_share_directory('buggy_sensors')
    pkg_buggy_mapping = get_package_share_directory('buggy_mapping')

    slam_params = os.path.join(pkg_buggy_mapping, 'config', 'slam_toolbox_mapping.yaml')
    p2l_params = os.path.join(pkg_buggy_mapping, 'config', 'pointcloud_to_laserscan.yaml')
    rviz_config = os.path.join(pkg_buggy_mapping, 'rviz', 'mapping.rviz')
    maps_dir = os.path.join(pkg_buggy_mapping, 'maps')

    bring_up_sim_arg = DeclareLaunchArgument(
        'bring_up_sim', default_value='false',
        description='true: also launch Gazebo+sensors so you can drive and add NEW scans '
                    'while editing (genuine continued mapping). false (default): pure graph '
                    'editing only, no sim needed at all.')
    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='office_track.world',
        description='Only used if bring_up_sim:=true - must match whatever world the '
                    'loaded map was actually built on.')
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value='0.0',
        description="Only used if bring_up_sim:=true - must match the spawn pose used "
                    "when the map was built (my_map2's was (0.0, -3.0, 0.0)).")
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='-3.0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    ground_filter_arg = DeclareLaunchArgument(
        'ground_filter', default_value='true',
        description='Only used if bring_up_sim:=true. false: skip segment_ground - '
                    'self_hit_filter runs on raw /velodyne_points instead.')
    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='my_map2',
        description='Basename (no extension) under buggy_mapping/maps/ to load and make '
                    'editable - loads <map_name>.data/.posegraph.')

    map_file_name = PathJoinSubstitution([maps_dir, LaunchConfiguration('map_name')])
    use_sim_time = ParameterValue(LaunchConfiguration('bring_up_sim'), value_type=bool)

    sensors_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buggy_sensors, 'launch', 'sensors_bringup.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('bring_up_sim')),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'gui': LaunchConfiguration('gui'),
            'ground_filter': LaunchConfiguration('ground_filter'),
        }.items(),
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[p2l_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', '/non_ground_points_filtered'),
            ('scan', '/scan'),
        ],
        condition=IfCondition(LaunchConfiguration('bring_up_sim')),
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        # async_slam_toolbox_node, NOT localization_slam_toolbox_node - this
        # is slam_toolbox_mapping.yaml's mode: mapping (editable graph),
        # not localize.launch.py's read-only-against-a-fixed-map mode.
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {
            'use_sim_time': use_sim_time,
            # The one thing that actually turns this into "continue
            # editing THIS saved graph" instead of starting a fresh one -
            # slam_toolbox's own "continued mapping" feature.
            'map_file_name': map_file_name,
            # Confirmed live: without this, the deserialize step fails
            # silently ("Map starting pose not specified" logged, but the
            # node keeps running as if nothing went wrong) and mapping
            # just starts fresh (a 1-vertex graph) instead of loading the
            # saved one. slam_toolbox_mapping.yaml never needed this
            # before - fresh mapping has no map to align a start pose
            # against. Same value/reasoning as slam_toolbox_localization.
            # yaml's own map_start_pose (see that file's comment): the
            # saved map's own origin IS wherever the vehicle was when it
            # was built, i.e. [0,0,0] in the map's own frame.
            'map_start_pose': [0.0, 0.0, 0.0],
        }],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        bring_up_sim_arg,
        world_file_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        gui_arg,
        rviz_arg,
        ground_filter_arg,
        map_name_arg,
        sensors_bringup,
        pointcloud_to_laserscan,
        slam_toolbox,
        rviz2,
    ])
