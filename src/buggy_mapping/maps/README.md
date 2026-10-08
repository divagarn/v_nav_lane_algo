# Saved maps

## 2D (slam_toolbox)

`ros2 launch buggy_mapping mapping_2d.launch.py` builds a map here as you
drive. Once you've covered the loop, save it:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/divagar/buggy_ros2/divagar_test1_ws/src/buggy_mapping/maps/<name>'}"
```

This is slam_toolbox's own serialization (`<name>.data` + `<name>.posegraph`)
- NOT the same thing as `nav2_map_server`'s `map_saver_cli` (`.yaml`/`.pgm`),
which produces a plain image-based map for tools like AMCL. This project
uses slam_toolbox's own localization mode for navigation later, which needs
the serialized pose graph, not a plain image map.

## 3D (octomap_server)

`ros2 launch buggy_mapping mapping_3d.launch.py` builds a 3D occupancy
octree as you drive (`/octomap_binary` topic + service, `octomap_msgs/msg/
Octomap`). Save it:

```bash
ros2 run buggy_mapping_tools save_octomap_node <path>.bt
```

This is a small custom node (`buggy_mapping_tools` package), not
octomap_server's own shipped `octomap_saver_node` CLI - that tool was
confirmed live to reject every filename tried ("Invalid file name or
extension" regardless of a valid `.bt`/`.ot` path, as a positional arg or
a ROS parameter, via `ros2 run` or the installed binary directly - root
cause not pinned down). `save_octomap_node` instead subscribes to
`/octomap_binary` and writes a correctly-formatted file via octomap's own
`AbstractOccupancyOcTree::writeBinary()` - round-trip verified live
(saved, then successfully reloaded via `octomap_server_static_node` with
no errors). Two more obvious-looking approaches were tried and confirmed
wrong first: writing the message's raw `data` field straight to a file
(missing octomap's required text header), and calling the base class's
generic `write()` instead of `writeBinary()` (produces the wrong header -
`.ot`'s "# Octomap OcTree file" instead of `.bt`'s "# Octomap OcTree
binary file"). See `buggy_mapping_tools/src/save_octomap_node.cpp` for
the full story.

**Important caveat**: octomap_server has no independent pose-correction of
its own (no scan-matching, no loop closure) - it only accumulates points
using whatever TF (odom -> base_link -> lidar_mount_link) is available at
each cloud's timestamp. The 2D slam_toolbox map above actively corrects
for odometry drift via scan matching/loop closure; this 3D map does not,
so it will show more drift over a long/large loop. Good enough for a
first 3D map of a driven area, not a substitute for the 2D SLAM pose
graph used for navigation.

**Important caveat**: octomap_server has no independent pose-correction of
its own (no scan-matching, no loop closure) - it only accumulates points
using whatever TF (odom -> base_link -> lidar_mount_link) is available at
each cloud's timestamp. The 2D slam_toolbox map above actively corrects
for odometry drift via scan matching/loop closure; this 3D map does not,
so it will show more drift over a long/large loop. Good enough for a
first 3D map of a driven area, not a substitute for the 2D SLAM pose
graph used for navigation.
