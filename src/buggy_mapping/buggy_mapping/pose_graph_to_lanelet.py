#!/usr/bin/env python3
"""Export the currently-loaded pose graph (via edit_pose_graph.launch.py)
into a standard Lanelet2 OSM-XML file - centerline vertices become a
lanelet's left/right boundary linestrings, offset by +-lane_half_width
perpendicular to the local path direction (same technique already used
for buggy_nav_costmap's own /free_lane_area marker).

WHY: editing the LIVE pose graph via slam_toolbox's interactive markers
turned out to require a real RViz client session that no external script
or tool can establish (confirmed live, see chat history) - RViz's own
Interact tool is also not working reliably in this environment. A static
OSM-XML file has none of that: any text editor, JOSM (if available), or
this package's own lanelet_editor.py can open and edit it directly, no
ROS, no live service protocol, no client-session requirement at all.

This is a ONE-TIME EXPORT, not a live bridge - edits made afterward (in
lanelet_editor.py or elsewhere) do NOT feed back into the running
slam_toolbox graph or the live navigation stack. It's for producing a
correctable, inspectable snapshot of the recorded route.

Coordinates use the local_x/local_y tag convention (no real GPS/lat-lon -
this project has none), the same approach Autoware's own sample maps use
for non-georeferenced Lanelet2 maps. lat/lon attributes are still present
on each node (required by the OSM XML schema) but are dummy zeros - only
local_x/local_y carry real position.

Usage (after launching edit_pose_graph.launch.py in another terminal):
    ros2 run buggy_mapping pose_graph_to_lanelet --ros-args \\
        -p output_file:=/path/to/output.osm -p lane_half_width:=1.3
"""
import math
import time
import xml.etree.ElementTree as ET
from xml.dom import minidom

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


class PoseGraphToLanelet(Node):

    def __init__(self):
        super().__init__('pose_graph_to_lanelet')
        self.declare_parameter('output_file', 'pose_graph_lanelet.osm')
        self.declare_parameter('lane_half_width', 1.3)
        self.declare_parameter('wait_sec', 5.0)
        self.output_file = self.get_parameter('output_file').value
        self.lane_half_width = float(self.get_parameter('lane_half_width').value)
        self.wait_sec = float(self.get_parameter('wait_sec').value)
        self.vertices = {}
        self.create_subscription(
            MarkerArray, '/slam_toolbox/graph_visualization', self._on_graph, 1)

    def _on_graph(self, msg):
        for m in msg.markers:
            if m.ns == 'slam_toolbox':
                self.vertices[m.id] = (m.pose.position.x, m.pose.position.y)

    def wait_for_graph(self):
        start = time.time()
        while rclpy.ok() and time.time() - start < self.wait_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
        return len(self.vertices) > 0

    def ordered_centerline(self):
        """Vertex IDs are assigned in recording order (confirmed all
        session via graph_visualization) - sorting by ID reproduces the
        driven path order."""
        return [self.vertices[i] for i in sorted(self.vertices)]

    def offset_boundaries(self, centerline):
        """Same perpendicular-offset technique as buggy_nav_costmap's own
        publishFreeLaneAreaMarker() - offset by lane_half_width to each
        side of the local path direction at each point."""
        n = len(centerline)
        left = []
        right = []
        for i in range(n):
            if i + 1 < n:
                dx = centerline[i + 1][0] - centerline[i][0]
                dy = centerline[i + 1][1] - centerline[i][1]
            else:
                dx = centerline[i][0] - centerline[i - 1][0]
                dy = centerline[i][1] - centerline[i - 1][1]
            length = math.hypot(dx, dy)
            if length < 1e-6:
                ox, oy = 0.0, 0.0
            else:
                ox = -dy / length * self.lane_half_width
                oy = dx / length * self.lane_half_width
            cx, cy = centerline[i]
            left.append((cx + ox, cy + oy))
            right.append((cx - ox, cy - oy))
        return left, right

    def write_osm(self, left, right):
        osm = ET.Element('osm', version='0.6', generator='pose_graph_to_lanelet')

        node_ids = {}
        next_id = [1]

        def add_node(x, y):
            nid = -next_id[0]  # negative IDs = locally-created, standard OSM convention
            next_id[0] += 1
            node = ET.SubElement(osm, 'node', id=str(nid), visible='true',
                                  version='1', lat='0.0', lon='0.0')
            ET.SubElement(node, 'tag', k='local_x', v=f'{x:.4f}')
            ET.SubElement(node, 'tag', k='local_y', v=f'{y:.4f}')
            ET.SubElement(node, 'tag', k='ele', v='0.0')
            return nid

        def add_way(points, way_type):
            nids = [add_node(x, y) for x, y in points]
            wid = -next_id[0]
            next_id[0] += 1
            way = ET.SubElement(osm, 'way', id=str(wid), visible='true', version='1')
            for nid in nids:
                ET.SubElement(way, 'nd', ref=str(nid))
            ET.SubElement(way, 'tag', k='type', v=way_type)
            ET.SubElement(way, 'tag', k='subtype', v='solid')
            return wid

        left_way_id = add_way(left, 'line_thin')
        right_way_id = add_way(right, 'line_thin')

        rel_id = -next_id[0]
        next_id[0] += 1
        rel = ET.SubElement(osm, 'relation', id=str(rel_id), visible='true', version='1')
        ET.SubElement(rel, 'member', type='way', ref=str(left_way_id), role='left')
        ET.SubElement(rel, 'member', type='way', ref=str(right_way_id), role='right')
        ET.SubElement(rel, 'tag', k='type', v='lanelet')
        ET.SubElement(rel, 'tag', k='subtype', v='road')
        ET.SubElement(rel, 'tag', k='location', v='urban')
        ET.SubElement(rel, 'tag', k='one_way', v='yes')

        rough = ET.tostring(osm, encoding='unicode')
        pretty = minidom.parseString(rough).toprettyxml(indent='  ')
        with open(self.output_file, 'w') as f:
            f.write(pretty)


def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphToLanelet()
    node.get_logger().info(
        f'Waiting up to {node.wait_sec}s for /slam_toolbox/graph_visualization...')
    if not node.wait_for_graph():
        node.get_logger().error(
            'No graph data received - is edit_pose_graph.launch.py running?')
        node.destroy_node()
        rclpy.shutdown()
        return

    centerline = node.ordered_centerline()
    node.get_logger().info(f'Got {len(centerline)} vertices, building lanelet boundaries...')
    left, right = node.offset_boundaries(centerline)
    node.write_osm(left, right)
    node.get_logger().info(
        f'Wrote {node.output_file}: {len(left)} left boundary points, '
        f'{len(right)} right boundary points, lane_half_width={node.lane_half_width}m.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
