#!/usr/bin/env python3
"""Pose-graph-referenced turn direction for freespace_grid_node's
from-scratch reactive planner.

freespace_grid_node has no saved map and no notion of a destination - it
just reactively picks whichever locally-clear arc looks best right now.
This node adds the missing "which way should the upcoming turn go" signal
on top of that, by reading slam_toolbox's recorded pose graph (loaded from
a saved map via slam_toolbox's LOCALIZATION mode) and reporting which way
the actual mapped path bends over the next turn_lookahead_m meters ahead of
the vehicle's current position.

Finds the pose-graph vertex nearest the vehicle, walks forward along the
graph (disambiguating direction by whichever neighbor best aligns with
current heading, since only vertex ARRAY ORDER survives - no edge list),
and compares the bearing to the point turn_lookahead_m out against current
heading.

Includes a sanity gate (nearest_vertex_max_dist): a badly-drifted or
not-yet-converged map->odom transform misplaces every pose-graph vertex by
the same rigid offset, and blindly using whichever vertex comes out
"nearest" after that is worse than not trusting the graph at all - if the
nearest vertex is still farther than this many meters away, this node
reports no direction this cycle instead of a confidently-wrong one.

Publishes:
  /turn_direction        (std_msgs/String)  "LEFT" / "RIGHT" / "STRAIGHT"
  /turn_distance_m       (std_msgs/Float32) meters until the recorded path
                                             actually starts bending away
                                             from current heading (NaN
                                             when not currently available)
  /turn_direction_marker (visualization_msgs/Marker) floating text, for RViz
  /turn_direction_path   (visualization_msgs/Marker) colored LINE_STRIP
                                             tracing the exact lookahead
                                             segment the call was based on
  /route_preview_path    (visualization_msgs/Marker) green LINE_STRIP -
                                             same pose-graph walk, much
                                             farther out (route_preview_
                                             distance_m, default 40m) -
                                             a long-range route preview
                                             alongside freespace_grid_node's
                                             own short-range golden-yellow
                                             suggested_lane
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


class PoseGraphTurnDirectionNode(Node):
    def __init__(self):
        super().__init__('pose_graph_turn_direction_node')

        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('pose_graph_topic', '/slam_toolbox/graph_visualization')
        # How far ahead (meters) along the recorded pose graph to look for
        # the upcoming turn.
        self.declare_parameter('turn_lookahead_m', 9.0)
        # Below this, a heading change from current_yaw is treated as noise
        # rather than a real upcoming corner.
        self.declare_parameter('snap_deg', 25.0)
        # Sanity gate - see module docstring.
        self.declare_parameter('nearest_vertex_max_dist', 5.0)
        self.declare_parameter('publish_rate', 5.0)
        # How many consecutive vertices to average over when deciding which
        # way is "forward" along the recorded path - see _lookahead_path's
        # local_direction_yaw for why this can't just be the single
        # immediate neighbor (one noisy/out-of-order vertex right next to
        # the nearest one was enough to send the whole downstream walk the
        # wrong way, confirmed live).
        self.declare_parameter('direction_disambiguation_vertices', 5)
        # Second, longer-range preview line (green) - same pose-graph walk
        # as the turn-direction one above, just much farther out: close to
        # (but under) freespace_grid_node's own max_ray_length (50m
        # default there), so it previews roughly as far as the LiDAR
        # itself is actually trusted to see, not just the near-term
        # turn_lookahead_m window used for the LEFT/RIGHT/STRAIGHT call.
        # Independent of turn_lookahead_m on purpose - raising this alone
        # does not change turn classification/urgency, only how far the
        # green preview line reaches.
        self.declare_parameter('route_preview_distance_m', 40.0)

        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.turn_lookahead_m = float(self.get_parameter('turn_lookahead_m').value)
        self.snap_deg = float(self.get_parameter('snap_deg').value)
        self.nearest_vertex_max_dist = float(self.get_parameter('nearest_vertex_max_dist').value)
        self.route_preview_distance_m = float(self.get_parameter('route_preview_distance_m').value)
        self.direction_disambiguation_vertices = int(
            self.get_parameter('direction_disambiguation_vertices').value)
        pose_graph_topic = self.get_parameter('pose_graph_topic').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._pose_graph_vertices_map = None
        # Frozen at whatever vertex count the graph had on the FIRST
        # callback (the originally-recorded route) - see
        # _pose_graph_callback's own comment for why this matters: without
        # it, the nearest-vertex search eventually starts landing on the
        # robot's own just-driven tail instead of the recorded route.
        self._original_vertex_count = None
        self.create_subscription(MarkerArray, pose_graph_topic, self._pose_graph_callback, 1)

        self.direction_pub = self.create_publisher(String, '/turn_direction', 1)
        self.distance_pub = self.create_publisher(Float32, '/turn_distance_m', 1)
        self.text_marker_pub = self.create_publisher(Marker, '/turn_direction_marker', 1)
        self.path_marker_pub = self.create_publisher(Marker, '/turn_direction_path', 1)
        # "Nearest assumption" alignment markers - see _publish_nearest_vertex_marker.
        # Published every cycle whenever ANY pose graph vertex exists,
        # independent of nearest_vertex_max_dist (that gate only decides
        # whether the graph is trustworthy enough to derive a turn
        # direction from) - so these still show the best-available match
        # even when the true pose is genuinely off the recorded route.
        self.nearest_vertex_pub = self.create_publisher(Marker, '/nearest_pose_graph_vertex', 1)
        self.nearest_link_pub = self.create_publisher(Marker, '/nearest_pose_graph_link', 1)
        self.nearest_text_pub = self.create_publisher(Marker, '/nearest_pose_graph_distance', 1)
        # Plain Float32 twin of the text marker above - not for RViz, for
        # actually measuring drift over a run: `ros2 topic echo` it, or
        # graph it with rqt_plot / record it in a bag, instead of trying to
        # eyeball whether the map is sliding by watching RViz. NaN
        # whenever no pose graph vertex is available at all (see
        # _nearest_vertex) - distinct from a real, large distance value.
        self.nearest_distance_pub = self.create_publisher(Float32, '/pose_graph_nearest_vertex_distance', 1)
        # Long-range (route_preview_distance_m) green preview line - see
        # that parameter's own comment and _publish_route_preview.
        self.route_preview_pub = self.create_publisher(Marker, '/route_preview_path', 1)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'Pose-graph turn direction: lookahead={self.turn_lookahead_m}m, '
            f'snap={self.snap_deg}deg, nearest_vertex_max_dist='
            f'{self.nearest_vertex_max_dist}m, fixed_frame="{self.fixed_frame}", '
            f'reading "{pose_graph_topic}".')

    def _pose_graph_callback(self, msg):
        """slam_toolbox's own graph_visualization MarkerArray - each vertex
        is a separate Marker(ns='slam_toolbox', type=SPHERE), position in
        .pose.position, published in frame_id 'map'.

        IMPORTANT - confirmed live: this node's LOCALIZATION mode doesn't
        just match against the frozen loaded map, it keeps APPENDING new
        vertices onto the END of this same array as the robot drives (its
        own just-driven trail, tacked onto the originally-recorded route).
        Left unbounded, the nearest-vertex search in _nearest_vertex/
        _lookahead_path eventually starts landing on one of those
        self-trail vertices instead of the actual recorded route - at that
        point "forward" (higher index) doesn't exist yet (the robot IS the
        tip), and "backward" just retraces the robot's own recent path
        instead of previewing the recorded route ahead, which is exactly
        what produced a route-preview line pointing backward instead of
        the direction actually being driven.

        _original_vertex_count freezes the ceiling at whatever count this
        callback saw FIRST (i.e., this node's own startup) - vertices
        appended after that are never used for nearest-vertex search or
        the walk, only the originally-recorded route is. Not a perfect
        boundary if a few self-trail vertices already existed the moment
        this node started, but it stops the ceiling from continuing to
        chase the robot indefinitely, which is what actually mattered
        here."""
        vertices = [
            (m.pose.position.x, m.pose.position.y)
            for m in msg.markers if m.ns == 'slam_toolbox'
        ]
        if self._original_vertex_count is None and vertices:
            self._original_vertex_count = len(vertices)
        self._pose_graph_vertices_map = np.array(vertices) if vertices else None

    def _vehicle_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.base_frame, Time(), timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'Vehicle pose TF lookup failed: {e}', throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _vertices_in_fixed_frame(self):
        """slam_toolbox publishes the pose graph in map frame - transform
        into fixed_frame (odom) via one rigid 2D transform (translation +
        yaw).

        Truncated to _original_vertex_count - see _pose_graph_callback's
        own comment for why anything appended after this node's startup
        (the robot's own live trail, not the recorded route) must never be
        a candidate for the nearest-vertex search."""
        if self._pose_graph_vertices_map is None or self._original_vertex_count is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.map_frame, Time(), timeout=Duration(seconds=0.2))
        except Exception:
            return None
        tx, ty = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        original = self._pose_graph_vertices_map[:self._original_vertex_count]
        vx = original[:, 0]
        vy = original[:, 1]
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        ox = tx + vx * cos_y - vy * sin_y
        oy = ty + vx * sin_y + vy * cos_y
        return np.column_stack([ox, oy])

    @staticmethod
    def _safe_nearest_index(d):
        """Index of the minimum FINITE value in d, or None if every entry
        is NaN/inf - i.e. no valid vertex to pick at all.

        Plain np.argmin is NOT safe here. Confirmed live (not
        hypothetical): np.argmin([50, 5, nan, 100]) returns 2 - the NaN's
        OWN index, not the true nearest index 1 - because NumPy's
        reduction propagates NaN once encountered (the running "current
        min" itself becomes NaN, so every later `x < current_min`
        comparison is False and the chosen index freezes wherever that
        first NaN appeared, anywhere in the array, not necessarily index
        0). A single vertex transforming to NaN (e.g. a momentarily
        degenerate map->odom transform) would otherwise silently return a
        garbage "nearest" index instead of being skipped, with no
        exception raised to catch it."""
        finite = np.isfinite(d)
        if not np.any(finite):
            return None
        return int(np.argmin(np.where(finite, d, np.inf)))

    def _nearest_vertex(self, x0, y0):
        """Nearest pose-graph vertex to (x0, y0), regardless of
        nearest_vertex_max_dist - that gate only decides whether the graph
        is trustworthy enough to derive a turn direction from (see
        _lookahead_path); this always reports the closest available match,
        so the alignment marker below still shows "the nearest assumption"
        even when the true pose is genuinely off the recorded line (bad
        localization, or just off-track), instead of publishing nothing
        the moment it stops being a confident match."""
        vertices = self._vertices_in_fixed_frame()
        if vertices is None or len(vertices) == 0:
            return None
        d = np.hypot(vertices[:, 0] - x0, vertices[:, 1] - y0)
        idx0 = self._safe_nearest_index(d)
        if idx0 is None:
            return None
        return float(vertices[idx0, 0]), float(vertices[idx0, 1]), float(d[idx0])

    def _publish_nearest_vertex_marker(self, stamp, x0, y0, nearest):
        """SPHERE at the nearest pose-graph vertex + a LINE_STRIP connecting
        it to the vehicle's actual current position, plus the gap distance
        as text - green/trusted within nearest_vertex_max_dist, orange
        past it (still shown - a stretched "nearest assumption" is more
        useful for judging alignment than no marker at all)."""
        sphere = Marker()
        sphere.header.frame_id = self.fixed_frame
        sphere.header.stamp = stamp
        sphere.ns = 'nearest_pose_graph_vertex'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.pose.orientation.w = 1.0

        link = Marker()
        link.header.frame_id = self.fixed_frame
        link.header.stamp = stamp
        link.ns = 'nearest_pose_graph_link'
        link.id = 0
        link.type = Marker.LINE_STRIP
        link.pose.orientation.w = 1.0
        link.scale.x = 0.08

        text = Marker()
        text.header.frame_id = self.fixed_frame
        text.header.stamp = stamp
        text.ns = 'nearest_pose_graph_distance'
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.orientation.w = 1.0
        text.scale.z = 0.4

        if nearest is None:
            sphere.action = link.action = text.action = Marker.DELETE
            self.nearest_vertex_pub.publish(sphere)
            self.nearest_link_pub.publish(link)
            self.nearest_text_pub.publish(text)
            self.nearest_distance_pub.publish(Float32(data=float('nan')))
            return

        vx, vy, dist = nearest
        self.nearest_distance_pub.publish(Float32(data=dist))
        trusted = dist <= self.nearest_vertex_max_dist
        color = (0.1, 0.9, 0.2, 0.95) if trusted else (1.0, 0.55, 0.0, 0.95)
        rgba = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])

        sphere.action = Marker.ADD
        sphere.pose.position.x = vx
        sphere.pose.position.y = vy
        sphere.pose.position.z = 0.2
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.6
        sphere.color = rgba
        self.nearest_vertex_pub.publish(sphere)

        link.action = Marker.ADD
        link.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.8)
        link.points = [Point(x=x0, y=y0, z=0.2), Point(x=vx, y=vy, z=0.2)]
        self.nearest_link_pub.publish(link)

        text.action = Marker.ADD
        text.pose.position.x = (x0 + vx) / 2.0
        text.pose.position.y = (y0 + vy) / 2.0
        text.pose.position.z = 0.6
        text.color = rgba
        text.text = f'{dist:.2f}m' + ('' if trusted else ' (past max)')
        self.nearest_text_pub.publish(text)

    def _lookahead_path(self, x0, y0, current_yaw, max_distance=None):
        """Walk the pose graph from whichever vertex is nearest (x0, y0)
        out to max_distance (turn_lookahead_m if not given), returning the
        list of (x, y) points walked (starting with the vehicle's own
        position) - or None if no graph/TF is available, or the nearest
        vertex is farther than nearest_vertex_max_dist (see module
        docstring's sanity-gate note).

        max_distance is a separate argument (not always self.turn_lookahead_m)
        so the same walk/disambiguation logic can also drive the much
        longer route_preview_distance_m green line without conflating the
        two - changing how far the preview line reaches must not change
        the near-term turn_lookahead_m used for LEFT/RIGHT/STRAIGHT.

        Only the vertex ARRAY ORDER is available (no edge list), assumed
        to match the order vertices were recorded in. The vehicle can now
        be retracing that path in either direction, so the walk direction
        is picked by whichever side's LOCAL DIRECTION (averaged over
        direction_disambiguation_vertices consecutive vertices, not just
        the single immediate neighbor - see local_direction_yaw below)
        better aligns with current_yaw."""
        if max_distance is None:
            max_distance = self.turn_lookahead_m
        vertices = self._vertices_in_fixed_frame()
        if vertices is None or len(vertices) < 2:
            return None

        d = np.hypot(vertices[:, 0] - x0, vertices[:, 1] - y0)
        idx0 = self._safe_nearest_index(d)
        if idx0 is None or d[idx0] > self.nearest_vertex_max_dist:
            return None
        n = len(vertices)

        def local_direction_yaw(i, step_dir):
            """Bearing from vertices[i] to whichever vertex is reached
            after walking up to direction_disambiguation_vertices steps in
            step_dir - NOT just the single immediate neighbor. A lone
            out-of-order or noisy vertex right next to idx0 can otherwise
            flip the chosen walk direction for the entire downstream path
            (confirmed live: the long route-preview line shooting off away
            from where the pose graph/map visibly continued, because the
            very first neighbor comparison happened to favor the wrong
            side) - averaging the bearing over several vertices smooths
            over that single-point noise without losing genuine short-
            radius turn responsiveness, since it's still a local window,
            not the whole path."""
            j = i
            last_valid = i
            for _ in range(self.direction_disambiguation_vertices):
                nj = j + step_dir
                if nj < 0 or nj >= n:
                    break
                last_valid = nj
                j = nj
            if last_valid == i:
                return None
            vx, vy = vertices[last_valid] - vertices[i]
            if vx == 0.0 and vy == 0.0:
                return None
            return math.atan2(vy, vx)

        def alignment(yaw):
            if yaw is None:
                return -2.0  # worse than any real cos() value
            diff = math.atan2(math.sin(yaw - current_yaw), math.cos(yaw - current_yaw))
            return math.cos(diff)

        fwd_yaw = local_direction_yaw(idx0, 1)
        bwd_yaw = local_direction_yaw(idx0, -1)
        step = 1 if alignment(fwd_yaw) >= alignment(bwd_yaw) else -1

        x, y = vertices[idx0]
        points = [(x0, y0), (float(x), float(y))]
        traveled = 0.0
        i = idx0
        while 0 <= i + step < n:
            nx, ny = vertices[i + step]
            seg = math.hypot(nx - x, ny - y)
            if traveled + seg >= max_distance:
                t_ = (max_distance - traveled) / seg if seg > 0 else 0.0
                points.append((x + t_ * (nx - x), y + t_ * (ny - y)))
                return points
            traveled += seg
            x, y = nx, ny
            points.append((float(x), float(y)))
            i += step
        return points  # graph ran out before max_distance - path just ends early

    def _distance_to_turn(self, points, current_yaw):
        """points[0] is the vehicle's own position. Cumulative distance to
        the first point whose bearing FROM THE VEHICLE deviates from
        current_yaw by at least snap_deg - i.e. how far ahead the recorded
        path actually starts bending. Returns turn_lookahead_m itself if
        it never deviates that much within the window."""
        x0, y0 = points[0]
        cumulative = 0.0
        prev_x, prev_y = points[0]
        for x, y in points[1:]:
            cumulative += math.hypot(x - prev_x, y - prev_y)
            prev_x, prev_y = x, y
            bearing = math.atan2(y - y0, x - x0)
            diff_deg = math.degrees(math.atan2(
                math.sin(bearing - current_yaw), math.cos(bearing - current_yaw)))
            if abs(diff_deg) >= self.snap_deg:
                return cumulative
        return self.turn_lookahead_m

    def _tick(self):
        pose = self._vehicle_pose()
        if pose is None:
            return
        x0, y0, yaw = pose
        stamp = self.get_clock().now().to_msg()

        self._publish_nearest_vertex_marker(stamp, x0, y0, self._nearest_vertex(x0, y0))

        path = self._lookahead_path(x0, y0, yaw)
        if path is None:
            self.direction_pub.publish(String(data='STRAIGHT'))
            self.distance_pub.publish(Float32(data=float('nan')))
            self._publish_text_marker(stamp, x0, y0, 'STRAIGHT', (1.0, 1.0, 1.0, 0.9), float('nan'))
            self._publish_path_marker(stamp, None, (1.0, 1.0, 1.0, 0.9))
            self._publish_route_preview(stamp, None)
            return

        far_x, far_y = path[-1]
        reference_yaw = math.atan2(far_y - y0, far_x - x0)
        diff_deg = math.degrees(math.atan2(
            math.sin(reference_yaw - yaw), math.cos(reference_yaw - yaw)))
        if abs(diff_deg) < self.snap_deg:
            direction, color = 'STRAIGHT', (1.0, 1.0, 1.0, 0.9)
        elif diff_deg > 0:
            direction, color = 'LEFT', (0.2, 0.4, 1.0, 0.95)
        else:
            direction, color = 'RIGHT', (1.0, 0.2, 0.2, 0.95)

        turn_distance = self._distance_to_turn(path, yaw)

        self.direction_pub.publish(String(data=direction))
        self.distance_pub.publish(Float32(data=float(turn_distance)))
        self._publish_text_marker(stamp, x0, y0, direction, color, turn_distance)
        self._publish_path_marker(stamp, path, color)

        preview_path = self._lookahead_path(x0, y0, yaw, self.route_preview_distance_m)
        self._publish_route_preview(stamp, preview_path)

    def _publish_route_preview(self, stamp, points):
        """Green LINE_STRIP tracing the pose graph out to
        route_preview_distance_m - a long-range preview of where the
        recorded route actually goes (bending left/right at a fork the
        same way the short-range turn_direction_path does, just much
        farther out), alongside freespace_grid_node's own short-range
        golden-yellow suggested_lane. Purely a visualization of the
        recorded route, same as turn_direction_path - it does not feed
        back into steering, so the existing reactive driving behavior
        (already direction-biased via /turn_direction + /turn_distance_m)
        is unaffected by this line existing."""
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = stamp
        marker.ns = 'route_preview_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD if points else Marker.DELETE
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.15
        marker.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.85)
        for x, y in (points or []):
            marker.points.append(Point(x=float(x), y=float(y), z=0.1))
        self.route_preview_pub.publish(marker)

    def _publish_text_marker(self, stamp, x, y, direction, color, turn_distance):
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = stamp
        marker.ns = 'turn_direction'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.8
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.6
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        marker.text = direction if math.isnan(turn_distance) else f'{direction} ({turn_distance:.1f}m)'
        self.text_marker_pub.publish(marker)

    def _publish_path_marker(self, stamp, points, color):
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = stamp
        marker.ns = 'turn_direction_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD if points else Marker.DELETE
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        for x, y in (points or []):
            marker.points.append(Point(x=float(x), y=float(y), z=0.2))
        self.path_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphTurnDirectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
