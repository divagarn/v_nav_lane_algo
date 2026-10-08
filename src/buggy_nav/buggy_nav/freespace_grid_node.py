#!/usr/bin/env python3
"""
Live, from-scratch free-space grid - no saved map, no AMCL, no Nav2 costmap.

Takes segment_ground's /non_ground_points (obstacle returns) AND
/ground_points (drivable ground-plane returns) as they arrive, transforms
each scan into a fixed frame (odom by default - odom starts at (0,0,0) at
launch and never resets, so it works as a stable accumulation frame without
needing any localization against a prior map), and ray-traces from the
sensor origin to every point: cells along the ray get a small FREE
(log-odds) nudge (the LiDAR beam passed through them unobstructed), and the
cell at the point itself gets an OCCUPIED nudge (non-ground) or a FREE one
(ground - it's the drivable surface itself). Using both streams matters: a
ground-facing LiDAR's returns are mostly ground hits, and ignoring them
(obstacle points only) throws away most of the evidence for "this area is
drivable," leaving large stretches of genuinely-observed clear ground
marked UNKNOWN for no reason.

Cells store a log-odds occupancy value (standard probabilistic occupancy
grid mapping, e.g. Thrun/Burgard/Fox), not a discrete state - each
observation nudges a cell's belief up or down by a fixed increment and
clamps at [-L_MAX, L_MAX], rather than overwriting it outright. This means
a single noisy or misclassified point can't instantly flip a well-observed
cell; it takes several consistent observations to move a confidently-known
cell's classification, while a cell still near 0 (unknown) responds
immediately to the first real observation. Ground-scan and non-ground-scan
rays both just nudge the same accumulator - no special-casing needed to
keep obstacle detection authoritative, the log-odds math does that
naturally (a cell nudged OCCUPIED many times needs many FREE nudges to undo).
Display classification (green/free, red/occupied, unrendered/unknown) is a
threshold read off the accumulated log-odds, with marker opacity scaled by
confidence - a cell right at the threshold renders faint, a
long-confirmed one renders solid.

The near-field blind-zone fill (see near_field_clear_radius below) uses a
deliberately weak prior nudge, not a direct classification - it's an
assumption standing in for missing sensor data, and should get overridden
quickly once real observations arrive, unlike genuine sensor hits.

The result is published as a MarkerArray - a flat CUBE_LIST (green=free/
drivable, red=occupied/non-drivable, opacity=confidence) plus an ARROW at
the vehicle's current pose pointing in its facing direction, plus a
suggested LINE_STRIP lane - for RViz.

This deliberately does not do localization correction or loop closure -
it's a local, live grid, good enough to see explored free space fill in
and update live as the vehicle drives; not a substitute for a real SLAM
back-end (e.g. RTAB-Map) if the goal is an accurate map over a large loop.

The grid is still bounded, just SPATIALLY, not by forgetting over time:
the recentering window (recenter_trigger_ratio) drops cells once they
scroll outside the current +-grid_half_extent view as the vehicle drives
far away - that's the whole memory limit. An optional TIME-based decay
also exists (decay_half_life_sec, see _apply_decay) that would additionally
fade a cell's belief toward unknown over real elapsed time even while
still inside the window - but it's OFF by default (0): confirmed live
that enabling it made the grid stop matching the real/recorded map, since
cells outside current sensor coverage faded out before the vehicle looped
back around to them. Set decay_half_life_sec > 0 to opt back into it.
"""
import math
import time
from collections import deque

import numpy as np
from scipy.ndimage import binary_dilation
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float32, String, Bool
from geometry_msgs.msg import Point
import tf2_ros


def logit(p):
    """Probability -> log-odds."""
    return math.log(p / (1.0 - p))


def quat_to_rot_matrix(x, y, z, w):
    """Quaternion -> 3x3 rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def bresenham_line(c0, r0, c1, r1):
    """Integer grid cells from (c0,r0) to (c1,r1), inclusive of both ends."""
    cells = []
    dc = abs(c1 - c0)
    dr = -abs(r1 - r0)
    sc = 1 if c0 < c1 else -1
    sr = 1 if r0 < r1 else -1
    err = dc + dr
    c, r = c0, r0
    while True:
        cells.append((c, r))
        if c == c1 and r == r1:
            break
        e2 = 2 * err
        if e2 >= dr:
            err += dr
            c += sc
        if e2 <= dc:
            err += dc
            r += sr
    return cells


class FreespaceGridNode(Node):
    def __init__(self):
        super().__init__('freespace_grid_node')

        self.declare_parameter('non_ground_topic', '/non_ground_points')
        self.declare_parameter('ground_topic', '/ground_points')
        # 0 (default): unlimited - process every incoming scan, same as
        # before this existed. Real hardware publishes /velodyne_points
        # (and therefore /non_ground_points, /ground_points) at its
        # native rate (e.g. 10Hz) vs the much lower rate this stack has
        # been tested against in sim so far - _mark_rays is still a plain
        # Python per-point ray-tracing loop (see that method's own
        # docstring for why it hasn't been vectorized: log-odds clamping
        # is applied after EACH individual nudge, so the result depends
        # on update order when multiple rays sweep the same cell in one
        # scan, making an exact vectorization fragile without live
        # profiling data confirming it's actually needed). Set this to a
        # real hardware-appropriate rate (e.g. 10.0, or lower if that's
        # still too slow) to explicitly DROP scans arriving faster than
        # that instead of letting them queue up - processing a backlogged
        # scan against a pose that's no longer current actively worsens
        # the TF-staleness problem rather than helping, so dropping is
        # safer than buffering here.
        self.declare_parameter('max_scan_process_rate_hz', 0.0)
        # When true, /ground_points scans are ignored entirely - the grid
        # only ever gets marked from non_ground (obstacle) rays, same as
        # before ground-point free-space marking was added. Off by default.
        self.declare_parameter('avoid_ground', False)
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('resolution', 0.3)
        # Half-extent kept comfortably larger than max_ray_length below, so
        # a ray from anywhere near the vehicle's current position can't run
        # off the grid's own array bounds before hitting its real range cap.
        self.declare_parameter('grid_half_extent', 60.0)
        # The grid window recenters on the sensor once it gets within this
        # fraction of grid_half_extent from an edge, instead of having a
        # hard boundary the vehicle can drive past (confirmed happening
        # live: a flat cutoff edge appeared once the vehicle got close to
        # the old fixed window's border). Recentering shifts the existing
        # data to match (no re-observation needed for what's still in view)
        # and discards whatever falls outside the new window - this is a
        # local, bounded-memory grid, not an unbounded map; far-away history
        # that scrolls out is gone, same tradeoff a rolling costmap makes.
        self.declare_parameter('recenter_trigger_ratio', 0.75)
        self.declare_parameter('voxel_size', 0.25)
        # The simulated Velodyne is actually configured for 1.0-100.0m range
        # (buggy.velodyne.xacro) - 25m was still throwing away most of what
        # it genuinely sees at any real distance (confirmed visually: raw
        # returns well past the grid's edge in RViz). Raised to 50m; still
        # capped short of the full 100m for ray-tracing cost, not because
        # the sensor can't see further.
        self.declare_parameter('max_ray_length', 50.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('heading_arrow_length', 2.5)
        # Time-based forgetting, independent of the spatial recentering
        # above - every cell's log-odds value decays toward 0 (unknown) by
        # half every decay_half_life_sec of real elapsed time (see
        # _apply_decay). Reverted back to 0 (disabled, indefinite memory
        # within the window) by default - confirmed live this made the
        # grid stop matching the real/recorded map (cells the vehicle
        # isn't currently re-scanning faded out well before it looped back
        # around to them, so the visible grid disagreed with what was
        # actually still there). Left in as an opt-in knob, not removed -
        # set a positive value to re-enable it if a future track/run
        # genuinely wants aged-out cells to fade.
        self.declare_parameter('decay_half_life_sec', 0.0)

        # --- Log-odds occupancy parameters (probabilities, converted to
        # log-odds internally) - standard occupancy-grid-mapping formulation.
        # A "hit" (obstacle observed at this cell) nudges belief toward
        # occupied; a "miss" (beam passed through, or ground observed here)
        # nudges toward free. Clamped so no cell becomes so confident that a
        # burst of real new evidence couldn't still move it.
        self.declare_parameter('p_hit', 0.7)
        self.declare_parameter('p_miss', 0.4)
        self.declare_parameter('p_min', 0.05)
        self.declare_parameter('p_max', 0.95)
        self.declare_parameter('p_occupied_threshold', 0.65)
        self.declare_parameter('p_free_threshold', 0.35)
        # LiDARs have a near-field blind zone right around their own mount
        # (beams that low/close never reach the ground before the vehicle's
        # own body occludes them), which shows up as a permanent UNKNOWN
        # wedge following the vehicle no matter how long it drives. The
        # vehicle is physically sitting there, so that space is presumed
        # traversable - but this is an ASSUMPTION standing in for missing
        # data, not a real observation, so it only nudges still-unknown
        # cells with a weak prior (p_near_field_assumed, close to 0.5) that
        # real sensor data can override in a single subsequent observation -
        # never touches a cell with any real evidence either way.
        self.declare_parameter('near_field_clear_radius', 4.0)
        self.declare_parameter('p_near_field_assumed', 0.42)
        # Buggy dimensions - used to inflate obstacles by half-width+margin
        # so a suggested lane never clips a wall, to bound candidate path
        # curvature to what the buggy can actually steer, and (half_length,
        # half_width) to draw the real rectangular footprint (see
        # _build_footprint_marker). Originally matched
        # global_costmap_mapped.yaml's footprint polygon (~1.5m half-width,
        # ~1.4m half-length) - deliberately overridden narrower/longer here
        # (1.0m/1.8m) rather than the real physical footprint; keep this in
        # sync with vehicle_footprint_margin below if the real vehicle's
        # actual size changes, since margin is meant to sit on top of the
        # true footprint, not compensate for an inaccurate one.
        self.declare_parameter('vehicle_half_width', 1.0)
        self.declare_parameter('vehicle_half_length', 1.8)
        self.declare_parameter('vehicle_footprint_margin', 0.3)
        # Costmap-style inflation halo drawn around every occupied cell, at
        # exactly the radius (vehicle_half_width + vehicle_footprint_margin
        # = self.clearance, set once both those are read below) the arc
        # planner's own footprint check (_arc_is_clear) already enforces -
        # NEW: previously that margin was only ever a number in a param
        # file, never actually visible. Drawn as a distinct orange band in
        # publish_grid's CUBE_LIST (see show_safety_region), same
        # radius-in-cells value _arc_is_clear's offset checks use, so
        # what's shown is exactly what's enforced, not a separately-tuned
        # approximation of it.
        self.declare_parameter('show_safety_region', True)
        self.declare_parameter('safety_region_alpha', 0.35)
        self.declare_parameter('wheelbase', 1.6)
        self.declare_parameter('max_steering_deg', 20.0)
        self.declare_parameter('lane_lookahead', 12.0)
        self.declare_parameter('lane_num_candidates', 15)
        # How far to each side to scan for left/right corridor clearance
        # when judging how centered a candidate arc is.
        self.declare_parameter('corridor_scan_limit', 8.0)
        # Among arcs whose clear distance is within this fraction of the best
        # one found, prefer the best-centered (equal clearance left/right)
        # rather than always chasing the single longest arc - keeps the lane
        # in the middle of the corridor instead of hugging one wall just
        # because that arc reached a few cm further.
        self.declare_parameter('lane_straight_preference_ratio', 0.85)
        # Exponential smoothing (0=frozen, 1=no smoothing) applied to the
        # chosen curvature between publishes, so the lane doesn't jump
        # between visibly different arcs frame to frame as new scan data
        # nudges which candidate comes out slightly ahead.
        self.declare_parameter('lane_smoothing_alpha', 0.3)
        # Matches steering_calculator_node's own default lookahead_distance
        # (cart_perception) - the arc's lateral offset AT this distance is
        # what gets published to /target_lateral_position for it to consume.
        self.declare_parameter('steering_lookahead_distance', 6.0)

        # --- Direction bias (from /turn_direction, /turn_distance_m -
        # pose_graph_turn_direction_node, when running) ---
        # How strongly a candidate arc's effective centering_error is
        # reduced (see _direction_bias_bonus) for curving toward the
        # currently-commanded turn direction. 0 disables the bias entirely
        # (pure reactive behavior, unchanged from before this was added).
        # Deliberately small relative to a typical centering_error spread
        # (a handful of meters) - direction only nudges/breaks ties among
        # arcs _plan_lane's own lane_straight_preference_ratio filter has
        # already judged comparably clear; it never overrides that safety
        # filter.
        self.declare_parameter('direction_bias_gain', 1.5)
        # /turn_direction hasn't been refreshed within this long -> treat
        # as STRAIGHT (no bias) rather than trusting a stale value. Same
        # graceful-degrade pattern as steering_calculator_node's own wheel
        # feedback timeout - a crashed/not-running turn-direction
        # publisher just means "no directional bias", never a stuck LEFT/
        # RIGHT forever.
        self.declare_parameter('turn_signal_timeout_sec', 2.0)

        # --- Reference-path following (from /route_preview_path -
        # pose_graph_turn_direction_node's green 40m line, when running) ---
        # Meters of centering_error "forgiven" per meter a candidate arc's
        # ENDPOINT sits away from the recorded route (the green line) -
        # this is what actually makes _plan_lane track the recorded
        # center-line's real shape, not just lean the right general
        # direction the way direction_bias_gain alone does. Same
        # subtracted-from-centering_error mechanism as direction_bias_gain
        # (only re-ranks candidates the obstacle-clearance safety filter
        # already judged comparably clear - never overrides it), and the
        # two stack: direction_bias_gain nudges toward the correct side
        # near a turn, this one additionally pulls toward the actual
        # recorded path's shape at every cycle, turn or not. 0 disables it
        # (candidates chosen purely by local clearance/centering, same as
        # before this existed).
        self.declare_parameter('reference_path_weight', 1.5)

        # --- Pose-graph turn direction (merged in from what used to be a
        # separate pose_graph_turn_direction_node, still available standalone
        # under that name if something else wants it independently) ---
        # MERGED, not just topic-connected, specifically to fix a real
        # confirmed-live problem: as two separate nodes on two independent
        # timers (grid at publish_rate, turn-direction at its own
        # publish_rate), each did its OWN separate TF lookup for the
        # vehicle's current pose - close in time, but never the exact same
        # read, and on weaker real-hardware compute the gap between them
        # could grow enough that the lane planner and the direction bias
        # were reasoning about the robot being in two subtly different
        # places in the same instant. Now _tick_pose_graph_turn_direction
        # is called from THIS node's own publish_grid, reusing the exact
        # same (pos, forward) _get_vehicle_pose() already fetched for
        # _plan_lane - one TF read, shared, not two independently-timed
        # ones - and self.turn_direction/self.turn_distance_m/
        # self._reference_path_points are set DIRECTLY (no publish-then-
        # subscribe-to-own-topic round trip) even though the same
        # /turn_direction, /turn_distance_m, /route_preview_path, etc.
        # topics are still published for RViz/external visibility.
        self.declare_parameter('enable_pose_graph_bias', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('pose_graph_topic', '/slam_toolbox/graph_visualization')
        self.declare_parameter('turn_lookahead_m', 9.0)
        self.declare_parameter('snap_deg', 25.0)
        self.declare_parameter('nearest_vertex_max_dist', 5.0)
        self.declare_parameter('direction_disambiguation_vertices', 5)
        self.declare_parameter('route_preview_distance_m', 40.0)
        # Robustness against a plausible-looking but WRONG nearest-vertex
        # pick (e.g. snapping toward the start of the recorded route -
        # "0th index") that survives NaN-filtering/vertex-freeze alone -
        # confirmed live, repeatedly, in carrot_path_publisher.py before
        # this was ported here. Keeps the last pose_graph_history_depth
        # ACCEPTED (start point, path) pairs per named walk (see
        # _robust_lookahead_path) - a fresh walk is only accepted if its
        # start point is within route_preview_max_jump_m of the MEDIAN
        # start point already in history (more robust than comparing only
        # against the single most recent entry - a lone bad entry already
        # in history can't by itself validate an equally-bad new one).
        # Rejected/failed cycles fall back to the most recent still-fresh
        # history entry instead.
        self.declare_parameter('pose_graph_history_depth', 3)
        # If this many consecutive REJECTED readings all agree with EACH
        # OTHER (within route_preview_max_jump_m), that's treated as
        # evidence the existing history was a bad seed, not that the new
        # readings are noise - history gets reset to trust the new
        # cluster instead of entrenching a wrong value forever. See
        # _robust_lookahead_path's own "FIX" comment for the confirmed
        # self-poisoning failure mode this closes.
        self.declare_parameter('pose_graph_reset_streak_threshold', 3)
        self.declare_parameter('route_preview_max_jump_m', 3.0)
        # Effectively unbounded (~11 days) by default - "store the last
        # known-good pose/path and use it until the next known-good one
        # is identified", not discarded just because time passed. The
        # real safeguard against trusting a BAD read is the median-jump
        # check above, not a clock.
        self.declare_parameter('route_preview_cache_timeout_sec', 999999.0)
        # See _pose_graph_callback's own comment - freezing on the very
        # first callback was fragile if that message arrived before
        # slam_toolbox finished publishing the complete loaded graph.
        self.declare_parameter('pose_graph_stabilize_count', 2)

        # --- Stuck / recovery ---
        # Consecutive publish_grid cycles with _plan_lane finding no clear
        # arc at all before starting an automatic recovery maneuver.
        self.declare_parameter('stuck_cycles_threshold', 5)
        # Signed - negative reverses. Held for recovery_duration_sec, then
        # the vehicle stops and holds for recovery_settle_sec (so the grid
        # can accumulate fresh observations from the new vantage point -
        # often enough by itself to un-block a candidate that was only
        # rejected for insufficient clearance right at the old position)
        # before _plan_lane is tried again.
        self.declare_parameter('recovery_reverse_speed_kmh', -1.0)
        self.declare_parameter('recovery_duration_sec', 3.0)
        self.declare_parameter('recovery_settle_sec', 1.0)
        # First recovery attempt reverses straight back; later attempts
        # alternate +/- this steering angle, so repeated retries don't all
        # reverse along the identical line if that's what got it stuck.
        self.declare_parameter('recovery_steer_deg_magnitude', 15.0)
        # After this many failed recovery attempts in a row, stop trying
        # automatically and report /nav_status "STUCK" instead of looping
        # forever - this is the "mapping area limitation" case: the local
        # grid genuinely has nowhere clear to go from here, and needs
        # either a wider view (drive it clear by hand, or relaunch the
        # localization/mapping stack for this area) rather than more
        # reversing.
        self.declare_parameter('recovery_max_attempts', 3)

        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.avoid_ground = bool(self.get_parameter('avoid_ground').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.half_extent = float(self.get_parameter('grid_half_extent').value)
        self.recenter_trigger_ratio = float(self.get_parameter('recenter_trigger_ratio').value)
        # World coordinates of the grid window's center - shifts as the grid
        # recenters, starting at the origin since odom itself starts there.
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_ray_length = float(self.get_parameter('max_ray_length').value)
        self.heading_arrow_length = float(self.get_parameter('heading_arrow_length').value)
        self.decay_half_life_sec = float(self.get_parameter('decay_half_life_sec').value)
        self._last_decay_time = self.get_clock().now()
        self.near_field_clear_radius = float(self.get_parameter('near_field_clear_radius').value)
        self.near_field_clear_radius_cells = int(round(
            self.near_field_clear_radius / self.resolution))

        # Log-odds increments/clamps/thresholds, derived once from the
        # probability parameters above.
        self.l_hit = logit(float(self.get_parameter('p_hit').value))
        self.l_miss = logit(float(self.get_parameter('p_miss').value))
        self.l_min = logit(float(self.get_parameter('p_min').value))
        self.l_max = logit(float(self.get_parameter('p_max').value))
        self.l_occ_thresh = logit(float(self.get_parameter('p_occupied_threshold').value))
        self.l_free_thresh = logit(float(self.get_parameter('p_free_threshold').value))
        self.l_near_field = logit(float(self.get_parameter('p_near_field_assumed').value))

        self.vehicle_half_width = float(self.get_parameter('vehicle_half_width').value)
        self.vehicle_half_length = float(self.get_parameter('vehicle_half_length').value)
        self.vehicle_footprint_margin = float(self.get_parameter('vehicle_footprint_margin').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steering_deg = float(self.get_parameter('max_steering_deg').value)
        self.lane_lookahead = float(self.get_parameter('lane_lookahead').value)
        self.lane_num_candidates = int(self.get_parameter('lane_num_candidates').value)
        self.corridor_scan_limit = float(self.get_parameter('corridor_scan_limit').value)
        self.lane_straight_preference_ratio = float(
            self.get_parameter('lane_straight_preference_ratio').value)
        self.lane_smoothing_alpha = float(self.get_parameter('lane_smoothing_alpha').value)
        self.steering_lookahead_distance = float(
            self.get_parameter('steering_lookahead_distance').value)
        self.smoothed_curvature = 0.0

        self.direction_bias_gain = float(self.get_parameter('direction_bias_gain').value)
        self.turn_signal_timeout_sec = float(self.get_parameter('turn_signal_timeout_sec').value)
        self.turn_direction = 'STRAIGHT'
        self.turn_distance_m = float('nan')
        self.last_turn_signal_time = self.get_clock().now()

        self.reference_path_weight = float(self.get_parameter('reference_path_weight').value)
        self._reference_path_points = None  # Nx2 numpy array in fixed_frame, or None
        self.last_reference_path_time = self.get_clock().now()

        self.enable_pose_graph_bias = bool(self.get_parameter('enable_pose_graph_bias').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.turn_lookahead_m = float(self.get_parameter('turn_lookahead_m').value)
        self.snap_deg = float(self.get_parameter('snap_deg').value)
        self.nearest_vertex_max_dist = float(self.get_parameter('nearest_vertex_max_dist').value)
        self.direction_disambiguation_vertices = int(
            self.get_parameter('direction_disambiguation_vertices').value)
        self.route_preview_distance_m = float(self.get_parameter('route_preview_distance_m').value)
        self._pose_graph_vertices_map = None
        # Frozen once the vertex count has been stable for
        # pose_graph_stabilize_count consecutive callbacks - see
        # _pose_graph_callback's own comment for why not on the first one.
        self._original_vertex_count = None
        self._pose_graph_last_len = None
        self._pose_graph_stable_count = 0
        self.pose_graph_stabilize_count = int(
            self.get_parameter('pose_graph_stabilize_count').value)
        # See pose_graph_history_depth's own comment - one deque per named
        # walk ('turn', 'preview'), populated by _robust_lookahead_path.
        self.pose_graph_history_depth = int(self.get_parameter('pose_graph_history_depth').value)
        self.pose_graph_reset_streak_threshold = int(
            self.get_parameter('pose_graph_reset_streak_threshold').value)
        self.route_preview_max_jump_m = float(self.get_parameter('route_preview_max_jump_m').value)
        self.route_preview_cache_timeout_sec = float(
            self.get_parameter('route_preview_cache_timeout_sec').value)
        self._pose_graph_walk_history = {}
        self._pose_graph_rejected_streak = {}

        self.stuck_cycles_threshold = int(self.get_parameter('stuck_cycles_threshold').value)
        self.recovery_reverse_speed_kmh = float(self.get_parameter('recovery_reverse_speed_kmh').value)
        self.recovery_duration_sec = float(self.get_parameter('recovery_duration_sec').value)
        self.recovery_settle_sec = float(self.get_parameter('recovery_settle_sec').value)
        self.recovery_steer_deg_magnitude = float(
            self.get_parameter('recovery_steer_deg_magnitude').value)
        self.recovery_max_attempts = int(self.get_parameter('recovery_max_attempts').value)
        self._stuck_count = 0
        self._recovery_state = None  # None | 'REVERSING' | 'SETTLING'
        self._recovery_state_until = None
        self._recovery_attempt = 0
        self._recovery_steer_deg = 0.0
        # Ackermann min turning radius: R = wheelbase / tan(max_steer). Guard
        # against a zero/near-zero max_steering_deg (would imply infinite
        # radius i.e. straight-only).
        steer_rad = math.radians(max(self.max_steering_deg, 0.5))
        self.min_turn_radius = self.wheelbase / math.tan(steer_rad)
        self.max_curvature = 1.0 / self.min_turn_radius
        self.clearance = self.vehicle_half_width + self.vehicle_footprint_margin
        self.show_safety_region = bool(self.get_parameter('show_safety_region').value)
        self.safety_region_alpha = float(self.get_parameter('safety_region_alpha').value)
        # Circular structuring element for the safety-region dilation (see
        # publish_grid) - radius in CELLS, computed once here since
        # self.clearance/self.resolution don't change at runtime.
        safety_radius_cells = max(1, int(round(self.clearance / self.resolution)))
        _yy, _xx = np.ogrid[
            -safety_radius_cells:safety_radius_cells + 1,
            -safety_radius_cells:safety_radius_cells + 1]
        self._safety_region_kernel = (_xx ** 2 + _yy ** 2) <= safety_radius_cells ** 2

        self.n_cells = int(round(2 * self.half_extent / self.resolution))
        # log-odds accumulator, 0.0 = p=0.5 = fully unknown.
        self.log_odds = np.zeros((self.n_cells, self.n_cells), dtype=np.float32)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, '/freespace_grid_markers', 1)
        self.target_lateral_pub = self.create_publisher(Float32, '/target_lateral_position', 1)
        # Recovery override consumed by steering_calculator_node - see that
        # node's own recovery_timeout_sec comment.
        self.recovery_active_pub = self.create_publisher(Bool, '/recovery_active', 1)
        self.recovery_speed_pub = self.create_publisher(Float32, '/recovery_speed_kmh', 1)
        self.recovery_steering_pub = self.create_publisher(Float32, '/recovery_steering_deg', 1)
        self.nav_status_pub = self.create_publisher(String, '/nav_status', 1)

        self.create_subscription(
            PointCloud2, self.get_parameter('non_ground_topic').value,
            self.non_ground_callback, 5)
        self.create_subscription(
            PointCloud2, self.get_parameter('ground_topic').value,
            self.ground_callback, 5)
        # Pose-graph turn direction, merged in - see enable_pose_graph_bias's
        # own comment for why this is one subscription to the raw pose
        # graph instead of subscribing to a separate node's derived /turn_
        # direction, /turn_distance_m, /route_preview_path topics. Absent
        # entirely (no map/localization launched, or slam_toolbox not yet
        # publishing), this just never receives anything and
        # _current_turn_direction()/_reference_path_bonus() gracefully
        # degrade to no bias via the same timeout pattern as before.
        self.create_subscription(
            MarkerArray, self.get_parameter('pose_graph_topic').value,
            self._pose_graph_callback, 1)
        self.direction_pub = self.create_publisher(String, '/turn_direction', 1)
        self.distance_pub = self.create_publisher(Float32, '/turn_distance_m', 1)
        self.text_marker_pub = self.create_publisher(Marker, '/turn_direction_marker', 1)
        self.path_marker_pub = self.create_publisher(Marker, '/turn_direction_path', 1)
        self.nearest_vertex_pub = self.create_publisher(Marker, '/nearest_pose_graph_vertex', 1)
        self.nearest_link_pub = self.create_publisher(Marker, '/nearest_pose_graph_link', 1)
        self.nearest_text_pub = self.create_publisher(Marker, '/nearest_pose_graph_distance', 1)
        self.nearest_distance_pub = self.create_publisher(
            Float32, '/pose_graph_nearest_vertex_distance', 1)
        self.route_preview_pub = self.create_publisher(Marker, '/route_preview_path', 1)
        # Flat circle around the vehicle at radius nearest_vertex_max_dist -
        # makes the actual trust boundary for nearest-vertex search
        # directly visible in RViz (is the nearest_pose_graph_vertex
        # marker inside or outside this ring?), instead of only
        # inferable from that marker's own green/orange color. Draws
        # exactly the value nearest_vertex_max_dist is set to, not a
        # separately-guessed display radius - same "show what's actually
        # enforced" approach as the safety-region inflation halo.
        self.nearest_vertex_search_radius_pub = self.create_publisher(
            Marker, '/nearest_pose_graph_search_radius', 1)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self.publish_grid)

        max_scan_rate = float(self.get_parameter('max_scan_process_rate_hz').value)
        self._min_scan_interval_sec = 1.0 / max_scan_rate if max_scan_rate > 0.0 else 0.0
        self._last_non_ground_process_time = None
        self._last_ground_process_time = None
        self._scans_dropped = 0

        # Hz/latency instrumentation - unlike segment_ground.py's own
        # _perf(), this node had none at all despite _mark_rays being a
        # plain per-point Python loop and publish_grid having already been
        # found lagging its target rate once before (see that method's own
        # history comment). Three independent streams, each logged the
        # same avg-ms/Hz way every 10 calls: non_ground/ground scans run on
        # message arrival at whatever rate their topic delivers,
        # publish_grid runs on its own publish_rate timer.
        self._perf_stats = {}

        self.scans_processed = 0
        self.get_logger().info(
            f'Freespace grid: {self.n_cells}x{self.n_cells} cells @ '
            f'{self.resolution}m/cell (+/-{self.half_extent}m), fixed_frame='
            f'"{self.fixed_frame}", no saved map / no AMCL - fresh accumulation only. '
            f'avoid_ground={self.avoid_ground} '
            f'({"ignoring /ground_points entirely" if self.avoid_ground else "using /ground_points for free-space marking"}). '
            f'Log-odds: l_hit={self.l_hit:.3f} l_miss={self.l_miss:.3f} '
            f'clamp=[{self.l_min:.3f},{self.l_max:.3f}] '
            f'display_thresholds=[{self.l_free_thresh:.3f},{self.l_occ_thresh:.3f}]. '
            f'decay_half_life_sec={self.decay_half_life_sec} '
            f'({"disabled - indefinite memory within the window" if self.decay_half_life_sec <= 0.0 else "unobserved cells fade to unknown over time"}).')
        self.get_logger().info(
            f'Lane planner: half_width={self.vehicle_half_width}m + '
            f'margin={self.vehicle_footprint_margin}m clearance, wheelbase='
            f'{self.wheelbase}m, max_steering={self.max_steering_deg}deg -> '
            f'min_turn_radius={self.min_turn_radius:.2f}m, lookahead='
            f'{self.lane_lookahead}m.')

    def _pose_graph_callback(self, msg):
        """slam_toolbox's own graph_visualization MarkerArray - each vertex
        is a separate Marker(ns='slam_toolbox', type=SPHERE), position in
        .pose.position, published in frame_id map_frame.

        Ported from the former standalone pose_graph_turn_direction_node -
        see enable_pose_graph_bias's own comment for why this is now
        merged directly into freespace_grid_node instead of a separate
        node's topics.

        IMPORTANT - confirmed live: slam_toolbox's LOCALIZATION mode keeps
        APPENDING new vertices onto the END of this same array as the
        robot drives (its own just-driven trail, tacked onto the
        originally-recorded route). Left unbounded, nearest-vertex search
        would eventually land on one of those self-trail vertices instead
        of the actual recorded route. _original_vertex_count freezes the
        ceiling once the vertex count is used to truncate future callbacks.

        FIX: freezing on the VERY FIRST non-empty callback (what this used
        to do) is fragile - if that first message arrives before
        slam_toolbox has finished publishing the complete loaded graph
        (a real possibility right after LOCALIZE_AT_POSE/startup), the
        freeze locks in a tiny count PERMANENTLY - not a transient bad
        read anything downstream can reject or fall back from, a
        structurally truncated graph from the very beginning. Confirmed
        live this produced exactly the "green line still pointing back
        toward the start of the route" symptom even with the NaN-filter,
        vertex-freeze, and jump-rejection fixes all already in place,
        because those all assume the underlying vertex count is at least
        trustworthy once frozen - this one wasn't. Now: only freeze once
        the count has been UNCHANGED for pose_graph_stabilize_count
        consecutive callbacks (i.e. the initial full-graph publish has
        visibly settled). Until stabilized, the FULL current array is
        used (best available) rather than blocking on a frozen count that
        might still be wrong."""
        vertices = [
            (m.pose.position.x, m.pose.position.y)
            for m in msg.markers if m.ns == 'slam_toolbox'
        ]
        if not vertices:
            return
        arr = np.array(vertices)
        n = len(arr)
        if self._original_vertex_count is None:
            if self._pose_graph_last_len == n:
                self._pose_graph_stable_count += 1
            else:
                self._pose_graph_stable_count = 1
            self._pose_graph_last_len = n
            if self._pose_graph_stable_count >= self.pose_graph_stabilize_count:
                self._original_vertex_count = n
                self.get_logger().info(
                    f'Pose graph vertex count stabilized at {n} - freezing here '
                    '(excludes any live trail appended from this point on).')
            self._pose_graph_vertices_map = arr
        else:
            self._pose_graph_vertices_map = arr[:self._original_vertex_count]

    def _vertices_in_fixed_frame(self):
        """slam_toolbox publishes the pose graph in map_frame - transform
        into fixed_frame (odom) via one rigid 2D transform (translation +
        yaw). Truncated to _original_vertex_count - see
        _pose_graph_callback's own comment for why anything appended after
        this node's startup must never be a candidate for the
        nearest-vertex search."""
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
        is NaN/inf. Plain np.argmin is NOT safe here - confirmed live that
        it returns the index of the FIRST NaN it encounters (NumPy's
        reduction propagates NaN once encountered), not the true nearest
        finite index, silently corrupting nearest-vertex picking whenever
        a distance computation produces one (e.g. a momentarily degenerate
        map->odom transform)."""
        finite = np.isfinite(d)
        if not np.any(finite):
            return None
        return int(np.argmin(np.where(finite, d, np.inf)))

    def _nearest_vertex(self, x0, y0):
        """Nearest pose-graph vertex to (x0, y0), regardless of
        nearest_vertex_max_dist - that gate only decides whether the graph
        is trustworthy enough to derive a turn direction from; this always
        reports the closest available match for the diagnostic marker."""
        vertices = self._vertices_in_fixed_frame()
        if vertices is None or len(vertices) == 0:
            return None
        d = np.hypot(vertices[:, 0] - x0, vertices[:, 1] - y0)
        idx0 = self._safe_nearest_index(d)
        if idx0 is None:
            return None
        return float(vertices[idx0, 0]), float(vertices[idx0, 1]), float(d[idx0])

    def _publish_nearest_vertex_search_radius(self, stamp, x0, y0):
        """Flat circle (LINE_STRIP ring, base_frame-height z) around the
        vehicle at radius nearest_vertex_max_dist - see
        nearest_vertex_search_radius_pub's own comment. Published every
        cycle unconditionally (doesn't depend on whether a match was
        actually found this cycle), so the boundary itself is always
        visible even while nothing currently sits inside it."""
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = stamp
        marker.ns = 'nearest_vertex_search_radius'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.6)
        n_pts = 48
        for i in range(n_pts + 1):
            theta = 2.0 * math.pi * i / n_pts
            p = Point()
            p.x = x0 + self.nearest_vertex_max_dist * math.cos(theta)
            p.y = y0 + self.nearest_vertex_max_dist * math.sin(theta)
            p.z = 0.05
            marker.points.append(p)
        self.nearest_vertex_search_radius_pub.publish(marker)

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
        vertex is farther than nearest_vertex_max_dist.

        Only the vertex ARRAY ORDER is available (no edge list), assumed
        to match the order vertices were recorded in. The vehicle can now
        be retracing that path in either direction, so the walk direction
        is picked by whichever side's LOCAL DIRECTION (averaged over
        direction_disambiguation_vertices consecutive vertices, not just
        the single immediate neighbor) better aligns with current_yaw."""
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
            step_dir - NOT just the single immediate neighbor, which a
            lone out-of-order/noisy vertex could otherwise flip."""
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

    def _robust_lookahead_path(self, x0, y0, current_yaw, max_distance, history_name):
        """_lookahead_path, guarded against a plausible-looking but WRONG
        nearest-vertex pick surviving even the NaN-filter/vertex-freeze/
        stabilization fixes - see pose_graph_history_depth's own comment.

        Keeps the last pose_graph_history_depth ACCEPTED (x, y, path,
        time) entries in self._pose_graph_walk_history[history_name] (a
        deque - 'turn' and 'preview' are tracked separately, since they're
        independent walks that can each succeed/fail on any given cycle).
        A fresh walk's start point is only ACCEPTED if it's within
        route_preview_max_jump_m of the MEDIAN start point already in
        history - a real vehicle can't move further than that between
        consecutive ~publish_rate cycles, and using the median (not just
        the single most recent entry) means one bad entry already in
        history can't by itself validate an equally bad new one. Rejected
        or failed cycles fall back to the most recent still-fresh (within
        route_preview_cache_timeout_sec) history entry instead.

        FIX (confirmed via direct offline test before this was applied): a
        naive "reject anything far from history" rule self-poisons - if
        the very FIRST accepted reading (when history is still empty, so
        nothing can be rejected yet) happens to be wrong, EVERY correct
        subsequent reading gets rejected forever afterward for disagreeing
        with that bad seed, since route_preview_cache_timeout_sec now
        defaults to effectively unbounded. self._pose_graph_rejected_streak
        tracks rejected readings that are mutually consistent WITH EACH
        OTHER (not with the existing history) - if pose_graph_reset_
        streak_threshold consecutive rejections all agree with each other,
        that's evidence they're the real, correct route and the EXISTING
        history is the outlier (e.g. seeded before localization settled),
        so history is reset to trust the new cluster instead of the old
        one. Verified this recovers from a bad seed within the streak
        threshold while genuinely scattered noise (each rejection
        disagreeing with the last) never falsely triggers a reset."""
        path = self._lookahead_path(x0, y0, current_yaw, max_distance)
        now = self.get_clock().now()
        history = self._pose_graph_walk_history.setdefault(
            history_name, deque(maxlen=self.pose_graph_history_depth))
        rejected = self._pose_graph_rejected_streak.setdefault(
            history_name, deque(maxlen=self.pose_graph_reset_streak_threshold))

        if path is not None and len(path) >= 2:
            start = path[0]
            if history:
                med_x = float(np.median([h[0] for h in history]))
                med_y = float(np.median([h[1] for h in history]))
                jump_ok = math.hypot(start[0] - med_x, start[1] - med_y) <= self.route_preview_max_jump_m
            else:
                jump_ok = True

            if jump_ok:
                history.append((start[0], start[1], path, now))
                rejected.clear()
                return path

            if rejected:
                rej_x = float(np.median([r[0] for r in rejected]))
                rej_y = float(np.median([r[1] for r in rejected]))
                consistent = math.hypot(start[0] - rej_x, start[1] - rej_y) <= self.route_preview_max_jump_m
            else:
                consistent = True
            if not consistent:
                rejected.clear()
            rejected.append((start[0], start[1]))

            if len(rejected) >= self.pose_graph_reset_streak_threshold:
                self.get_logger().warn(
                    f'Pose-graph walk ({history_name}): {len(rejected)} consecutive '
                    'mutually-consistent readings disagreed with existing history - '
                    'trusting the new cluster instead (existing history was likely a '
                    'bad seed).', throttle_duration_sec=2.0)
                history.clear()
                history.append((start[0], start[1], path, now))
                rejected.clear()
                return path

            self.get_logger().warn(
                f'Pose-graph walk ({history_name}) start point jumped implausibly from '
                'recent history - rejecting as a likely bad nearest-vertex pick, reusing '
                'the last accepted path instead.', throttle_duration_sec=2.0)

        if history:
            _, _, last_path, last_time = history[-1]
            age = (now - last_time).nanoseconds / 1e9
            if age <= self.route_preview_cache_timeout_sec:
                return last_path
        return None

    def _distance_to_turn(self, points, current_yaw):
        """points[0] is the vehicle's own position. Cumulative distance to
        the first point whose bearing FROM THE VEHICLE deviates from
        current_yaw by at least snap_deg. Returns turn_lookahead_m itself
        if it never deviates that much within the window."""
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

    def _publish_turn_direction_text_marker(self, stamp, x, y, direction, color, turn_distance):
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

    def _publish_turn_direction_path_marker(self, stamp, points, color):
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

    def _publish_route_preview_marker(self, stamp, points):
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

    def _tick_pose_graph_turn_direction(self, x0, y0, current_yaw, stamp):
        """The former standalone pose_graph_turn_direction_node's _tick(),
        called from publish_grid using the SAME (pos, forward) already
        fetched there for _plan_lane - see enable_pose_graph_bias's own
        comment. Sets self.turn_direction/self.turn_distance_m/
        self._reference_path_points DIRECTLY (no publish-then-subscribe-
        to-own-topic round trip) while still publishing every topic the
        old standalone node did, for RViz/external visibility."""
        self._publish_nearest_vertex_search_radius(stamp, x0, y0)
        self._publish_nearest_vertex_marker(stamp, x0, y0, self._nearest_vertex(x0, y0))

        path = self._robust_lookahead_path(x0, y0, current_yaw, self.turn_lookahead_m, 'turn')
        if path is None:
            self.turn_direction = 'STRAIGHT'
            self.turn_distance_m = float('nan')
            self.last_turn_signal_time = self.get_clock().now()
            self.direction_pub.publish(String(data='STRAIGHT'))
            self.distance_pub.publish(Float32(data=float('nan')))
            self._publish_turn_direction_text_marker(
                stamp, x0, y0, 'STRAIGHT', (1.0, 1.0, 1.0, 0.9), float('nan'))
            self._publish_turn_direction_path_marker(stamp, None, (1.0, 1.0, 1.0, 0.9))
            self._reference_path_points = None
            self._publish_route_preview_marker(stamp, None)
            return

        far_x, far_y = path[-1]
        reference_yaw = math.atan2(far_y - y0, far_x - x0)
        diff_deg = math.degrees(math.atan2(
            math.sin(reference_yaw - current_yaw), math.cos(reference_yaw - current_yaw)))
        if abs(diff_deg) < self.snap_deg:
            direction, color = 'STRAIGHT', (1.0, 1.0, 1.0, 0.9)
        elif diff_deg > 0:
            direction, color = 'LEFT', (0.2, 0.4, 1.0, 0.95)
        else:
            direction, color = 'RIGHT', (1.0, 0.2, 0.2, 0.95)

        turn_distance = self._distance_to_turn(path, current_yaw)

        self.turn_direction = direction
        self.turn_distance_m = turn_distance
        self.last_turn_signal_time = self.get_clock().now()
        self.direction_pub.publish(String(data=direction))
        self.distance_pub.publish(Float32(data=float(turn_distance)))
        self._publish_turn_direction_text_marker(stamp, x0, y0, direction, color, turn_distance)
        self._publish_turn_direction_path_marker(stamp, path, color)

        preview_path = self._robust_lookahead_path(
            x0, y0, current_yaw, self.route_preview_distance_m, 'preview')
        if preview_path:
            self._reference_path_points = np.array(preview_path)
            self.last_reference_path_time = self.get_clock().now()
        else:
            self._reference_path_points = None
        self._publish_route_preview_marker(stamp, preview_path)

    def _current_turn_direction(self):
        """STRAIGHT (no bias) whenever /turn_direction hasn't been
        refreshed recently - see turn_signal_timeout_sec's own comment."""
        age = (self.get_clock().now() - self.last_turn_signal_time).nanoseconds / 1e9
        if age > self.turn_signal_timeout_sec:
            return 'STRAIGHT'
        return self.turn_direction

    def _direction_bias_bonus(self, k):
        """Bonus (higher = more preferred) for candidate curvature k,
        based on the externally-published turn direction/distance. desired
        is 0 for STRAIGHT (or when the signal is stale/absent), which
        makes this an exact no-op - ordinary corridor driving with no
        turn commanded is bit-for-bit unaffected. Scaled by 'urgency': 0
        when the reported turn is farther away than this planner's own
        lane_lookahead horizon (not yet relevant to a locally-clear arc
        choice), rising to 1 as it comes within/at that lookahead."""
        desired = {'LEFT': 1.0, 'RIGHT': -1.0}.get(self._current_turn_direction(), 0.0)
        if desired == 0.0 or self.max_curvature <= 0.0:
            return 0.0
        if math.isnan(self.turn_distance_m):
            urgency = 1.0
        else:
            urgency = max(0.0, min(1.0, 1.0 - self.turn_distance_m / self.lane_lookahead))
        alignment = desired * (k / self.max_curvature)  # +1 fully aligned, -1 fully opposed
        return self.direction_bias_gain * urgency * alignment

    def _reference_path_bonus(self, ex, ey):
        """Bonus (higher = more preferred, same meters units as
        centering_error so it can be subtracted from it directly) for how
        close a candidate arc's ENDPOINT (ex, ey) sits to the recorded
        route (the green /route_preview_path line) - checked at the
        endpoint for the same reason candidate_centering elsewhere in this
        codebase is: (pos[0],pos[1]) is the same fixed point for every
        candidate, so checking there would mostly measure "how close is
        the recorded route to here right now" rather than "does THIS
        candidate's own endpoint actually track it". 0 (no preference)
        whenever no reference path is available or it's gone stale - same
        graceful-degrade pattern as _direction_bias_bonus."""
        age = (self.get_clock().now() - self.last_reference_path_time).nanoseconds / 1e9
        if self._reference_path_points is None or age > self.turn_signal_timeout_sec:
            return 0.0
        pts = self._reference_path_points
        if len(pts) < 2:
            return 0.0
        a = pts[:-1]
        b = pts[1:]
        ab = b - a
        ab_len_sq = np.sum(ab * ab, axis=1)
        ab_len_sq[ab_len_sq == 0.0] = 1e-9
        ap = np.array([ex, ey]) - a
        t = np.clip(np.sum(ap * ab, axis=1) / ab_len_sq, 0.0, 1.0)
        closest = a + ab * t[:, None]
        dists = np.hypot(closest[:, 0] - ex, closest[:, 1] - ey)
        return -self.reference_path_weight * float(np.min(dists))

    def world_to_cell(self, x, y):
        col = int(math.floor((x - self.origin_x + self.half_extent) / self.resolution))
        row = int(math.floor((y - self.origin_y + self.half_extent) / self.resolution))
        return col, row

    def cell_to_world(self, col, row):
        x = (col + 0.5) * self.resolution - self.half_extent + self.origin_x
        y = (row + 0.5) * self.resolution - self.half_extent + self.origin_y
        return x, y

    def in_bounds(self, col, row):
        return 0 <= col < self.n_cells and 0 <= row < self.n_cells

    def _is_occupied(self, row, col):
        return self.log_odds[row, col] >= self.l_occ_thresh

    def _should_process_scan(self, last_time_attr):
        """See max_scan_process_rate_hz's own comment - True (process
        this one) unless the rate limit is enabled and we're still within
        _min_scan_interval_sec of the last one we actually processed on
        this same stream. Explicitly drops the message rather than
        deferring it - a scan processed late, against a pose that's no
        longer current, actively hurts more than one dropped outright."""
        if self._min_scan_interval_sec <= 0.0:
            return True
        last = getattr(self, last_time_attr)
        now = self.get_clock().now()
        if last is not None:
            elapsed = (now - last).nanoseconds / 1e9
            if elapsed < self._min_scan_interval_sec:
                self._scans_dropped += 1
                if self._scans_dropped % 100 == 0:
                    self.get_logger().info(
                        f'Dropped {self._scans_dropped} scans so far to stay within '
                        f'max_scan_process_rate_hz - normal at a higher incoming rate '
                        f'than this is configured to process.', throttle_duration_sec=5.0)
                return False
        setattr(self, last_time_attr, now)
        return True

    def non_ground_callback(self, msg: PointCloud2):
        if not self._should_process_scan('_last_non_ground_process_time'):
            return
        t0 = time.time()
        decoded = self._decode_scan(msg)
        if decoded is None:
            return
        pts_world, origin_col, origin_row = decoded
        self._mark_rays(pts_world, origin_col, origin_row, hit=True)
        self._clear_near_field(origin_col, origin_row)
        self._log_progress()
        self._log_rate('non_ground_scan', t0, f' | {len(pts_world)} pts')

    def ground_callback(self, msg: PointCloud2):
        if self.avoid_ground:
            return
        if not self._should_process_scan('_last_ground_process_time'):
            return
        t0 = time.time()
        decoded = self._decode_scan(msg)
        if decoded is None:
            return
        pts_world, origin_col, origin_row = decoded
        self._mark_rays(pts_world, origin_col, origin_row, hit=False)
        self._log_progress()
        self._log_rate('ground_scan', t0, f' | {len(pts_world)} pts')

    def _log_rate(self, stream, t0, extra=''):
        """Same avg-ms/Hz pattern as segment_ground.py's own _perf() - one
        shared accumulator per named stream, logged every 10 calls, so real
        throughput numbers exist once real hardware's actual scan rate is
        what's driving this instead of sim's RTF-throttled one."""
        dt = time.time() - t0
        s = self._perf_stats.setdefault(stream, {'tt': 0.0, 'cnt': 0})
        s['tt'] += dt
        s['cnt'] += 1
        if s['cnt'] % 10 == 0:
            avg = s['tt'] / s['cnt']
            hz = 1.0 / avg if avg > 0 else 0.0
            self.get_logger().info(
                f'{stream}: {dt * 1000:.1f}ms (avg {avg * 1000:.1f}ms {hz:.1f}Hz){extra}')

    def _decode_scan(self, msg: PointCloud2):
        """Common decode/filter/downsample/transform pipeline. Returns
        (pts_world, origin_col, origin_row) or None if the scan can't be used."""
        n = msg.width * msg.height
        if n == 0:
            return None

        try:
            # Latest available transform, not msg.header.stamp - this sim's
            # clock/executor timing has been unreliable all day, and a
            # slightly-stale transform is far less harmful here than an
            # extrapolation exception dropping the whole scan.
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, msg.header.frame_id, Time(),
                timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed, skipping scan: {e}',
                                    throttle_duration_sec=5.0)
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        rot = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        origin = np.array([t.x, t.y, t.z])

        # Fast structured-dtype decode (same pattern as segment_ground/obstacle_detector)
        step = msg.point_step
        off = {}
        for f in msg.fields:
            if f.name in ('x', 'y', 'z'):
                off[f.name] = f.offset
        if len(off) < 3:
            return None
        dt = np.dtype({'names': ['x', 'y', 'z'],
                       'formats': ['<f4', '<f4', '<f4'],
                       'offsets': [off['x'], off['y'], off['z']],
                       'itemsize': step})
        raw = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
        pts = np.column_stack([raw['x'], raw['y'], raw['z']]).astype(np.float64)
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if len(pts) == 0:
            return None

        # Range-limit before transforming/downsampling - far returns are the
        # least reliable and the most expensive to ray-trace.
        rng = np.linalg.norm(pts, axis=1)
        pts = pts[rng <= self.max_ray_length]
        if len(pts) == 0:
            return None

        # Voxel downsample (same 3D-hash pattern used elsewhere in this codebase)
        vox_idx = np.floor(pts / self.voxel_size).astype(np.int64)
        keys = vox_idx[:, 0] * 4_000_000 + vox_idx[:, 1] * 2_000 + vox_idx[:, 2]
        _, uniq_idx = np.unique(keys, return_index=True)
        pts = pts[uniq_idx]

        self._maybe_recenter(origin[0], origin[1])

        pts_world = pts @ rot.T + origin
        origin_col, origin_row = self.world_to_cell(origin[0], origin[1])
        return pts_world, origin_col, origin_row

    def _maybe_recenter(self, sensor_x, sensor_y):
        """Shift the grid window to stay centered on the vehicle, once it
        gets within recenter_trigger_ratio of an edge - instead of a hard
        boundary the vehicle can drive past."""
        dx = sensor_x - self.origin_x
        dy = sensor_y - self.origin_y
        trigger = self.half_extent * self.recenter_trigger_ratio
        if abs(dx) < trigger and abs(dy) < trigger:
            return
        shift_cols = int(round(dx / self.resolution))
        shift_rows = int(round(dy / self.resolution))
        if shift_cols == 0 and shift_rows == 0:
            return
        self._shift_grid(shift_cols, shift_rows)
        self.get_logger().info(
            f'Freespace grid recentered: shifted ({shift_cols},{shift_rows}) cells, '
            f'new window center=({self.origin_x:.1f},{self.origin_y:.1f})m.')

    def _shift_grid(self, shift_cols, shift_rows):
        """Shift grid contents so old cell (r,c) becomes new cell
        (r-shift_rows, c-shift_cols) - equivalent to moving the window's
        world origin by (shift_cols*resolution, shift_rows*resolution).
        Cells that fall outside the new window are discarded (this is a
        local, bounded-memory grid, not an unbounded map); newly-exposed
        cells start fully UNKNOWN (0.0)."""
        new_grid = np.zeros_like(self.log_odds)
        n = self.n_cells
        r_old0, r_old1 = max(0, shift_rows), min(n, n + shift_rows)
        c_old0, c_old1 = max(0, shift_cols), min(n, n + shift_cols)
        r_new0 = max(0, -shift_rows)
        c_new0 = max(0, -shift_cols)
        h = r_old1 - r_old0
        w = c_old1 - c_old0
        if h > 0 and w > 0:
            new_grid[r_new0:r_new0 + h, c_new0:c_new0 + w] = \
                self.log_odds[r_old0:r_old0 + h, c_old0:c_old0 + w]
        self.log_odds = new_grid
        self.origin_x += shift_cols * self.resolution
        self.origin_y += shift_rows * self.resolution

    def _mark_rays(self, pts_world, origin_col, origin_row, hit):
        """Ray-trace from (origin_col,origin_row) to every point's cell,
        nudging swept cells toward FREE (l_miss) and the endpoint cell
        toward OCCUPIED (l_hit, non-ground points) or FREE (l_miss, ground
        points - the hit itself is evidence of drivable ground). Log-odds
        accumulation means repeated agreeing observations reinforce a cell
        and a single new disagreeing one can't instantly flip it - no
        special-casing needed to keep obstacle detection authoritative over
        ground-point rays or vice versa, unlike a plain overwrite scheme."""
        endpoints = []
        for p in pts_world:
            col, row = self.world_to_cell(p[0], p[1])
            if not self.in_bounds(col, row):
                continue
            endpoints.append((col, row))
            for c, r in bresenham_line(origin_col, origin_row, col, row):
                if (c, r) == (col, row) or not self.in_bounds(c, r):
                    continue
                self.log_odds[r, c] = max(self.l_min, self.log_odds[r, c] + self.l_miss)
        increment = self.l_hit if hit else self.l_miss
        for col, row in endpoints:
            self.log_odds[row, col] = min(self.l_max, max(
                self.l_min, self.log_odds[row, col] + increment))

    def _log_progress(self):
        self.scans_processed += 1
        if self.scans_processed % 40 == 0:
            occ_mask = self.log_odds >= self.l_occ_thresh
            free_mask = self.log_odds <= self.l_free_thresh
            occ = int(np.count_nonzero(occ_mask))
            free = int(np.count_nonzero(free_mask))
            self.get_logger().info(
                f'Freespace grid: {self.scans_processed} scans processed, '
                f'{occ + free} cells known ({occ} occupied, {free} free).')

    def _clear_near_field(self, center_col, center_row):
        """Give still-fully-unknown cells within near_field_clear_radius of the
        sensor a weak FREE prior (p_near_field_assumed, not the full p_miss
        confidence a real observation would get) - stands in for the LiDAR's
        unavoidable blind zone right around itself. Only touches cells at
        exactly 0.0 (never yet observed either way); a single real
        observation afterward, in either direction, fully overrides it.
        """
        rad = self.near_field_clear_radius_cells
        if rad <= 0:
            return
        c0, c1 = max(0, center_col - rad), min(self.n_cells, center_col + rad + 1)
        r0, r1 = max(0, center_row - rad), min(self.n_cells, center_row + rad + 1)
        if c0 >= c1 or r0 >= r1:
            return
        cc, rr = np.meshgrid(np.arange(c0, c1), np.arange(r0, r1))
        within = (cc - center_col) ** 2 + (rr - center_row) ** 2 <= rad ** 2
        region = self.log_odds[r0:r1, c0:c1]
        region[within & (region == 0.0)] = self.l_near_field

    def _apply_decay(self):
        """Leaky-integrator forgetting - see decay_half_life_sec's own
        comment. Applied here (once per publish_grid call) rather than a
        separate timer so it's driven by actual elapsed wall/sim time
        (measured from the clock, not assumed from publish_rate), robust
        to timer jitter or publish_rate changing at runtime."""
        if self.decay_half_life_sec <= 0.0:
            return
        now = self.get_clock().now()
        dt = (now - self._last_decay_time).nanoseconds / 1e9
        self._last_decay_time = now
        if dt <= 0.0:
            return
        factor = 0.5 ** (dt / self.decay_half_life_sec)
        self.log_odds *= factor

    def publish_grid(self):
        t0 = time.time()
        self._apply_decay()
        occ_mask = self.log_odds >= self.l_occ_thresh
        free_mask = self.log_odds <= self.l_free_thresh

        # Safety-region overlay - a costmap-style inflation halo around
        # every occupied cell, at exactly the radius (self.clearance =
        # vehicle_half_width + vehicle_footprint_margin) _arc_is_clear's
        # own footprint check already enforces, so what's drawn is
        # exactly what's being enforced, not a separately-eyeballed
        # approximation. binary_dilation with a precomputed circular
        # kernel instead of a naive per-cell distance check (same
        # vectorization reasoning as the grid-marker/_lateral_clearance
        # fixes elsewhere in this file). Confined to currently-FREE cells
        # only - occupied cells stay solid red, not relabeled.
        inflated_mask = (
            binary_dilation(occ_mask, structure=self._safety_region_kernel) & free_mask
            if self.show_safety_region else None)

        rows, cols = np.nonzero(occ_mask | free_mask)
        if len(rows) == 0:
            return

        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        grid_marker = Marker()
        grid_marker.header.frame_id = self.fixed_frame
        grid_marker.header.stamp = stamp
        grid_marker.ns = 'freespace_grid'
        grid_marker.id = 0
        grid_marker.type = Marker.CUBE_LIST
        grid_marker.action = Marker.ADD
        grid_marker.pose.orientation.w = 1.0
        grid_marker.scale.x = self.resolution
        grid_marker.scale.y = self.resolution
        grid_marker.scale.z = 0.02  # flat, overlay-like rather than a raised block

        # Vectorized (numpy) instead of a plain Python loop over every known
        # cell doing cell_to_world()/threshold-branching/message-construction
        # per cell - confirmed live this loop was the actual bottleneck:
        # publish_grid was only achieving ~0.9Hz against a configured 10Hz
        # publish_rate once the grid had enough known cells accumulated
        # (tens of thousands after sustained exploration), which throttled
        # not just this marker's redraw rate but _plan_lane's own steering-
        # target recompute too, since both run in this same function.
        # Semi-transparent so this reads as a drivable/non-drivable overlay
        # on the ground, not solid opaque blocks - opacity scales with
        # confidence (how far past its display threshold a cell's log-odds
        # sits), so a freshly-crossed-the-line cell looks faint and a
        # long-confirmed one looks solid.
        xs = (cols.astype(np.float64) + 0.5) * self.resolution - self.half_extent + self.origin_x
        ys = (rows.astype(np.float64) + 0.5) * self.resolution - self.half_extent + self.origin_y
        lv = self.log_odds[rows, cols].astype(np.float64)
        occ = lv >= self.l_occ_thresh
        occ_conf = np.clip((lv - self.l_occ_thresh) / max(1e-3, self.l_max - self.l_occ_thresh), 0.0, 1.0)
        free_conf = np.clip((self.l_free_thresh - lv) / max(1e-3, self.l_free_thresh - self.l_min), 0.0, 1.0)
        if inflated_mask is not None:
            inflated = inflated_mask[rows, cols]
        else:
            inflated = np.zeros_like(occ)
        # Occupied: red. Inflated-but-free (the safety region): orange,
        # fixed alpha (safety_region_alpha) rather than confidence-scaled -
        # it's a hard geometric buffer around a real obstacle, not itself
        # a belief that strengthens with more observations. Plain free:
        # green, same confidence-scaled opacity as before.
        r_ch = np.where(occ, 0.9, np.where(inflated, 1.0, 0.0))
        g_ch = np.where(occ, 0.0, np.where(inflated, 0.55, 0.8))
        a_ch = np.where(
            occ, 0.25 + 0.5 * occ_conf,
            np.where(inflated, self.safety_region_alpha, 0.15 + 0.4 * free_conf))

        # visualization_msgs/Marker.points and .colors are lists of real
        # message objects - rclpy has no bulk/vectorized way to fill these,
        # so this final list-comprehension step is still O(N) in Python,
        # just now doing plain object construction from already-computed
        # values instead of per-element arithmetic/branching/function calls.
        grid_marker.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in zip(xs, ys)]
        grid_marker.colors = [
            ColorRGBA(r=float(r), g=float(g), b=0.0, a=float(a))
            for r, g, a in zip(r_ch, g_ch, a_ch)
        ]
        markers.markers.append(grid_marker)

        pose = self._get_vehicle_pose()
        if pose is not None:
            pos, forward = pose
            markers.markers.append(self._build_heading_arrow(stamp, pos, forward))
            markers.markers.append(self._build_footprint_marker(stamp, pos, forward))

            # Same (pos, forward) TF read used for _plan_lane below - see
            # enable_pose_graph_bias's own comment for why this must run
            # BEFORE _plan_lane (so _direction_bias_bonus/_reference_path_
            # bonus see this cycle's values, not last cycle's) and why it's
            # merged in here instead of a separately-timed node.
            if self.enable_pose_graph_bias:
                yaw = math.atan2(forward[1], forward[0])
                self._tick_pose_graph_turn_direction(pos[0], pos[1], yaw, stamp)

            if self._recovery_state is not None:
                self._tick_recovery()
            else:
                lane_marker = self._plan_lane(stamp, pos, forward)
                if lane_marker is not None:
                    markers.markers.append(lane_marker)
                    # Published every cycle (not just on a stuck->OK
                    # transition) so /nav_status is a reliable heartbeat -
                    # downstream consumers (freespace_uart_bridge,
                    # steering_calculator_node's navigation-enabled gate)
                    # need to tell "actively fine" apart from "nothing's
                    # publishing at all" via staleness, which a
                    # transition-only OK can't support.
                    self.nav_status_pub.publish(String(data='OK'))
                    self._stuck_count = 0
                    self._recovery_attempt = 0
                else:
                    self._stuck_count += 1
                    if self._stuck_count >= self.stuck_cycles_threshold:
                        self._start_recovery()

        self.marker_pub.publish(markers)
        self._log_rate('publish_grid', t0, f' | {len(rows)} cells')

    def _start_recovery(self):
        """No clear arc for stuck_cycles_threshold cycles in a row - back
        up (straight on the first try, alternating +/- recovery_steer_deg_
        magnitude on later ones) so the vehicle isn't stuck reasoning about
        the exact same too-close geometry every cycle, then settle
        (stopped) for recovery_settle_sec before letting _plan_lane try
        again from the new vantage point. See recovery_max_attempts' own
        comment for what happens if this genuinely can't recover."""
        self._recovery_attempt += 1
        if self._recovery_attempt > self.recovery_max_attempts:
            self.get_logger().error(
                f'Recovery exhausted recovery_max_attempts ({self.recovery_max_attempts}) - '
                'stopping automatic recovery. The local grid has nowhere clear to go from '
                'here: drive it clear manually, or relaunch the mapping/localization stack '
                'for this area and resume once the pose graph covers it.')
            self.nav_status_pub.publish(String(data='STUCK'))
            self._recovery_attempt = 0
            self._stuck_count = 0
            self._recovery_state = None
            self._publish_recovery(False, 0.0, 0.0)
            return

        if self._recovery_attempt == 1:
            self._recovery_steer_deg = 0.0
        else:
            sign = 1.0 if self._recovery_attempt % 2 == 0 else -1.0
            self._recovery_steer_deg = sign * self.recovery_steer_deg_magnitude

        self.get_logger().warn(
            f'No clear arc for {self._stuck_count} cycles - recovery attempt '
            f'{self._recovery_attempt}/{self.recovery_max_attempts}: reversing at '
            f'{self.recovery_reverse_speed_kmh}km/h, steer={self._recovery_steer_deg:.0f}deg, '
            f'for {self.recovery_duration_sec}s.')
        self._recovery_state = 'REVERSING'
        self._recovery_state_until = self.get_clock().now() + Duration(seconds=self.recovery_duration_sec)
        self.nav_status_pub.publish(String(data='RECOVERING'))
        self._publish_recovery(True, self.recovery_reverse_speed_kmh, self._recovery_steer_deg)

    def _tick_recovery(self):
        now = self.get_clock().now()
        if now < self._recovery_state_until:
            # Republished every cycle (not just once in _start_recovery) -
            # same heartbeat reasoning as the OK case above.
            self.nav_status_pub.publish(String(data='RECOVERING'))
            self._publish_recovery(True, self.recovery_reverse_speed_kmh, self._recovery_steer_deg)
            return
        if self._recovery_state == 'REVERSING':
            # Reverse leg done - hold still so the grid can accumulate a
            # few fresh observations from this new vantage point (often
            # enough by itself to un-block a candidate that was only
            # rejected for insufficient clearance right at the old spot)
            # before _plan_lane is tried again.
            self._recovery_state = 'SETTLING'
            self._recovery_state_until = now + Duration(seconds=self.recovery_settle_sec)
            self._publish_recovery(True, 0.0, 0.0)
            return
        # SETTLING finished - hand control back; the next cycle's normal
        # _plan_lane call decides whether it's actually clear now.
        self._recovery_state = None
        self._recovery_state_until = None
        self._stuck_count = 0
        self._publish_recovery(False, 0.0, 0.0)

    def _publish_recovery(self, active, speed_kmh, steering_deg):
        self.recovery_active_pub.publish(Bool(data=active))
        self.recovery_speed_pub.publish(Float32(data=float(speed_kmh)))
        self.recovery_steering_pub.publish(Float32(data=float(steering_deg)))

    def _get_vehicle_pose(self):
        """Current (position xyz, forward-direction unit vector) of base_frame in fixed_frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.base_frame, Time(), timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'Vehicle pose: TF lookup failed: {e}',
                                    throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        rot = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        forward = rot @ np.array([1.0, 0.0, 0.0])  # local +X axis, rotated into fixed_frame
        return np.array([t.x, t.y, t.z]), forward

    def _build_heading_arrow(self, stamp, pos, forward):
        """ARROW marker at the vehicle's current pose, pointing in its facing direction."""
        arrow = Marker()
        arrow.header.frame_id = self.fixed_frame
        arrow.header.stamp = stamp
        arrow.ns = 'vehicle_heading'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.points = [
            Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]) + 0.3),
            Point(x=float(pos[0] + forward[0] * self.heading_arrow_length),
                  y=float(pos[1] + forward[1] * self.heading_arrow_length),
                  z=float(pos[2]) + 0.3),
        ]
        arrow.scale.x = 0.15  # shaft diameter
        arrow.scale.y = 0.3   # head diameter
        arrow.scale.z = 0.4   # head length
        arrow.color = ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.95)
        return arrow

    def _build_footprint_marker(self, stamp, pos, forward):
        """The vehicle's REAL physical footprint (not the clearance-
        inflated one _arc_is_clear checks against - vehicle_half_width +
        vehicle_footprint_margin - this is the plain half_width/
        half_length box itself), drawn as a flat rectangle at its live
        pose every cycle. Same dimensions as global_costmap_mapped.yaml's
        own footprint polygon (see vehicle_half_width's own comment) -
        lets you visually judge whether the yellow lane/green lines
        actually leave enough real room, not just whatever the inflated
        safety margin assumed."""
        yaw = math.atan2(forward[1], forward[0])
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = stamp
        marker.ns = 'vehicle_footprint'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = 0.05  # just above the flat grid overlay, avoids z-fighting
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x = 2.0 * self.vehicle_half_length
        marker.scale.y = 2.0 * self.vehicle_half_width
        marker.scale.z = 0.1
        marker.color = ColorRGBA(r=0.0, g=0.6, b=1.0, a=0.35)
        return marker

    def _lateral_clearance(self, x, y, theta, sign, max_search):
        """Distance from (x,y), perpendicular to theta (sign=+1/-1 for either
        side), until an OCCUPIED cell or the grid edge - capped at max_search.

        Vectorized (numpy) instead of a step-by-step Python loop -
        confirmed live this was the dominant remaining cost in
        publish_grid after the grid-marker loop was already fixed:
        _plan_lane calls this ~15 candidates x ~13 samples x 2 sides per
        cycle, each running up to corridor_scan_limit/resolution (~26)
        sequential Python iterations - roughly 10,000+ per-cell lookups a
        cycle. Verified exactly equivalent to the original sequential
        version via a 5000-trial fuzz test plus targeted edge cases
        (all-clear, immediately-blocked, out-of-bounds, max_search smaller
        than one resolution step) before this replaced it - same
        first-unsafe-cell semantics, not an approximation."""
        step = self.resolution
        n = int(max_search / step)
        if n <= 0:
            return max_search
        d = np.arange(1, n + 1, dtype=np.float64) * step
        sin_t, cos_t = math.sin(theta), math.cos(theta)
        ex = x - sign * d * sin_t
        ey = y + sign * d * cos_t
        col = np.floor((ex - self.origin_x + self.half_extent) / self.resolution).astype(np.int64)
        row = np.floor((ey - self.origin_y + self.half_extent) / self.resolution).astype(np.int64)
        out_of_bounds = (col < 0) | (col >= self.n_cells) | (row < 0) | (row >= self.n_cells)
        # Occupied lookup computed for every index (including out-of-bounds
        # ones, via clipping) purely to keep this branch-free/vectorized -
        # a clipped/garbage value at an out-of-bounds index never matters,
        # since `unsafe` below is already True there from out_of_bounds
        # alone (same short-circuit behavior the original's `or` had).
        safe_col = np.clip(col, 0, self.n_cells - 1)
        safe_row = np.clip(row, 0, self.n_cells - 1)
        occupied = self.log_odds[safe_row, safe_col] >= self.l_occ_thresh
        unsafe = out_of_bounds | occupied
        if not unsafe.any():
            return max_search
        first_unsafe = int(np.argmax(unsafe))  # first True - numpy argmax ties break to first occurrence
        return d[first_unsafe] - step

    def _arc_is_clear(self, pos, yaw, curvature):
        """Walk an arc of given curvature from (pos,yaw), checking the buggy's
        footprint clearance at each step, and how centered it stays between
        the left/right obstacles along the way. Returns (clear_distance,
        points, avg_centering_error) - lower centering_error is more centered."""
        step = self.resolution
        n_steps = int(self.lane_lookahead / step)
        points = []
        clear_distance = 0.0
        centering_error_sum = 0.0
        centering_samples = 0
        for i in range(1, n_steps + 1):
            s = i * step
            if abs(curvature) < 1e-4:
                lx, ly, ltheta = s, 0.0, 0.0
            else:
                r = 1.0 / curvature
                ltheta = s * curvature
                lx, ly = r * math.sin(ltheta), r * (1 - math.cos(ltheta))
            wx = pos[0] + lx * math.cos(yaw) - ly * math.sin(yaw)
            wy = pos[1] + lx * math.sin(yaw) + ly * math.cos(yaw)
            world_theta = yaw + ltheta

            # Sample the buggy's left/right edges (perpendicular to its
            # heading AT THIS POINT on the arc) in addition to the centerline,
            # so the check reflects the full footprint width, not just a
            # zero-width curve through the middle of the corridor.
            blocked = False
            # world_theta is constant across the 3 offset checks below -
            # hoisted out of what was a per-offset math.sin/cos call each
            # (same values every time, just avoids recomputing them 3x).
            sin_wt, cos_wt = math.sin(world_theta), math.cos(world_theta)
            for offset in (0.0, self.clearance, -self.clearance):
                ex = wx - offset * sin_wt
                ey = wy + offset * cos_wt
                col, row = self.world_to_cell(ex, ey)
                if not self.in_bounds(col, row) or self._is_occupied(row, col):
                    blocked = True
                    break
            if blocked:
                break
            clear_distance = s
            points.append((wx, wy))

            # How far off-center is this point? A path can be fully
            # obstacle-clear (the check above) while still hugging one wall -
            # sample every few steps (perpendicular scans aren't free) to
            # keep this cheap.
            if i % 3 == 0:
                left = self._lateral_clearance(wx, wy, world_theta, 1, self.corridor_scan_limit)
                right = self._lateral_clearance(wx, wy, world_theta, -1, self.corridor_scan_limit)
                centering_error_sum += abs(left - right)
                centering_samples += 1

        avg_centering_error = (centering_error_sum / centering_samples
                                if centering_samples else 0.0)
        return clear_distance, points, avg_centering_error

    def _plan_lane(self, stamp, pos, forward):
        """Pick a clear arc (bounded by the buggy's min turn radius), biased
        toward staying centered in the corridor when a well-centered option
        is nearly as good as the single longest one, smoothed over time so
        it doesn't jump between updates - and return it as a LINE_STRIP
        marker, or None if nothing usable. Also publishes /target_lateral_position
        for cart_perception's existing steering_calculator_node (pure-pursuit
        steering + PID against real wheel feedback, and its own speed
        reduction for sharp turns) to consume - the lane isn't just drawn,
        it drives the actual steering/speed control."""
        yaw = math.atan2(forward[1], forward[0])
        curvatures = np.linspace(-self.max_curvature, self.max_curvature, self.lane_num_candidates)

        results = [(k, *self._arc_is_clear(pos, yaw, k)) for k in curvatures]
        max_distance = max(dist for _, dist, _, _ in results)
        if max_distance < self.resolution:
            self.smoothed_curvature = 0.0
            return None

        # Don't always chase the single longest arc - among everything
        # within lane_straight_preference_ratio of the best distance found,
        # take the best-centered (most equal left/right clearance) rather
        # than whichever happened to reach a few cm further.
        threshold = max_distance * self.lane_straight_preference_ratio
        good_enough = [r for r in results if r[1] >= threshold]
        # Direction bias + reference-path bias (see _direction_bias_bonus/
        # _reference_path_bonus) are both subtracted from the primary
        # (centering_error) sort key, not added as a separate criterion -
        # they only re-rank candidates the safety filter above already
        # judged comparably clear: direction bias leans toward the right
        # general side near a turn, reference-path bias additionally pulls
        # toward the recorded route's actual shape every cycle. Zero bonus
        # from both (no map/localization running) reproduces the original
        # (r[3], abs(r[0])) ordering exactly.
        def score(r):
            k, dist, points, centering_error = r
            ex, ey = points[-1] if points else (pos[0], pos[1])
            bias = self._direction_bias_bonus(k) + self._reference_path_bonus(ex, ey)
            return (centering_error - bias, abs(k))

        chosen_k, chosen_dist, chosen_points, _ = min(good_enough, key=score)

        # Smooth the chosen curvature toward the previous frame's value so
        # the visualized lane eases between arcs instead of snapping.
        alpha = self.lane_smoothing_alpha
        smoothed_k = alpha * chosen_k + (1 - alpha) * self.smoothed_curvature
        final_dist, final_points, _ = self._arc_is_clear(pos, yaw, smoothed_k)
        if final_dist < self.resolution:
            # The smoothed curvature isn't actually clear right now (e.g. a
            # new obstacle just appeared where the old heading was taking
            # us) - snap to the unsmoothed choice instead of stalling.
            smoothed_k, final_dist, final_points = chosen_k, chosen_dist, chosen_points
        self.smoothed_curvature = smoothed_k

        if not final_points:
            return None

        self._publish_target_lateral_position(smoothed_k)

        lane = Marker()
        lane.header.frame_id = self.fixed_frame
        lane.header.stamp = stamp
        lane.ns = 'suggested_lane'
        lane.id = 0
        lane.type = Marker.LINE_STRIP
        lane.action = Marker.ADD
        lane.pose.orientation.w = 1.0
        lane.scale.x = 0.25
        lane.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.95)
        lane.points = [Point(x=float(pos[0]), y=float(pos[1]), z=0.15)]
        lane.points += [Point(x=float(x), y=float(y), z=0.15) for x, y in final_points]
        return lane

    def _publish_target_lateral_position(self, curvature):
        """The chosen arc's local lateral offset at steering_lookahead_distance -
        exactly what front_view_occupancy_map already publishes on this same
        topic in the reactive pipeline, so steering_calculator_node needs no
        changes to consume a lane chosen from this from-scratch grid instead."""
        s = self.steering_lookahead_distance
        if abs(curvature) < 1e-4:
            ly = 0.0
        else:
            r = 1.0 / curvature
            ly = r * (1 - math.cos(s * curvature))
        msg = Float32()
        msg.data = float(ly)
        self.target_lateral_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FreespaceGridNode()
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
