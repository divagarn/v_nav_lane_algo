#!/usr/bin/env python3
"""Local path planner front-end.

Earlier version of this node just synthesized a naive straight line ahead
of the vehicle and handed it to the local controller (MPPI/DWB) as the
FollowPath reference, trusting the controller to deviate from it around
obstacles. In practice the controller's path-following critics/costs could
still pull it into a wall that the naive line ran straight through - there
was no actual path *planning* happening, just reactive trajectory scoring.

This version asks planner_server (nav2_navfn_planner, over the rolling
local costmap built from /non_ground_points_filtered) to compute a real,
obstacle-avoiding path from the vehicle's current pose out to a goal point
~carrot_distance meters ahead, then hands that computed path to the local
controller instead. Re-requested on a timer so the goal continuously
tracks ahead of the vehicle's latest pose.

The goal direction is NOT simply "current heading" - that degenerates
exactly at real corners, where the corridor bends away from wherever the
vehicle currently happens to be pointed, and NavFn can only ever try to
reach a point straight ahead: it can bend the path a little to dodge the
nearest wall, but there's nothing pulling it to actually commit to the
turn, so the vehicle just nudges along the wall instead of turning.
_pick_goal_heading scans a spread of candidate headings against
/global_costmap/costmap (the same costmap NavfnPlanner itself plans over)
and measures how far each stays clear of lethal/inflated/unknown cells,
then targets whichever direction is genuinely open - biased toward
current heading so it doesn't zigzag when several directions are equally
clear.

The scan distance (scan_distance) is deliberately larger than the actual
placed goal distance (carrot_distance): with a car-like Ackermann vehicle
that has a real minimum turning radius (4.4m here) and can't rotate in
place or reverse out of a mistake, waiting until the wall is only
carrot_distance away to notice the corner leaves no room left to complete
the arc - DWB ends up maxed out on steering with almost no forward speed,
unable to finish the turn, and just freezes. Scanning further out lets it
notice the corner and start bending toward it while there's still enough
room to sweep a physically achievable turn.

Goal heading smoothing (added when the global planner became
SmacPlannerHybrid): NavfnPlanner is a Dijkstra search over position only -
it completely ignores the goal PoseStamped's orientation, so a picked
heading that flipped a few degrees cycle to cycle (raycast noise near
_pick_goal_heading's switching margin, not just real corners) had no
effect on the planned path shape. SmacPlannerHybrid is a real kinematic
planner where the goal orientation is a hard endpoint constraint the curve
must terminate facing - the same per-cycle heading noise that was
invisible before now reshapes the entire planned curve every ~1s
(measured live: end-of-path heading oscillating by 5-15deg practically
every cycle, even mid-corridor with nothing nearby).

_smoothed_yaw damps that small, noise-scale wobble but deliberately does
NOT lag on a large, genuine heading change (see GOAL_HEADING_SNAP_DEG): a
first pass here used a plain low-pass filter on every change, which
technically fixed the noise but then took 5-8 cycles (5-8s) to converge on
a real ~90deg corner - during which the vehicle kept driving forward at
up to 2.2m/s, eating right into the extra reaction room scan_distance was
supposed to buy it. Confirmed live: the vehicle drove itself into an
inside corner with no >=4.4m-radius Dubin path left to reach a large,
obviously-open area it should have caught much earlier, and
SmacPlannerHybrid returned "no valid path found" for minutes straight
(not a bug - genuinely no forward-only path existed from that position).
Large, clearly-real corrections now snap through in one cycle; only
sub-threshold wobble gets damped.

Centerline tracking (straight sections only): the costmap-inflation-
gradient approach to centering (local_costmap.yaml/planner_costmap.yaml's
inflation_radius tuned to roughly half the corridor width, see those
files' comments) is an indirect approximation - it depends on a specific
corridor width and breaks if width varies along the route. _centerline_lateral_bias()
instead directly clusters /non_ground_points_filtered into a left-wall and
a right-wall per forward-distance slice and biases the goal toward their
literal midpoint - a more precise center for confirmed straight corridor
segments than an inflation-gradient guess. Deliberately NOT used at
corners/openings/T-junctions, where one side's wall may be missing,
farther away, or absent entirely: the confidence check below (enough
consecutive both-sides bins, roughly consistent width) is what SmacPlannerHybrid's
turning-radius search and carrot_path_publisher's gap-scan already handle
correctly (see this whole module's history above) - this feature only
ever nudges the goal sideways when it's confident it's looking at a plain
two-wall corridor, and falls back to zero bias (i.e. today's behavior)
the instant that confidence check fails, rather than trying to be clever
about partial/ambiguous wall data.

Goal-seeking (/goal_pose, e.g. RViz's "2D Goal Pose"): everything above is
purely reactive - it has no notion of a destination, just "stay clear of
obstacles, keep going". Confirmed live (tools/carrot_heading_tuner.py, an
L/T-shaped test corridor) that this made every carrot goal converge on the
same left-biased route regardless of where the actual destination was, and
that it would happily drive straight through/past a goal that didn't
happen to also be wherever looked clearest. With a goal set,
_pick_goal_heading centers its candidate scan on the bearing to that goal
instead of current heading (widened to +-165deg, since reaching a goal
that's behind a corridor bend can require a heading nowhere near the
direct bearing until the corridor opens up) and caps goal_distance at the
remaining distance to the goal so it stops there rather than overshooting.
request_plan itself stops replanning entirely once within goal_xy_tolerance
of the goal, holding until a new /goal_pose arrives.

use_global_planner=False with a goal set goes one step further:
_split_into_clear_segments builds a whole multi-waypoint polyline to the
goal in a single request_plan() cycle (by calling _pick_goal_heading
repeatedly, waypoint_step_m at a time, internally - not one carrot per
real replan tick), so TEB gets a path that already bends around a corner
instead of a single short segment it'd otherwise only see one piece of
per cycle.

use_lane_following=True (independent of use_global_planner - checked
first in request_plan, wins over both other modes whenever a goal is
set) is a third, structurally different technique: _build_lane_path
computes the medial axis (skeleton) of the local costmap's free space -
the set of points equidistant from the nearest walls on every side, i.e.
the true geometric centerline of every corridor/junction/turn in the
map - and searches along it from the vehicle's position to the goal,
instead of picking headings step by step. Confirmed live
(tools/carrot_heading_tuner.py) that _split_into_clear_segments/
_pick_goal_heading's centering_weight has a real ceiling (oscillates
above ~2.5 on a test corridor) and still only weakly centers in wide-open
areas; the skeleton is exactly centered everywhere, independent of
corridor width, with no oscillation risk since it's a real graph search,
not a greedy per-step heuristic. The raw skeleton path is pixel-jagged
(a discretized 1-cell-wide ridge), so it's simplified via Douglas-Peucker
(bounded perpendicular deviation from the true skeleton shape, NOT "cut
straight to whichever point is still technically clear" - that greedy
alternative was tried first and throws away the whole centering benefit,
producing the exact same corner-hugging diagonal _split_into_clear_
segments already had a problem with, confirmed by rendering both side by
side) before being handed to _build_straight_path the same as the other
two modes' waypoint lists.

auto_return_to_start (buggy_mapping's closed-loop navigation): drive the
whole mapped loop once and stop back where you started, from a single
"2D Goal Pose" click, rather than needing a second manual goal for the
return leg or looping forever. start_pose_xy is recorded once, from the
node's very first odometry message. When the first goal is reached,
request_plan auto-chains exactly one more goal - start_pose_xy - instead
of holding; reaching THAT one (detected by the just-reached goal already
being start_pose_xy) holds for real. Works with all three techniques
above, since it only ever changes what latest_goal_xy is, the same input
each of them already reads.
"""

import math

import networkx as nx
import numpy as np
import rclpy
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Pose
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float32
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import ComputePathToPose, FollowPath
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

LETHAL_THRESHOLD = 90  # nav2_costmap_2d: 253 lethal / 254 inscribed / 0-252 inflation decay

# base_link -> velodyne static mount offset (velodyne_static_tf.py), no
# rotation currently (roll=pitch=yaw=0), so sensor-frame -> base_link-frame
# is just this translation. Same values as self_hit_filter.py, duplicated
# rather than imported since that file is a standalone node script, not a
# shared library, and it's only two numbers.
SENSOR_OFFSET_X = 1.73
SENSOR_OFFSET_Y = 0.0

# Waypoint spacing for _build_straight_path (use_global_planner=False mode).
STRAIGHT_PATH_STEP_M = 0.5


class CarrotPathPublisher(Node):

    def __init__(self):
        super().__init__('carrot_path_publisher')

        self.carrot_distance = self.declare_parameter('carrot_distance', 5.0).value
        self.scan_distance = self.declare_parameter('scan_distance', 9.0).value
        self.resend_period_sec = self.declare_parameter('resend_period_sec', 1.0).value
        self.controller_id = self.declare_parameter('controller_id', 'FollowPath').value
        self.planner_id = self.declare_parameter('planner_id', 'GridBased').value
        # Cosmetic only (startup log line below) - this node has no way to
        # introspect which plugin planner_server actually loaded for
        # GridBased, so the launch file is responsible for passing the
        # matching name (SmacPlannerHybrid, ThetaStarPlanner, ...). Defaults
        # to SmacPlannerHybrid so existing launch files that don't set this
        # keep logging what they always have.
        self.global_planner_name = self.declare_parameter(
            'global_planner_name', 'SmacPlannerHybrid').value
        # Action name the ComputePathToPose client below actually connects
        # to. Defaults to the plain name planner_server itself always uses,
        # so existing Theta*/Smac launches (which run through planner_server
        # directly) need no changes. Override this when the real planner is
        # a standalone node instead (e.g. rrt_star_planner_node.py) - a
        # plain launch-time `remappings=` on either this node or the other
        # one does NOT rename an rclpy ActionClient/ActionServer (confirmed
        # live: `ros2 node info` kept showing the unremapped name on both
        # ends), so the actual target has to be set here as a real parameter
        # instead.
        self.planner_action_name = self.declare_parameter(
            'planner_action_name', 'compute_path_to_pose').value
        # False skips planner_server (SmacPlannerHybrid) entirely: instead of
        # ComputePathToPose, a plain interpolated straight line from the
        # current pose to the same obstacle-aware goal _pick_goal_heading
        # already picks (see that method - it's still costmap-driven, not a
        # blind forward line) is handed straight to TEB via FollowPath. TEB's
        # own local-costmap obstacle avoidance is what keeps it off walls in
        # this mode, with no kinematic global search upstream of it. See
        # _build_straight_path.
        self.use_global_planner = self.declare_parameter('use_global_planner', True).value
        # Low-pass filter weight applied to the picked goal heading each
        # cycle (see module docstring's "Goal heading smoothing" note) -
        # 1.0 disables smoothing entirely (old NavfnPlanner-era behavior).
        self.goal_heading_alpha = self.declare_parameter('goal_heading_alpha', 0.4).value
        # Above this, a heading change is treated as a real corner (snap
        # immediately) rather than frame-to-frame raycast noise (damp via
        # goal_heading_alpha) - comfortably above the ~5-15deg noise band
        # measured live mid-corridor, comfortably below a genuine turn.
        self.goal_heading_snap_deg = self.declare_parameter('goal_heading_snap_deg', 25.0).value
        # How much farther a candidate heading must stay clear than the
        # current best before it's allowed to win (see _pick_goal_heading).
        # 0.3m was tuned back when NavfnPlanner ignored goal orientation
        # entirely, so a spurious switch only nudged the *reference* path a
        # little and MPPI/DWB/TEB just tracked it - harmless. It's too
        # small now: normal raycast/costmap noise on a wide-open straight
        # (e.g. the 7m corridor in rect_loop_track.world) still exceeds
        # 0.3m often enough to swap between +-15deg candidates almost every
        # cycle, and since the goal orientation is now a hard endpoint
        # constraint for SmacPlannerHybrid, that swap visibly S-curves the
        # vehicle down what should be a straight line. Real corners show a
        # multi-meter clearance gap between continuing straight and turning
        # (measured live: often 5m+), so raising this doesn't blunt corner
        # detection at all.
        self.heading_switch_margin_m = self.declare_parameter('heading_switch_margin_m', 1.5).value
        # Goal-seeking mode only (see _pick_goal_heading docstring):
        # "closest to the goal" alone has no preference for staying away
        # from walls, so around a bend it'll hug the inside edge as tight
        # as clear_distance allows - confirmed live with
        # tools/carrot_heading_tuner.py by drawing an actual curved
        # corridor and watching the picked path graze the inner wall the
        # whole way around instead of favoring the middle. 0 disables
        # centering entirely. 2.0 (rather than an equal-weight 1.0) is
        # deliberate: measured on a wide-then-narrow test corridor in that
        # same tool, 2.0 roughly halved the path's peak offset from the
        # true centerline (4.36m -> 2.06m) with no regression on a tight
        # 90deg corridor test. Don't casually raise this further - above
        # ~2.5 in the same test the vehicle got torn between "avoid the
        # left wall" and "avoid the right wall" candidates scoring nearly
        # equally and oscillated in place rather than committing to
        # either; this is a real ceiling on the greedy single-step scoring
        # here, not just an untried "more is better" knob. corridor_check_
        # distance caps how far sideways is worth checking - reward stops
        # accruing past this much clearance on the tighter side, so a
        # candidate in wide-open space doesn't keep scoring better forever
        # just for being in the middle of nowhere.
        self.centering_weight = self.declare_parameter('centering_weight', 2.0).value
        self.corridor_check_distance = self.declare_parameter('corridor_check_distance', 3.0).value
        # _clear_distance returns the distance to the FIRST unsafe
        # (lethal/inflated/unknown) cell along a ray - using that value
        # directly as the goal distance places the goal right at that
        # cell's boundary, zero margin, confirmed live via /carrot_marker
        # (goal sphere sitting visibly inside the inflated costmap region
        # at a corridor corner). Subtracted from every clear-distance-based
        # placement (candidate_advance) and used as the minimum acceptable
        # sideways clearance at a candidate's endpoint (candidate_score's
        # centering term, goal-seeking mode) - keeps placed goals solidly
        # inside the confirmed-safe zone instead of grazing its edge. 1.0m
        # default: comfortably more than _clear_distance's own 0.2m step
        # granularity, less than the vehicle's ~1.45m half-width so it
        # doesn't shrink valid corridors more than necessary.
        self.goal_safety_margin = self.declare_parameter('goal_safety_margin', 1.0).value
        # Soft bias toward slam_toolbox's recorded pose graph (the actual
        # trajectory driven during mapping) - see pose_graph_callback's own
        # comment for the data source. Same weighted-bonus pattern as
        # centering_weight, applied in BOTH reactive and goal-seeking mode
        # (unlike centering, which was already goal-seeking-capable before
        # the reactive-mode fix - this one was designed for both from the
        # start). Exists specifically to compensate for costmap_topic
        # defaulting to the local costmap (no static map layer, so nothing
        # else here knows "the map says a corridor continues past current
        # sensor range" - staying near the pose graph is what keeps the
        # vehicle on ground it's already confirmed driveable instead of
        # wandering into open-but-never-driven map area).
        self.pose_graph_weight = self.declare_parameter('pose_graph_weight', 1.5).value
        self.pose_graph_max_influence = self.declare_parameter('pose_graph_max_influence', 5.0).value
        # Switch between the EXISTING pose_graph_weight bias above (nearest
        # vertex ANYWHERE in the whole recorded graph - no direction
        # awareness at all, can pull toward a nearby parallel corridor or a
        # loop-closure segment behind/beside the vehicle) and a directional
        # alternative: bias toward the green /route_preview_path line
        # specifically (the direction-disambiguated walk ahead, same one
        # _publish_route_preview draws - see _pose_graph_lookahead_path).
        # false (default): unchanged existing behavior. true: candidate_score
        # uses reference_path_weight/_point_to_polyline_dist against that
        # walked path instead - applies to BOTH candidate_score branches
        # (reactive/no-goal and goal-seeking), since graph_term is shared by
        # both, same as pose_graph_weight always was.
        self.use_route_preview_reference = self.declare_parameter(
            'use_route_preview_reference', False).value
        self.reference_path_weight = self.declare_parameter('reference_path_weight', 1.5).value
        # --- Arc-based lane planner (only active when
        # use_route_preview_reference:=true AND no manual /goal_pose is
        # set - see _plan_lane_arc). This is the SAME algorithm as
        # cart_navigation_ws's freespace_grid_node._plan_lane/_arc_is_clear
        # (curvature-candidate arc evaluation, not a single straight
        # heading+distance like _pick_goal_heading above), ported here so
        # candidate_score's straight-ray picker - which cannot represent a
        # smoothly curving path at all, only re-picks a new straight ray
        # every cycle - can be swapped for the arc-following one on
        # request. Reads THIS node's own costmap (self.latest_costmap)
        # instead of freespace_grid_node's custom log-odds grid, and its
        # direction-bias/reference-path-bias inputs come directly from
        # this node's own already-computed reference_yaw/route-preview
        # walk (in-process) rather than subscribing to /turn_direction,
        # /turn_distance_m, /route_preview_path over topics the way
        # freespace_grid_node (a separate node) had to.
        self.lane_wheelbase = self.declare_parameter('lane_wheelbase', 1.6).value
        self.lane_max_steering_deg = self.declare_parameter('lane_max_steering_deg', 20.0).value
        self.lane_lookahead = self.declare_parameter('lane_lookahead', 12.0).value
        self.lane_num_candidates = int(self.declare_parameter('lane_num_candidates', 15).value)
        self.corridor_scan_limit = self.declare_parameter('corridor_scan_limit', 8.0).value
        self.lane_straight_preference_ratio = self.declare_parameter(
            'lane_straight_preference_ratio', 0.85).value
        # ABSOLUTE (not relative to the best candidate found this cycle)
        # minimum clear distance a candidate arc must reach to be
        # considered at all - see _plan_lane_arc's own comment for why
        # this replaced lane_straight_preference_ratio as the actual
        # safety filter: confirmed live (repeatedly) that the ratio-based
        # filter excluded the CORRECT direction outright whenever it was
        # a tighter/shorter-clearance curving arc than an unrelated wide-
        # open straight opening elsewhere - no bias weight could ever pull
        # toward a candidate that was filtered out before scoring even
        # ran. An absolute floor only asks "is this arc safe enough to
        # drive", not "is this arc as long as the single longest one
        # found", so a correct-but-shorter curving arc stays eligible.
        self.lane_min_safety_distance = self.declare_parameter(
            'lane_min_safety_distance', 2.0).value
        # 5 DISCRETE, individually-visualized candidates (forward, near/far
        # left, near/far right) instead of a silent lane_num_candidates-way
        # continuous sweep - lets you actually SEE all 5 options against
        # the green line in RViz (/lane_candidates) instead of only ever
        # seeing the single winner, to make it possible to tell whether a
        # bad pick is a scoring problem or a "the right option was never
        # generated" problem. Fractions are of lane_max_curvature.
        self.lane_candidate_inner_fraction = self.declare_parameter(
            'lane_candidate_inner_fraction', 0.4).value
        self.lane_candidate_outer_fraction = self.declare_parameter(
            'lane_candidate_outer_fraction', 1.0).value
        self.lane_candidates_pub = self.create_publisher(MarkerArray, '/lane_candidates', 1)
        self.lane_smoothing_alpha = self.declare_parameter('lane_smoothing_alpha', 0.3).value
        self.lane_vehicle_half_width = self.declare_parameter('lane_vehicle_half_width', 1.0).value
        self.lane_vehicle_footprint_margin = self.declare_parameter(
            'lane_vehicle_footprint_margin', 0.3).value
        self.direction_bias_gain = self.declare_parameter('direction_bias_gain', 1.5).value
        self.smoothed_curvature = 0.0
        # Ackermann min turning radius: R = wheelbase / tan(max_steer) - same
        # formula freespace_grid_node uses for its own max_curvature.
        lane_steer_rad = math.radians(max(self.lane_max_steering_deg, 0.5))
        self.lane_min_turn_radius = self.lane_wheelbase / math.tan(lane_steer_rad)
        self.lane_max_curvature = 1.0 / self.lane_min_turn_radius
        self.lane_clearance = self.lane_vehicle_half_width + self.lane_vehicle_footprint_margin
        self._last_golden_lane_points = None
        # use_global_planner=False + a real /goal_pose: how far each of
        # _split_into_clear_segments' internal steps advances (independent
        # of carrot_distance - this is building a whole path in one
        # request_plan() cycle, not one per-cycle carrot), and how many
        # such steps it's allowed to take before giving up on a still-
        # distant goal. 1.5m keeps individual steps small enough to
        # reliably corner (see that method's docstring for why large,
        # greedy steps fail); 50 steps at 1.5m covers 75m of travel, far
        # more than one scan_distance/carrot_distance-scale corridor
        # segment would ever need - cheap to compute (all raycasts stay
        # within latest_costmap's own rolling window), so a generous
        # budget costs nothing but a few extra iterations once actually
        # near the goal.
        self.waypoint_step_m = self.declare_parameter('waypoint_step_m', 1.5).value
        self.max_waypoint_steps = self.declare_parameter('max_waypoint_steps', 50).value
        # Independent of use_global_planner - see module docstring's
        # "use_lane_following" paragraph for why this is a structurally
        # different technique (medial-axis skeleton search) rather than
        # another tuning of the greedy step-by-step approach. Wins over
        # both other modes in request_plan whenever True and a goal is set.
        self.use_lane_following = self.declare_parameter('use_lane_following', False).value
        # Douglas-Peucker tolerance (max perpendicular deviation, meters)
        # for simplifying the raw pixel-jagged skeleton path down to a few
        # waypoints - see _build_lane_path's docstring for why this must
        # be deviation-bounded simplification, not greedy "shortcut to
        # whichever point is still clear".
        self.lane_simplify_epsilon = self.declare_parameter('lane_simplify_epsilon', 0.3).value
        # False (default): reactive/no-destination mode (use_global_planner
        # False, no /goal_pose set) picks exactly one carrot per cycle, same
        # as it always has. True: build a short multi-waypoint chain instead
        # (_build_reactive_chain), same waypoint_step_m-spaced repeated-
        # _pick_goal_heading technique _split_into_clear_segments already
        # uses for a real destination, just with no (gx,gy) to stop at -
        # instead it keeps stepping until the cumulative chain length
        # reaches scan_distance (the same budget already used for turn-
        # scanning), capped by max_waypoint_steps as a safety bound. Lets
        # /sub_goals + the bent /plan this produces be inspected in RViz
        # even with no manual goal set, instead of only ever seeing the
        # single reactive carrot point.
        self.use_reactive_chain = self.declare_parameter('use_reactive_chain', False).value
        self._smoothed_yaw = None
        odom_topic = self.declare_parameter(
            'odom_topic', '/ackermann_steering_controller/odometry').value

        # Centerline tracking params - see module docstring's "Centerline
        # tracking" note. Distances in base_link-frame (forward=x, left=y).
        self.centerline_enabled = self.declare_parameter('centerline_enabled', True).value
        self.centerline_bin_size = self.declare_parameter('centerline_bin_size', 1.0).value
        # Below this many both-sides bins, there isn't enough confirmed
        # corridor ahead to trust a centerline - falls back to zero bias.
        self.centerline_min_bins = self.declare_parameter('centerline_min_bins', 4).value
        # A "wall" candidate must be within this lateral band to count -
        # too close (< side_min) risks catching the vehicle's own
        # near-field noise self_hit_filter didn't fully remove; too far
        # (> side_max) risks pairing with something that isn't actually
        # this corridor's wall (e.g. a distant wall past an opening).
        self.centerline_side_min = self.declare_parameter('centerline_side_min', 0.5).value
        self.centerline_side_max = self.declare_parameter('centerline_side_max', 5.0).value
        # Corridor width (left_wall - right_wall) must stay within this
        # much of itself across all valid bins - a real straight corridor
        # has near-constant width; a corner or opening does not, and this
        # is what actually keeps the feature out of corners (not a
        # separate "is this a corner" flag).
        self.centerline_width_tolerance = self.declare_parameter('centerline_width_tolerance', 1.0).value
        # Sanity cap on how far the goal can be nudged sideways in one
        # cycle, independent of the confidence check above.
        self.centerline_max_bias = self.declare_parameter('centerline_max_bias', 2.0).value
        self.latest_cloud_xy = None
        self.create_subscription(
            PointCloud2, '/non_ground_points_filtered', self.cloud_callback, 2)

        self.latest_odom = None
        # Recorded once, from the vehicle's very first odometry message -
        # see odom_callback and auto_return_to_start below.
        self.start_pose_xy = None
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 1)

        # Optional fixed destination (RViz "2D Goal Pose" publishes here by
        # convention). Without one, _pick_goal_heading falls back to its
        # original purely-reactive behavior (scan around current heading,
        # go wherever's clearest, no notion of "arriving" anywhere) - see
        # that method's docstring for why a goal changes the scan entirely.
        self.latest_goal_xy = None
        self.goal_xy_tolerance = self.declare_parameter('goal_xy_tolerance', 0.5).value
        # For closed-loop map-based navigation (buggy_mapping): drive one
        # full lap and stop back where you started, rather than needing a
        # second manual goal click for the return leg or looping forever.
        # When the FIRST /goal_pose goal is reached, automatically chains
        # exactly one more goal - start_pose_xy - instead of holding; once
        # THAT one is reached (i.e. we're back at the recorded start), it
        # holds for real. See request_plan's "goal reached" branch.
        self.auto_return_to_start = self.declare_parameter('auto_return_to_start', False).value
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 1)

        # /global_costmap/costmap is planner_server's own internal costmap -
        # when use_global_planner is False, planner_server isn't launched at
        # all (see mppi/real/bag_navigate.launch.py's IfCondition on
        # planner_server), so that topic never publishes and latest_costmap
        # stays None forever. _pick_goal_heading's very first check
        # (latest_costmap is None -> return current_yaw unchanged) then
        # fires every single cycle - not "scanning and finding nothing
        # open", just never scanning at all, which is why the straight-line
        # fallback previously drove dead ahead through obstacles instead of
        # turning. controller_server's own local_costmap runs unconditionally
        # regardless of use_global_planner, so that's the correct source to
        # fall back to here.
        default_costmap_topic = (
            '/global_costmap/costmap' if self.use_global_planner else '/local_costmap/costmap')
        costmap_topic = self.declare_parameter('costmap_topic', default_costmap_topic).value
        self.latest_costmap = None
        # (height, width) numpy view of the same data as latest_costmap.data -
        # kept alongside it (not a replacement) so _lateral_clearance_arc can
        # do vectorized numpy lookups (same approach as freespace_grid_node's
        # own _lateral_clearance) instead of a per-cell Python loop, while
        # _clear_distance/_is_path_safe etc. keep reading the raw message
        # unchanged.
        self._costmap_grid = None
        costmap_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            OccupancyGrid, costmap_topic, self.costmap_callback, costmap_qos)

        # See costmap_callback's own comment for what this actually does -
        # cancels + immediately replans (not just at the next 1Hz cycle)
        # the moment a fresh costmap shows the executing path crossing an
        # inflated/lethal cell. Most useful paired with costmap_topic
        # pointed at the local costmap, whose obstacle layer updates as
        # fast as the sensor pipeline runs (15Hz), not just at replan time.
        self.path_safety_check = self.declare_parameter('path_safety_check', True).value
        self._current_path = None
        self._current_follow_goal_handle = None
        # See costmap_callback's own comment - caps how often the
        # immediate cancel-and-replan-on-unsafe-path trigger can actually
        # fire, so a borderline path can't cause a runaway cancel-replan
        # loop that never lets the controller produce a stable command.
        self._last_safety_replan_time = None
        self.path_safety_replan_cooldown_sec = float(
            self.declare_parameter('path_safety_replan_cooldown_sec', 0.5).value)

        # slam_toolbox's pose graph - see pose_graph_callback's own comment
        # for the data source and why a TF lookup is needed here (published
        # in the 'map' frame, everything else in this node works in 'odom').
        self._pose_graph_vertices_map = None
        # See pose_graph_callback's own comment - frozen once the vertex
        # count has been stable for pose_graph_stabilize_count consecutive
        # callbacks, not on the first message (fragile - see that
        # method's "FIX #2" comment).
        self._original_vertex_count = None
        self._pose_graph_last_len = None
        self._pose_graph_stable_count = 0
        self.pose_graph_stabilize_count = int(
            self.declare_parameter('pose_graph_stabilize_count', 2).value)
        # Last confidently-computed reference_yaw (the pose-graph bearing
        # used for /turn_direction, the arc planner's direction bias, and
        # the straight-ray picker's fallback) + when it was set. A
        # transient TF hiccup or a cycle where every pose-graph vertex
        # happens to get filtered as non-finite (see _pose_graph_vertices_
        # in_odom's own NaN-filtering comment) would otherwise make
        # reference_yaw drop straight to "no bias" for that one cycle -
        # this caches the last good value so a momentary glitch doesn't
        # show up as the golden lane/turn-direction marker flickering back
        # toward raw reactive behavior and snapping back.
        #
        # reference_yaw_cache_timeout_sec defaults effectively unbounded
        # (~11 days) - the last known-good pose-graph bearing is kept and
        # used until a genuinely fresh one replaces it, not discarded just
        # because time passed, per confirmed live behavior: expiring it
        # after a few seconds meant a longer localization hiccup dropped
        # straight to "no bias" instead of holding the last trusted
        # direction. Set a smaller value here if "no bias after N seconds
        # of no data" is actually wanted in a given deployment.
        self._last_valid_reference_yaw = None
        self._last_valid_reference_yaw_time = None
        self.declare_parameter('reference_yaw_cache_timeout_sec', 999999.0)
        self.reference_yaw_cache_timeout_sec = float(
            self.get_parameter('reference_yaw_cache_timeout_sec').value)

        # Same problem, one level up: reference_yaw above caches a single
        # bearing, but the green line ITSELF (the full walked polyline -
        # what /route_preview_path actually draws, and what _plan_lane_arc
        # scores candidates against) had no caching at all - confirmed
        # live this could still show the green line snapping its START
        # point back toward the beginning of the recorded route
        # ("0th index") on a bad cycle, even with the NaN-vertex/vertex-
        # freeze fixes already in place, because a plausible-looking
        # (non-NaN) but WRONG nearest-vertex pick is a different failure
        # mode those fixes don't catch. _route_preview_caches holds one
        # {'path','time'} entry per named walk (see _robust_pose_graph_
        # path) - 'long' for the main green line/reference_path_weight
        # bias, 'mid' for the new second, shorter reference line below.
        # Same effectively-unbounded default as reference_yaw_cache_
        # timeout_sec above, same reason: "store the last known pose of
        # the nearest pose graph point so we can use that until the next
        # known one is identified" - the real safeguard against trusting a
        # BAD read is route_preview_max_jump_m (rejects implausible
        # jumps), not a clock. A time-based expiry on top of that would
        # only throw away a still-good cached path for no reason other
        # than elapsed time.
        self._route_preview_caches = {}
        self.declare_parameter('route_preview_cache_timeout_sec', 999999.0)
        self.route_preview_cache_timeout_sec = float(
            self.get_parameter('route_preview_cache_timeout_sec').value)
        # A real vehicle cannot jump this many meters between two
        # consecutive ~resend_period_sec cycles - a fresh walk whose START
        # point (nearest vertex found this cycle) moved further than this
        # from the last ACCEPTED start point is rejected as a bad
        # nearest-vertex pick, not a real relocalization, and the last
        # accepted path is reused instead (see _robust_pose_graph_path).
        self.declare_parameter('route_preview_max_jump_m', 3.0)
        self.route_preview_max_jump_m = float(
            self.get_parameter('route_preview_max_jump_m').value)
        # Second, shorter reference line - "2-3 lines at different
        # lengths" so a single long walk jumping onto the wrong branch at
        # a junction isn't the only signal steering toward. turn_lookahead_m
        # (9m default) already gives a SHORT bearing-only signal for
        # direction_bias_gain; route_preview_distance_m (40m) gives the
        # LONG full-shape line; this fills the middle distance with its
        # own full polyline (not just a bearing), so _plan_lane_arc can
        # weight agreement across three scales instead of two.
        self.declare_parameter('route_preview_distance_mid_m', 20.0)
        self.route_preview_distance_mid_m = float(
            self.get_parameter('route_preview_distance_mid_m').value)
        self.declare_parameter('reference_path_mid_weight', 1.0)
        self.reference_path_mid_weight = float(
            self.get_parameter('reference_path_mid_weight').value)
        self.route_preview_mid_pub = self.create_publisher(Marker, '/route_preview_path_mid', 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        pose_graph_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            MarkerArray, '/slam_toolbox/graph_visualization', self.pose_graph_callback,
            pose_graph_qos)

        self._planner_client = ActionClient(self, ComputePathToPose, self.planner_action_name)
        self._controller_client = ActionClient(self, FollowPath, 'follow_path')

        # Same computed Path also published to a plain topic for RViz -
        # the action goals themselves aren't visible there (Fixed Frame:
        # odom to see it).
        self.plan_pub = self.create_publisher(Path, '/plan', 1)
        # Visualizes what _centerline_lateral_bias() actually detected each
        # cycle (base_link frame) - green when confident enough to bias the
        # goal, red when it bailed out (not enough both-sides bins, or
        # width too inconsistent - see that method's docstring). Lets you
        # watch the real detected wall midpoints against the track's own
        # lane stripe/geometry instead of just trusting the math.
        self.centerline_marker_pub = self.create_publisher(Marker, '/centerline_marker', 1)
        # The chosen /goal_pose itself (a plain sphere) - republished every
        # request_plan cycle (not just once in goal_pose_callback) because
        # this publisher, like plan_pub/centerline_marker_pub above, has no
        # transient-local durability: a late-joining RViz would otherwise
        # never see a goal that was set before it connected.
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 1)
        # The immediate sub-goal actually handed to the planner THIS cycle -
        # distinct from /goal_marker above, which only ever shows the final
        # manually-clicked destination (and only exists at all once one is
        # set). This publishes every request_plan cycle regardless of
        # whether a manual goal exists: with no goal set, it's the reactive
        # carrot _pick_goal_heading picked on its own (no destination, just
        # "stay clear and keep going" - see that method's docstring); with a
        # goal set, it's the goal-seeking candidate _pick_goal_heading chose
        # toward it (usually much closer than the real goal, capped by
        # carrot_distance/clearance) - i.e. exactly what compute_path_to_pose
        # was actually asked to reach, cycle by cycle.
        self.carrot_marker_pub = self.create_publisher(Marker, '/carrot_marker', 1)
        # The "golden yellow lane" - a plain LINE_STRIP from the vehicle's
        # current position straight to /carrot_marker's endpoint, i.e. the
        # actual reactive/goal-seeking direction candidate_score picked
        # THIS cycle. Distinct from /plan (whatever use_global_planner's
        # RRT*/straight-line technique produced FROM that same endpoint,
        # which can look like a winding search result rather than a
        # simple picked direction) and from /route_preview_path (the
        # green line - the pose graph's own recorded shape, not what the
        # planner is actually about to drive). See _publish_golden_lane.
        self.golden_lane_pub = self.create_publisher(Marker, '/golden_lane_marker', 1)
        # use_lane_following's specific path (see _build_lane_path) - drawn
        # separately from the generic /plan Path (which every mode
        # publishes identically) so you can actually tell, in RViz, that a
        # given route came from the skeleton search and not the greedy
        # step-by-step techniques.
        self.lane_marker_pub = self.create_publisher(Marker, '/lane_marker', 1)
        # The actual subgoal waypoints the straight-segment/lane-following
        # techniques split the main /goal_pose into, as plain data (not a
        # Marker) - "ros2 topic echo /sub_goals" or an RViz PoseArray
        # display shows exactly which intermediate points the controller is
        # being aimed at, one at a time, instead of just the dense
        # interpolated /plan. Not published for SmacPlannerHybrid: that
        # technique hands back one dense path from the action server, it
        # doesn't produce independently-planned subgoals the way the other
        # two do.
        self.sub_goals_pub = self.create_publisher(PoseArray, '/sub_goals', 1)
        # Which way the upcoming turn goes (LEFT/RIGHT/STRAIGHT), derived
        # from the same goal_yaw _pick_goal_heading already computed by
        # scanning scan_distance ahead for corners - not a separate
        # lookahead. Plain String for anything that just wants the label
        # (dashboard, logging), Marker (floating text above the vehicle)
        # for RViz. See _publish_turn_direction.
        self.turn_direction_pub = self.create_publisher(String, '/turn_direction', 1)
        self.turn_marker_pub = self.create_publisher(Marker, '/turn_direction_marker', 1)
        # How far ahead along slam_toolbox's recorded pose graph
        # _pose_graph_lookahead_point walks before reporting the upcoming
        # turn - see that method's own docstring.
        self.turn_lookahead_m = self.declare_parameter('turn_lookahead_m', 9.0).value
        # Long-range green preview line along the pose graph - see
        # _publish_route_preview. Separate publisher/param from
        # turn_direction_marker above: this is a much longer look-ahead
        # (route_preview_distance_m, default matches this project's LiDAR
        # max-range convention) meant to show the overall shape of the
        # upcoming route, not just flag the next single turn.
        self.route_preview_pub = self.create_publisher(Marker, '/route_preview_path', 1)
        self.route_preview_distance_m = self.declare_parameter('route_preview_distance_m', 40.0).value
        # How many consecutive vertices _pose_graph_lookahead_path averages
        # over (each direction) to pick the walk direction - see that
        # method's own docstring for the single-neighbor-noise bug this
        # fixes.
        self.direction_disambiguation_vertices = self.declare_parameter(
            'direction_disambiguation_vertices', 5).value
        # Debug/diagnostic visualization of which pose-graph vertex
        # nearest-vertex search actually picked this cycle - see
        # _publish_nearest_vertex_marker.
        self.nearest_vertex_marker_pub = self.create_publisher(
            Marker, '/nearest_pose_graph_vertex', 1)
        self.nearest_vertex_distance_pub = self.create_publisher(
            Float32, '/pose_graph_nearest_vertex_distance', 1)
        self.nearest_vertex_max_dist = self.declare_parameter('nearest_vertex_max_dist', 5.0).value

        self._planning_in_progress = False

        self.timer = self.create_timer(self.resend_period_sec, self.request_plan)

        planner_desc = (
            f'via planner_server/{self.global_planner_name}' if self.use_global_planner
            else 'via a straight line to the picked goal (global planner disabled)'
        )
        if self.use_lane_following:
            planner_desc += ' (use_lane_following: wins over this whenever a goal is set)'
        if self.use_reactive_chain:
            planner_desc += ' (use_reactive_chain: multi-waypoint chain even with no goal set)'
        self.get_logger().info(
            f'Local path planner initialized (goal {self.carrot_distance}m ahead, '
            f'scanning {self.scan_distance}m ahead for turns, replans every '
            f'{self.resend_period_sec}s {planner_desc}, '
            f'goal_heading_alpha={self.goal_heading_alpha}, '
            f'goal_safety_margin={self.goal_safety_margin}m, '
            f'pose_graph_weight={self.pose_graph_weight}, '
            f'path_safety_check={self.path_safety_check}, '
            f'costmap_topic={costmap_topic}, '
            f'centerline_enabled={self.centerline_enabled}, odom from {odom_topic})'
        )

    def odom_callback(self, msg):
        if self.start_pose_xy is None:
            # Recorded once, from the very first odometry message - "where
            # it was start" for auto_return_to_start, not wherever the
            # vehicle happens to be when the first /goal_pose arrives.
            self.start_pose_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.get_logger().info(
                f'Start pose recorded: ({self.start_pose_xy[0]:.2f}, {self.start_pose_xy[1]:.2f})')
        self.latest_odom = msg

    def goal_pose_callback(self, msg):
        self.latest_goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self._smoothed_yaw = None  # don't drag the old (possibly reactive) heading into a fresh goal
        self.get_logger().info(
            f'New goal received: ({self.latest_goal_xy[0]:.2f}, {self.latest_goal_xy[1]:.2f})')

    def costmap_callback(self, msg):
        self.latest_costmap = msg
        self._costmap_grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        if not self.path_safety_check or self._current_path is None:
            return
        if self._is_path_safe(self._current_path, msg):
            return
        # FIX (confirmed live): this check runs on every costmap update -
        # update_frequency in local_costmap.yaml, 15Hz - much faster than
        # request_plan's own resend_period_sec (2Hz default). A path that
        # sits close to an obstacle boundary (the arc planner's
        # lane_min_safety_distance is a much tighter margin than an
        # open-space RRT* path typically has) can trip this on nearly
        # every costmap tick, and each trip used to cancel + replan
        # IMMEDIATELY and unconditionally - if the FRESH replan also ends
        # up borderline (likely, for the same underlying reason the first
        # one was), this became a tight cancel-replan loop firing up to
        # 15x/second: a brand new FollowPath goal sent before the
        # controller ever had a stable window to produce even one
        # cmd_vel, which looked exactly like the vehicle having simply
        # stopped even though the system was technically still "working".
        # path_safety_replan_cooldown_sec caps how often this immediate-
        # replan path can actually fire (default matches resend_period_
        # sec's own 0.5s cadence) - a genuinely new obstacle is still
        # caught, just not re-triggered faster than the controller can
        # realistically respond to.
        now = self.get_clock().now()
        if (self._last_safety_replan_time is not None and
                (now - self._last_safety_replan_time).nanoseconds / 1e9
                < self.path_safety_replan_cooldown_sec):
            return
        self._last_safety_replan_time = now
        # Fresh costmap shows the path we're currently executing crosses an
        # inflated/lethal cell (new obstacle, or the local costmap's
        # rolling window caught up with something the picker's raycasts
        # missed last cycle) - don't wait for the next 1Hz request_plan
        # tick, cancel now and replan immediately. self._current_path is
        # cleared unconditionally here (not just on successful cancel) so
        # a second costmap message arriving before the cancel/replan
        # completes doesn't re-trigger on the same now-stale path.
        self.get_logger().warn(
            'Executing path now crosses an unsafe cell - cancelling and replanning immediately.',
            throttle_duration_sec=1.0)
        self._current_path = None
        if self._current_follow_goal_handle is not None:
            self._current_follow_goal_handle.cancel_goal_async()
            self._current_follow_goal_handle = None
        self.request_plan()

    def _is_path_safe(self, path, costmap):
        """True if every pose in path sits on a free/inflated-but-passable
        cell of costmap - same LETHAL_THRESHOLD convention as _clear_distance,
        just checking an already-computed path instead of raycasting a new
        candidate. Off-the-known-window points count as unsafe (matches
        _clear_distance's own "ran off the window -> treat as the limit"
        stance, just inverted to a hard fail here since this is a safety
        check, not a clearance measurement)."""
        info = costmap.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = costmap.data
        for pose_stamped in path.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            gx = int((x - ox) / res)
            gy = int((y - oy) / res)
            if gx < 0 or gx >= w or gy < 0 or gy >= h:
                return False
            val = data[gy * w + gx]
            if val < 0 or val >= LETHAL_THRESHOLD:
                return False
        return True

    def pose_graph_callback(self, msg):
        """slam_toolbox's own graph_visualization MarkerArray - confirmed
        live (no dedicated "get pose graph" service exists in this
        slam_toolbox version, only SerializePoseGraph, which writes to a
        FILE, not a queryable structure) the only live-accessible source
        of pose-graph vertex positions. Each vertex is a separate
        Marker(ns='slam_toolbox', type=SPHERE), position in .pose.position
        - NOT a single POINTS/SPHERE_LIST marker with a .points[] array as
        might be assumed. Published in frame_id 'map' (confirmed live),
        NOT 'odom' like everything else in this node - stored here as-is;
        _pose_graph_vertices_in_odom() does the actual map->odom transform,
        once per request_plan cycle rather than once per marker here.

        FIX (confirmed live, same underlying slam_toolbox behavior as
        cart_navigation_ws): this topic keeps growing while driving -
        slam_toolbox appends the vehicle's OWN live trajectory as new
        vertices onto the same graph, not just the originally-recorded
        mapping route. Nearest-vertex search was therefore sometimes
        picking the robot's own most recent trail vertex (literally
        wherever it currently is) instead of the actual mapped route,
        which corrupts the walk-direction pick in
        _pose_graph_lookahead_path (the green/turn-direction line pointing
        backward - the vehicle's trail immediately behind it looks
        "nearest" too).

        FIX #2 (confirmed live: the green line kept pointing back toward
        the start of the route even AFTER fix #1 above, the NaN filter,
        and the jump-rejection cache - none of those explained it, because
        this is a DIFFERENT bug): freezing _original_vertex_count on the
        VERY FIRST non-empty callback is fragile - if THAT first message
        happens to arrive before slam_toolbox has finished publishing the
        complete loaded pose graph (a real possibility right after
        LOCALIZE_AT_POSE/startup), the freeze locks in a tiny vertex count
        PERMANENTLY, for the rest of the run - not a transient bad read
        anything downstream can reject or fall back from, a structurally
        truncated graph from the very beginning. Now: don't freeze on
        message #1. Track the vertex count across consecutive callbacks
        and only freeze once it's been UNCHANGED for
        _pose_graph_stabilize_count callbacks in a row (default 2) - i.e.
        once the initial full-graph publish has visibly settled and
        growth has slowed to the live-trail-appending phase this fix
        exists to exclude. Until stabilized, the FULL current array is
        used (best available, not artificially truncated) rather than
        blocking on a frozen count that might still be wrong."""
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

    def _pose_graph_vertices_in_odom(self):
        """Transform the stored map-frame pose graph vertices into odom
        frame via a live TF lookup - returns None (bias gracefully
        disabled, same pattern as centerline_enabled/latest_cloud_xy
        elsewhere in this file) if no graph received yet or the map->odom
        transform isn't available yet (e.g. slam_toolbox still starting
        up). Done as one rigid 2D transform (translation + yaw rotation)
        applied to the whole vertex array at once, not a per-vertex TF
        call - map->odom is a single rigid transform at any given instant,
        not per-point.

        FIX (confirmed live): a small localization/map misalignment can
        make this TF lookup momentarily degenerate (e.g. an
        in-flight/settling transform during a correction), producing NaN
        in tx/ty/yaw and therefore in the whole transformed vertex array.
        _safe_nearest_index already guards the nearest-vertex SELECTION
        step against that, but the WALK in _pose_graph_lookahead_path
        (stepping vertex-to-vertex once a start index is picked) has no
        such guard - a NaN vertex a few hops in silently corrupts every
        segment-length/direction calculation after it, and plain
        np.argmin elsewhere would return the FIRST NaN's index outright
        (confirmed: index 0 if that's where the NaN lands), which reads
        as nearest-vertex search snapping back to the start of the route.
        Filtering non-finite rows out HERE, the single place every
        consumer (_pose_graph_lookahead_path, _publish_nearest_vertex_
        marker, pose_graph_bonus, route_preview_bonus) reads the odom-
        frame vertices from, closes the gap for all of them at once."""
        if self._pose_graph_vertices_map is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform('odom', 'map', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        yaw = self._yaw_from_quaternion(t.transform.rotation)
        vx = self._pose_graph_vertices_map[:, 0]
        vy = self._pose_graph_vertices_map[:, 1]
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        ox = tx + vx * cos_y - vy * sin_y
        oy = ty + vx * sin_y + vy * cos_y
        vertices = np.column_stack([ox, oy])
        finite_mask = np.isfinite(vertices).all(axis=1)
        if not finite_mask.all():
            self.get_logger().warn(
                f'Dropped {int((~finite_mask).sum())} non-finite pose-graph vertex/vertices '
                'after map->odom transform (localization/TF glitch) - excluded from nearest-'
                'vertex search and the walk.', throttle_duration_sec=5.0)
        return vertices[finite_mask]

    def cloud_callback(self, msg):
        if msg.width == 0:
            self.latest_cloud_xy = None
            return
        pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        # Sensor frame -> base_link frame (forward=x, left=y), same
        # translation-only conversion as self_hit_filter.py.
        xy = pts[:, :2].copy()
        xy[:, 0] += SENSOR_OFFSET_X
        xy[:, 1] += SENSOR_OFFSET_Y
        self.latest_cloud_xy = xy

    def _centerline_lateral_bias(self, max_forward):
        """Return a lateral offset (base_link-frame, +y=left) to nudge the
        goal toward the literal midpoint between confirmed left/right
        corridor walls, or None if not confident this is a plain straight
        corridor (see module docstring's "Centerline tracking" note)."""
        if not self.centerline_enabled or self.latest_cloud_xy is None:
            return None

        xy = self.latest_cloud_xy
        bin_edges = np.arange(1.0, max_forward, self.centerline_bin_size)
        bin_xs = []
        centers = []
        widths = []
        for x_lo in bin_edges:
            x_hi = x_lo + self.centerline_bin_size
            in_bin = xy[(xy[:, 0] >= x_lo) & (xy[:, 0] < x_hi)]
            if in_bin.shape[0] == 0:
                continue
            y = in_bin[:, 1]
            left = y[(y > self.centerline_side_min) & (y < self.centerline_side_max)]
            right = y[(y < -self.centerline_side_min) & (y > -self.centerline_side_max)]
            if left.size == 0 or right.size == 0:
                continue
            left_wall = left.min()   # nearest point on the left
            right_wall = right.max()  # nearest point on the right (least negative)
            bin_xs.append(x_lo + self.centerline_bin_size / 2.0)
            centers.append((left_wall + right_wall) / 2.0)
            widths.append(left_wall - right_wall)

        confident = (
            len(centers) >= self.centerline_min_bins
            and max(widths) - min(widths) <= self.centerline_width_tolerance
        ) if centers else False

        bias = None
        if confident:
            bias = float(np.mean(centers))
            bias = max(-self.centerline_max_bias, min(self.centerline_max_bias, bias))

        self._publish_centerline_marker(bin_xs, centers, confident)
        return bias

    def _publish_centerline_marker(self, bin_xs, centers, confident):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "centerline_detected"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        if confident:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 1.0
        else:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 0.6
        for x, y in zip(bin_xs, centers):
            p = Point()
            # float(): numpy.float32/float64 (from np.arange/array.min/max
            # above) fail geometry_msgs' field type assertion, which isn't
            # caught anywhere upstream - it just kills the node outright.
            p.x, p.y, p.z = float(x), float(y), 0.1
            marker.points.append(p)
        self.centerline_marker_pub.publish(marker)

    def _publish_goal_marker(self, header, x, y, publisher, ns="goal",
                              color=(1.0, 0.0, 1.0, 0.9)):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        publisher.publish(marker)

    def _publish_golden_lane(self, header, x0, y0, gx, gy, arc_points=None):
        """See golden_lane_pub's own comment. Two rendering modes:

        - arc_points given (use_route_preview_reference's arc planner
          picked this cycle, see _plan_lane_arc): draws the vehicle's
          position followed by every sampled point along the CHOSEN
          CURVED arc - a real bending line, same as freespace_grid_node's
          own yellow "suggested_lane" marker in cart_navigation_ws.
        - arc_points is None (the straight-ray _pick_goal_heading picked
          instead): the original plain two-point line from the vehicle to
          the candidate endpoint /carrot_marker already marks - this is
          not an approximation of a curve, it's the literal straight ray
          that heading+distance picker chose."""
        marker = Marker()
        marker.header = header
        marker.ns = "golden_lane"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.84, 0.0, 0.95
        pts = [(x0, y0)] + list(arc_points) if arc_points else [(x0, y0), (gx, gy)]
        for x, y in pts:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.05
            marker.points.append(p)
        self.golden_lane_pub.publish(marker)

    @staticmethod
    def _safe_nearest_index(d):
        """NaN-safe replacement for np.argmin - confirmed via direct
        testing that plain np.argmin, when the array contains any NaN,
        returns the index of the FIRST NaN it encounters (not necessarily
        a real nearest point), which silently corrupts nearest-vertex
        picking whenever a distance computation produces one (e.g. a
        malformed/degenerate marker). Masks non-finite entries out before
        arg-minning; returns None if nothing valid remains."""
        finite = np.isfinite(d)
        if not finite.any():
            return None
        return int(np.argmin(np.where(finite, d, np.inf)))

    @staticmethod
    def _point_to_polyline_dist(path_xy, ex, ey):
        """Perpendicular distance from (ex, ey) to the nearest SEGMENT of
        path_xy (an (N,2) array of ordered points, e.g. from
        _pose_graph_lookahead_path) - not just the nearest vertex. Used by
        the route-preview-referenced candidate scoring (see
        use_route_preview_reference) so a candidate between two widely-
        spaced pose-graph vertices still scores by its true distance to the
        drawn line, not to whichever endpoint happens to be closer."""
        if path_xy is None or len(path_xy) < 2:
            return None
        p = np.array([ex, ey])
        a = path_xy[:-1]
        b = path_xy[1:]
        ab = b - a
        ab_len2 = np.sum(ab * ab, axis=1)
        ab_len2 = np.where(ab_len2 < 1e-9, 1e-9, ab_len2)
        t = np.clip(np.sum((p - a) * ab, axis=1) / ab_len2, 0.0, 1.0)
        proj = a + t[:, None] * ab
        d = np.hypot(proj[:, 0] - ex, proj[:, 1] - ey)
        return float(d.min())

    def _pose_graph_lookahead_path(self, x0, y0, current_yaw, max_distance):
        """Walk along slam_toolbox's recorded pose graph (the actual path
        driven during mapping - same data source as pose_graph_weight's
        bias, see pose_graph_callback's own comment for why a TF lookup is
        needed) starting from whichever vertex is nearest (x0, y0), out to
        max_distance meters, and return every point visited along the way
        (odom frame) as a list, or None if no graph/TF is available yet
        (same graceful-disable pattern as pose_graph_weight elsewhere in
        this file). Shared by _pose_graph_lookahead_point (turn_lookahead_m,
        only the final point matters) and _publish_route_preview
        (route_preview_distance_m, the whole polyline is drawn) - same
        nearest-vertex + direction-disambiguation walk, just parameterized
        on how far to go and keeping every intermediate point instead of
        only the last one.

        Only the graph's vertex ARRAY ORDER is available (no edge list -
        see pose_graph_callback), assumed to match the order vertices were
        recorded in, i.e. consecutive array entries are consecutive points
        along whatever was driven during mapping. The vehicle can now be
        retracing that same path in either the recorded direction or its
        reverse, so the walk direction (increasing vs decreasing index
        from the nearest vertex) is picked by whichever side better aligns
        with current_yaw - averaged over direction_disambiguation_vertices
        consecutive vertices in each direction (confirmed live: a SINGLE
        immediate neighbor is noisy enough on its own - two vertices that
        happen to sit close together, or slightly out of recording order,
        can flip which side "wins" outright, which read as the green/
        turn-direction line pointing backward)."""
        vertices = self._pose_graph_vertices_in_odom()
        if vertices is None or len(vertices) < 2:
            return None

        d = np.hypot(vertices[:, 0] - x0, vertices[:, 1] - y0)
        idx0 = self._safe_nearest_index(d)
        if idx0 is None:
            return None
        n = len(vertices)

        def local_direction_yaw(i, step_dir):
            """Average bearing over up to direction_disambiguation_vertices
            consecutive steps from i in step_dir, instead of just the
            single immediate neighbor - see this method's own docstring."""
            j = i
            dx = dy = 0.0
            count = 0
            for _ in range(self.direction_disambiguation_vertices):
                nj = j + step_dir
                if nj < 0 or nj >= n:
                    break
                dx += vertices[nj][0] - vertices[j][0]
                dy += vertices[nj][1] - vertices[j][1]
                j = nj
                count += 1
            return math.atan2(dy, dx) if count > 0 else None

        def alignment(yaw):
            if yaw is None:
                return -2.0  # worse than any real cos() value - never picked over a real neighbor
            diff = math.atan2(math.sin(yaw - current_yaw), math.cos(yaw - current_yaw))
            return math.cos(diff)

        fwd_yaw = local_direction_yaw(idx0, 1)
        bwd_yaw = local_direction_yaw(idx0, -1)
        step = 1 if alignment(fwd_yaw) >= alignment(bwd_yaw) else -1

        x, y = vertices[idx0]
        path = [(x, y)]
        traveled = 0.0
        i = idx0
        while 0 <= i + step < n:
            nx, ny = vertices[i + step]
            seg = math.hypot(nx - x, ny - y)
            if traveled + seg >= max_distance:
                t = (max_distance - traveled) / seg if seg > 0 else 0.0
                path.append((x + t * (nx - x), y + t * (ny - y)))
                return path
            traveled += seg
            x, y = nx, ny
            path.append((x, y))
            i += step

        return path  # graph ran out before max_distance - return what there was

    def _costmap_cell_is_clear(self, x, y):
        """True if (x,y) (odom frame, matching the costmap's own frame)
        reads as a safe costmap cell - not lethal/inflated (>=
        LETHAL_THRESHOLD), not unknown (<0), and not off the costmap's
        current window. True whenever no costmap has arrived yet - this is
        a "don't rule the point out" default, not a safety claim; callers
        that need an actual safety guarantee already have _clear_distance/
        _arc_is_clear_costmap for that."""
        costmap = self.latest_costmap
        if costmap is None:
            return True
        info = costmap.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        gx = int((x - ox) / res)
        gy = int((y - oy) / res)
        if gx < 0 or gx >= w or gy < 0 or gy >= h:
            return False
        val = costmap.data[gy * w + gx]
        return not (val < 0 or val >= LETHAL_THRESHOLD)

    def _pose_graph_lookahead_point(self, x0, y0, current_yaw):
        """The point turn_lookahead_m ahead along the pose graph, used as
        the bearing target for turn-direction/direction-bias - see
        _pose_graph_lookahead_path for the actual walk.

        FIX: the recorded route can end up blocked at exactly that
        distance - a new/temporary obstacle sitting on the old path, or
        the route drifting slightly into a wall due to a small
        localization misalignment (see this file's other NaN-vertex
        fixes for the same underlying drift symptom). Biasing steering
        toward a point that's currently marked lethal/unknown on the live
        costmap would aim it at/through the very obstacle the costmap
        already knows about. Instead of taking that point outright, walk
        the SAME path further (up to route_preview_distance_m, reusing
        the one already-computed polyline rather than a second TF/walk)
        and use the first point from turn_lookahead_m onward that's
        actually costmap-clear - i.e. skip forward to the next usable
        point on the recorded route instead of aiming at a blocked one."""
        path = self._pose_graph_lookahead_path(x0, y0, current_yaw, self.route_preview_distance_m)
        if not path:
            return None
        cum = 0.0
        idx_at_target = len(path) - 1
        for i in range(1, len(path)):
            cum += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if cum >= self.turn_lookahead_m:
                idx_at_target = i
                break
        for x, y in path[idx_at_target:]:
            if self._costmap_cell_is_clear(x, y):
                return (x, y)
        return path[-1]  # nothing clear found out to route_preview_distance_m either - best available

    def _publish_turn_direction(self, header, position, current_yaw, reference_yaw):
        """LEFT/RIGHT/STRAIGHT text marker + plain String topic for the
        upcoming turn - reference_yaw is the bearing from the vehicle's
        current position to the point turn_lookahead_m ahead along
        slam_toolbox's recorded pose graph (see
        _pose_graph_lookahead_point), i.e. "which way does the actual
        mapped path bend over the next turn_lookahead_m meters", not the
        reactive planner's own picked heading. Positive yaw is CCW (left,
        since +y is left of forward here), so a positive
        current_yaw->reference_yaw diff means the upcoming turn is to the
        left. goal_heading_snap_deg (reused here, same threshold
        _pick_goal_heading uses to decide "real corner" vs "noise") is the
        cutoff for calling it a turn at all: below it, flashing LEFT/RIGHT
        would just be reacting to ordinary heading noise, not an actual
        upcoming corner."""
        diff_deg = math.degrees(math.atan2(
            math.sin(reference_yaw - current_yaw), math.cos(reference_yaw - current_yaw)))
        if abs(diff_deg) < self.goal_heading_snap_deg:
            direction, color = 'STRAIGHT', (1.0, 1.0, 1.0, 0.9)
        elif diff_deg > 0:
            direction, color = 'LEFT', (1.0, 0.65, 0.0, 0.95)
        else:
            direction, color = 'RIGHT', (1.0, 0.65, 0.0, 0.95)

        self.turn_direction_pub.publish(String(data=direction))

        marker = Marker()
        marker.header = header
        marker.ns = "turn_direction"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z + 1.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.6
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = direction
        self.turn_marker_pub.publish(marker)

    def _robust_pose_graph_path(self, x0, y0, current_yaw, max_distance, cache_name):
        """_pose_graph_lookahead_path, guarded against the failure mode
        that survived even the NaN/vertex-freeze fixes: a nearest-vertex
        pick that is perfectly finite (no NaN) but simply WRONG - snapping
        onto a distant, spatially-unrelated vertex (e.g. near the start of
        the recorded route) for one cycle due to a borderline TF/
        localization moment. A real vehicle cannot move more than
        route_preview_max_jump_m between two consecutive ~resend_period_sec
        cycles, so a fresh walk whose START point lands further than that
        from the last ACCEPTED start point is rejected outright and the
        last accepted path is reused instead (within
        route_preview_cache_timeout_sec) - "index 0" (or any other
        implausible jump) never reaches the published line or the arc
        planner's bias, it just gets ignored for that cycle.

        cache_name selects which independent {'path','time'} slot in
        _route_preview_caches to check/update - 'long' and 'mid' track
        separately since they're different walks (different max_distance)
        that can independently succeed or fail on any given cycle."""
        path = self._pose_graph_lookahead_path(x0, y0, current_yaw, max_distance)
        now = self.get_clock().now()
        cache = self._route_preview_caches.setdefault(cache_name, {'path': None, 'time': None})
        if path is not None and len(path) >= 2:
            start = path[0]
            prev = cache['path']
            jump_ok = (
                prev is None or
                math.hypot(start[0] - prev[0][0], start[1] - prev[0][1]) <= self.route_preview_max_jump_m)
            if jump_ok:
                cache['path'] = path
                cache['time'] = now
                return path
            self.get_logger().warn(
                f'Pose-graph path walk ({cache_name}) start point jumped '
                f'{math.hypot(start[0] - prev[0][0], start[1] - prev[0][1]):.1f}m from last '
                'cycle\'s accepted start - rejecting as an implausible nearest-vertex pick, '
                'reusing the last accepted path instead.', throttle_duration_sec=2.0)
        if (cache['path'] is not None and cache['time'] is not None and
                (now - cache['time']).nanoseconds / 1e9 <= self.route_preview_cache_timeout_sec):
            return cache['path']
        return None

    def _publish_route_preview(self, header, x0, y0, current_yaw):
        """Long-range (route_preview_distance_m, default 40m - matching
        this project's LiDAR max-range convention) green preview line
        along slam_toolbox's recorded pose graph, bending the same
        direction the actual mapped route bends - a further look down the
        road than turn_direction_marker's single turn_lookahead_m point,
        via the same shared walk (_pose_graph_lookahead_path) just carried
        out further and keeping every intermediate vertex instead of only
        the final one. Routed through _robust_pose_graph_path (see its own
        docstring) rather than the raw walk directly, so one bad cycle's
        nearest-vertex pick doesn't show up as the line snapping
        somewhere implausible. Returns the path actually published (or
        the cached one reused), so callers needing the same polyline for
        scoring (see _plan_lane_arc) don't have to redo the walk. Returns
        None (publishes nothing this cycle) when neither a fresh nor a
        cached path is available."""
        path = self._robust_pose_graph_path(
            x0, y0, current_yaw, self.route_preview_distance_m, 'long')
        if not path:
            return None
        marker = Marker()
        marker.header = header
        marker.ns = "route_preview"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 0.2, 0.85
        for x, y in path:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.05
            marker.points.append(p)
        self.route_preview_pub.publish(marker)
        return path

    def _publish_route_preview_mid(self, header, x0, y0, current_yaw):
        """Second, shorter reference line (route_preview_distance_mid_m,
        default 20m) - see route_preview_distance_mid_m's own comment for
        why: a single long walk can jump onto the wrong branch at a
        junction, so a second, shorter walk gives _plan_lane_arc an
        independent, more locally-reliable direction signal to weigh
        alongside the long one. Same robust-caching treatment as the long
        line, own cache slot ('mid') since it's an independent walk.
        Drawn in a lighter/brighter green so it's visually distinct from
        the main route-preview line in RViz."""
        path = self._robust_pose_graph_path(
            x0, y0, current_yaw, self.route_preview_distance_mid_m, 'mid')
        if not path:
            return None
        marker = Marker()
        marker.header = header
        marker.ns = "route_preview_mid"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.10
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.4, 1.0, 0.5, 0.9
        for x, y in path:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.06
            marker.points.append(p)
        self.route_preview_mid_pub.publish(marker)
        return path

    def _publish_nearest_vertex_marker(self, header, x0, y0):
        """Visualizes which pose-graph vertex nearest-vertex search
        actually picked this cycle (the starting point _pose_graph_
        lookahead_path's walk uses for both turn_direction_marker and
        route_preview_path) - green sphere when within
        nearest_vertex_max_dist, orange when it's still the best
        available match but further than that (e.g. before the vehicle
        has re-entered the mapped route). Always reports the best-
        available match regardless of that threshold - this is a
        diagnostic aid for exactly the "picked a stale/wrong vertex"
        class of bug that pose_graph_callback's vertex-count freeze
        fixes, so it deliberately does NOT hide a bad pick behind the
        same graceful-disable pattern used elsewhere in this file."""
        vertices = self._pose_graph_vertices_in_odom()
        if vertices is None or len(vertices) == 0:
            return
        d = np.hypot(vertices[:, 0] - x0, vertices[:, 1] - y0)
        idx0 = self._safe_nearest_index(d)
        if idx0 is None:
            return
        dist = float(d[idx0])
        vx, vy = vertices[idx0]

        marker = Marker()
        marker.header = header
        marker.ns = "nearest_pose_graph_vertex"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(vx)
        marker.pose.position.y = float(vy)
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.6
        if dist <= self.nearest_vertex_max_dist:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 0.9
        else:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.55, 0.0, 0.9
        self.nearest_vertex_marker_pub.publish(marker)
        self.nearest_vertex_distance_pub.publish(Float32(data=dist))

    def _publish_lane_marker(self, header, waypoints_xy):
        marker = Marker()
        marker.header = header
        marker.ns = "lane"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 1.0, 1.0
        for x, y in waypoints_xy:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.05
            marker.points.append(p)
        self.lane_marker_pub.publish(marker)

    def _publish_sub_goals(self, header, waypoints_xy):
        """waypoints_xy: the subgoal list a technique split the main goal
        into (_split_into_clear_segments or _build_lane_path's simplified
        output) - each pose's orientation points at the next waypoint (last
        one repeats the previous segment's heading, or stays identity if
        there's only one point)."""
        array = PoseArray()
        array.header = header
        n = len(waypoints_xy)
        for i, (x, y) in enumerate(waypoints_xy):
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            if i + 1 < n:
                nx, ny = waypoints_xy[i + 1]
                heading = math.atan2(ny - y, nx - x)
            elif n > 1:
                px, py = waypoints_xy[i - 1]
                heading = math.atan2(y - py, x - px)
            else:
                heading = 0.0
            qx, qy, qz, qw = self._yaw_to_quaternion(heading)
            pose.orientation.x, pose.orientation.y = qx, qy
            pose.orientation.z, pose.orientation.w = qz, qw
            array.poses.append(pose)
        self.sub_goals_pub.publish(array)

    def _yaw_from_quaternion(self, q):
        # Z-axis yaw only, matches quaternion_from_yaw used elsewhere in this project
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _yaw_to_quaternion(self, yaw):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def _clear_distance(self, x0, y0, heading, max_dist, step=0.2):
        """March from (x0, y0) along heading in the costmap frame, return
        distance to the first lethal/inflated/unknown cell, capped at
        max_dist. Costmap frame matches x0/y0's frame (odom)."""
        costmap = self.latest_costmap
        info = costmap.info
        w, h = info.width, info.height
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = costmap.data
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        d = 0.0
        while d < max_dist:
            d += step
            gx = int((x0 + d * cos_h - ox) / res)
            gy = int((y0 + d * sin_h - oy) / res)
            if gx < 0 or gx >= w or gy < 0 or gy >= h:
                return d  # ran off the known window - treat edge as the limit
            val = data[gy * w + gx]
            if val < 0 or val >= LETHAL_THRESHOLD:
                return d
        return max_dist

    def _lateral_clearance_arc(self, x, y, theta, sign, max_search):
        """Distance from (x,y), perpendicular to theta (sign=+1/-1 for
        either side), until a lethal/inflated/unknown costmap cell or the
        costmap edge - capped at max_search. Vectorized numpy, ported
        directly from freespace_grid_node's own _lateral_clearance
        (cart_navigation_ws) - same first-unsafe-cell semantics, just
        reading this node's cached _costmap_grid/LETHAL_THRESHOLD instead
        of that node's custom log-odds grid. That node's own docstring
        notes this was the dominant remaining per-cycle cost once
        vectorized (~15 candidates x ~13 samples x 2 sides, each up to
        corridor_scan_limit/resolution sequential steps) - ported already-
        vectorized rather than repeating that performance lesson here."""
        costmap = self.latest_costmap
        if costmap is None or self._costmap_grid is None:
            return 0.0
        info = costmap.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        step = res
        n = int(max_search / step)
        if n <= 0:
            return max_search
        d = np.arange(1, n + 1, dtype=np.float64) * step
        sin_t, cos_t = math.sin(theta), math.cos(theta)
        ex = x - sign * d * sin_t
        ey = y + sign * d * cos_t
        col = np.floor((ex - ox) / res).astype(np.int64)
        row = np.floor((ey - oy) / res).astype(np.int64)
        out_of_bounds = (col < 0) | (col >= w) | (row < 0) | (row >= h)
        safe_col = np.clip(col, 0, w - 1)
        safe_row = np.clip(row, 0, h - 1)
        val = self._costmap_grid[safe_row, safe_col]
        unsafe = out_of_bounds | (val < 0) | (val >= LETHAL_THRESHOLD)
        if not unsafe.any():
            return max_search
        first_unsafe = int(np.argmax(unsafe))
        return d[first_unsafe] - step

    def _arc_is_clear_costmap(self, x0, y0, yaw, curvature):
        """Walk an arc of given curvature from (x0,y0,yaw), checking the
        vehicle's footprint clearance (lane_clearance = lane_vehicle_
        half_width + lane_vehicle_footprint_margin) at each step against
        this node's own costmap, and how centered it stays between the
        left/right obstacles along the way. Returns (clear_distance,
        points, avg_centering_error).

        VECTORIZED (numpy) - confirmed live on real hardware that the
        original plain-Python per-step loop (up to lane_lookahead/
        resolution steps - e.g. 200 at lane_lookahead=20m, resolution=
        0.1m - times up to 3 footprint offsets each, called for every one
        of _plan_lane_arc's 5 candidates, multiple times per request_plan
        cycle) was a real, measurable slowdown on weaker real-hardware
        CPUs, the same class of bottleneck freespace_grid_node's own
        _lateral_clearance/grid-marker loop hit in cart_navigation_ws
        before THOSE got vectorized. All per-step positions/footprint
        checks are computed as whole-array numpy ops here instead of a
        Python for-loop; only the still-inherently-sequential centering-
        sample calls to _lateral_clearance_arc (itself already vectorized
        internally) remain a (much shorter, ~1-in-3-steps) Python loop.

        Verified via a 3000-trial fuzz test plus targeted edge cases
        (all-clear straight/curved, tiny lookahead, high curvature, a
        negative-offset-from-origin region) against the original
        sequential implementation before this replaced it - exact same
        first-unsafe-step semantics, not an approximation. Uses the SAME
        int()-truncation cell-index convention as _clear_distance/
        _costmap_cell_is_clear elsewhere in this file (NOT np.floor,
        which disagrees with int() truncation for negative values and
        would silently shift results near the low edge of the costmap -
        confirmed by the fuzz test catching exactly this when first
        tried)."""
        costmap = self.latest_costmap
        if costmap is None:
            return 0.0, [], 0.0
        info = costmap.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        step = res
        n_steps = int(self.lane_lookahead / step)
        if n_steps <= 0:
            return 0.0, [], 0.0

        s = np.arange(1, n_steps + 1, dtype=np.float64) * step
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        if abs(curvature) < 1e-4:
            lx = s.copy()
            ly = np.zeros_like(s)
            ltheta = np.zeros_like(s)
        else:
            r = 1.0 / curvature
            ltheta = s * curvature
            lx = r * np.sin(ltheta)
            ly = r * (1.0 - np.cos(ltheta))
        wx = x0 + lx * cos_yaw - ly * sin_yaw
        wy = y0 + lx * sin_yaw + ly * cos_yaw
        world_theta = yaw + ltheta
        sin_wt = np.sin(world_theta)
        cos_wt = np.cos(world_theta)

        unsafe_any_offset = np.zeros(n_steps, dtype=bool)
        for offset in (0.0, self.lane_clearance, -self.lane_clearance):
            ex = wx - offset * sin_wt
            ey = wy + offset * cos_wt
            # (x/res).astype(int64) truncates toward zero, same as plain
            # Python int() - see this method's own docstring for why that
            # (not np.floor) is required here.
            col = ((ex - ox) / res).astype(np.int64)
            row = ((ey - oy) / res).astype(np.int64)
            out_of_bounds = (col < 0) | (col >= w) | (row < 0) | (row >= h)
            safe_col = np.clip(col, 0, w - 1)
            safe_row = np.clip(row, 0, h - 1)
            val = self._costmap_grid[safe_row, safe_col]
            unsafe = out_of_bounds | (val < 0) | (val >= LETHAL_THRESHOLD)
            unsafe_any_offset |= unsafe

        if unsafe_any_offset.any():
            n_valid = int(np.argmax(unsafe_any_offset))
        else:
            n_valid = n_steps

        if n_valid == 0:
            return 0.0, [], 0.0

        clear_distance = float(s[n_valid - 1])
        points = list(zip(wx[:n_valid].tolist(), wy[:n_valid].tolist()))

        centering_error_sum = 0.0
        centering_samples = 0
        for i in range(2, n_valid, 3):
            left = self._lateral_clearance_arc(
                wx[i], wy[i], world_theta[i], 1, self.corridor_scan_limit)
            right = self._lateral_clearance_arc(
                wx[i], wy[i], world_theta[i], -1, self.corridor_scan_limit)
            centering_error_sum += abs(left - right)
            centering_samples += 1

        avg_centering_error = (centering_error_sum / centering_samples
                                if centering_samples else 0.0)
        return clear_distance, points, avg_centering_error

    def _direction_bias_bonus_arc(self, k, current_yaw, reference_yaw):
        """Bonus (higher = more preferred) for candidate curvature k,
        based on reference_yaw (this node's own pose-graph-derived bearing
        to the turn_lookahead_m point - see _publish_turn_direction, the
        SAME value that decides LEFT/RIGHT/STRAIGHT there) - in-process,
        not subscribed over a topic the way freespace_grid_node's own
        _direction_bias_bonus read /turn_direction from a separate node.
        desired is 0 (no bias) whenever the bearing difference is within
        goal_heading_snap_deg (same STRAIGHT threshold _publish_turn_
        direction itself uses), which makes this an exact no-op for
        ordinary corridor driving with no turn coming up.

        DEVIATION from the original: freespace_grid_node additionally
        scaled this by an "urgency" factor (0 far from the turn, rising to
        1 at turn_distance_m<=0) derived from a separately-published
        /turn_distance_m. This node has no equivalent distance-to-turn
        signal computed elsewhere, so that scaling is omitted here - bias
        magnitude/direction still tracks curvature alignment the same way,
        just without fading in as the turn approaches."""
        if self.lane_max_curvature <= 0.0:
            return 0.0
        diff = math.atan2(
            math.sin(reference_yaw - current_yaw), math.cos(reference_yaw - current_yaw))
        if abs(math.degrees(diff)) < self.goal_heading_snap_deg:
            return 0.0
        desired = 1.0 if diff > 0 else -1.0
        alignment = desired * (k / self.lane_max_curvature)  # +1 fully aligned, -1 fully opposed
        return self.direction_bias_gain * alignment

    def _publish_lane_candidates(self, header, x0, y0, results, names, chosen_k):
        """Draws ALL 5 candidates from _plan_lane_arc as separate LINE_
        STRIPs on /lane_candidates - the chosen one in full bright yellow
        (same styling as the final golden_lane marker), the other 4 dim/
        thin - so it's possible to SEE what the planner actually
        considered and compare each one against the green line directly
        in RViz, instead of only ever seeing the single winner and having
        to guess whether a bad pick was a scoring problem or "the right
        option was never even generated"."""
        markers = MarkerArray()
        for name, (k, dist, points, centering_error) in zip(names, results):
            marker = Marker()
            marker.header = header
            marker.ns = f"lane_candidate_{name}"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            is_chosen = abs(k - chosen_k) < 1e-6
            marker.scale.x = 0.18 if is_chosen else 0.07
            if is_chosen:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.84, 0.0, 0.95
            else:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.8, 0.55, 0.05, 0.4
            for x, y in [(x0, y0)] + list(points):
                p = Point()
                p.x, p.y, p.z = float(x), float(y), 0.04
                marker.points.append(p)
            markers.markers.append(marker)
        self.lane_candidates_pub.publish(markers)

    def _plan_lane_arc(self, header, x0, y0, yaw, reference_yaw, route_preview_path,
                        route_preview_mid_path=None):
        """Pick a clear arc, primarily biased toward following the pose-
        graph route-preview line(s) ("no matter what" among safe options -
        see below), with centering (equal left/right clearance) and
        gentlest-curvature as tiebreakers, smoothed over time so it
        doesn't jump between updates.

        5 DISCRETE candidates (forward, near/far left, near/far right -
        lane_candidate_inner_fraction/lane_candidate_outer_fraction of
        lane_max_curvature) instead of a lane_num_candidates-way
        continuous sweep - each one published individually on
        /lane_candidates (see _publish_lane_candidates) so what the
        planner actually considered is visible in RViz, not just the
        winner. Each is walked out to lane_lookahead meters (raise this
        launch arg, e.g. to 20.0, for a longer reach).

        REVISED scoring from the original cart_navigation_ws
        freespace_grid_node port: that version's safety filter
        (lane_straight_preference_ratio) only let candidates within a
        FRACTION of the single longest clear distance found THIS CYCLE be
        scored by the bias at all - confirmed live, repeatedly, that this
        silently excluded the genuinely correct direction whenever it was
        a tighter/shorter-clearance curving arc (following an actual
        bend) than an unrelated wide-open straight opening elsewhere. No
        bias weight can ever pull toward a candidate that was filtered
        out before scoring even ran. Now: the safety filter is an
        ABSOLUTE floor (lane_min_safety_distance, "is this arc safe
        enough to drive") instead of relative to the best one found, and
        the PRIMARY sort key is distance to the route-preview line(s)
        (route_preview_path/route_preview_mid_path, weighted by
        reference_path_weight/reference_path_mid_weight) minus the
        pose-graph direction bias - i.e. among everything safe, whichever
        candidate is actually closest to the recorded route wins "no
        matter what" (even if a less-aligned candidate reached further),
        with centering_error only breaking ties among comparably-close
        candidates, and driving centered within whichever direction wins.

        route_preview_mid_path (optional) adds a second, independent
        point-to-polyline distance at a shorter lookahead distance
        (route_preview_distance_mid_m) - a single long walk jumping onto
        the wrong branch at a junction is now only one of three direction
        signals (this, reference_yaw's turn_lookahead_m bearing, and
        route_preview_path's long line) instead of the only one.

        Reactive only - like the original, this has no concept of a
        destination at all, so the caller only invokes this while no
        manual /goal_pose is set.

        Returns (goal_x, goal_y, arc_points, final_yaw) or None if no
        candidate reaches even lane_min_safety_distance of clear distance
        (no stuck/recovery state machine ported - the caller falls back
        to the existing straight-ray _pick_goal_heading when this returns
        None, same graceful-degrade pattern used throughout this file,
        rather than stalling)."""
        if self.latest_costmap is None:
            return None
        curvatures = [
            0.0,
            self.lane_candidate_inner_fraction * self.lane_max_curvature,
            self.lane_candidate_outer_fraction * self.lane_max_curvature,
            -self.lane_candidate_inner_fraction * self.lane_max_curvature,
            -self.lane_candidate_outer_fraction * self.lane_max_curvature,
        ]
        names = ['forward', 'left_near', 'left_far', 'right_near', 'right_far']
        results = [(k, *self._arc_is_clear_costmap(x0, y0, yaw, k)) for k in curvatures]
        max_distance = max(dist for _, dist, _, _ in results)
        res = self.latest_costmap.info.resolution
        if max_distance < res:
            self.smoothed_curvature = 0.0
            return None

        good_enough = [r for r in results if r[1] >= self.lane_min_safety_distance]
        if not good_enough:
            # Nothing reaches the absolute safety floor this cycle (e.g. a
            # genuinely tight/cluttered spot) - fall back to whichever
            # candidate got furthest, rather than giving up outright.
            good_enough = [max(results, key=lambda r: r[1])]

        def score(r):
            k, dist, points, centering_error = r
            ex, ey = points[-1] if points else (x0, y0)
            direction_bonus = self._direction_bias_bonus_arc(k, yaw, reference_yaw)
            ref_dist = 0.0
            if route_preview_path is not None:
                d = self._point_to_polyline_dist(route_preview_path, ex, ey)
                if d is not None:
                    ref_dist += self.reference_path_weight * d
            if route_preview_mid_path is not None:
                d_mid = self._point_to_polyline_dist(route_preview_mid_path, ex, ey)
                if d_mid is not None:
                    ref_dist += self.reference_path_mid_weight * d_mid
            # Primary: distance to the recorded route (lower is better,
            # direction_bonus subtracted so a well-aligned curvature still
            # counts when no route-preview line is available at all - both
            # route_preview_path/route_preview_mid_path None makes this
            # term purely -direction_bonus, degrading gracefully toward
            # "just lean the right general way"). Secondary: centering.
            # Tertiary: gentlest curvature among ties.
            return (ref_dist - direction_bonus, centering_error, abs(k))

        chosen_k, chosen_dist, chosen_points, _ = min(good_enough, key=score)
        self._publish_lane_candidates(header, x0, y0, results, names, chosen_k)

        alpha = self.lane_smoothing_alpha
        smoothed_k = alpha * chosen_k + (1 - alpha) * self.smoothed_curvature
        final_dist, final_points, _ = self._arc_is_clear_costmap(x0, y0, yaw, smoothed_k)
        if final_dist < res:
            # Smoothed curvature isn't actually clear right now (e.g. a new
            # obstacle just appeared where the old heading was taking us) -
            # snap to the unsmoothed choice instead of stalling.
            smoothed_k, final_dist, final_points = chosen_k, chosen_dist, chosen_points
        self.smoothed_curvature = smoothed_k

        if not final_points:
            return None

        ex, ey = final_points[-1]
        final_yaw = yaw + final_dist * smoothed_k
        return ex, ey, final_points, final_yaw

    def _pick_goal_heading(self, x0, y0, current_yaw, smoothed_yaw):
        """Return (new_smoothed_yaw, goal_distance).

        With no latest_goal_xy set, this is the original purely-reactive
        picker: scans candidate headings around current_yaw out to
        scan_distance and picks whichever stays clear the longest, biased
        toward current_yaw so it doesn't zigzag between two similarly-open
        directions. That version has NO notion of a destination - it just
        keeps going wherever's open, turning only when the costmap forces
        it to, and ties between equally-open directions are broken by
        whichever offset happens to be checked first (the fixed
        (15,-15,30,-30,...) order below), which in practice means it
        favors turning left at a symmetric junction for no reason tied to
        where you actually want to go. Fine for "just don't hit anything",
        useless for "go to this specific point" - confirmed with
        tools/carrot_heading_tuner.py on an L/T-shaped corridor: all three
        of a north goal, a south goal, and a "stay in this corridor" goal
        produced the *same* left-biased path, because nothing here ever
        looked at where any of those goals actually were.

        With latest_goal_xy set, the scan is centered on the bearing to
        that goal instead of current_yaw, widened to +-165deg (from +-90):
        reaching a goal that's, say, due north from deep inside an
        east-west corridor legitimately requires committing to a heading
        nowhere near the direct bearing until the corridor opens up, and
        the narrower spread used for the no-goal case couldn't find that
        far a candidate at all.

        Crucially, goal-seeking mode canNOT just reuse the no-goal case's
        "pick whichever candidate has the longest raw clear_distance,
        re-centered on the goal bearing instead of current heading" - raw
        clearance has no idea a candidate is actually useful, only that
        it's open, and will happily prefer heading almost straight back
        down the empty corridor already driven through over continuing
        toward the goal, since "already-driven-through empty corridor"
        reads as more clear than "corridor ahead, not yet explored,
        curving toward the goal". Confirmed live with
        tools/carrot_heading_tuner.py: a raw-clearance version of this,
        merely re-centered on the goal bearing, made the simulated vehicle
        oscillate in place, repeatedly "deciding" a ~180deg reversal was
        the best option, and it never reached an L-shaped corridor's goal
        at all.

        So goal-seeking mode scores each candidate by the resulting
        distance to the goal after advancing along it (by however far
        _clear_distance says is safe, capped at carrot_distance/remaining
        distance to goal) - not by raw clearance. A direct, unobstructed
        shot at the goal scores 0 (lands exactly on it) and always wins.
        When blocked, whichever candidate's reachable endpoint ends up
        closest to the goal wins - reversing away from the goal always
        scores worse than continuing forward, no matter how much open
        corridor is behind the vehicle, which is exactly the missing
        constraint above. heading_switch_margin_m is reused as-is (still
        meters, still "how much better before switching"), just measured
        against this distance-to-goal score instead of raw clearance.

        Distance-to-goal alone still isn't enough for a good-looking path,
        just a legal one: it has no preference for staying away from
        walls, so around a bend it'll happily hug the inside edge as tight
        as _clear_distance allows, since that's the shortest technically-
        clear route - confirmed live by drawing an actual curved corridor
        in tools/carrot_heading_tuner.py and watching the picked path
        graze the inner wall the whole way around instead of favoring the
        middle. candidate_centering adds a second term: two perpendicular
        raycasts at each candidate's own prospective ENDPOINT (not
        (x0,y0) - checked there first, it had zero effect on the winner,
        since that's the same fixed point for every candidate and mostly
        just measures "how open is it here", not "does this specific
        candidate end up near a wall"), capped at corridor_check_distance,
        scored by whichever side is tighter (min, not average - wide open
        on one side shouldn't hide grazing a wall on the other). Weighted
        by centering_weight and added directly to the goal-distance score
        (both already in meters).

        smoothed_yaw is threaded through explicitly (returned alongside
        goal_distance) rather than read/written on self._smoothed_yaw
        directly, so _split_into_clear_segments can run its own,
        independent smoothing thread when it calls this repeatedly to
        build a whole multi-step path in one request_plan() cycle, without
        clobbering the live per-cycle smoothing state this method's other
        caller (request_plan's reactive/no-destination path) relies on."""
        has_goal = self.latest_goal_xy is not None
        if has_goal:
            gx, gy = self.latest_goal_xy
            dist_to_goal = math.hypot(gx - x0, gy - y0)
            base_heading = math.atan2(gy - y0, gx - x0)
        else:
            dist_to_goal = None
            base_heading = current_yaw

        if self.latest_costmap is None:
            return base_heading, (min(self.carrot_distance, dist_to_goal) if has_goal else self.carrot_distance)

        # Fetched once per _pick_goal_heading call (one TF lookup / one
        # walk), not once per candidate. See use_route_preview_reference's
        # own comment: this switches which pose-graph signal graph_term
        # (below, used by BOTH candidate_score branches - reactive/no-goal
        # and goal-seeking) is biased toward.
        if self.use_route_preview_reference:
            # Directional: the same walked, direction-disambiguated path
            # _publish_route_preview draws (green line) - None whenever no
            # graph/TF is available yet, or the walk couldn't get started.
            route_preview_path = (
                self._pose_graph_lookahead_path(x0, y0, current_yaw, self.route_preview_distance_m)
                if self.reference_path_weight != 0.0 else None)
            route_preview_path = (
                np.array(route_preview_path)
                if route_preview_path is not None and len(route_preview_path) >= 2 else None)
        else:
            # Existing behavior, unchanged: nearest vertex ANYWHERE in the
            # whole recorded graph - no direction awareness. pose_graph_
            # bonus degrades to 0 (no bias) whenever no graph/TF is
            # available yet, same graceful-disable pattern as
            # centerline_enabled elsewhere here.
            pose_graph_vertices = (
                self._pose_graph_vertices_in_odom() if self.pose_graph_weight != 0.0 else None)

        def pose_graph_bonus(ex, ey):
            if pose_graph_vertices is None or len(pose_graph_vertices) == 0:
                return 0.0
            d = np.hypot(pose_graph_vertices[:, 0] - ex, pose_graph_vertices[:, 1] - ey)
            return max(0.0, self.pose_graph_max_influence - float(d.min()))

        def route_preview_bonus(ex, ey):
            dist = self._point_to_polyline_dist(route_preview_path, ex, ey)
            return 0.0 if dist is None else max(0.0, self.pose_graph_max_influence - dist)

        def graph_bonus_term(ex, ey):
            if self.use_route_preview_reference:
                return self.reference_path_weight * route_preview_bonus(ex, ey)
            return self.pose_graph_weight * pose_graph_bonus(ex, ey)

        def candidate_advance(clear):
            # See goal_safety_margin's own comment: clear is the distance to
            # the first UNSAFE cell, not a safe stopping point - pull back
            # by the margin before it ever becomes the placed goal distance.
            clear = max(0.0, clear - self.goal_safety_margin)
            if has_goal:
                return max(min(clear, self.carrot_distance, dist_to_goal), 0.0)
            return max(min(clear, self.carrot_distance), 1.0)

        def candidate_centering(heading, ex, ey):
            # Checked at the candidate's prospective ENDPOINT (ex,ey), not
            # (x0,y0) - (x0,y0) is the same fixed point for every
            # candidate, so checking there mostly measures "how open is it
            # here" rather than "does THIS candidate's own endpoint sit
            # near a wall", which barely distinguishes between candidates
            # at all (confirmed live: had zero effect on the winning
            # heading until moved to the endpoint).
            if self.centering_weight == 0.0:
                return 0.0
            left = self._clear_distance(ex, ey, heading + math.pi / 2, self.corridor_check_distance)
            right = self._clear_distance(ex, ey, heading - math.pi / 2, self.corridor_check_distance)
            return min(left, right)

        def candidate_score(heading, clear):
            # Centering now applies in BOTH modes (previously reactive mode
            # returned bare `clear` - raw clearance only, no centering
            # influence at all, safe or not). Confirmed live via
            # /carrot_marker + RViz: with no goal set, driving a curved
            # section, the picked heading consistently hugged whichever
            # side happened to be marginally more open instead of the
            # corridor's actual middle - _centerline_lateral_bias doesn't
            # help here either, since it deliberately disables itself at
            # exactly this kind of corner/curve (see its own docstring).
            # This is the same weighted-bonus pattern goal-seeking mode
            # already used, just no longer gated on has_goal.
            advance = candidate_advance(clear)
            ex = x0 + advance * math.cos(heading)
            ey = y0 + advance * math.sin(heading)
            side_clearance = candidate_centering(heading, ex, ey)
            if side_clearance < self.goal_safety_margin:
                # Endpoint sits closer to a side wall (e.g. a corner cutting
                # in from off the main ray) than the safety margin allows -
                # actively penalize rather than just under-reward, so a
                # corner-hugging candidate never beats one with real margin.
                centering_term = -self.centering_weight * (self.goal_safety_margin - side_clearance)
            else:
                centering_term = self.centering_weight * side_clearance
            graph_term = graph_bonus_term(ex, ey)
            if not has_goal:
                return clear + centering_term + graph_term
            goal_term = -math.hypot(gx - ex, gy - ey)  # bigger (closer to 0) is better, like clear
            return goal_term + centering_term + graph_term

        best_heading = base_heading
        best_clear = self._clear_distance(x0, y0, base_heading, self.scan_distance)
        best_score = candidate_score(base_heading, best_clear)

        # Widening spread: nearer-to-preferred options are checked first
        # and only displaced by a clearly better (not just marginally
        # different) alternative, so it doesn't turn every cycle chasing
        # noise between near-equal directions. Goal-seeking mode needs the
        # wider +-165deg spread (see docstring above); the no-goal
        # reactive mode keeps the original +-90deg - deliberately can't
        # see "behind" it, since without a destination there's nothing to
        # justify considering a near-reverse turn.
        offsets_deg = (
            (15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90, 105, -105, 120, -120, 135, -135, 150, -150, 165, -165)
            if has_goal else
            (15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90))
        for offset_deg in offsets_deg:
            heading = base_heading + math.radians(offset_deg)
            clear = self._clear_distance(x0, y0, heading, self.scan_distance)
            score = candidate_score(heading, clear)
            if score > best_score + self.heading_switch_margin_m:
                best_score = score
                best_clear = clear
                best_heading = heading

        goal_dist = candidate_advance(best_clear)

        # Low-pass filter on the heading itself (angle-wrap-safe): converge
        # toward best_heading over a few cycles rather than snapping to it,
        # so a genuinely better opening still wins out but frame-to-frame
        # raycast noise doesn't reshape SmacPlannerHybrid's entire curve
        # every cycle. See module docstring.
        if smoothed_yaw is None:
            smoothed_yaw = best_heading
        else:
            diff = math.atan2(
                math.sin(best_heading - smoothed_yaw),
                math.cos(best_heading - smoothed_yaw))
            if abs(math.degrees(diff)) >= self.goal_heading_snap_deg:
                smoothed_yaw = best_heading  # real corner - commit now, don't lag
            else:
                smoothed_yaw += self.goal_heading_alpha * diff

        return smoothed_yaw, goal_dist

    def _split_into_clear_segments(self, x0, y0, gx, gy, max_steps):
        """use_global_planner=False + a real /goal_pose set: instead of one
        straight segment capped at whatever's reachable this single cycle
        (the old behavior, still used below when there's no fixed goal to
        aim a multi-segment path at), build a whole multi-waypoint polyline
        from (x0,y0) to (gx,gy) in one shot by repeatedly calling
        _pick_goal_heading and advancing waypoint_step_m at a time - i.e.
        this IS request_plan's own per-cycle logic, just run several times
        back-to-back within a single cycle instead of once per real
        resend_period_sec tick, so TEB gets a longer, already-bent
        reference path instead of one short segment it has to wait several
        real cycles to see the rest of.

        Two earlier versions of this tried to be a proper geometric
        "insert one detour waypoint, recurse" splitter with their own
        scoring loop, and both were wrong in ways only found by actually
        running them (tools/carrot_heading_tuner.py) on an L/T-junction,
        not by inspection:
          1. Scored each candidate by the goal-distance reached after
             advancing the candidate's own FULL clear_distance. A diagonal
             heading with only ~6m of clearance can score better (ends up
             numerically closer to the goal) than continuing another ~12m
             down the actual corridor at ~0deg, because the diagonal
             points more directly at the goal - so one greedy jump locks
             onto the diagonal, quickly runs out of room hugging the
             inside wall, and never finds its way through.
          2. Capped each step's advance at a small fixed distance to force
             many small steps (the right instinct), but then floored that
             capped advance at a minimum step so it was never allowed to
             stay still - which, once genuinely cornered (clearance near
             zero), forced it forward anyway, punching straight through
             the wall in small increments, and every step after that
             started from an already-inside-a-wall position, so its own
             clear_distance readings were meaningless from then on.

        _pick_goal_heading doesn't have either problem: its own
        goal_distance is already bounded by real measured clearance (never
        forced past what's actually open), and calling it repeatedly with
        a small waypoint_step_m re-evaluates from a fresh position often
        enough that a momentarily-attractive diagonal never gets to commit
        the vehicle too far into a corner before the next call corrects
        course. Confirmed live: this is the version that actually reaches
        the goal cleanly on the L/T-junction test.

        Returns a list of (x, y) waypoints starting with (x0,y0) - ending
        with (gx,gy) if it converges within max_steps, or wherever it got
        to otherwise (TEB's own local-costmap obstacle avoidance is the
        final backstop here, same as it always was for the single-segment
        case, so a shorter-than-hoped polyline is still strictly better
        than none)."""
        x, y = x0, y0
        yaw = math.atan2(gy - y0, gx - x0)
        smoothed_yaw = None  # independent of self._smoothed_yaw - see _pick_goal_heading's docstring
        waypoints = [(x, y)]
        for _ in range(max_steps):
            dist = math.hypot(gx - x, gy - y)
            if dist < STRAIGHT_PATH_STEP_M:
                break
            smoothed_yaw, goal_dist = self._pick_goal_heading(x, y, yaw, smoothed_yaw)
            advance = min(self.waypoint_step_m, goal_dist, dist)
            x = x + advance * math.cos(smoothed_yaw)
            y = y + advance * math.sin(smoothed_yaw)
            yaw = smoothed_yaw
            waypoints.append((x, y))
            if math.hypot(gx - x, gy - y) < STRAIGHT_PATH_STEP_M:
                break
        return waypoints

    def _build_reactive_chain(self, x0, y0, current_yaw):
        """use_reactive_chain=True, no /goal_pose set: builds a short
        multi-waypoint chain using the same repeated-_pick_goal_heading
        technique _split_into_clear_segments uses for a real destination -
        just with no (gx, gy) to march toward or stop at.
        self.latest_goal_xy is None throughout this branch, so each
        _pick_goal_heading call runs in its plain reactive scoring mode
        (clearance + centering + pose-graph bias), same as the single-
        carrot case - just re-evaluated fresh every waypoint_step_m
        instead of once per real request_plan cycle, so a turn gets
        discovered progressively across several bending waypoints rather
        than only showing up as one single far carrot point (see
        _split_into_clear_segments' own docstring for why small repeated
        steps, not one big jump, is what actually handles corners well).

        Stops once the cumulative chain length reaches scan_distance (the
        same budget already used for turn-scanning - the chain reaches
        exactly as far as the turn-detection logic already looks ahead) or
        max_waypoint_steps, whichever comes first. Unlike the goal-seeking
        version there's no destination to converge on, so a distance
        budget is the only sensible stopping rule here.

        Returns a list of (x, y) waypoints starting with (x0, y0)."""
        x, y = x0, y0
        yaw = current_yaw
        smoothed_yaw = None  # independent of self._smoothed_yaw - see _pick_goal_heading's docstring
        waypoints = [(x, y)]
        traveled = 0.0
        for _ in range(self.max_waypoint_steps):
            if traveled >= self.scan_distance:
                break
            smoothed_yaw, goal_dist = self._pick_goal_heading(x, y, yaw, smoothed_yaw)
            advance = min(self.waypoint_step_m, goal_dist)
            x = x + advance * math.cos(smoothed_yaw)
            y = y + advance * math.sin(smoothed_yaw)
            yaw = smoothed_yaw
            traveled += advance
            waypoints.append((x, y))
        return waypoints

    def _build_lane_path(self, x0, y0, gx, gy):
        """use_lane_following=True + a real /goal_pose set: navigate along
        the medial axis (skeleton) of latest_costmap's free space instead
        of picking headings step by step - see module docstring's
        "use_lane_following" paragraph for the full rationale. Returns a
        list of (x, y) waypoints from (x0,y0) to (gx,gy), or None if no
        skeleton path connects them (logged, caller falls through to
        whatever the other modes would have done - see request_plan)."""
        info = self.latest_costmap.info
        w, h = info.width, info.height
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        grid = np.array(self.latest_costmap.data, dtype=np.int16).reshape(h, w)

        free = (grid >= 0) & (grid < LETHAL_THRESHOLD)
        skeleton = skeletonize(free)
        ys, xs = np.nonzero(skeleton)
        if len(ys) == 0:
            return None

        pts = np.stack([ys, xs], axis=1)
        tree = cKDTree(pts)

        graph = nx.Graph()
        idx_of = {(int(y), int(x)): True for y, x in pts}
        for y, x in pts:
            y, x = int(y), int(x)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    if (y + dy, x + dx) in idx_of:
                        graph.add_edge((y, x), (y + dy, x + dx), weight=math.hypot(dy, dx) * res)

        def nearest_skeleton_cell(wx, wy):
            cell_y = (wy - oy) / res
            cell_x = (wx - ox) / res
            _, i = tree.query([cell_y, cell_x])
            return (int(pts[i][0]), int(pts[i][1]))

        start_cell = nearest_skeleton_cell(x0, y0)
        goal_cell = nearest_skeleton_cell(gx, gy)

        try:
            path_cells = nx.shortest_path(graph, start_cell, goal_cell, weight='weight')
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return None

        path_xy = [(ox + (cx + 0.5) * res, oy + (cy + 0.5) * res) for cy, cx in path_cells]
        return self._simplify_path_dp(path_xy, self.lane_simplify_epsilon)

    def _simplify_path_dp(self, points, epsilon):
        """Ramer-Douglas-Peucker simplification: keeps the path within
        epsilon meters (perpendicular distance) of the original polyline's
        shape at every point, discarding only points that don't bend the
        path more than that - NOT the same as "shortcut to the farthest
        point that's still technically collision-clear" (a materially
        worse alternative that was tried first for _build_lane_path - it
        throws away the skeleton's centering entirely, since a wall-
        hugging diagonal shortcut can easily still be "technically clear".
        Deviation-bounded simplification is what actually preserves the
        skeleton's true centered shape while still cutting hundreds of
        raw pixel-jagged skeleton cells down to a handful of waypoints)."""
        if len(points) < 3:
            return points

        def perpendicular_distance(p, a, b):
            (px, py), (ax, ay), (bx, by) = p, a, b
            dx, dy = bx - ax, by - ay
            if dx == 0.0 and dy == 0.0:
                return math.hypot(px - ax, py - ay)
            t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
            proj_x, proj_y = ax + t * dx, ay + t * dy
            return math.hypot(px - proj_x, py - proj_y)

        dmax = 0.0
        index = 0
        for i in range(1, len(points) - 1):
            d = perpendicular_distance(points[i], points[0], points[-1])
            if d > dmax:
                index = i
                dmax = d

        if dmax > epsilon:
            left = self._simplify_path_dp(points[:index + 1], epsilon)
            right = self._simplify_path_dp(points[index:], epsilon)
            return left[:-1] + right
        return [points[0], points[-1]]

    def _build_straight_path(self, header, waypoints_xy):
        """Interpolates a nav_msgs/Path at STRAIGHT_PATH_STEP_M spacing
        through a polyline of (x, y) waypoints (either the single
        [start, goal] pair from the old no-destination fallback, or
        _split_into_clear_segments' multi-segment result). Orientation at
        each point faces that segment's own direction, not one fixed
        heading for the whole path, so the path itself actually bends at
        each detour waypoint - TEB tracks orientation as well as position,
        so a single fixed heading across a bent polyline would fight the
        turn instead of describing it."""
        path = Path()
        path.header = header
        for seg_idx in range(len(waypoints_xy) - 1):
            (sx, sy), (ex, ey) = waypoints_xy[seg_idx], waypoints_xy[seg_idx + 1]
            seg_dist = math.hypot(ex - sx, ey - sy)
            seg_yaw = math.atan2(ey - sy, ex - sx)
            qx, qy, qz, qw = self._yaw_to_quaternion(seg_yaw)
            steps = max(1, int(seg_dist / STRAIGHT_PATH_STEP_M))
            start_i = 0 if seg_idx == 0 else 1  # skip the point shared with the previous segment
            for i in range(start_i, steps + 1):
                t = i / steps
                pose_stamped = PoseStamped()
                pose_stamped.header = header
                pose_stamped.pose.position.x = sx + t * (ex - sx)
                pose_stamped.pose.position.y = sy + t * (ey - sy)
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.x = qx
                pose_stamped.pose.orientation.y = qy
                pose_stamped.pose.orientation.z = qz
                pose_stamped.pose.orientation.w = qw
                path.poses.append(pose_stamped)
        return path

    def _dispatch_follow_path(self, path):
        self.plan_pub.publish(path)
        # Tracked for costmap_callback's path-safety monitor - see that
        # method's own comment. Set even if path_safety_check is disabled
        # (cheap either way; the check itself is what's gated on the flag).
        self._current_path = path

        if not self._controller_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('follow_path action server not available yet', throttle_duration_sec=5.0)
            return

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = self.controller_id
        send_future = self._controller_client.send_goal_async(follow_goal)
        send_future.add_done_callback(self._on_follow_goal_response)

    def _on_follow_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is not None and goal_handle.accepted:
            self._current_follow_goal_handle = goal_handle

    def request_plan(self):
        if self.latest_odom is None:
            self.get_logger().warn('No odometry received yet, skipping plan request', throttle_duration_sec=5.0)
            return

        if self.latest_goal_xy is not None:
            # Republished every cycle (not just once in goal_pose_callback)
            # since this publisher has no transient-local durability - a
            # late-joining RViz would otherwise never see a goal that was
            # set before it connected. Kept alive through the "holding"
            # early-return below too, so the marker doesn't disappear once
            # arrived.
            self._publish_goal_marker(
                self.latest_odom.header, self.latest_goal_xy[0], self.latest_goal_xy[1],
                self.goal_marker_pub)
            pose_now = self.latest_odom.pose.pose
            remaining = math.hypot(
                self.latest_goal_xy[0] - pose_now.position.x,
                self.latest_goal_xy[1] - pose_now.position.y)
            if remaining <= self.goal_xy_tolerance:
                # auto_return_to_start (closed-loop map navigation): the
                # FIRST goal reached auto-chains one more goal - the
                # recorded start_pose_xy - instead of holding, so one
                # manual "2D Goal Pose" click drives the whole lap and
                # back, not just the outbound leg. Detected by whether the
                # goal just reached IS start_pose_xy already (the return
                # leg) or not (the outbound leg, or any leg before it) -
                # only ever chains once per goal, so this can't loop
                # forever even if start_pose_xy and a later manually-set
                # goal happen to coincide.
                if self.auto_return_to_start and self.start_pose_xy is not None:
                    dist_to_start = math.hypot(
                        self.start_pose_xy[0] - self.latest_goal_xy[0],
                        self.start_pose_xy[1] - self.latest_goal_xy[1])
                    if dist_to_start > self.goal_xy_tolerance:
                        self.latest_goal_xy = self.start_pose_xy
                        self._smoothed_yaw = None
                        self.get_logger().info(
                            'Leg complete - returning to recorded start '
                            f'({self.start_pose_xy[0]:.2f}, {self.start_pose_xy[1]:.2f}).')
                        return
                    self.get_logger().info(
                        'Closed loop complete - back at start, holding.', throttle_duration_sec=5.0)
                    return
                # Deliberately don't clear latest_goal_xy here: doing so would
                # make _pick_goal_heading fall back to its reactive, no-
                # destination mode next cycle and drive off again. Holding
                # means "keep re-detecting arrived and doing nothing" until a
                # genuinely new /goal_pose overwrites latest_goal_xy.
                self.get_logger().info(
                    'Goal reached (within %.2fm) - holding, no further replans until a '
                    'new /goal_pose arrives.' % self.goal_xy_tolerance, throttle_duration_sec=5.0)
                return

        # use_lane_following wins over both other modes whenever a goal is
        # set (see module docstring) - falls through to whatever
        # use_global_planner would otherwise do if no costmap is available
        # yet or no skeleton path connects start to goal (e.g. genuinely
        # disconnected free-space regions in the current rolling window).
        if self.use_lane_following and self.latest_goal_xy is not None:
            pose = self.latest_odom.pose.pose
            header = self.latest_odom.header
            gx, gy = self.latest_goal_xy
            if self.latest_costmap is not None:
                waypoints_xy = self._build_lane_path(pose.position.x, pose.position.y, gx, gy)
                if waypoints_xy is not None:
                    self._publish_lane_marker(header, waypoints_xy)
                    self._publish_sub_goals(header, waypoints_xy)
                    path = self._build_straight_path(header, waypoints_xy)
                    self._dispatch_follow_path(path)
                    return
                self.get_logger().warn(
                    'use_lane_following: no skeleton path found connecting current pose to '
                    'goal through this costmap window - falling back to the straight-segment '
                    'technique this cycle.', throttle_duration_sec=5.0)

        # use_global_planner=False with a real destination set: build the
        # whole straight-segment polyline to it in one shot (see
        # _split_into_clear_segments) rather than the single
        # carrot-distance-capped segment used below - that single-segment
        # path is deliberately reachable-by-construction every cycle (its
        # endpoint is chosen specifically because the line to it is clear),
        # so applying the splitter to IT would never find anything to
        # split; it only does something useful aimed at the actual,
        # possibly-much-farther-than-carrot_distance goal.
        if not self.use_global_planner and self.latest_goal_xy is not None:
            pose = self.latest_odom.pose.pose
            header = self.latest_odom.header
            gx, gy = self.latest_goal_xy
            if self.latest_costmap is None:
                waypoints_xy = [(pose.position.x, pose.position.y), (gx, gy)]
            else:
                waypoints_xy = self._split_into_clear_segments(
                    pose.position.x, pose.position.y, gx, gy, self.max_waypoint_steps)
            self._publish_sub_goals(header, waypoints_xy)
            path = self._build_straight_path(header, waypoints_xy)
            self._dispatch_follow_path(path)
            return

        if self.use_global_planner:
            if self._planning_in_progress:
                return
            if not self._planner_client.wait_for_server(timeout_sec=0.2):
                self.get_logger().warn('compute_path_to_pose action server not available yet', throttle_duration_sec=5.0)
                return

        pose = self.latest_odom.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation)
        header = self.latest_odom.header  # frame_id "odom", matches costmaps' global_frame

        # Pose-graph-referenced turn direction: which way the actual
        # mapped path bends over the next turn_lookahead_m meters, not
        # which way the reactive/global planner happens to be steering
        # this cycle. Computed BEFORE goal-heading picking now (used as a
        # direct in-process bias input by _plan_lane_arc below, not just
        # for the /turn_direction topic/marker).
        #
        # Three-tier fallback: (1) a genuinely fresh far_point this cycle,
        # (2) the last confidently-computed reference_yaw, if it's still
        # within reference_yaw_cache_timeout_sec - covers a transient TF
        # hiccup or a cycle where every pose-graph vertex got filtered as
        # non-finite, so a momentary glitch doesn't make the bias drop to
        # zero and snap back for one cycle, (3) current_yaw (no bias) once
        # even the cache has gone stale - e.g. slam_toolbox genuinely not
        # running/available yet, same graceful-disable pattern used
        # throughout this file.
        far_point = self._pose_graph_lookahead_point(pose.position.x, pose.position.y, yaw)
        now = self.get_clock().now()
        if far_point is not None:
            reference_yaw = math.atan2(
                far_point[1] - pose.position.y, far_point[0] - pose.position.x)
            self._last_valid_reference_yaw = reference_yaw
            self._last_valid_reference_yaw_time = now
        elif (self._last_valid_reference_yaw is not None and
              (now - self._last_valid_reference_yaw_time).nanoseconds / 1e9
              <= self.reference_yaw_cache_timeout_sec):
            reference_yaw = self._last_valid_reference_yaw
        else:
            reference_yaw = yaw
        self._publish_turn_direction(header, pose.position, yaw, reference_yaw)
        route_preview_pts = self._publish_route_preview(header, pose.position.x, pose.position.y, yaw)
        route_preview_mid_pts = self._publish_route_preview_mid(
            header, pose.position.x, pose.position.y, yaw)
        self._publish_nearest_vertex_marker(header, pose.position.x, pose.position.y)

        # Arc-based lane planner (use_route_preview_reference:=true) - see
        # _plan_lane_arc's own docstring. Reactive-only (no destination
        # concept, same as the original freespace_grid_node logic this is
        # ported from), so it only takes over while no manual /goal_pose is
        # set; goal-seeking always uses the straight-ray picker below.
        # Reuses the paths the two publish calls above just computed
        # (already robust-cached, see _robust_pose_graph_path) rather than
        # walking the pose graph a third/fourth time this cycle.
        arc_result = None
        if self.use_route_preview_reference and self.latest_goal_xy is None:
            route_preview_np = (
                np.array(route_preview_pts)
                if route_preview_pts is not None and len(route_preview_pts) >= 2 else None)
            route_preview_mid_np = (
                np.array(route_preview_mid_pts)
                if route_preview_mid_pts is not None and len(route_preview_mid_pts) >= 2 else None)
            arc_result = self._plan_lane_arc(
                header, pose.position.x, pose.position.y, yaw, reference_yaw,
                route_preview_np, route_preview_mid_np)

        self._last_golden_lane_points = None
        if arc_result is not None:
            goal_x, goal_y, arc_points, goal_yaw = arc_result
            self._smoothed_yaw = goal_yaw
            self._last_golden_lane_points = arc_points
        else:
            self._smoothed_yaw, goal_dist = self._pick_goal_heading(
                pose.position.x, pose.position.y, yaw, self._smoothed_yaw)
            goal_yaw = self._smoothed_yaw
            goal_x = pose.position.x + goal_dist * math.cos(goal_yaw)
            goal_y = pose.position.y + goal_dist * math.sin(goal_yaw)

            # Straight-corridor centerline nudge (see module docstring) -
            # only meaningful for the straight-ray picker; the arc planner
            # already finds a centered, obstacle-aware path of its own, so
            # this is skipped whenever arc_result was used above. Bias is
            # in base_link-frame (+y=left of the vehicle's current actual
            # heading, not goal_yaw), so it's rotated by the vehicle's
            # current yaw, not goal_yaw, before being added to the goal
            # position.
            lateral_bias = self._centerline_lateral_bias(goal_dist)
            if lateral_bias is not None:
                goal_x += -math.sin(yaw) * lateral_bias
                goal_y += math.cos(yaw) * lateral_bias

        qx, qy, qz, qw = self._yaw_to_quaternion(goal_yaw)

        start = PoseStamped()
        start.header = header
        start.pose = pose

        goal_pose = PoseStamped()
        goal_pose.header = header
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        self._publish_goal_marker(
            header, goal_x, goal_y, self.carrot_marker_pub, ns="carrot",
            color=(0.1, 0.9, 1.0, 0.95))
        self._publish_golden_lane(
            header, pose.position.x, pose.position.y, goal_x, goal_y,
            arc_points=self._last_golden_lane_points)

        if arc_result is not None:
            # The chosen lane candidate (see _plan_lane_arc/golden_lane)
            # IS the driven path this cycle - dispatch its own sampled
            # curve points directly (through FollowPath/TEB, same as
            # every other path this file dispatches), bypassing RRT*/the
            # straight-ray planner entirely, instead of only handing RRT*
            # the arc's ENDPOINT as a target and letting it compute a
            # DIFFERENT (possibly not-matching) path of its own toward it.
            # _build_straight_path interpolates short segments between
            # consecutive (already-dense, ~costmap-resolution-spaced) arc
            # points with each segment's own heading, so the dispatched
            # path actually bends the same way the arc/golden lane does.
            arc_waypoints = [(pose.position.x, pose.position.y)] + list(self._last_golden_lane_points)
            path = self._build_straight_path(header, arc_waypoints)
            self._dispatch_follow_path(path)
            return

        if not self.use_global_planner:
            # latest_goal_xy is None here (that case returns earlier, via
            # _split_into_clear_segments) - no real destination to split
            # toward, just the single reactive carrot segment - unless
            # use_reactive_chain is set, in which case build a short
            # multi-waypoint chain the same way, see that param's own
            # comment and _build_reactive_chain's docstring.
            if self.use_reactive_chain:
                waypoints_xy = self._build_reactive_chain(
                    start.pose.position.x, start.pose.position.y, yaw)
                self._publish_sub_goals(header, waypoints_xy)
            else:
                waypoints_xy = [
                    (start.pose.position.x, start.pose.position.y),
                    (goal_pose.pose.position.x, goal_pose.pose.position.y)]
            path = self._build_straight_path(header, waypoints_xy)
            self._dispatch_follow_path(path)
            return

        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose
        goal.start = start
        goal.use_start = True
        goal.planner_id = self.planner_id

        self._planning_in_progress = True
        send_future = self._planner_client.send_goal_async(goal)
        send_future.add_done_callback(self._plan_goal_response)

    def _plan_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('compute_path_to_pose goal rejected', throttle_duration_sec=5.0)
            self._planning_in_progress = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._plan_result)

    def _plan_result(self, future):
        self._planning_in_progress = False
        try:
            path = future.result().result.path
        except Exception as exc:
            self.get_logger().warn(f'compute_path_to_pose failed: {exc}', throttle_duration_sec=5.0)
            return

        if not path.poses:
            self.get_logger().warn('Planner returned an empty path', throttle_duration_sec=5.0)
            return

        self._dispatch_follow_path(path)


def main(args=None):
    rclpy.init(args=args)
    node = CarrotPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
