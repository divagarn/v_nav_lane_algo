#!/usr/bin/env python3
"""RRT* (Rapidly-exploring Random Tree Star) global planner - a standalone
ROS2 action server, NOT a nav2_core::GlobalPlanner/planner_server plugin.

Why standalone: no off-the-shelf RRT planner exists for nav2 Humble
(checked: no ros-humble-nav2-rrt-planner or equivalent on apt - only
ros-iron-multidim-rrt-planner, a different ROS distro's frontier-
EXPLORATION package, not a compute_path_to_pose-compatible global
planner). Writing one as a real nav2_core plugin would mean a new C++/
CMake package + pluginlib registration + compilation; writing it as its
own Python node hosting the SAME action (nav2_msgs/action/
ComputePathToPose) carrot_path_publisher already calls is a drop-in
alternative with none of that - carrot_path_publisher.py needs ZERO code
changes, only a launch-time remap of its action client's target (see
navigate_algo_rrt_star.launch.py).

ARCHITECTURE NOTE - why planner_server still runs alongside this node in
that launch file: planner_server's compute_path_to_pose action always
exists once the node is up, regardless of which GridBased plugin is
configured - there is no way to disable just that part of it. Two nodes
both hosting an action under the exact same name ('compute_path_to_pose')
in the same namespace is a real conflict - confirmed live earlier this
project's history ("Ignoring unexpected goal response. There may be more
than one action server for the action 'compute_path_to_pose'" - seen when
stale duplicate nodes lingered). So planner_server is kept running ONLY
for the /global_costmap/costmap it builds/publishes (from
global_costmap_mapped.yaml's static+obstacle+inflation layers - the same
topic carrot_path_publisher's own _pick_goal_heading/candidate_centering
already subscribes to independently) - its OWN action server is simply
never targeted, since carrot_path_publisher's action client gets remapped
to THIS node's action name instead.

ALGORITHM: standard RRT* over the OccupancyGrid this node receives on
/global_costmap/costmap (2D, no vehicle kinematics in the search itself -
same design choice as this project's ThetaStarPlanner: the search treats
the vehicle as a point, and TEB's own min_turning_radius constraint
(teb_controller.yaml) is what actually keeps the EXECUTED trajectory
physically drivable, regardless of what the global planner proposed - see
that config's own comment for the full reasoning, unchanged here):

  1. Sample a random point (goal_bias fraction of the time: sample the
     goal directly instead, standard RRT goal-biasing).
  2. Find the nearest existing tree node (vectorized numpy distance calc,
     not a KD-tree - the local search regions here are small enough that
     this stays fast without added complexity).
  3. Steer from that nearest node toward the sample by at most step_size.
  4. Collision-check the new edge by sampling the costmap along it (any
     cell >= lethal_threshold, or actually every remaining check point,
     rejects the edge).
  5. Among existing nodes within rewire_radius of the new point, pick
     whichever gives the lowest total cost-to-come as the new node's
     parent (not just the nearest one) - this + step 6 is the "star" in
     RRT*, what makes it converge toward a good path instead of just any
     valid one.
  6. Rewire: for each of those same nearby nodes, check whether routing
     THROUGH the new node would lower ITS cost - if so, re-parent it.
     (Does not propagate cost improvements to further descendants - a
     standard simplification in lightweight RRT* implementations; the
     resulting path is still collision-free and a real (if not perfectly
     globally optimal) improvement over plain RRT.)
  7. Cost isn't just Euclidean edge length - see clearance_weight in
     rrt_star_planner.yaml for why (same centering rationale as
     ThetaStarPlanner's w_traversal_cost).
  8. Repeat until max_iterations or planning_timeout_sec, whichever
     first - keep the best (lowest-cost) path found to anywhere within
     goal_tolerance of the actual goal.

Returns an empty nav_msgs/Path (goal_handle.abort()) if the start or goal
itself is on/past an impassable cell, or if no path to the goal was found
within budget - same failure contract planner_server's own action already
has, so carrot_path_publisher's existing "Planner returned an empty path"
handling needs no changes either.
"""

import math
import random
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose


class RRTStarPlanner(Node):

    def __init__(self):
        super().__init__('rrt_star_planner')

        self.max_iterations = self.declare_parameter('max_iterations', 3000).value
        self.planning_timeout_sec = self.declare_parameter('planning_timeout_sec', 2.0).value
        self.goal_bias = self.declare_parameter('goal_bias', 0.1).value
        self.step_size = self.declare_parameter('step_size', 1.0).value
        self.rewire_radius = self.declare_parameter('rewire_radius', 2.0).value
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.5).value
        self.clearance_weight = self.declare_parameter('clearance_weight', 4.0).value
        self.lethal_threshold = self.declare_parameter('lethal_threshold', 90).value
        self.sample_margin = self.declare_parameter('sample_margin', 6.0).value
        # 1 (default): single RRT* search per goal request, same as always.
        # >1: run that many independent searches for the SAME start/goal
        # and keep whichever result has the lowest total cost (the same
        # length + clearance_weight cost _plan already computes - so "best"
        # already accounts for staying centered/away from walls, not just
        # raw path length). RRT* is randomized - a single run can, by
        # chance, produce a noticeably worse tree through a tight turn;
        # running it several times and keeping the best is the standard
        # mitigation. planning_timeout_sec is divided across the trials
        # (not multiplied) so total worst-case latency for one goal request
        # stays about the same as a single run at num_trials=1, rather than
        # growing to num_trials x planning_timeout_sec.
        self.num_trials = self.declare_parameter('num_trials', 1).value
        # Defaults to the global costmap (planner_server's, static+obstacle+
        # inflation, full saved map) for backward compatibility, but
        # navigate_algo_rrt_star.launch.py overrides this to
        # /local_costmap/costmap (controller_server's, rolling 30x30m
        # window, live sensor data only, NO static map layer - see that
        # yaml's own comment) - trades map-wide awareness for a costmap
        # that reacts to newly-sensed obstacles as fast as the sensor
        # pipeline itself updates, not just at this node's own replan
        # cadence. carrot_path_publisher's own costmap_topic parameter
        # should be set to the SAME topic in that case, so its heading-
        # picker's raycasts see the same obstacles this planner does.
        self.costmap_topic = self.declare_parameter(
            'costmap_topic', '/global_costmap/costmap').value

        self.latest_costmap = None
        costmap_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            OccupancyGrid, self.costmap_topic, self._costmap_cb, costmap_qos)

        self.path_pub = self.create_publisher(Path, '/rrt_star_plan', 1)

        # A plain launch-time `remappings=[('compute_path_to_pose', ...)]`
        # on this node does NOT rename the ActionServer - confirmed live:
        # `ros2 node info` still showed /compute_path_to_pose after that
        # remap, unlike a topic/service remap on the same node, which DOES
        # take effect. Declaring the actual name as a parameter instead and
        # using it directly here sidesteps that rclpy behavior entirely.
        self.action_name = self.declare_parameter(
            'action_name', 'compute_path_to_pose').value
        self._action_server = ActionServer(
            self, ComputePathToPose, self.action_name, self._execute_callback)

        self._rng = random.Random()

        self.get_logger().info(
            f'RRT* planner initialized (max_iterations={self.max_iterations}, '
            f'num_trials={self.num_trials}, '
            f'planning_timeout_sec={self.planning_timeout_sec}, step_size={self.step_size}m, '
            f'rewire_radius={self.rewire_radius}m, clearance_weight={self.clearance_weight}, '
            f'costmap={self.costmap_topic}, '
            f'hosting {self.action_name} - set action_name if planner_server\'s own '
            f'action of the same name is also present)'
        )

    def _costmap_cb(self, msg):
        self.latest_costmap = msg

    def _cell_value(self, grid, info, x, y):
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        if gx < 0 or gx >= info.width or gy < 0 or gy >= info.height:
            return 100  # off the known map - treat as impassable, not free
        return grid[gy, gx]

    def _is_free(self, grid, info, x, y):
        v = self._cell_value(grid, info, x, y)
        return 0 <= v < self.lethal_threshold

    def _collision_free(self, grid, info, p1, p2):
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        steps = max(1, int(dist / (info.resolution * 2.0)))
        for i in range(steps + 1):
            t = i / steps
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            if not self._is_free(grid, info, x, y):
                return False
        return True

    def _edge_cost(self, grid, info, p1, p2):
        """Euclidean length plus a clearance penalty (quadratic in the
        sampled costmap value along the edge) - see this module's own
        docstring / rrt_star_planner.yaml's clearance_weight comment for
        why this, not raw length alone, is what biases the result toward
        the corridor center."""
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if self.clearance_weight == 0.0:
            return dist
        steps = max(1, int(dist / (info.resolution * 2.0)))
        penalty = 0.0
        for i in range(steps + 1):
            t = i / steps
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            v = max(self._cell_value(grid, info, x, y), 0)
            penalty += self.clearance_weight * (v * v) / (100.0 * 100.0)
        return dist + penalty

    def _plan(self, start, goal, timeout_sec=None):
        """RRT* search. start/goal are (x, y) tuples in the costmap's own
        frame (matches the costmap message's header.frame_id - 'odom'
        throughout this workspace, same as carrot_path_publisher assumes).
        timeout_sec overrides self.planning_timeout_sec for this call when
        given - see num_trials' own comment for why (dividing the budget
        across multiple trial runs).

        Returns (list of (x, y) waypoints from start to goal, total cost of
        that path), or (None, None) if no path was found within budget."""
        if timeout_sec is None:
            timeout_sec = self.planning_timeout_sec
        msg = self.latest_costmap
        info = msg.info
        grid = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)

        if not self._is_free(grid, info, *start):
            self.get_logger().warn('RRT*: start pose is on/past an impassable cell.')
            return None, None
        if not self._is_free(grid, info, *goal):
            self.get_logger().warn('RRT*: goal pose is on/past an impassable cell.')
            return None, None

        min_x = min(start[0], goal[0]) - self.sample_margin
        max_x = max(start[0], goal[0]) + self.sample_margin
        min_y = min(start[1], goal[1]) - self.sample_margin
        max_y = max(start[1], goal[1]) + self.sample_margin

        nodes = [start]
        parents = [-1]
        costs = [0.0]
        nodes_arr = np.array([start], dtype=np.float64)

        best_goal_idx = None
        best_goal_cost = float('inf')

        t_start = time.time()
        for _ in range(self.max_iterations):
            if time.time() - t_start > timeout_sec:
                break

            if self._rng.random() < self.goal_bias:
                sample = goal
            else:
                sample = (self._rng.uniform(min_x, max_x), self._rng.uniform(min_y, max_y))

            deltas = nodes_arr - np.array(sample)
            dists_sq = np.einsum('ij,ij->i', deltas, deltas)
            nearest_idx = int(np.argmin(dists_sq))
            nearest = nodes[nearest_idx]

            d = math.hypot(sample[0] - nearest[0], sample[1] - nearest[1])
            if d < 1e-6:
                continue
            step = min(self.step_size, d)
            new_pt = (
                nearest[0] + step * (sample[0] - nearest[0]) / d,
                nearest[1] + step * (sample[1] - nearest[1]) / d,
            )

            if not self._is_free(grid, info, *new_pt):
                continue
            if not self._collision_free(grid, info, nearest, new_pt):
                continue

            deltas = nodes_arr - np.array(new_pt)
            dists_sq = np.einsum('ij,ij->i', deltas, deltas)
            near_indices = np.where(dists_sq <= self.rewire_radius ** 2)[0].tolist()

            best_parent = nearest_idx
            best_cost = costs[nearest_idx] + self._edge_cost(grid, info, nearest, new_pt)
            for i in near_indices:
                if i == nearest_idx:
                    continue
                if self._collision_free(grid, info, nodes[i], new_pt):
                    c = costs[i] + self._edge_cost(grid, info, nodes[i], new_pt)
                    if c < best_cost:
                        best_cost = c
                        best_parent = i

            new_idx = len(nodes)
            nodes.append(new_pt)
            parents.append(best_parent)
            costs.append(best_cost)
            nodes_arr = np.vstack([nodes_arr, new_pt])

            for i in near_indices:
                if i == best_parent:
                    continue
                if self._collision_free(grid, info, new_pt, nodes[i]):
                    c = best_cost + self._edge_cost(grid, info, new_pt, nodes[i])
                    if c < costs[i]:
                        parents[i] = new_idx
                        costs[i] = c

            if math.hypot(new_pt[0] - goal[0], new_pt[1] - goal[1]) <= self.goal_tolerance:
                if best_cost < best_goal_cost:
                    best_goal_cost = best_cost
                    best_goal_idx = new_idx

        if best_goal_idx is None:
            self.get_logger().warn(
                f'RRT*: no path found to goal within {self.max_iterations} iterations / '
                f'{timeout_sec}s ({len(nodes)} nodes explored).')
            return None, None

        path = []
        idx = best_goal_idx
        while idx != -1:
            path.append(nodes[idx])
            idx = parents[idx]
        path.reverse()
        path.append(goal)
        return path, best_goal_cost

    def _build_path_msg(self, waypoints_xy, header):
        path = Path()
        path.header = header
        for seg_idx in range(len(waypoints_xy) - 1):
            (sx, sy), (ex, ey) = waypoints_xy[seg_idx], waypoints_xy[seg_idx + 1]
            yaw = math.atan2(ey - sy, ex - sx)
            qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = sx
            pose.pose.position.y = sy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)
        if waypoints_xy:
            last = PoseStamped()
            last.header = header
            last.pose.position.x = waypoints_xy[-1][0]
            last.pose.position.y = waypoints_xy[-1][1]
            if path.poses:
                last.pose.orientation = path.poses[-1].pose.orientation
            else:
                last.pose.orientation.w = 1.0
            path.poses.append(last)
        return path

    def _execute_callback(self, goal_handle):
        request = goal_handle.request
        result = ComputePathToPose.Result()

        if self.latest_costmap is None:
            self.get_logger().warn('RRT*: no costmap received yet - aborting.')
            goal_handle.abort()
            return result

        start_pose = request.start.pose if request.use_start else None
        if start_pose is None:
            self.get_logger().warn(
                'RRT*: use_start=False (current-pose start) is not supported by this node - '
                'carrot_path_publisher always sets use_start=True, so this should not happen.')
            goal_handle.abort()
            return result

        start = (start_pose.position.x, start_pose.position.y)
        goal = (request.goal.pose.position.x, request.goal.pose.position.y)
        header = self.latest_costmap.header

        # num_trials=1 (default): identical to before - one _plan() call at
        # the full planning_timeout_sec. num_trials>1: run that many
        # independent searches, budget divided so total worst-case latency
        # doesn't grow with num_trials - see that param's own comment for
        # why. Keep whichever trial's path has the lowest total cost
        # (already accounts for the clearance penalty, not just length).
        per_trial_timeout = self.planning_timeout_sec / self.num_trials
        best_waypoints = None
        best_cost = float('inf')
        t0 = time.time()
        for _ in range(self.num_trials):
            waypoints, cost = self._plan(start, goal, timeout_sec=per_trial_timeout)
            if waypoints is not None and cost < best_cost:
                best_cost = cost
                best_waypoints = waypoints
        planning_time = time.time() - t0
        waypoints = best_waypoints

        if self.num_trials > 1:
            if waypoints is not None:
                self.get_logger().info(
                    f'RRT*: {self.num_trials} trials ({per_trial_timeout:.2f}s each), '
                    f'best cost={best_cost:.2f}')
            else:
                self.get_logger().info(
                    f'RRT*: all {self.num_trials} trials failed to reach the goal.')

        if waypoints is None:
            goal_handle.abort()
            return result

        path_msg = self._build_path_msg(waypoints, header)
        self.path_pub.publish(path_msg)

        result.path = path_msg
        result.planning_time.sec = int(planning_time)
        result.planning_time.nanosec = int((planning_time % 1.0) * 1e9)
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RRTStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
