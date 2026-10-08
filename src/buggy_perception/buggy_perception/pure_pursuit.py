"""Standalone pure-pursuit steering geometry - no ROS dependency.

Identical copy of buggy_nav/buggy_nav/pure_pursuit.py - duplicated here
rather than cross-imported to avoid a circular package dependency
(buggy_nav already depends on buggy_perception for the real-launch
actuation nodes; this stays consistent with how this workspace already
handles other small shared logic, e.g. the UART protocol constants
duplicated across uart_interface.py/uart_bridge.py/check_uart.py/etc.
rather than centralized). Keep both copies in sync if this changes.

Given the vehicle's current pose and a path (or a single carrot/goal
point), computes the steering angle needed to curve toward it - pure
geometry, the classic Pure Pursuit algorithm. Deliberately does NOT need
a speed input at all: unlike the atan2(angular_z*wheelbase, speed)
recovery this workspace's speed_governor.py/steering_uart_bridge.py used
to use (which requires knowing which speed TEB's Twist was computed for -
the exact ambiguity that caused a real steering spike bug earlier), this
works directly from (pose, target point) - nothing to get inconsistent.

Bicycle-model curvature to reach a target point at (lookahead_dist, alpha)
in the vehicle's own frame (alpha = angle from vehicle heading to the
target, lookahead_dist = straight-line distance to it):

    curvature = 2*sin(alpha) / lookahead_dist
    steering_angle = atan(wheelbase * curvature)

This is the same relationship used everywhere else in this workspace
(min_turning_radius = wheelbase / tan(max_steering_angle)), just solved
in the other direction: target point -> required curvature -> steering
angle, instead of commanded Twist -> curvature -> steering angle.
"""
import math


def world_to_vehicle_frame(px, py, pose_x, pose_y, pose_yaw):
    """Transform a world-frame point into the vehicle's own frame
    (x forward, y left), given the vehicle's world pose."""
    dx = px - pose_x
    dy = py - pose_y
    cos_yaw = math.cos(-pose_yaw)
    sin_yaw = math.sin(-pose_yaw)
    x_v = dx * cos_yaw - dy * sin_yaw
    y_v = dx * sin_yaw + dy * cos_yaw
    return x_v, y_v


def find_lookahead_point(pose_x, pose_y, pose_yaw, path, lookahead_dist):
    """First path point at or beyond lookahead_dist from the current
    position, walking forward along the path. Falls back to the path's
    last point if the whole path is shorter than lookahead_dist (matches
    carrot_path_publisher's own candidate_advance clamp-to-remaining-path
    behavior - never overshoots past the end)."""
    if not path:
        return None
    for (px, py) in path:
        dist = math.hypot(px - pose_x, py - pose_y)
        if dist >= lookahead_dist:
            return (px, py)
    return path[-1]


def compute_steering_angle(pose_x, pose_y, pose_yaw, target_x, target_y,
                            wheelbase, max_steering_deg):
    """Pure-pursuit steering angle (degrees, +right/-left - this
    project's convention, matching the real UART/SteerSpeed hardware,
    OPPOSITE of the standard math/ROS convention angular.z uses (+left) -
    see steering_uart_bridge.py's own sign note) to curve toward
    (target_x, target_y) from the given pose. Returns a dict with the
    angle plus the intermediate values, for test/debug visibility rather
    than a bare number. alpha_deg/curvature are left in the standard
    (+left) math convention - only steering_deg is flipped, at the very
    end, to this project's convention."""
    x_v, y_v = world_to_vehicle_frame(target_x, target_y, pose_x, pose_y, pose_yaw)
    lookahead_dist = math.hypot(x_v, y_v)
    if lookahead_dist < 1e-6:
        return {'steering_deg': 0.0, 'lookahead_dist': 0.0, 'alpha_deg': 0.0,
                'curvature': 0.0, 'clamped': False, 'behind': False}

    alpha = math.atan2(y_v, x_v)
    behind = x_v < 0.0
    curvature = 2.0 * math.sin(alpha) / lookahead_dist
    steering_rad = math.atan(wheelbase * curvature)
    # Flip here: steering_rad above is in the standard (+left) convention
    # (positive alpha/curvature = target to the left = positive angular.z
    # in ROS's own convention). This project's steering_deg is +right
    # instead (matches SteerSpeed/the real UART hardware - see this
    # function's own docstring) - same flip the old atan2(-angular_z*...)
    # formula applied to angular_z before this module existed.
    steering_deg = -math.degrees(steering_rad)
    clamped = abs(steering_deg) > max_steering_deg
    steering_deg = max(-max_steering_deg, min(max_steering_deg, steering_deg))

    return {
        'steering_deg': steering_deg,
        'lookahead_dist': lookahead_dist,
        'alpha_deg': math.degrees(alpha),
        'curvature': curvature,
        'clamped': clamped,
        'behind': behind,
    }


def compute_steering_for_path(pose_x, pose_y, pose_yaw, path, wheelbase,
                               max_steering_deg, lookahead_dist):
    """Full pipeline: pick the lookahead point from a path, then compute
    steering toward it."""
    target = find_lookahead_point(pose_x, pose_y, pose_yaw, path, lookahead_dist)
    if target is None:
        return {'steering_deg': 0.0, 'lookahead_dist': 0.0, 'alpha_deg': 0.0,
                'curvature': 0.0, 'clamped': False, 'behind': False, 'target': None}
    result = compute_steering_angle(
        pose_x, pose_y, pose_yaw, target[0], target[1], wheelbase, max_steering_deg)
    result['target'] = target
    return result
