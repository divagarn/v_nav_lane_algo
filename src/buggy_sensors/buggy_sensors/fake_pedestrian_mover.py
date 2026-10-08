#!/usr/bin/env python3
"""Sim-only test prop mover: repositions a plain (non-actor) Gazebo model
along a fixed rectangular loop, for exercising dynamic_obstacle_detector_
node's moving-obstacle classification against something that actually
moves.

Two Gazebo <actor>-based approaches were tried and abandoned before this
(see office_track.world's own comment on the walking_person_loop model for
the full story): a <script><trajectory> actor only animates the visual
mesh, never moving its physics/collision pose (the LiDAR never saw it at
all); libActorPlugin.so does move the real collision, but turns at a fixed
fraction of the remaining angle PER PHYSICS STEP rather than per second of
real time, which combined with this project's reduced-rate physics and a
typically-low real-time factor on a dev machine made even a single turn
take minutes of real time - and even then it only wanders a hardcoded
random box, not a precise loop.

This node sidesteps both problems: it computes an exact point on the loop
from elapsed TIME (this node's own clock, so use_sim_time keeps it
correctly tied to simulated time) and calls Gazebo's own /set_entity_state
service directly - no physics-engine turning/convergence involved at all.
"""

import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose


class FakePedestrianMover(Node):

    def __init__(self):
        super().__init__('fake_pedestrian_mover')

        self.entity_name = self.declare_parameter('entity_name', 'walking_person_loop').value
        self.z = self.declare_parameter('z', 0.85).value
        self.update_rate_hz = self.declare_parameter('update_rate_hz', 10.0).value

        # Same 4-corner rectangle originally specified: (2,-3) -> (2,-1) ->
        # (5,-1) -> (5,-3) -> back to (2,-3), ~1 m/s (matches the speed
        # used in this project's own offline verification of the moving-
        # obstacle classifier). Cumulative times define a piecewise-linear
        # path; the loop repeats every self.corners[-1][0] seconds.
        self.corners = [
            (0.0, 2.0, -3.0),
            (2.0, 2.0, -1.0),
            (5.0, 5.0, -1.0),
            (7.0, 5.0, -3.0),
            (10.0, 2.0, -3.0),
        ]

        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / self.update_rate_hz, self.on_timer)

        self.get_logger().info(
            f"fake_pedestrian_mover: looping '{self.entity_name}' around a "
            f'{self._loop_length():.1f}m rectangle, period '
            f'{self.corners[-1][0]:.1f}s, via /set_entity_state.')

    def _loop_length(self):
        total = 0.0
        for i in range(len(self.corners) - 1):
            _, x0, y0 = self.corners[i]
            _, x1, y1 = self.corners[i + 1]
            total += math.hypot(x1 - x0, y1 - y0)
        return total

    def _position_at(self, t):
        period = self.corners[-1][0]
        t = t % period
        for i in range(len(self.corners) - 1):
            t0, x0, y0 = self.corners[i]
            t1, x1, y1 = self.corners[i + 1]
            if t0 <= t <= t1:
                frac = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                x = x0 + frac * (x1 - x0)
                y = y0 + frac * (y1 - y0)
                yaw = math.atan2(y1 - y0, x1 - x0)
                return x, y, yaw
        # Shouldn't reach here given the modulo above, but fail safe to
        # the loop's start point rather than raising mid-timer-callback.
        _, x0, y0 = self.corners[0]
        return x0, y0, 0.0

    def on_timer(self):
        if not self.client.service_is_ready():
            # Gazebo's gazebo_ros_state plugin may still be coming up -
            # skip this cycle rather than block the timer waiting for it.
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        x, y, yaw = self._position_at(elapsed)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = self.z
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        state = EntityState()
        state.name = self.entity_name
        state.pose = pose
        state.reference_frame = 'world'

        request = SetEntityState.Request()
        request.state = state
        self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = FakePedestrianMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
