#!/usr/bin/env python3
"""Interactive CLI to edit a pose graph loaded via edit_pose_graph.launch.py.

`list`/`show`/`save`/`undo` are fully confirmed reliable (read straight
from /slam_toolbox/graph_visualization and the documented slam_toolbox
services). `move` is NOT - see below.

CONFIRMED LIVE: `move` DOES NOT WORK from this process, and no amount of
retrying different marker-name strings will fix it. slam_toolbox's real
"move a vertex" mechanism is the interactive_markers protocol - publish
visualization_msgs/InteractiveMarkerFeedback (MOUSE_DOWN -> POSE_UPDATE ->
MOUSE_UP) referencing a marker by name, then call
/slam_toolbox/manual_loop_closure (empty request - it acts on whatever was
just fed back) to commit it. Tested this directly: 5 different plausible
marker-name conventions ('5', 'node_5', 'vertex_5', 'marker_5', 'pose_5')
against a real vertex, each followed by manual_loop_closure - EVERY one
returned success from the service call, with ZERO actual position change,
identically across all 5. That consistency rules out "wrong name format"
as the cause: /slam_toolbox/update (the topic that would show the real
marker names) also never sent this process a single message in 20+
seconds, despite `ros2 topic info` confirming a live publisher. Together
this means slam_toolbox's interactive marker server never recognizes a
bare script/CLI process as a real client at all - RViz's own client must
be doing some session-establishment step this tool can't trigger, and
that's a protocol-level gap this tool cannot work around without
slam_toolbox's own source (not available on this machine, only the
compiled binary).

`move` is kept below anyway (marked accordingly) in case a future
slam_toolbox version or a real client-session workaround changes this -
its own self-verification will immediately reveal if it starts working.
For actually moving vertices today, RViz's own Interact tool (see
edit_pose_graph.launch.py's docstring) is the confirmed-working path.

Usage (after launching edit_pose_graph.launch.py in another terminal):
    ros2 run buggy_mapping pose_graph_editor

Commands:
    list                       - list all vertex IDs and positions
    show <id>                  - show one vertex's exact position
    move <id> <x> <y> [yaw_deg]
                                - move vertex <id> to (x, y[, yaw]), commit
                                  as a loop closure, then verify the result
    undo                       - clear pending changes (before a move's
                                  own commit - see manual_loop_closure's
                                  own comment; cannot undo an already-
                                  committed move)
    save <path>                - serialize the pose graph to <path>
                                  (no extension - writes <path>.data/
                                  .posegraph). Save to a NEW name, not
                                  over the map you loaded.
    refresh                    - re-fetch the current vertex list
    quit / exit
"""
import cmd
import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from visualization_msgs.msg import InteractiveMarkerFeedback, MarkerArray
from slam_toolbox.srv import LoopClosure, SerializePoseGraph


class PoseGraphEditorNode(Node):

    def __init__(self):
        super().__init__('pose_graph_editor')
        self.vertices = {}  # id -> (x, y)
        self._lock = threading.Lock()
        self.create_subscription(
            MarkerArray, '/slam_toolbox/graph_visualization', self._on_graph, 1)
        self.feedback_pub = self.create_publisher(
            InteractiveMarkerFeedback, '/slam_toolbox/feedback', 10)
        self.loop_closure_cli = self.create_client(LoopClosure, '/slam_toolbox/manual_loop_closure')
        self.clear_changes_cli = self.create_client(Empty, '/slam_toolbox/clear_changes')
        self.serialize_cli = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')

    def _on_graph(self, msg):
        verts = {}
        for m in msg.markers:
            if m.ns == 'slam_toolbox':
                verts[m.id] = (m.pose.position.x, m.pose.position.y)
        with self._lock:
            self.vertices = verts

    def snapshot(self):
        with self._lock:
            return dict(self.vertices)

    def _yaw_to_quat(self, yaw_deg):
        yaw = math.radians(yaw_deg)
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def send_feedback_sequence(self, vertex_id, x, y, yaw_deg):
        """MOUSE_DOWN -> POSE_UPDATE -> MOUSE_UP, marker_name = str(vertex_id)
        - see this module's own docstring for why that name is a guess."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        qx, qy, qz, qw = self._yaw_to_quat(yaw_deg)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        for event_type in (
            InteractiveMarkerFeedback.MOUSE_DOWN,
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            fb = InteractiveMarkerFeedback()
            fb.header.frame_id = 'map'
            fb.header.stamp = self.get_clock().now().to_msg()
            fb.client_id = 'pose_graph_editor_tool'
            fb.marker_name = str(vertex_id)
            fb.control_name = ''
            fb.event_type = event_type
            fb.pose = pose
            self.feedback_pub.publish(fb)
            time.sleep(0.05)

    def call_manual_loop_closure(self, timeout_sec=3.0):
        if not self.loop_closure_cli.wait_for_service(timeout_sec=timeout_sec):
            return False, 'service /slam_toolbox/manual_loop_closure not available'
        future = self.loop_closure_cli.call_async(LoopClosure.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return False, 'manual_loop_closure call timed out'
        return True, None

    def call_clear_changes(self, timeout_sec=3.0):
        if not self.clear_changes_cli.wait_for_service(timeout_sec=timeout_sec):
            return False, 'service /slam_toolbox/clear_changes not available'
        future = self.clear_changes_cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return False, 'clear_changes call timed out'
        return True, None

    def call_serialize(self, filename, timeout_sec=5.0):
        if not self.serialize_cli.wait_for_service(timeout_sec=timeout_sec):
            return False, 'service /slam_toolbox/serialize_map not available'
        req = SerializePoseGraph.Request()
        req.filename = filename
        future = self.serialize_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return False, 'serialize_map call timed out'
        resp = future.result()
        if resp.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            return False, f'serialize_map returned failure code {resp.result}'
        return True, None


class Shell(cmd.Cmd):
    intro = (
        "Pose graph editor. 'list' to see vertices, 'help' for all commands.\n"
        "Reminder: 'move' is a best-effort mechanism (see this tool's module "
        "docstring) - it always tells you whether the edit actually took effect, "
        "checked against the real graph, not just whether the calls succeeded."
    )
    prompt = 'posegraph> '

    def __init__(self, node):
        super().__init__()
        self.node = node

    def do_list(self, arg):
        """list - show all vertex IDs and positions."""
        verts = self.node.snapshot()
        if not verts:
            print('No graph data received yet - is edit_pose_graph.launch.py running?')
            return
        for vid in sorted(verts):
            x, y = verts[vid]
            print(f'  {vid:5d}  x={x:8.3f}  y={y:8.3f}')
        print(f'{len(verts)} vertices total.')

    def do_show(self, arg):
        """show <id> - show one vertex's exact position."""
        try:
            vid = int(arg.strip())
        except ValueError:
            print('Usage: show <id>')
            return
        verts = self.node.snapshot()
        if vid not in verts:
            print(f'No vertex {vid} in the current graph.')
            return
        x, y = verts[vid]
        print(f'vertex {vid}: x={x:.3f} y={y:.3f}')

    def do_move(self, arg):
        """move <id> <x> <y> [yaw_deg] - move a vertex, commit, and verify.
        CONFIRMED NOT WORKING from this process (see module docstring) -
        kept in case a future slam_toolbox build changes this; the
        verification step below will say so immediately either way."""
        parts = arg.split()
        if len(parts) not in (3, 4):
            print('Usage: move <id> <x> <y> [yaw_deg]')
            return
        try:
            vid = int(parts[0])
            x, y = float(parts[1]), float(parts[2])
            yaw_deg = float(parts[3]) if len(parts) == 4 else 0.0
        except ValueError:
            print('Usage: move <id> <x> <y> [yaw_deg] (all numeric)')
            return

        before = self.node.snapshot()
        if vid not in before:
            print(f'No vertex {vid} in the current graph - use "list" or "refresh" first.')
            return
        old_x, old_y = before[vid]

        print(f'Sending feedback for vertex {vid}: ({old_x:.3f},{old_y:.3f}) -> ({x:.3f},{y:.3f})...')
        self.node.send_feedback_sequence(vid, x, y, yaw_deg)
        time.sleep(0.3)  # let slam_toolbox process the feedback before committing

        ok, err = self.node.call_manual_loop_closure()
        if not ok:
            print(f'manual_loop_closure call failed: {err}')
            return

        time.sleep(0.5)  # let the graph re-publish after the optimizer re-solves
        rclpy.spin_once(self.node, timeout_sec=1.0)
        after = self.node.snapshot()
        new_x, new_y = after.get(vid, (old_x, old_y))
        moved = math.hypot(new_x - old_x, new_y - old_y)
        target_dist = math.hypot(new_x - x, new_y - y)

        if moved < 0.01:
            print(
                f'NO CHANGE (as expected - see this module\'s docstring): vertex {vid} '
                f'is still at ({new_x:.3f},{new_y:.3f}). This mechanism was confirmed not '
                'to work from a bare script/CLI process across 5 different marker-name '
                'attempts - use RViz\'s own Interact tool instead '
                '(see edit_pose_graph.launch.py\'s docstring).')
        elif target_dist < 0.1:
            print(f'Moved as requested: vertex {vid} is now at ({new_x:.3f},{new_y:.3f}).')
        else:
            print(
                f'Vertex {vid} moved to ({new_x:.3f},{new_y:.3f}) - close to requested but not '
                f'exact ({target_dist:.3f}m off), likely nudged further by the graph optimizer '
                'resolving other constraints. This is expected, not necessarily wrong.')

    def do_undo(self, arg):
        """undo - clear pending (not-yet-committed) changes."""
        ok, err = self.node.call_clear_changes()
        print('Cleared pending changes.' if ok else f'clear_changes failed: {err}')

    def do_save(self, arg):
        """save <path> - serialize the pose graph to <path> (no extension)."""
        path = arg.strip()
        if not path:
            print('Usage: save <path-without-extension>')
            return
        ok, err = self.node.call_serialize(path)
        print(f'Saved to {path}.data/.posegraph' if ok else f'Save failed: {err}')

    def do_refresh(self, arg):
        """refresh - re-fetch the current vertex list."""
        rclpy.spin_once(self.node, timeout_sec=1.0)
        print(f'{len(self.node.snapshot())} vertices currently known.')

    def do_quit(self, arg):
        """quit - exit the tool."""
        return True

    def do_exit(self, arg):
        """exit - exit the tool."""
        return True

    def do_EOF(self, arg):
        """Handle stdin closing (piped input, Ctrl-D) - exit cleanly instead
        of looping forever re-reading EOF as an unrecognized command."""
        print()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphEditorNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(1.0)  # let the first graph_visualization message arrive

    try:
        Shell(node).cmdloop()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
