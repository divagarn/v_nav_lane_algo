#!/usr/bin/env python3
"""Standalone (no ROS dependency at runtime) click-and-drag editor for the
Lanelet2 OSM-XML files pose_graph_to_lanelet.py produces.

WHY NO ROS DEPENDENCY MATTERS: editing the LIVE pose graph via slam_toolbox
required establishing a real interactive-markers client session that no
external tool could reach (confirmed live - see chat history). This tool
sidesteps that completely by operating on a plain XML FILE instead of a
live ROS service - there is no protocol wall to hit, dragging a point here
is just a mouse-event callback on a matplotlib plot, and saving is just
rewriting the same XML structure with new coordinates.

Usage:
    ros2 run buggy_mapping lanelet_editor /path/to/pose_graph_lanelet.osm

Controls:
    Left-click + drag a point (either boundary) to move it.
    's' key or the Save button - write to <input>_edited.osm (never
        overwrites the file you opened).
    'r' key or the Reset button - reload original positions.
    Close the window when done.
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.widgets import Button  # noqa: E402


class LaneletData:
    """Parses/rewrites the exact structure pose_graph_to_lanelet.py
    produces: two 'line_thin' ways (left/right boundary), one 'lanelet'
    relation linking them by role. Doesn't attempt to support arbitrary
    third-party Lanelet2 files - only this project's own export format."""

    def __init__(self, path):
        self.path = path
        self.tree = ET.parse(path)
        self.root = self.tree.getroot()
        self.nodes = {}  # id -> [x, y] (mutable list so drags update in place)
        for node in self.root.findall('node'):
            nid = node.get('id')
            x = y = 0.0
            for tag in node.findall('tag'):
                if tag.get('k') == 'local_x':
                    x = float(tag.get('v'))
                elif tag.get('k') == 'local_y':
                    y = float(tag.get('v'))
            self.nodes[nid] = [x, y]

        ways = self.root.findall('way')
        if len(ways) < 2:
            raise ValueError(f'Expected at least 2 ways (left/right boundary), found {len(ways)}')
        self.left_way = ways[0]
        self.right_way = ways[1]

    def way_node_ids(self, way):
        return [nd.get('ref') for nd in way.findall('nd')]

    def way_points(self, way):
        return [self.nodes[nid] for nid in self.way_node_ids(way)]

    def save(self, out_path):
        for node in self.root.findall('node'):
            nid = node.get('id')
            x, y = self.nodes[nid]
            for tag in node.findall('tag'):
                if tag.get('k') == 'local_x':
                    tag.set('v', f'{x:.4f}')
                elif tag.get('k') == 'local_y':
                    tag.set('v', f'{y:.4f}')
        rough = ET.tostring(self.root, encoding='unicode')
        pretty = minidom.parseString(rough).toprettyxml(indent='  ')
        with open(out_path, 'w') as f:
            f.write(pretty)


class DraggableEditor:
    PICK_RADIUS_PX = 12

    def __init__(self, data: LaneletData):
        self.data = data
        self.original_positions = {nid: list(xy) for nid, xy in data.nodes.items()}
        self.dragging_nid = None

        self.fig, self.ax = plt.subplots(figsize=(10, 9))
        self.fig.subplots_adjust(bottom=0.12)
        self.ax.set_aspect('equal')
        self.ax.set_title(
            'Left-click + drag a point to move it. Press "s" to save, "r" to reset.')

        left_ids = data.way_node_ids(data.left_way)
        right_ids = data.way_node_ids(data.right_way)
        self.left_ids = left_ids
        self.right_ids = right_ids

        (self.left_line,) = self.ax.plot([], [], 'b.-', label='Left boundary', markersize=6)
        (self.right_line,) = self.ax.plot([], [], 'r.-', label='Right boundary', markersize=6)
        self.ax.legend(loc='upper right')
        self.ax.grid(True, alpha=0.3)
        self._redraw()

        save_ax = self.fig.add_axes([0.7, 0.02, 0.1, 0.05])
        reset_ax = self.fig.add_axes([0.81, 0.02, 0.1, 0.05])
        self.save_button = Button(save_ax, 'Save')
        self.reset_button = Button(reset_ax, 'Reset')
        self.save_button.on_clicked(lambda event: self.save())
        self.reset_button.on_clicked(lambda event: self.reset())

        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    def _redraw(self):
        lx = [self.data.nodes[nid][0] for nid in self.left_ids]
        ly = [self.data.nodes[nid][1] for nid in self.left_ids]
        rx = [self.data.nodes[nid][0] for nid in self.right_ids]
        ry = [self.data.nodes[nid][1] for nid in self.right_ids]
        self.left_line.set_data(lx, ly)
        self.right_line.set_data(rx, ry)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw_idle()

    def _nearest_node(self, event):
        if event.xdata is None or event.ydata is None:
            return None
        best_nid, best_dist_px = None, None
        for nid in self.left_ids + self.right_ids:
            x, y = self.data.nodes[nid]
            disp = self.ax.transData.transform((x, y))
            dist_px = ((disp[0] - event.x) ** 2 + (disp[1] - event.y) ** 2) ** 0.5
            if best_dist_px is None or dist_px < best_dist_px:
                best_nid, best_dist_px = nid, dist_px
        if best_dist_px is not None and best_dist_px <= self.PICK_RADIUS_PX:
            return best_nid
        return None

    def on_press(self, event):
        if event.button != 1 or event.inaxes != self.ax:
            return
        self.dragging_nid = self._nearest_node(event)

    def on_motion(self, event):
        if self.dragging_nid is None or event.inaxes != self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        self.data.nodes[self.dragging_nid][0] = event.xdata
        self.data.nodes[self.dragging_nid][1] = event.ydata
        self._redraw()

    def on_release(self, event):
        self.dragging_nid = None

    def on_key(self, event):
        if event.key == 's':
            self.save()
        elif event.key == 'r':
            self.reset()

    def save(self):
        base, ext = os.path.splitext(self.data.path)
        out_path = f'{base}_edited{ext}'
        self.data.save(out_path)
        print(f'Saved to {out_path}')
        self.ax.set_title(f'Saved to {out_path}')
        self.fig.canvas.draw_idle()

    def reset(self):
        for nid, xy in self.original_positions.items():
            self.data.nodes[nid][0] = xy[0]
            self.data.nodes[nid][1] = xy[1]
        self._redraw()
        print('Reset to original positions.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('osm_file', help='Path to a pose_graph_to_lanelet.py-produced .osm file')
    args = parser.parse_args()

    if not os.path.isfile(args.osm_file):
        print(f'File not found: {args.osm_file}', file=sys.stderr)
        sys.exit(1)

    data = LaneletData(args.osm_file)
    editor = DraggableEditor(data)
    plt.show()


if __name__ == '__main__':
    main()
