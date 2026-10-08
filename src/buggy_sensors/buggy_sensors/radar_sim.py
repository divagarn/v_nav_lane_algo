#!/usr/bin/env python3
"""Gazebo-fed stand-in for the real UMRR-A4 Type 171 Automotive radar
(smartmicro_ros2_radars/umrr_ros2_driver, CAN-based, sensor model
umrra4_can_v1_2_1, pub_type "target" per that package's
radar.params.template.yaml). Gazebo Classic has no native mmWave radar
sensor plugin, so this derives synthetic radar targets from the SAME
obstacle geometry the Velodyne pipeline already sees, narrowed to a
plausible forward radar FOV and coarsened into a sparse target list - a
real radar returns tens of discrete targets per cycle, not a dense LiDAR
sweep, so echoing every LiDAR point back out unchanged would look nothing
like real radar data.

Reads /non_ground_points_filtered - segment_ground (torch-based ground
removal) + self_hit_filter (both in buggy_perception, toggled via
sensors_bringup.launch.py's ground_filter argument) publish this
regardless of whether ground_filter is on or off (self_hit_filter's own
input remaps between /non_ground_points and raw /velodyne_points - see
that node's launch wiring - but its OUTPUT topic name never changes), so
this subscription is correct either way. Same behavior as the original
old-workspace version now that ground segmentation exists in this
workspace too.

Publishes on the REAL driver's exact topic names and PointCloud2 field
layout - verified directly from umrr_ros2_driver/src/smartmicro_radar_node.cpp's
RadarPoint struct (x, y, z, radial_speed, power, rcs, noise, snr,
azimuth_angle, elevation_angle, range - 11x float32, in that order) and
its can_publishers() topic names - so anything built to consume the real
driver (RViz's smart_rviz_plugin, obstacle fusion) works identically
whether fed by real hardware or this stand-in:
  smart_radar/can_targets_0       (sensor_msgs/PointCloud2)
  smart_radar/can_targetheader_0  (umrr_ros2_msgs/CanTargetHeader)

radial_speed is always 0.0 here - this track's walls are static and there
is no ground-truth velocity model for anything else in these worlds.
power/rcs/noise/snr are synthetic placeholder constants (no real RF
propagation model), chosen as plausible for a large flat reflector like a
wall, not measured or calibrated values.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from umrr_ros2_msgs.msg import CanTargetHeader

# base_link -> umrr mount offset: front bumper, centered, facing forward -
# unlike a rear-axle-measured mount, there's no real vehicle measurement
# for this sensor yet, so this is a plausible forward mount given the
# vehicle's ~2.8m chassis length (buggy.urdf.xacro's chassis_length) -
# flag/replace with a measured value once the real sensor is actually
# mounted.
RADAR_OFFSET_X = 2.2
RADAR_OFFSET_Y = 0.0
RADAR_OFFSET_Z = 0.5

# Plausible forward automotive long-range radar FOV/range - narrower and
# farther-seeing than the Velodyne's 360deg/15m-effective sweep, matching
# UMRR-A4's documented wide-angle mode characteristics (not this specific
# unit's calibrated spec sheet).
AZIMUTH_FOV_DEG = 60.0
ELEVATION_FOV_DEG = 15.0
MAX_RANGE_M = 60.0

# Real radar output is a sparse target list (tens of targets), not a dense
# point cloud - bin returns into a coarse azimuth/range grid and keep only
# the nearest point per bin, mimicking real angular/range resolution
# limits rather than echoing every LiDAR point back out as a "target".
AZIMUTH_BIN_DEG = 2.0
RANGE_BIN_M = 1.0

# Synthetic RF placeholders (no real radar-cross-section/propagation
# model) - plausible magnitude for a large flat reflector, not measured.
SYNTHETIC_RCS = 10.0
SYNTHETIC_POWER = -40.0
SYNTHETIC_NOISE = -90.0
SYNTHETIC_SNR = 25.0

# velodyne sensor frame (lidar_mount_link, per buggy.velodyne.xacro's
# frame_name) -> base_link, translation only (no rotation on the mount).
LIDAR_SENSOR_OFFSET_X = 1.73
LIDAR_SENSOR_OFFSET_Y = 0.0

RADAR_FIELD_NAMES = [
    'x', 'y', 'z', 'radial_speed', 'power', 'rcs',
    'noise', 'snr', 'azimuth_angle', 'elevation_angle', 'range',
]


class RadarSim(Node):
    def __init__(self):
        super().__init__('radar_sim')

        self.publish_rate_hz = self.declare_parameter('publish_rate_hz', 15.0).value

        self.latest_cloud_xyz = None
        self.create_subscription(
            PointCloud2, '/non_ground_points_filtered', self.cloud_callback, 2)

        self.target_pub = self.create_publisher(PointCloud2, 'smart_radar/can_targets_0', 5)
        self.header_pub = self.create_publisher(
            CanTargetHeader, 'smart_radar/can_targetheader_0', 5)

        self._broadcast_static_tf()

        self._cycle_count = 0
        self.create_timer(1.0 / self.publish_rate_hz, self.publish_targets)

        self.get_logger().info(
            f'radar_sim initialized (UMRR-A4 Type 171 Automotive stand-in): '
            f'FOV=+-{AZIMUTH_FOV_DEG}deg az / +-{ELEVATION_FOV_DEG}deg el, '
            f'max_range={MAX_RANGE_M}m, {self.publish_rate_hz}Hz, frame=umrr, '
            f'topics=smart_radar/can_targets_0 + can_targetheader_0'
        )

    def _broadcast_static_tf(self):
        broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "umrr"
        t.transform.translation.x = RADAR_OFFSET_X
        t.transform.translation.y = RADAR_OFFSET_Y
        t.transform.translation.z = RADAR_OFFSET_Z
        t.transform.rotation.w = 1.0
        broadcaster.sendTransform(t)

    def cloud_callback(self, msg):
        if msg.width == 0:
            self.latest_cloud_xyz = None
            return
        pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        xyz = pts[:, :3].copy()
        # velodyne sensor frame -> base_link -> umrr, both translation-only
        # mounts, so this collapses to one combined offset.
        xyz[:, 0] += LIDAR_SENSOR_OFFSET_X - RADAR_OFFSET_X
        xyz[:, 1] += LIDAR_SENSOR_OFFSET_Y - RADAR_OFFSET_Y
        xyz[:, 2] += -RADAR_OFFSET_Z
        self.latest_cloud_xyz = xyz

    def _select_targets(self):
        """FOV-crop, range-crop, then keep only the nearest point per
        (azimuth, range) bin - turns a dense LiDAR-derived cloud into a
        sparse, radar-plausible target list."""
        xyz = self.latest_cloud_xyz
        if xyz is None or xyz.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32)

        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        rng = np.sqrt(x * x + y * y + z * z)
        az = np.degrees(np.arctan2(y, x))
        el = np.degrees(np.arctan2(z, np.sqrt(x * x + y * y)))

        in_fov = (
            (rng > 0.5) & (rng < MAX_RANGE_M) &
            (np.abs(az) < AZIMUTH_FOV_DEG) &
            (np.abs(el) < ELEVATION_FOV_DEG)
        )
        if not np.any(in_fov):
            return np.empty((0, 3), dtype=np.float32)

        xyz_f, rng_f, az_f = xyz[in_fov], rng[in_fov], az[in_fov]
        az_bin = np.round(az_f / AZIMUTH_BIN_DEG).astype(np.int64)
        rng_bin = np.round(rng_f / RANGE_BIN_M).astype(np.int64)
        keys = az_bin * 100000 + rng_bin

        # Keep the closest-range point within each (azimuth,range) bin -
        # sorting by range then dropping duplicate keys keeps the first
        # (nearest) occurrence per bin.
        order = np.argsort(rng_f)
        _, first_idx = np.unique(keys[order], return_index=True)
        selected = order[first_idx]
        return xyz_f[selected]

    def publish_targets(self):
        targets_xyz = self._select_targets()
        n = targets_xyz.shape[0]
        now = self.get_clock().now()

        points = np.zeros((n, 11), dtype=np.float32)
        if n > 0:
            x, y, z = targets_xyz[:, 0], targets_xyz[:, 1], targets_xyz[:, 2]
            points[:, 0] = x
            points[:, 1] = y
            points[:, 2] = z
            points[:, 3] = 0.0  # radial_speed - static obstacles only, see module docstring
            points[:, 4] = SYNTHETIC_POWER
            points[:, 5] = SYNTHETIC_RCS
            points[:, 6] = SYNTHETIC_NOISE
            points[:, 7] = SYNTHETIC_SNR
            points[:, 8] = np.degrees(np.arctan2(y, x))
            points[:, 9] = np.degrees(np.arctan2(z, np.sqrt(x * x + y * y)))
            points[:, 10] = np.sqrt(x * x + y * y + z * z)

        msg = PointCloud2()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "umrr"
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(RADAR_FIELD_NAMES)
        ]
        msg.is_bigendian = False
        msg.point_step = 4 * len(RADAR_FIELD_NAMES)
        msg.row_step = msg.point_step * n
        msg.is_dense = True
        msg.data = points.tobytes()
        self.target_pub.publish(msg)

        sec, nanosec = now.seconds_nanoseconds()
        header = CanTargetHeader()
        header.frame_id = "umrr"
        header.cycle_time = 1.0 / self.publish_rate_hz
        header.number_of_targets = min(n, 255)
        header.cycle_count = self._cycle_count
        header.acquisition_setup = 0
        header.time_stamp = sec
        header.acq_ts_fraction = nanosec / 1e9
        self.header_pub.publish(header)
        self._cycle_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = RadarSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
