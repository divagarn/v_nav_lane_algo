#!/usr/bin/env python3
"""
Ground segmentation with three selectable backends (SEGMENT_GROUND_METHOD
env var: gpu/cpu/segment - see that var's own comment below).

gpu/cpu pipeline (identical logic, GPU=PyTorch CUDA, CPU=NumPy):
  1. Distance / height pre-filter
  2. Voxel downsample seed candidates
  3. Height-band seed selection
  4. Batched RANSAC plane fit (coarse; batched on both GPU and CPU)
  5. Refine plane on coarse inliers (fine)
  6. Slope-aware adaptive threshold - classify all points
  7. Statistical outlier rejection on far-range ground (k-NN)
  8. Publish ground / non-ground (zero-copy binary PointCloud2)

segment pipeline (no torch, CPU-only, fewer cores, no O(n^2) step - see
SegmentLinefit section below for the full algorithm): angular-segment +
near-to-far line-walk ground model, then the same adaptive-threshold
classification (steps 6/8 above) against each point's local line instead
of one global plane.

gpu/cpu were originally "Optimised for NVIDIA Jetson AGX Orin 64 GB
(JetPack 6) and desktop GPUs" - segment has no such dependency at all.
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import time
from sensor_msgs_py import point_cloud2 as pc2

# SEGMENT_GROUND_METHOD selects the backend before any torch import happens,
# not just the node's own branch - 'cpu'/'segment' skip torch.cuda.is_
# available() AND the CUDA warm-up call below entirely. On Jetson, a
# PyTorch wheel that doesn't match the installed JetPack/CUDA version can
# make that warm-up call itself hang (not error), which looks like
# everything sharing the GPU context (RViz included) froze - this is the
# way to rule that specific failure mode in or out. 'segment' additionally
# means torch is never touched at all (see SegmentLinefit below), so it
# has zero GPU/driver dependency.
#   gpu     - CUDA RANSAC (_gpu), falls back to 'cpu' if CUDA unavailable.
#   cpu     - NumPy RANSAC + SOR (_cpu) - same algorithm as 'gpu', slower.
#   segment - Himmelsbach-style angular-segment ground filter
#             (_segment_linefit) - no RANSAC, no O(n^2) step, no torch.
_METHOD = os.environ.get('SEGMENT_GROUND_METHOD', 'gpu').lower()
if _METHOD not in ('gpu', 'cpu', 'segment'):
    _METHOD = 'gpu'

# -- Try CUDA via PyTorch (only if the 'gpu' method was requested - 'cpu'
# and 'segment' never import torch at all, so neither has any GPU/driver
# dependency) --
if _METHOD == 'gpu':
    try:
        import torch
        HAS_CUDA = torch.cuda.is_available()
        if HAS_CUDA:
            _DEV = torch.device('cuda')
            torch.zeros(1, device=_DEV)  # warm-up
        else:
            _DEV = torch.device('cpu')
    except ImportError:
        HAS_CUDA = False
        torch = None
        _DEV = None
else:
    HAS_CUDA = False
    torch = None
    _DEV = None

# -- Constants --
VOXEL_SIZE              = 0.15
MAX_DISTANCE            = 25.0
LIDAR_HEIGHT            = 1.00
MIN_HEIGHT              = -1.60
MAX_HEIGHT              = 2.00
SEED_Z_MIN              = -1.45
SEED_Z_MAX              = -0.45
MAX_GROUND_ANGLE        = 15.0
RANSAC_ITERATIONS       = 300
RANSAC_THRESHOLD_COARSE = 0.20
RANSAC_THRESHOLD_FINE   = 0.12
MIN_GROUND_POINTS       = 50
BASE_GROUND_THRESH      = 0.15
DISTANCE_SCALE          = 0.012
# Tightened from 0.35/0.25/0.30 - a short (e.g. ~0.9m) real-world object at
# meaningful range could get fully swallowed as ground: VLP16's ~2deg
# vertical beam spacing means only one (sometimes zero) beam actually hits
# a short object at range, and the previous, range-widened band was thick
# enough to absorb that single return entirely, leaving nothing to reach
# local_costmap. Tighter band = less of that risk, traded against letting
# more genuine ground-plane noise/small road texture back into
# /non_ground_points - watch for false-positive costmap obstacles on a
# real (bumpy/uneven) surface after this change, not just missed real ones.
MAX_ADAPTIVE_THRESH     = 0.25
GROUND_MAX_Z_ABOVE      = 0.15
GROUND_MAX_Z_BELOW      = 0.20
STAT_OUTLIER_K          = 15
STAT_OUTLIER_STD        = 2.0
RANSAC_BATCH            = 64  # GPU hypotheses evaluated per loop


# ====================================================================
#  GPU helpers
# ====================================================================

def _g(arr):
    """NumPy -> contiguous GPU tensor."""
    return torch.from_numpy(np.ascontiguousarray(arr)).to(_DEV, non_blocking=True)


def g_voxel_ds(pts, vs):
    """Voxel downsample on GPU using scatter_add."""
    vox = torch.floor(pts / vs).to(torch.int64)
    keys = vox[:, 0] * 2_000_000 + vox[:, 1] * 2_000 + vox[:, 2]
    u, inv = torch.unique(keys, return_inverse=True)
    nv = len(u)
    s = torch.zeros(nv, 3, dtype=torch.float64, device=_DEV)
    s.scatter_add_(0, inv.unsqueeze(1).expand(-1, 3), pts.double())
    c = torch.zeros(nv, dtype=torch.float64, device=_DEV)
    c.scatter_add_(0, inv, torch.ones(len(pts), dtype=torch.float64, device=_DEV))
    return (s / c.unsqueeze(1)).float()


def g_ransac(pts, dt, mi, ma, batch=RANSAC_BATCH):
    """Batched RANSAC plane fit on GPU."""
    n = pts.shape[0]
    if n < 3:
        return torch.tensor([0., 0., 1., 0.], device=_DEV), \
               torch.zeros(n, dtype=torch.bool, device=_DEV)
    cl = float(np.cos(np.radians(ma)))
    bc = 0
    beq = torch.tensor([0., 0., 1., 0.], device=_DEV)
    bm = torch.zeros(n, dtype=torch.bool, device=_DEV)
    for _ in range(max(1, mi // batch)):
        ix = torch.randint(0, n, (batch, 3), device=_DEV)
        p1, p2, p3 = pts[ix[:, 0]], pts[ix[:, 1]], pts[ix[:, 2]]
        nm = torch.linalg.cross(p2 - p1, p3 - p1)
        nl = torch.linalg.norm(nm, dim=1, keepdim=True)
        ok = nl.squeeze(1) > 1e-8
        nm = nm / nl.clamp(min=1e-8)
        # Flip normals so z > 0
        flip = nm[:, 2] < 0
        nm[flip] *= -1
        ok = ok & (nm[:, 2].abs() > cl)
        d = -(nm * p1).sum(1)
        # Compute distances for all points against all batch hypotheses: (n, batch)
        dist = torch.abs(pts @ nm.T + d.unsqueeze(0))
        ic = (dist < dt).sum(0)
        ic[~ok] = 0
        bi = torch.argmax(ic)
        cv = ic[bi].item()
        if cv > bc:
            bc = cv
            nr = nm[bi]
            beq = torch.tensor([nr[0].item(), nr[1].item(), nr[2].item(),
                                d[bi].item()], device=_DEV)
            bm = dist[:, bi] < dt
    return beq, bm


def g_sor(pts, k, sm):
    """Statistical outlier rejection via batched k-NN on GPU."""
    n = pts.shape[0]
    if n < k + 1:
        return torch.ones(n, dtype=torch.bool, device=_DEV)
    md = torch.empty(n, device=_DEV)
    ch = min(n, 8192)
    for s in range(0, n, ch):
        e = min(s + ch, n)
        d = torch.cdist(pts[s:e].unsqueeze(0), pts.unsqueeze(0)).squeeze(0)
        tv, _ = torch.topk(d, k + 1, dim=1, largest=False)
        md[s:e] = tv[:, 1:].mean(1)
    mu, sig = md.mean(), md.std()
    return md < (mu + sm * sig)


# ====================================================================
#  CPU fallback helpers
# ====================================================================

def c_voxel_ds(pts, vs):
    """Voxel downsample on CPU."""
    vi = np.floor(pts / vs).astype(np.int32)
    keys = vi[:, 0] * 2_000_000 + vi[:, 1] * 2_000 + vi[:, 2]
    _, inv, cnt = np.unique(keys, return_inverse=True, return_counts=True)
    s = np.zeros((len(cnt), 3), dtype=np.float64)
    np.add.at(s, inv, pts)
    return (s / cnt[:, None]).astype(np.float32)


def c_ransac(pts, dt, mi, ma, batch=RANSAC_BATCH):
    """Batched RANSAC plane fit on CPU (numpy-vectorized, mirrors g_ransac's
    batching so the CPU fallback pays numpy dispatch overhead once per
    64-hypothesis batch instead of once per single hypothesis)."""
    n = len(pts)
    if n < 3:
        return np.array([0., 0., 1., 0.]), np.zeros(n, dtype=bool)
    cl = np.cos(np.radians(ma))
    bc = 0
    beq = np.array([0., 0., 1., 0.])
    bm = np.zeros(n, dtype=bool)
    for _ in range(max(1, mi // batch)):
        ix = np.random.randint(0, n, size=(batch, 3))
        p1, p2, p3 = pts[ix[:, 0]], pts[ix[:, 1]], pts[ix[:, 2]]
        nm = np.cross(p2 - p1, p3 - p1)
        nl = np.linalg.norm(nm, axis=1, keepdims=True)
        ok = nl[:, 0] > 1e-8
        nm = nm / np.clip(nl, 1e-8, None)
        flip = nm[:, 2] < 0
        nm[flip] *= -1
        ok = ok & (np.abs(nm[:, 2]) > cl)
        d = -(nm * p1).sum(1)
        dist = np.abs(pts @ nm.T + d[None, :])
        ic = (dist < dt).sum(0)
        ic[~ok] = 0
        bi = int(np.argmax(ic))
        cv = int(ic[bi])
        if cv > bc:
            bc = cv
            beq = np.array([nm[bi, 0], nm[bi, 1], nm[bi, 2], d[bi]])
            bm = dist[:, bi] < dt
    return beq, bm


def c_sor(pts, k, sm):
    """Statistical outlier rejection on CPU via chunked distance."""
    n = len(pts)
    if n < k + 1:
        return np.ones(n, dtype=bool)
    md = np.empty(n, dtype=np.float64)
    ch = 4000
    for s in range(0, n, ch):
        e = min(s + ch, n)
        diff = pts[None, :, :] - pts[s:e, None, :]
        d2 = (diff ** 2).sum(2)
        kp = np.partition(d2, k, axis=1)[:, :k + 1]
        ks = np.sort(kp, axis=1)[:, 1:k + 1]
        md[s:e] = np.mean(np.sqrt(ks), axis=1)
    mu, sig = md.mean(), md.std()
    return md < (mu + sm * sig)


# ====================================================================
#  SegmentLinefit - Himmelsbach-style angular-segment ground filter
#  (CPU, pure NumPy, no torch/GPU dependency at all)
#
#  Himmelsbach, Hummel, Franke, Gross, "Fast Segmentation of 3D Point
#  Clouds for Ground Vehicles", IEEE Intelligent Vehicles Symposium 2010 -
#  the standard real-time ground-vehicle LiDAR ground filter (also the CPU
#  baseline Patchwork++ itself is benchmarked against). Unlike RANSAC, it
#  fits no global plane: it slices the scan into angular segments (pie
#  slices from the sensor), bins each segment by range, and walks each
#  segment's bins near-to-far, extending a local ground line while the
#  slope/height step between consecutive bins stays small and starting
#  over whenever it doesn't. This naturally follows gentle floor
#  variation (RANSAC's single global plane can't) and needs neither
#  RANSAC's iteration loop nor c_sor's O(n^2) step - the only Python-level
#  loop below walks occupied (segment, bin) CELLS, not raw points, so for
#  a VLP16 scan it's typically a few thousand iterations of plain scalar
#  arithmetic: cheap, and nothing here is large-matrix/multi-threaded-BLAS
#  shaped, so it stays light on CPU cores too.
# ====================================================================

SEG_NUM_SEGMENTS    = 180    # 2deg azimuth slices around the sensor
SEG_BIN_SIZE_M       = 0.5   # range bin width within each segment
SEG_MAX_SLOPE        = np.tan(np.radians(10.0))  # max ground slope between bins
SEG_MAX_HEIGHT_JUMP  = 0.20  # max |z| step between consecutive ground bins


def segment_linefit_ground(pts):
    """pts: (N,3) already distance/height pre-filtered. Returns
    (ground_cell_mask, expected_z) per point - ground_cell_mask is True
    where the point's (segment, bin) cell was accepted as part of a
    ground line; expected_z is that line's height at the point's own
    range, for the caller to run the same adaptive-threshold check the
    RANSAC backends use against their global plane."""
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = np.sqrt(x * x + y * y)
    ang = np.arctan2(y, x) % (2.0 * np.pi)

    seg_idx = np.minimum(
        (ang / (2.0 * np.pi / SEG_NUM_SEGMENTS)).astype(np.int64), SEG_NUM_SEGMENTS - 1)
    bin_idx = np.maximum((r / SEG_BIN_SIZE_M).astype(np.int64), 0)

    # Group points into (segment, bin) cells - same integer-key + np.unique
    # hashing trick c_voxel_ds already uses for voxel downsampling.
    key = seg_idx * 100_000 + bin_idx
    u, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    ng = len(u)

    # Per-cell prototype: lowest point in the cell (most likely ground
    # return) and the cell's mean range.
    proto_z = np.full(ng, np.inf)
    np.minimum.at(proto_z, inv, z)
    proto_r = np.zeros(ng)
    np.add.at(proto_r, inv, r)
    proto_r /= cnt
    proto_seg = u // 100_000

    # Near-to-far walk per segment, sorted so cells of the same segment
    # are contiguous and ordered by range.
    order = np.lexsort((proto_r, proto_seg))
    s_seg, s_r, s_z = proto_seg[order], proto_r[order], proto_z[order]

    cell_ground = np.zeros(ng, dtype=bool)
    cell_anchor_r = np.zeros(ng)
    cell_anchor_z = np.zeros(ng)
    cell_end_r = np.zeros(ng)
    cell_end_z = np.zeros(ng)

    prev_seg = -1
    prev_r = prev_z = 0.0
    have_prev = False
    for i in range(ng):
        seg = s_seg[i]
        if seg != prev_seg:
            have_prev = False
        rr, zz = s_r[i], s_z[i]
        if not have_prev:
            accept = SEED_Z_MIN <= zz <= SEED_Z_MAX
        else:
            dr = max(rr - prev_r, 1e-3)
            accept = (abs((zz - prev_z) / dr) <= SEG_MAX_SLOPE and
                      abs(zz - prev_z) <= SEG_MAX_HEIGHT_JUMP)
        idx = order[i]
        if accept:
            cell_anchor_r[idx] = prev_r if have_prev else rr
            cell_anchor_z[idx] = prev_z if have_prev else zz
            cell_end_r[idx] = rr
            cell_end_z[idx] = zz
            cell_ground[idx] = True
            prev_r, prev_z, have_prev = rr, zz, True
        prev_seg = seg

    ground_cell = cell_ground[inv]
    a_r, a_z = cell_anchor_r[inv], cell_anchor_z[inv]
    e_r, e_z = cell_end_r[inv], cell_end_z[inv]
    with np.errstate(divide='ignore', invalid='ignore'):
        slope = np.where(e_r > a_r, (e_z - a_z) / np.maximum(e_r - a_r, 1e-3), 0.0)
    expected_z = a_z + slope * (r - a_r)
    return ground_cell, expected_z


# ====================================================================
#  Zero-copy binary PointCloud2 publisher
# ====================================================================

_PC_FIELDS = [
    PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.UINT32,  count=1),
]


def fast_pub(header, pts, pub, is_ground):
    """Build PointCloud2 binary data in NumPy - zero per-point Python loops."""
    n = len(pts)
    if n == 0:
        return
    if is_ground:
        d = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2)
        mx = max(d.max(), 1e-6)
        g = (255 * (0.5 + 0.5 * (1.0 - d / mx))).astype(np.uint32)
        rgb = g << 8
    else:
        h = pts[:, 2]
        hmn, hmx = h.min(), h.max()
        rng = max(hmx - hmn, 0.01)
        nr = (h - hmn) / rng
        r = (255 * (1.0 - nr)).astype(np.uint32)
        g = (255 * np.abs(nr - 0.5) * 2).astype(np.uint32)
        b = (255 * nr).astype(np.uint32)
        rgb = (r << 16) | (g << 8) | b
    buf = np.empty((n, 4), dtype=np.float32)
    buf[:, :3] = pts[:, :3]
    buf[:, 3] = rgb.view(np.float32)
    m = PointCloud2()
    m.header = header
    m.height = 1
    m.width = n
    m.fields = _PC_FIELDS
    m.is_bigendian = False
    m.point_step = 16
    m.row_step = 16 * n
    m.data = buf.tobytes()
    m.is_dense = True
    pub.publish(m)


# ====================================================================
#  ROS 2 Node
# ====================================================================

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('segment_ground')
        self.gpub = self.create_publisher(PointCloud2, '/ground_points', 2)
        self.ngpub = self.create_publisher(PointCloud2, '/non_ground_points', 2)
        self.tt = 0.0
        self.cnt = 0
        # _METHOD already resolved at import time (SEGMENT_GROUND_METHOD) -
        # 'gpu' falls back to 'cpu' here if CUDA actually isn't available.
        self.method = _METHOD if (_METHOD != 'gpu' or HAS_CUDA) else 'cpu'
        self.use_gpu = (self.method == 'gpu')

        if self.use_gpu:
            self.last_eq = torch.tensor([0., 0., 1., -LIDAR_HEIGHT], device=_DEV)
        else:
            self.last_eq = np.array([0., 0., 1., -LIDAR_HEIGHT])

        self.create_subscription(PointCloud2, '/velodyne_points', self.callback, 1)

        be = {
            'gpu': "CUDA ({})".format(torch.cuda.get_device_name(0)) if self.use_gpu else None,
            'cpu': "CPU (NumPy RANSAC)",
            'segment': "CPU (NumPy, Himmelsbach-style segment line-fit)",
        }[self.method]
        self.get_logger().info("Ground segmentation -- backend: {}".format(be))
        if self.method == 'segment':
            self.get_logger().info(
                "segments={} bin={}m max_slope<{:.0f}deg max_height_jump={}m".format(
                    SEG_NUM_SEGMENTS, SEG_BIN_SIZE_M, 10.0, SEG_MAX_HEIGHT_JUMP))
        else:
            self.get_logger().info(
                "RANSAC coarse={} fine={} angle<{}deg iters={} batch={}".format(
                    RANSAC_THRESHOLD_COARSE, RANSAC_THRESHOLD_FINE,
                    MAX_GROUND_ANGLE, RANSAC_ITERATIONS, RANSAC_BATCH))

    def callback(self, msg):
        try:
            t0 = time.time()
            # Fast binary decode — structured dtype, no per-point Python loop
            n = msg.width * msg.height
            if n < MIN_GROUND_POINTS:
                return
            step = msg.point_step
            off = {}
            for f in msg.fields:
                if f.name in ('x', 'y', 'z'):
                    off[f.name] = f.offset
            if len(off) < 3:
                return
            dt = np.dtype({'names': ['x', 'y', 'z'],
                           'formats': ['<f4', '<f4', '<f4'],
                           'offsets': [off['x'], off['y'], off['z']],
                           'itemsize': step})
            raw = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
            pts_np = np.column_stack([raw['x'], raw['y'], raw['z']])
            valid = np.isfinite(pts_np).all(axis=1)
            if not valid.all():
                pts_np = pts_np[valid]
            nt = len(pts_np)
            if nt < MIN_GROUND_POINTS:
                return
            if self.method == 'gpu':
                self._gpu(msg, pts_np, nt, t0)
            elif self.method == 'segment':
                self._segment_linefit(msg, pts_np, nt, t0)
            else:
                self._cpu(msg, pts_np, nt, t0)
        except Exception as e:
            self.get_logger().error("Ground seg error: {}".format(e))
            import traceback
            self.get_logger().error(traceback.format_exc())

    # -- GPU path --
    def _gpu(self, msg, pts_np, nt, t0):
        p = _g(pts_np)

        # 1 - pre-filter
        rad = torch.sqrt(p[:, 0] ** 2 + p[:, 1] ** 2)
        keep = (rad < MAX_DISTANCE) & (p[:, 2] > MIN_HEIGHT) & (p[:, 2] < MAX_HEIGHT)
        f = p[keep]
        if f.shape[0] < MIN_GROUND_POINTS:
            return

        # 2 - seeds
        sm = (f[:, 2] >= SEED_Z_MIN) & (f[:, 2] <= SEED_Z_MAX)
        seeds = f[sm]
        if seeds.shape[0] < MIN_GROUND_POINTS:
            zt = torch.quantile(f[:, 2], 0.20)
            seeds = f[f[:, 2] <= zt]
        if seeds.shape[0] < MIN_GROUND_POINTS:
            return

        # 3 - downsample seeds
        sd = g_voxel_ds(seeds, VOXEL_SIZE) if seeds.shape[0] > 5000 else seeds

        # 4 - coarse RANSAC
        eq, im = g_ransac(sd, RANSAC_THRESHOLD_COARSE, RANSAC_ITERATIONS, MAX_GROUND_ANGLE)
        if im.sum().item() < MIN_GROUND_POINTS:
            eq = self.last_eq
        else:
            # 5 - refine
            ci = sd[im]
            if ci.shape[0] > 3:
                eq2, _ = g_ransac(ci, RANSAC_THRESHOLD_FINE,
                                  RANSAC_ITERATIONS // 2, MAX_GROUND_ANGLE)
                if abs(eq2[2].item()) > np.cos(np.radians(MAX_GROUND_ANGLE)):
                    eq = eq2
            self.last_eq = eq

        a, b, c, d = eq

        # 6 - classify all points
        signed = p[:, 0] * a + p[:, 1] * b + p[:, 2] * c + d
        absd = signed.abs()
        ra = torch.sqrt(p[:, 0] ** 2 + p[:, 1] ** 2)
        nb = 0.08 * torch.clamp(1.0 - ra / 3.0, min=0.0)
        adapt = torch.clamp(BASE_GROUND_THRESH + DISTANCE_SCALE * ra + nb,
                            max=MAX_ADAPTIVE_THRESH)
        gm = (absd < adapt) & (signed < GROUND_MAX_Z_ABOVE) & \
             (signed > -GROUND_MAX_Z_BELOW) & (p[:, 2] < MAX_HEIGHT)

        # 7 - SOR (far only)
        gi = torch.where(gm)[0]
        if gi.shape[0] > STAT_OUTLIER_K + 1:
            gp = p[gi]
            gr = torch.sqrt(gp[:, 0] ** 2 + gp[:, 1] ** 2)
            far = gr > 4.0
            fp = gp[far]
            if fp.shape[0] > STAT_OUTLIER_K + 1:
                sk = min(STAT_OUTLIER_K, 10) if fp.shape[0] > 8000 else STAT_OUTLIER_K
                smf = g_sor(fp, sk, STAT_OUTLIER_STD)
                sf = torch.ones(gi.shape[0], dtype=torch.bool, device=_DEV)
                fi = torch.where(far)[0]
                sf[fi[~smf]] = False
                gm[gi[~sf]] = False

        gm_np = gm.cpu().numpy()
        fast_pub(msg.header, pts_np[gm_np], self.gpub, True)
        fast_pub(msg.header, pts_np[~gm_np], self.ngpub, False)
        self._perf(t0, int(gm_np.sum()), nt - int(gm_np.sum()), nt)

    # -- CPU path --
    def _cpu(self, msg, pts_np, nt, t0):
        p = pts_np

        rad = np.sqrt(p[:, 0] ** 2 + p[:, 1] ** 2)
        keep = (rad < MAX_DISTANCE) & (p[:, 2] > MIN_HEIGHT) & (p[:, 2] < MAX_HEIGHT)
        f = p[keep]
        if len(f) < MIN_GROUND_POINTS:
            return

        sm = (f[:, 2] >= SEED_Z_MIN) & (f[:, 2] <= SEED_Z_MAX)
        seeds = f[sm]
        if len(seeds) < MIN_GROUND_POINTS:
            zt = np.percentile(f[:, 2], 20)
            seeds = f[f[:, 2] <= zt]
        if len(seeds) < MIN_GROUND_POINTS:
            return

        sd = c_voxel_ds(seeds, VOXEL_SIZE) if len(seeds) > 5000 else seeds

        eq, im = c_ransac(sd, RANSAC_THRESHOLD_COARSE, RANSAC_ITERATIONS, MAX_GROUND_ANGLE)
        if im.sum() < MIN_GROUND_POINTS:
            eq = self.last_eq
        else:
            ci = sd[im]
            if len(ci) > 3:
                eq2, _ = c_ransac(ci, RANSAC_THRESHOLD_FINE,
                                  RANSAC_ITERATIONS // 2, MAX_GROUND_ANGLE)
                if abs(eq2[2]) > np.cos(np.radians(MAX_GROUND_ANGLE)):
                    eq = eq2
            self.last_eq = eq

        a, b, c, d = eq
        signed = p[:, 0] * a + p[:, 1] * b + p[:, 2] * c + d
        absd = np.abs(signed)
        ra = np.sqrt(p[:, 0] ** 2 + p[:, 1] ** 2)
        nb = 0.08 * np.maximum(1.0 - ra / 3.0, 0.0)
        adapt = np.minimum(BASE_GROUND_THRESH + DISTANCE_SCALE * ra + nb,
                           MAX_ADAPTIVE_THRESH)
        gm = (absd < adapt) & (signed < GROUND_MAX_Z_ABOVE) & \
             (signed > -GROUND_MAX_Z_BELOW) & (p[:, 2] < MAX_HEIGHT)

        gi = np.where(gm)[0]
        if len(gi) > STAT_OUTLIER_K + 1:
            gp = p[gi]
            gr = np.sqrt(gp[:, 0] ** 2 + gp[:, 1] ** 2)
            far = gr > 4.0
            fp = gp[far]
            if len(fp) > STAT_OUTLIER_K + 1:
                sk = min(STAT_OUTLIER_K, 10) if len(fp) > 8000 else STAT_OUTLIER_K
                smf = c_sor(fp, sk, STAT_OUTLIER_STD)
                sf = np.ones(len(gi), dtype=bool)
                fi = np.where(far)[0]
                sf[fi[~smf]] = False
                gm[gi[~sf]] = False

        fast_pub(msg.header, p[gm], self.gpub, True)
        fast_pub(msg.header, p[~gm], self.ngpub, False)
        self._perf(t0, int(gm.sum()), nt - int(gm.sum()), nt)

    # -- Segment line-fit path (CPU, no torch) --
    def _segment_linefit(self, msg, pts_np, nt, t0):
        p = pts_np

        rad = np.sqrt(p[:, 0] ** 2 + p[:, 1] ** 2)
        keep = (rad < MAX_DISTANCE) & (p[:, 2] > MIN_HEIGHT) & (p[:, 2] < MAX_HEIGHT)
        f = p[keep]
        if len(f) < MIN_GROUND_POINTS:
            return

        ground_cell, expected_z = segment_linefit_ground(f)

        signed = f[:, 2] - expected_z
        absd = np.abs(signed)
        ra = np.sqrt(f[:, 0] ** 2 + f[:, 1] ** 2)
        nb = 0.08 * np.maximum(1.0 - ra / 3.0, 0.0)
        adapt = np.minimum(BASE_GROUND_THRESH + DISTANCE_SCALE * ra + nb,
                           MAX_ADAPTIVE_THRESH)
        gm_f = ground_cell & (absd < adapt) & (signed < GROUND_MAX_Z_ABOVE) & \
            (signed > -GROUND_MAX_Z_BELOW)

        gm = np.zeros(nt, dtype=bool)
        gm[keep] = gm_f

        fast_pub(msg.header, p[gm], self.gpub, True)
        fast_pub(msg.header, p[~gm], self.ngpub, False)
        self._perf(t0, int(gm.sum()), nt - int(gm.sum()), nt)

    def _perf(self, t0, ng, nng, nt):
        dt = time.time() - t0
        self.tt += dt
        self.cnt += 1
        if self.cnt % 10 == 0:
            avg = self.tt / self.cnt
            hz = 1.0 / avg if avg > 0 else 0
            pct = 100.0 * ng / nt if nt else 0
            self.get_logger().info(
                "Ground: {} ({:.1f}%) | {} non-gnd | {:.1f}ms (avg {:.1f}ms {:.1f}Hz)".format(
                    ng, pct, nng, dt * 1000, avg * 1000, hz))


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
