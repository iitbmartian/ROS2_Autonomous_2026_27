"""Terrain map quality against ground truth: elevation RMSE and negative-obstacle timing.

    ros2 run rover_nav2 terrain_eval --world crater_field --out ~/eval/crater.csv
    (run alongside a sim, or a bag play, for --duration seconds or until Ctrl-C)

Every /terrain/local map is moved into the world frame through ground truth, so localisation
drift does not count as mapping error: T_world_odom = T_world_base(ground truth) *
T_odom_base(TF)^-1 at the map's stamp. Each finite cell within --radius of the rover is compared
with the ground-truth heightmap (ground_truth.py).

Negative obstacles (with --craters, crater_field's craters.yaml): for every crater wider than the
wheel track, the first time /terrain/local_features flags at least --min-cells cells inside its
rim, against the first time the rover's footprint (ground truth) reaches the rim. SPEC.md's
Phase 2 gate: every such crater flagged before the footprint reaches it.

Gazebo's ground-truth /odom has its x, y origin at the spawn point, which every goal set puts at
the world origin with yaw 0. Its z is already the height above the world origin: on marsyard the
rover at rest reads z -0.0006 while the map built from its own odometry puts the ground at
+0.002 under a base_footprint at 0, so base_footprint sits on the ground and nothing is added.
--spawn-z exists for spawns elsewhere.
"""

import argparse
import bisect
import csv
import math
import os
import sys
import threading
import time

import numpy as np
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
import yaml
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from rover_nav2.benchmark import ground_truth as gt
from rover_nav2.benchmark.metrics import quat_to_rpy


def gridmap_layer(msg, name):
    """Unwrapped layer as rows along -x, columns along -y (grid_map order), or None."""
    if name not in msg.layers:
        return None
    arr = msg.data[msg.layers.index(name)]
    cols, rows = arr.layout.dim[0].size, arr.layout.dim[1].size
    a = np.array(arr.data, dtype=float).reshape(cols, rows).T
    return np.roll(a, (-msg.outer_start_index, -msg.inner_start_index), axis=(0, 1))


def gridmap_cell_xy(msg, shape):
    """Map-frame x, y of every cell centre, for a grid of the given shape."""
    res = msg.info.resolution
    rows, cols = shape
    cx, cy = msg.info.pose.position.x, msg.info.pose.position.y
    xs = cx + msg.info.length_x / 2.0 - (np.arange(rows) + 0.5) * res
    ys = cy + msg.info.length_y / 2.0 - (np.arange(cols) + 0.5) * res
    return np.meshgrid(xs, ys, indexing='ij')


def matrix(x, y, z, roll, pitch, yaw):
    T = np.eye(4)
    T[:3, :3] = gt.rpy_matrix(roll, pitch, yaw)
    T[:3, 3] = (x, y, z)
    return T


class GroundTruthGrid:

    def __init__(self, npz):
        d = np.load(npz)
        self.z = d['z'].astype(float)
        self.x_min, self.y_max, self.res = float(d['x_min']), float(d['y_max']), float(d['res'])

    def sample(self, x, y):
        """Bilinear between cell centres, NaN outside."""
        u = (x - self.x_min) / self.res - 0.5
        v = (self.y_max - y) / self.res - 0.5
        rows, cols = self.z.shape
        ok = (u >= 0) & (u <= cols - 1) & (v >= 0) & (v <= rows - 1)
        u, v = np.clip(u, 0, cols - 1), np.clip(v, 0, rows - 1)
        # The last cell centre interpolates from the cell before it with weight 1, exactly.
        j = np.minimum(np.floor(u).astype(int), cols - 2)
        i = np.minimum(np.floor(v).astype(int), rows - 2)
        fu, fv = u - j, v - i
        z = self.z
        h = (z[i, j] * (1 - fu) * (1 - fv) + z[i, j + 1] * fu * (1 - fv)
             + z[i + 1, j] * (1 - fu) * fv + z[i + 1, j + 1] * fu * fv)
        return np.where(ok, h, np.nan)


def footprint_reaches(x, y, yaw, length, width, cx, cy, r):
    """True when the footprint rectangle centred at (x, y) with heading yaw touches the disc."""
    dx, dy = cx - x, cy - y
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    qx = min(max(lx, -length / 2), length / 2)
    qy = min(max(ly, -width / 2), width / 2)
    return math.hypot(lx - qx, ly - qy) <= r


class Evaluator(Node):

    def __init__(self, a, gtg, craters, phys):
        super().__init__('terrain_eval', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.a, self.gtg, self.phys = a, gtg, phys
        self.craters = [c for c in craters if c['diameter'] > phys['track']]
        self.lock = threading.Lock()
        self.gt_t, self.gt_pose = [], []
        self.rows = []
        self.sq_sum, self.n_cells, self.abs_max = 0.0, 0, 0.0
        self.first_seen = {}       # crater index -> stamp of first detection
        self.first_reach = {}      # crater index -> stamp the footprint reached the rim
        self.min_dist = {}
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 100)
        self.create_subscription(GridMap, a.map_topic, self.on_map, 2)
        if self.craters:
            self.create_subscription(GridMap, a.features_topic, self.on_features, 2)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        r, pi, y = quat_to_rpy(q.x, q.y, q.z, q.w)
        with self.lock:
            self.gt_t.append(t)
            self.gt_pose.append((p.x, p.y, p.z + self.a.spawn_z, r, pi, y))
        for k, c in enumerate(self.craters):
            d = math.hypot(c['x'] - p.x, c['y'] - p.y)
            self.min_dist[k] = min(self.min_dist.get(k, 1e9), d)
            if k not in self.first_reach and footprint_reaches(
                    p.x, p.y, y, self.phys['footprint_length'], self.phys['footprint_width'],
                    c['x'], c['y'], c['diameter'] / 2):
                self.first_reach[k] = t

    def gt_at(self, t):
        with self.lock:
            k = bisect.bisect_left(self.gt_t, t)
            if not self.gt_t or k >= len(self.gt_t) or (k == 0 and self.gt_t[0] - t > 0.05):
                return None
            if k > 0 and t - self.gt_t[k - 1] < self.gt_t[k] - t:
                k -= 1
            if abs(self.gt_t[k] - t) > 0.05:
                return None
            return self.gt_pose[k]

    def world_from_map(self, msg):
        """T_world_mapframe at the map's stamp, or None."""
        t = Time.from_msg(msg.header.stamp)
        pose = self.gt_at(t.nanoseconds * 1e-9)
        if pose is None:
            return None, None
        try:
            tf = self.tf.lookup_transform(msg.header.frame_id, self.a.base_frame, t)
        except TransformException:
            return None, None
        tr, q = tf.transform.translation, tf.transform.rotation
        T_map_base = matrix(tr.x, tr.y, tr.z, *quat_to_rpy(q.x, q.y, q.z, q.w))
        T_world_base = matrix(*pose)
        return T_world_base @ np.linalg.inv(T_map_base), pose

    def cells_world(self, msg, layer):
        a = gridmap_layer(msg, layer)
        if a is None:
            return None
        T, pose = self.world_from_map(msg)
        if T is None:
            return None
        X, Y = gridmap_cell_xy(msg, a.shape)
        P = np.stack([X.ravel(), Y.ravel(), np.nan_to_num(a.ravel()), np.ones(a.size)])
        W = T @ P
        return a.ravel(), W[0], W[1], W[2], pose

    def on_map(self, msg):
        out = self.cells_world(msg, 'elevation')
        if out is None:
            return
        z_map, wx, wy, wz, pose = out
        near = np.hypot(wx - pose[0], wy - pose[1]) <= self.a.radius
        ok = near & np.isfinite(z_map)
        g = self.gtg.sample(wx[ok], wy[ok])
        err = wz[ok] - g
        err = err[np.isfinite(err)]
        if len(err) == 0:
            return
        self.sq_sum += float(np.sum(err ** 2))
        self.n_cells += len(err)
        self.abs_max = max(self.abs_max, float(np.max(np.abs(err))))
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.rows.append((round(t, 3), len(err), float(np.sqrt(np.mean(err ** 2))),
                          float(np.mean(err)), float(np.percentile(np.abs(err), 95)),
                          round(pose[0], 3), round(pose[1], 3)))

    def on_features(self, msg):
        out = self.cells_world(msg, 'negative_obstacle')
        if out is None:
            return
        neg, wx, wy, _, _ = out
        flagged = neg == 1.0
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        for k, c in enumerate(self.craters):
            if k in self.first_seen:
                continue
            inside = np.hypot(wx - c['x'], wy - c['y']) <= c['diameter'] / 2
            if np.count_nonzero(flagged & inside) >= self.a.min_cells:
                self.first_seen[k] = t


def summary(ev):
    lines = []
    rmse = math.sqrt(ev.sq_sum / ev.n_cells) if ev.n_cells else float('nan')
    lines.append(f'elevation within {ev.a.radius} m: RMSE {rmse:.4f} m over {ev.n_cells} cell '
                 f'samples in {len(ev.rows)} maps, max |error| {ev.abs_max:.3f} m')
    if ev.rows:
        per = [r[2] for r in ev.rows]
        lines.append(f'  per-map RMSE: median {np.median(per):.4f}, p95 {np.percentile(per, 95):.4f}'
                     f', worst {max(per):.4f} m. Mean error (bias) median '
                     f'{np.median([r[3] for r in ev.rows]):+.4f} m')
    if ev.craters:
        passed = failed = 0
        lines.append(f"craters wider than the track ({ev.phys['track']} m): {len(ev.craters)}")
        for k, c in enumerate(ev.craters):
            seen, reach = ev.first_seen.get(k), ev.first_reach.get(k)
            if reach is not None:
                ok = seen is not None and seen < reach
                passed += ok
                failed += not ok
                verdict = 'PASS' if ok else 'FAIL'
            else:
                verdict = 'not reached'
            lines.append(
                f"  ({c['x']:6.2f}, {c['y']:6.2f}) d {c['diameter']:.1f} m: closest "
                f"{ev.min_dist.get(k, float('nan')):.2f} m, flagged "
                f"{'-' if seen is None else f'{seen:.1f}s'}, footprint reached "
                f"{'-' if reach is None else f'{reach:.1f}s'}: {verdict}")
        lines.append(f'  reached craters flagged first: {passed} of {passed + failed}')
    return '\n'.join(lines)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--world', required=True, help='world name, for the ground-truth heightmap')
    p.add_argument('--gt-dir', default=os.path.expanduser('~/.ros/rover_3d_nav/ground_truth'))
    p.add_argument('--craters', default=None, help='craters.yaml, enables the crater check')
    p.add_argument('--out', required=True, help='per-map CSV, summary next to it as .txt')
    p.add_argument('--radius', type=float, default=6.0)
    p.add_argument('--duration', type=float, default=0.0, help='wall s, 0 = until Ctrl-C')
    p.add_argument('--map-topic', default='/terrain/local')
    p.add_argument('--features-topic', default='/terrain/local_features')
    p.add_argument('--base-frame', default='base_footprint')
    p.add_argument('--spawn-z', type=float, default=0.0)
    p.add_argument('--min-cells', type=int, default=3)
    a = p.parse_args(sys.argv[1:] if argv is None else argv)

    npz = os.path.join(a.gt_dir, f'{a.world}.npz')
    if not os.path.exists(npz):
        sys.exit(f'{npz} missing: run ros2 run rover_nav2 ground_truth_heightmap {a.world}.sdf '
                 f'--out {npz}')
    craters = []
    if a.craters:
        with open(a.craters) as f:
            craters = yaml.safe_load(f)['craters']
    from rover_traversability import physical_params
    phys, _ = physical_params.load()

    # Own the SIGINT: rclpy's handler shuts the context down mid-spin, which can surface as
    # ExternalShutdownException or RCLError and lose the results.
    # A job started with & from a script inherits SIGINT as ignored, so set it explicitly.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    ev = Evaluator(a, GroundTruthGrid(npz), craters, phys)
    t0 = time.time()
    take_errors = 0
    try:
        while rclpy.ok() and (a.duration <= 0 or time.time() - t0 < a.duration):
            try:
                rclpy.spin_once(ev, timeout_sec=0.2)
            except RuntimeError as e:
                # rclpy now and then fails to convert one taken message (seen on rock_field after
                # 7 h), losing one map is better than losing the whole evaluation.
                take_errors += 1
                if take_errors > 50:
                    raise
                print(f'skipped a message that failed to take ({take_errors}): {e}',
                      file=sys.stderr)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'cells', 'rmse_m', 'mean_err_m', 'p95_abs_err_m', 'gt_x', 'gt_y'])
        w.writerows(ev.rows)
    text = summary(ev)
    with open(os.path.splitext(a.out)[0] + '.txt', 'w') as f:
        f.write(text + '\n')
    print(text)
    ev.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
