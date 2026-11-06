"""Snapshot of a running 3D stack: raw layers to .npz and a six-panel figure.

    python3 snapshot.py NAME WORLD [--out DIR]

Run inside simenv.sh. Captures /terrain/local, /terrain/local_features, /local_costmap/costmap
and /global_costmap/costmap once. Panels: elevation, elevation minus ground truth (needs
ground_truth/<WORLD>.npz), best utilisation, the check that limits it, local and global costmap.
The local map is moved into the world frame through ground truth, like terrain_eval:
T_world_odom = T_world_base(Gazebo's /odom) * T_odom_base(TF)^-1 at the map's stamp, so odometry
drift does not count as mapping error. Without a ground-truth pose it falls back to odom = world.
Output: DIR/NAME.npz and DIR/NAME.{png,svg}, DIR defaults to ~/rover_3d_nav_results/snapshots.
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from matplotlib.colors import ListedColormap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import figstyle as fs  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.time import Time  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402
from rover_nav2.benchmark.terrain_eval import (  # noqa: E402
    gridmap_cell_xy, gridmap_layer, matrix, quat_to_rpy)

HOME = os.path.expanduser('~')
CHECKS = ['tip', 'clearance', 'articulation', 'step', 'roughness', 'hole']


def grab():
    got = {}
    rclpy.init()
    n = Node('snapshot', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    tf = Buffer()
    TransformListener(tf, n)
    odom = []
    n.create_subscription(Odometry, '/odom', lambda m: odom.append(m), 100)
    rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(GridMap, '/terrain/local', lambda m: got.setdefault('elev', m), rel)
    n.create_subscription(GridMap, '/terrain/local_features',
                          lambda m: got.setdefault('feat', m), rel)
    n.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                          lambda m: got.setdefault('lcm', m), 1)
    n.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                          lambda m: got.setdefault('gcm', m), latched)
    t0 = time.time()
    while len(got) < 4 and time.time() - t0 < 40:
        rclpy.spin_once(n, timeout_sec=0.2)
    # TF and ground truth need a moment more than the latched maps.
    t1 = time.time()
    while time.time() - t1 < 4.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    got['T_world_odom'] = None
    if 'elev' in got and odom:
        m = got['elev']
        try:
            # Latest transform, and the ground-truth pose nearest to it. A snapshot is taken with
            # the rover at rest, so the map's own stamp is not needed.
            t = tf.lookup_transform(m.header.frame_id, 'base_footprint', rclpy.time.Time())
            ts = Time.from_msg(t.header.stamp).nanoseconds * 1e-9
            o = min(odom, key=lambda q: abs(Time.from_msg(q.header.stamp).nanoseconds * 1e-9 - ts))
            dt = abs(Time.from_msg(o.header.stamp).nanoseconds * 1e-9 - ts)
            tr, q = t.transform.translation, t.transform.rotation
            T_mb = matrix(tr.x, tr.y, tr.z, *quat_to_rpy(q.x, q.y, q.z, q.w))
            p, oq = o.pose.pose.position, o.pose.pose.orientation
            T_wb = matrix(p.x, p.y, p.z, *quat_to_rpy(oq.x, oq.y, oq.z, oq.w))
            if dt < 0.1:
                got['T_world_odom'] = T_wb @ np.linalg.inv(T_mb)
        except Exception as e:  # noqa: BLE001
            print(f'no transform to the world frame, assuming odom = world: {e}')
    n.destroy_node()
    rclpy.shutdown()
    return got


def grid_of(occ):
    w, h, r = occ.info.width, occ.info.height, occ.info.resolution
    c = np.array(occ.data, dtype=float).reshape(h, w)
    c[c < 0] = np.nan
    ox, oy = occ.info.origin.position.x, occ.info.origin.position.y
    return c, (ox, ox + w * r, oy, oy + h * r)


def gt_on(xs, ys, world):
    p = os.path.join(HOME, '.ros/rover_3d_nav/ground_truth', f'{world}.npz')
    if not os.path.exists(p):
        return None
    d = np.load(p)
    z, x0, y1, r = d['z'], float(d['x_min']), float(d['y_max']), float(d['res'])
    i = np.clip(((y1 - ys) / r).astype(int), 0, z.shape[0] - 1)
    j = np.clip(((xs - x0) / r).astype(int), 0, z.shape[1] - 1)
    return z[i, j]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name')
    ap.add_argument('world')
    ap.add_argument('--out', default=os.path.join(HOME, 'rover_3d_nav_results/snapshots'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    got = grab()
    missing = {'elev', 'feat', 'lcm', 'gcm'} - set(got)
    if missing:
        print(f'missing topics: {sorted(missing)}')
    data = {}
    if 'elev' in got:
        m = got['elev']
        z = gridmap_layer(m, 'elevation')
        X, Y = gridmap_cell_xy(m, z.shape)
        T = got.get('T_world_odom')
        if T is not None:
            P = np.stack([X.ravel(), Y.ravel(), np.nan_to_num(z.ravel()), np.ones(z.size)])
            W = T @ P
            drift = np.hypot(*(T[:2, 3]))
            print(f'odom to world offset {drift:.2f} m, yaw {np.degrees(np.arctan2(T[1, 0], T[0, 0])):+.1f} deg')
            X, Y = W[0].reshape(z.shape), W[1].reshape(z.shape)
            z = np.where(np.isfinite(z), W[2].reshape(z.shape), np.nan)
            m.info.pose.position.x, m.info.pose.position.y = (T @ np.array(
                [m.info.pose.position.x, m.info.pose.position.y, 0, 1]))[:2]
        data.update(elevation=z, x=X, y=Y, robot=np.array([m.info.pose.position.x,
                                                           m.info.pose.position.y]))
        gt = gt_on(X.ravel(), Y.ravel(), a.world)
        if gt is not None:
            data['error'] = z - gt.reshape(z.shape)
    if 'feat' in got:
        f = got['feat']
        for k in ('best_utilisation', 'safe_yaw_mask', 'limiting_check', 'step', 'slope',
                  'roughness', 'negative_obstacle'):
            if k in f.layers:
                data[k] = gridmap_layer(f, k)
    for key in ('lcm', 'gcm'):
        if key in got:
            c, ext = grid_of(got[key])
            data[key] = c
            data[key + '_extent'] = np.array(ext)
    np.savez_compressed(os.path.join(a.out, a.name + '.npz'), **data)

    fs.apply()
    fig, axes = fs.plt.subplots(2, 3, figsize=(15, 9.5))
    rx, ry = data.get('robot', (0.0, 0.0))

    def show(ax, img, title, cmap, vmin=None, vmax=None, label='', extent=None):
        if img is None:
            ax.set_visible(False)
            return
        if extent is None:
            ext = (data['x'].min(), data['x'].max(), data['y'].min(), data['y'].max())
        im = ax.imshow(np.asarray(img), cmap=cmap, vmin=vmin, vmax=vmax,
                       extent=extent if extent is not None else ext, origin='upper',
                       interpolation='nearest')
        ax.plot(rx, ry, 'o', ms=7, color=fs.INK, mec=fs.SURFACE, mew=2)
        ax.set_title(title, loc='left')
        ax.grid(False)
        ax.set_aspect('equal')
        cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02)
        cb.outline.set_visible(False)
        if label:
            cb.set_label(label, color=fs.INK2)
        return cb

    def img_of(layer):
        """Layer as an image with +x to the right and +y up, from the cells' own coordinates."""
        if layer not in data:
            return None
        A, X, Y = np.asarray(data[layer]), data['x'], data['y']
        if abs(X[1, 0] - X[0, 0]) > abs(X[0, 1] - X[0, 0]):   # x runs down the rows
            A, X, Y = A.T, X.T, Y.T
        if X[0, -1] < X[0, 0]:
            A, Y = A[:, ::-1], Y[:, ::-1]
        if Y[-1, 0] > Y[0, 0]:
            A = A[::-1, :]
        return A

    show(axes[0, 0], img_of('elevation'), 'elevation (local map)', fs.SEQ_BLUE, label='m')
    show(axes[0, 1], img_of('error'), 'elevation minus ground truth', fs.DIVERGING,
         -0.05, 0.05, 'm')
    show(axes[0, 2], img_of('best_utilisation'), 'best utilisation u (lethal above 1)',
         fs.SEQ_BLUE, 0, 1.2)
    cb = show(axes[1, 0], img_of('limiting_check'), 'check that sets u',
              ListedColormap(fs.CATEGORICAL[:6]), -0.5, 5.5)
    if cb is not None:
        cb.set_ticks(range(6))
        cb.set_ticklabels(CHECKS)
    for ax, key, title in ((axes[1, 1], 'lcm', 'local costmap'),
                           (axes[1, 2], 'gcm', 'global costmap')):
        if key in data:
            c = data[key][::-1, :]
            show(ax, c, title, fs.SEQ_BLUE, 0, 100, 'cost (100 lethal)',
                 extent=tuple(data[key + '_extent']))
        else:
            ax.set_visible(False)
    if 'gcm' in data:
        axes[1, 2].set_xlim(rx - 12, rx + 12)
        axes[1, 2].set_ylim(ry - 12, ry + 12)
    stamp = time.strftime('%Y-%m-%d %H:%M')
    fig.suptitle(f'{a.name}  ({a.world}, {stamp})', color=fs.INK, fontsize=12)
    fig.tight_layout()
    fs.save(fig, os.path.join(a.out, a.name))
    if 'error' in data:
        e = data['error'][np.isfinite(data['error'])]
        print(f'error vs ground truth: mean {e.mean() * 100:+.2f} cm, rms '
              f'{np.sqrt(np.mean(e ** 2)) * 100:.2f} cm over {e.size} cells')
    print(os.path.join(a.out, a.name + '.png'))


if __name__ == '__main__':
    main()
