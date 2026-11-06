#!/usr/bin/env python3
"""Terrain grids from a bag at the moment the rover has been static longest (or at --t seconds),
layers drawn in world frame with rock footprints. Args: bag dir, grid topic, out png [t]."""
import math, re, sys
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
W = '/home/ram/ROS2_Autonomous_2026_27/src/rover_gazebo/worlds/rock_field.sdf'
rocks = []
for m in re.finditer(r'<model name="(rock_\d+)">(.*?)</model>', open(W).read(), re.S):
    x, y, z = [float(v) for v in re.search(r'<pose>([^<]+)</pose>', m.group(2)).group(1).split()[:3]]
    e = re.search(r'<radii>([^<]+)</radii>', m.group(2)); b = re.search(r'<size>([^<]+)</size>', m.group(2))
    v = [float(q) for q in (e or b).group(1).split()]
    if b and not e: v = [q / 2 for q in v]
    rocks.append((x, y, max(v[0], v[1]), z + v[2]))

def decode(m):
    """grid_map msg -> dict layer -> (rows along -x, cols along -y) unwrapped, plus extent."""
    i = m.info; res = i.resolution
    rows, cols = int(round(i.length_x / res)), int(round(i.length_y / res))
    out = {}
    for name, arr in zip(m.layers, m.data):
        a = np.array(arr.data, np.float32).reshape(cols, rows).T  # column-major
        a = np.roll(a, (-m.outer_start_index, -m.inner_start_index), axis=(0, 1))
        out[name] = a
    cx, cy = i.pose.position.x, i.pose.position.y
    # row 0 at +x edge, col 0 at +y edge
    return out, (cx - i.length_x / 2, cx + i.length_x / 2, cy - i.length_y / 2, cy + i.length_y / 2), m.header.frame_id

bag, topic, png = sys.argv[1:4]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
grids, od = [], []
t_first = None
while r.has_next():
    tp, data, t = r.read_next()
    t_first = t_first or t
    if (t - t_first) * 1e-9 > 400: break
    if tp == '/odom':
        m = deserialize_message(data, get_message(types[tp])); od.append((t * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y))
    elif tp == topic:
        grids.append((t * 1e-9, data))
od = np.array(od)
if len(sys.argv) > 4:
    tsel = od[0, 0] + float(sys.argv[4])
else:  # middle of the longest static spell
    best = (0, 0); i0 = 0
    for k in range(len(od)):
        while math.hypot(od[k, 1] - od[i0, 1], od[k, 2] - od[i0, 2]) > 0.15: i0 += 1
        if od[k, 0] - od[i0, 0] > best[1] - best[0]: best = (od[i0, 0], od[k, 0])
    tsel = sum(best) / 2
    print(f'static {best[1]-best[0]:.0f} s')
t, data = min(grids, key=lambda g: abs(g[0] - tsel))
m = deserialize_message(data, get_message(types[topic]))
L, ext, frame = decode(m)
rx, ry = od[np.searchsorted(od[:, 0], t), 1:3]
print(f't={t-od[0,0]:.0f}s rover ({rx:.2f},{ry:.2f}) frame {frame} layers {list(L)} extent {ext}')
show = [k for k in ('elevation', 'step', 'slope', 'roughness', 'negative_obstacle', 'best_utilisation', 'safe_yaw_mask', 'observations') if k in L]
n = len(show); fig, axs = plt.subplots(2, (n + 1) // 2, figsize=(5 * ((n + 1) // 2), 9)); axs = np.ravel(axs)
for ax, k in zip(axs, show):
    a = L[k][::-1, ::-1].T  # -> rows along +y, cols along +x, origin lower
    im = ax.imshow(a, origin='lower', extent=ext, cmap='viridis')
    fig.colorbar(im, ax=ax, shrink=0.7); ax.set_title(k)
    for x, y, rad, top in rocks:
        if ext[0] < x < ext[1] and ext[2] < y < ext[3]:
            ax.add_patch(plt.Circle((x, y), rad, fill=False, color='r', lw=0.8))
    ax.plot(rx, ry, 'w^', ms=8); ax.plot(10, 0, 'm*', ms=10)
    fin = a[np.isfinite(a)]
    if len(fin): print(f'  {k:18s} min {fin.min():.3f} p50 {np.median(fin):.3f} p99 {np.percentile(fin, 99):.3f} max {fin.max():.3f}')
fig.suptitle(f'{topic} t={t-od[0,0]:.0f}s'); fig.tight_layout(); fig.savefig(png, dpi=100)
