#!/usr/bin/env python3
"""Keeps the last /replay/off and /replay/on global feature maps and, on SIGINT, compares them:
cells where the fine step raised the step, where that crosses step_max (lethal by step), where the
best utilisation rose, and a map of the differences (png). Args: out prefix."""
import signal, sys
import numpy as np, rclpy, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from rover_nav2.benchmark.terrain_eval import gridmap_layer, gridmap_cell_xy

STEP_MAX = 0.10
got = {}
rclpy.init(); n = Node('replay_cmp')
for k in ('off', 'on'):
    n.create_subscription(GridMap, f'/replay/{k}', lambda m, k=k: got.__setitem__(k, m), 1)
stop = [False]
signal.signal(signal.SIGINT, lambda *a: stop.__setitem__(0, True))
while not stop[0]:
    rclpy.spin_once(n, timeout_sec=0.2)
if 'off' not in got or 'on' not in got:
    print('missing maps', list(got)); sys.exit(1)
off, on = got['off'], got['on']
s0, s1 = gridmap_layer(off, 'step'), gridmap_layer(on, 'step')
u0, u1 = gridmap_layer(off, 'best_utilisation'), gridmap_layer(on, 'best_utilisation')
d = s1 - s0
ok = np.isfinite(d)
print(f'cells with step {ok.sum()}, raised >0.02 m: {(d[ok] > 0.02).sum()}, '
      f'newly over step_max: {((s1 > STEP_MAX) & ~(s0 > STEP_MAX) & ok).sum()}')
lu = np.isfinite(u0) & np.isfinite(u1)
print(f'best utilisation: cells newly >= 1: {((u1 >= 1) & (u0 < 1) & lu).sum()}, '
      f'mean rise where raised {np.nanmean((u1 - u0)[lu & (d > 0.02)]) if (lu & (d > 0.02)).any() else 0:.3f}')
X, Y = gridmap_cell_xy(on, s1.shape)
idx = np.argwhere(ok & (d > 0.02))
order = np.argsort(-d[ok & (d > 0.02)])
print('largest raises (x, y, step off -> on, u off -> on):')
for i, j in idx[order][:25]:
    x, y = X[i, j], Y[i, j]
    print(f'  {x:6.2f} {y:6.2f}  {s0[i, j]:.3f} -> {s1[i, j]:.3f}   {u0[i, j]:.2f} -> {u1[i, j]:.2f}')
fig, ax = plt.subplots(1, 2, figsize=(11, 5))
ax[0].imshow(s0, vmin=0, vmax=0.2); ax[0].set_title('global step, fine off')
im = ax[1].imshow(np.where(ok, d, np.nan), vmin=0, vmax=0.15); ax[1].set_title('fine on minus off')
fig.colorbar(im, ax=ax[1]); fig.savefig(sys.argv[1] + '.png', dpi=110)
