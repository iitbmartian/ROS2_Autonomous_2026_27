"""Elevation map minus ground truth in a box, binned by range from the rover. Usage: map_vs_gt.py TOPIC GT.npz xmin xmax ymin ymax"""
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from grid_map_msgs.msg import GridMap
from rover_nav2.benchmark.terrain_eval import gridmap_layer, gridmap_cell_xy
topic, gtp = sys.argv[1], sys.argv[2]; x0, x1, y0, y1 = map(float, sys.argv[3:7])
gt = np.load(gtp); Z = gt['z']; gx, gy, gr = float(gt['x_min']), float(gt['y_max']), float(gt['res'])
got = {}
rclpy.init(); n = Node('map_vs_gt')
n.create_subscription(GridMap, topic, lambda m: got.setdefault('m', m), QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
t0 = time.time()
while not got and time.time() - t0 < 40: rclpy.spin_once(n, timeout_sec=0.2)
m = got['m']; z = gridmap_layer(m, 'elevation'); X, Y = gridmap_cell_xy(m, z.shape)
sel = np.isfinite(z) & (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
i = ((gy - Y[sel]) / gr).astype(int); j = ((X[sel] - gx) / gr).astype(int)
e = z[sel] - Z[i, j]
d = np.hypot(X[sel] - m.info.pose.position.x, Y[sel] - m.info.pose.position.y)
print(f'{topic}: {sel.sum()} cells, error mean {e.mean()*100:+.2f} cm, std {e.std()*100:.2f} cm, |e|>5cm {np.mean(np.abs(e) > 0.05):.1%}')
for lo in range(0, 6):
    s = (d >= lo) & (d < lo + 1)
    if s.sum() > 10: print(f'  range {lo}-{lo+1} m: n {s.sum()} mean {e[s].mean()*100:+.2f} std {e[s].std()*100:.2f} cm')
