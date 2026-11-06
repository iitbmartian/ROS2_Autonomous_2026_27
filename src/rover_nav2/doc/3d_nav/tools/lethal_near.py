"""Terrain-lethal cells (no safe heading or u > 1) within 1.6 m of the rover, with the layers that explain them."""
import time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from grid_map_msgs.msg import GridMap
from rover_nav2.benchmark.terrain_eval import gridmap_layer, gridmap_cell_xy
got = {}
rclpy.init(); n = Node('lethal_near')
n.create_subscription(GridMap, '/terrain/local_features', lambda m: got.setdefault('f', m),
                      QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
t0 = time.time()
while not got and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.2)
f = got['f']
L = {k: gridmap_layer(f, k) for k in f.layers}
X, Y = gridmap_cell_xy(f, L['elevation'].shape)
cx, cy = f.info.pose.position.x, f.info.pose.position.y
d = np.hypot(X - cx, Y - cy)
sel = (d < 1.6) & ((L['safe_yaw_mask'] == 0) | (L['best_utilisation'] > 1))
keys = [k for k in ['elevation', 'variance', 'step', 'roughness', 'slope', 'best_utilisation', 'limiting_check'] if k in L]
print('lethal cells within 1.6 m:', sel.sum(), ' limiting checks:', np.unique(L['limiting_check'][sel], return_counts=True))
for i, j in list(zip(*np.nonzero(sel)))[:12]:
    print(f'dx {X[i,j]-cx:+.2f} dy {Y[i,j]-cy:+.2f}', ' '.join(f'{k} {L[k][i,j]:.4f}' for k in keys))
