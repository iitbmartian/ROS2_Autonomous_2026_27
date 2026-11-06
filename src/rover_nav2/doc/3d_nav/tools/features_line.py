"""Feature layers along a line. Usage: features_line.py TOPIC x0 y0 x1 y1 [n]"""
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from grid_map_msgs.msg import GridMap
from rover_nav2.benchmark.terrain_eval import gridmap_layer, gridmap_cell_xy
topic = sys.argv[1]; x0, y0, x1, y1 = map(float, sys.argv[2:6]); N = int(sys.argv[6]) if len(sys.argv) > 6 else 18
got = {}
rclpy.init(); n = Node('features_line')
n.create_subscription(GridMap, topic, lambda m: got.setdefault('f', m), QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
t0 = time.time()
while not got and time.time() - t0 < 40: rclpy.spin_once(n, timeout_sec=0.2)
f = got['f']; L = {k: gridmap_layer(f, k) for k in f.layers}
X, Y = gridmap_cell_xy(f, L['elevation'].shape)
keys = [k for k in ['elevation', 'variance', 'slope', 'step', 'roughness', 'safe_yaw_mask', 'best_utilisation', 'limiting_check'] if k in L]
print(topic, f.header.frame_id, 'res', f.info.resolution, '|', ' '.join(keys))
for t in np.linspace(0, 1, N):
    x, y = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
    k = np.argmin((X - x) ** 2 + (Y - y) ** 2)
    print(f'{x:5.2f} {y:5.2f} ', ' '.join(f'{L[q].flat[k]:8.4f}' for q in keys))
