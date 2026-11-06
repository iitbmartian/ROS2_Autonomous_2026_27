"""Local costmap and feature map around the rover: cost classes within 1.5 m of the centre."""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from grid_map_msgs.msg import GridMap
from rover_nav2.benchmark.terrain_eval import gridmap_layer, gridmap_cell_xy
got = {}
class N(Node):
    def __init__(s):
        super().__init__('costmap_near')
        s.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: got.setdefault('cm', m), 1)
        s.create_subscription(GridMap, '/terrain/local_features', lambda m: got.setdefault('f', m),
                              QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
rclpy.init(); n = N(); t = time.time()
while len(got) < 2 and time.time() - t < 30: rclpy.spin_once(n, timeout_sec=0.2)
cm = got['cm']; w, h, r = cm.info.width, cm.info.height, cm.info.resolution
c = np.array(cm.data).reshape(h, w)
cx, cy = w // 2, h // 2
yy, xx = np.mgrid[0:h, 0:w]
d = np.hypot(xx - cx, yy - cy) * r
for lo, hi in [(0, 0.76), (0.76, 1.01), (1.01, 1.5)]:
    band = c[(d >= lo) & (d < hi)]
    print(f'costmap {lo:.2f}-{hi:.2f} m: lethal(100) {np.mean(band == 100):.0%}, 99 {np.mean(band == 99):.0%}, '
          f'unknown-cost(78) {np.mean(band == 78):.0%}, low(<50) {np.mean((band >= 0) & (band < 50)):.0%}')
f = got['f']
u = gridmap_layer(f, 'best_utilisation'); m = gridmap_layer(f, 'safe_yaw_mask'); z = gridmap_layer(f, 'elevation')
X, Y = gridmap_cell_xy(f, z.shape)
dd = np.hypot(X - f.info.pose.position.x, Y - f.info.pose.position.y)
for lo, hi in [(0, 0.76), (0.76, 1.01), (1.01, 1.5)]:
    sel = (dd >= lo) & (dd < hi)
    print(f'features {lo:.2f}-{hi:.2f} m: elevation known {np.mean(np.isfinite(z[sel])):.0%}, verdict known {np.mean(np.isfinite(u[sel])):.0%}, '
          f'no safe heading {np.mean(m[sel] == 0):.0%}, u>1 {np.mean(u[sel] > 1):.0%}')
