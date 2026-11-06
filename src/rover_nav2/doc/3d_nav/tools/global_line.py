"""Global costmap values along a straight line from (x0, y0) to (x1, y1). Usage: global_line.py x0 y0 x1 y1"""
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
x0, y0, x1, y1 = map(float, sys.argv[1:5])
got = {}
rclpy.init(); n = Node('global_line')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: got.setdefault('g', m), qos)
t0 = time.time()
while not got and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.2)
g = got['g']; w, h, r = g.info.width, g.info.height, g.info.resolution
ox, oy = g.info.origin.position.x, g.info.origin.position.y
c = np.array(g.data).reshape(h, w)
print(f'global costmap {w}x{h} at {r} m, origin ({ox:.1f}, {oy:.1f}), unknown share {np.mean(c < 0):.0%}')
row = []
for t in np.linspace(0, 1, 41):
    x, y = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
    i, j = int((y - oy) / r), int((x - ox) / r)
    row.append(f'{x:.1f}:{c[i, j]}')
print(' '.join(row))
