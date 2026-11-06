"""Print one sample each of: local costmap value histogram, and per-layer finite share and range
of /terrain/local_features. Exits after both or 60 s."""
import collections, math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from grid_map_msgs.msg import GridMap

class P(Node):
    def __init__(s):
        super().__init__('terrain_stats')
        s.done = set()
        s.create_subscription(OccupancyGrid, '/local_costmap/costmap', s.cm, 1)
        s.create_subscription(GridMap, '/terrain/local_features', s.gm, 1)
    def cm(s, m):
        if 'cm' in s.done: return
        c = collections.Counter(m.data)
        rng = lambda a, b: sum(v for k, v in c.items() if a <= k <= b)
        print('local_costmap', f'{m.info.width}x{m.info.height}', 'stamp', m.header.stamp.sec,
              {'unknown': c[-1], 'free 0': c[0], '1-49': rng(1, 49), '50-98': rng(50, 98),
               'inscribed 99': c[99], 'lethal 100': c[100]}, flush=True)
        s.done.add('cm')
    def gm(s, m):
        if 'gm' in s.done: return
        out = {}
        for name, arr in zip(m.layers, m.data):
            fin = [x for x in arr.data if not math.isnan(x)]
            out[name] = (round(len(fin) / max(1, len(arr.data)), 3),
                         round(min(fin), 3) if fin else None, round(max(fin), 3) if fin else None)
        print('features', m.header.frame_id, 'stamp', m.header.stamp.sec, out, flush=True)
        s.done.add('gm')

rclpy.init(); n = P(); t = time.time()
while len(n.done) < 2 and time.time() - t < 60:
    rclpy.spin_once(n, timeout_sec=0.5)
