#!/usr/bin/env python3
"""rock_field bag: when the collision monitor holds the rover, how far the nearest hazard point
is from the rover centre, what the local costmap says there, and how many plan poses sit on
hazard cells. Arg: bag dir."""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1]), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
odom, haz, cm, lcm, plan = [], [], [], [], []
while r.has_next():
    topic, data, t = r.read_next()
    if topic not in ('/odom', '/terrain/hazard_points', '/collision_monitor_state',
                     '/local_costmap/costmap', '/plan'):
        continue
    m = deserialize_message(data, get_message(types[topic]))
    t = t * 1e-9
    if topic == '/odom':
        p = m.pose.pose; odom.append((t, p.position.x, p.position.y))
    elif topic == '/terrain/hazard_points':
        pts = np.array([[q[0], q[1]] for q in point_cloud2.read_points(m, ('x', 'y'), skip_nans=True)])
        haz.append((t, m.header.frame_id, pts))
    elif topic == '/collision_monitor_state':
        cm.append((t, m.action_type))
    elif topic == '/local_costmap/costmap':
        lcm.append((t, m))
    elif topic == '/plan':
        plan.append((t, np.array([[q.pose.position.x, q.pose.position.y] for q in m.poses])))

def at(seq, t):
    i = np.searchsorted([s[0] for s in seq], t)
    return seq[min(max(i - 1, 0), len(seq) - 1)]

print(f'odom {len(odom)} hazard {len(haz)} cm_state {len(cm)} local_costmap {len(lcm)} plan {len(plan)}')
print('hazard frame', haz[0][1] if haz else None, ', odom gt frame is world, map starts at spawn')
names = {0: 'none', 1: 'stop', 2: 'slowdown', 3: 'approach', 4: 'limit'}
from collections import Counter
print('cm states', Counter(names.get(a, a) for _, a in cm))
# Each change into a holding state: rover pose, nearest hazard, its local cost.
prev = 0
for t, a in cm:
    if a != 0 and prev == 0:
        _, x, y = at(odom, t)
        _, fr, pts = at(haz, t)
        d = np.hypot(pts[:, 0] - x, pts[:, 1] - y) if len(pts) else np.array([np.inf])
        k = int(np.argmin(d))
        _, c = at(lcm, t)
        info = c.info
        def cost(px, py):
            i = int((px - info.origin.position.x) / info.resolution)
            j = int((py - info.origin.position.y) / info.resolution)
            if 0 <= i < info.width and 0 <= j < info.height:
                return c.data[j * info.width + i]
            return None
        near = int((d < 0.5).sum())
        hx, hy = (pts[k] if len(pts) else (math.nan, math.nan))
        print(f't {t - cm[0][0]:6.1f} {names.get(a, a):9s} rover ({x:5.2f},{y:5.2f}) cost {cost(x, y)} '
              f'nearest hazard {d[k]:.2f} m at ({hx:5.2f},{hy:5.2f}) cost there {cost(hx, hy)}, '
              f'{near} hazards within 0.5 m')
    prev = a
# Plan poses on or next to hazard cells (0.1 m), per plan message, sampled.
hits = []
for t, pp in plan[::max(1, len(plan) // 20)]:
    _, fr, pts = at(haz, t)
    if len(pts) == 0 or len(pp) == 0:
        continue
    dm = np.min(np.hypot(pp[:, None, 0] - pts[None, :, 0], pp[:, None, 1] - pts[None, :, 1]), axis=1)
    hits.append(int((dm < 0.1).sum()))
print('plan poses within 0.1 m of a hazard point, sampled plans:', hits)
