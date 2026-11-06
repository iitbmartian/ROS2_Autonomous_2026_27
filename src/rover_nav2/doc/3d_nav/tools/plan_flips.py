#!/usr/bin/env python3
"""Do consecutive global plans flip sides? For each /plan, the lateral offset of the point 1.5 m
along the path from the rover, and the max distance between consecutive plans over their first 3 m.
Arg: bag dir."""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1]), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/plan']))
plans = []
while r.has_next():
    _, data, t = r.read_next()
    m = deserialize_message(data, Path)
    p = np.array([[q.pose.position.x, q.pose.position.y] for q in m.poses])
    if len(p) > 2:
        plans.append((t * 1e-9, p))

def head(p, L):
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
    return p[s <= L], s

def resample(p, L=3.0, n=30):
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
    q = np.linspace(0, min(L, s[-1]), n)
    return np.stack([np.interp(q, s, p[:, 0]), np.interp(q, s, p[:, 1])], 1)

t0 = plans[0][0]
prev = None
big = 0
for t, p in plans:
    q = resample(p)
    d = np.max(np.hypot(*(q - prev).T)) if prev is not None else 0.0
    big += d > 0.3
    prev = q
print(f'{len(plans)} plans, {big} consecutive pairs differ by more than 0.3 m in the first 3 m')
# print a few around the largest jumps
prev = None
rows = []
for t, p in plans:
    q = resample(p)
    if prev is not None:
        rows.append((np.max(np.hypot(*(q - prev).T)), t - t0, q[0], q[15], prev[15]))
    prev = q
for d, t, a, b, c in sorted(rows, key=lambda r: -r[0])[:8]:
    print(f't {t:6.1f} jump {d:.2f} m, start ({a[0]:.2f},{a[1]:.2f}), 1.5 m point now ({b[0]:.2f},{b[1]:.2f}) before ({c[0]:.2f},{c[1]:.2f})')
