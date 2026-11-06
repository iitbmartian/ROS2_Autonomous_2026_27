#!/usr/bin/env python3
"""Unsafe margin around each tall rock: 16 rays from the rock outline outward, distance to the first
cell with a safe yaw (safe_yaw_mask != 0). Median over rays that leave the rock clear of other tall
rocks within 1.5 m. Args: bag dir, topic, t seconds."""
import math, re, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
W = '/home/ram/ROS2_Autonomous_2026_27/src/rover_gazebo/worlds/rock_field.sdf'
rocks = []
for m in re.finditer(r'<model name="(rock_\w+)">(.*?)</model>', open(W).read(), re.S):
    b = m.group(2); x, y, z, _, _, yaw = [float(v) for v in re.search(r'<pose>([^<]+)</pose>', b).group(1).split()]
    e = re.search(r'<radii>([^<]+)</radii>', b); s = re.search(r'<size>([^<]+)</size>', b)
    if s: hx, hy, hz = [float(v) / 2 for v in s.group(1).split()]; box = True
    else: hx, hy, hz = [float(v) for v in e.group(1).split()]; box = False
    rocks.append((m.group(1), x, y, yaw, hx, hy, z + hz, box))
bag, topic, tq = sys.argv[1], sys.argv[2], float(sys.argv[3])
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
t0 = None; msg = None
while r.has_next():
    tp, data, t = r.read_next(); t0 = t0 or t
    if tp == topic and (t - t0) * 1e-9 >= tq: msg = deserialize_message(data, get_message(types[tp])); break
i = msg.info; res = i.resolution; rows, cols = int(round(i.length_x / res)), int(round(i.length_y / res))
L = {}
for name, arr in zip(msg.layers, msg.data):
    a = np.array(arr.data, np.float32).reshape(cols, rows).T
    L[name] = np.roll(a, (-msg.outer_start_index, -msg.inner_start_index), axis=(0, 1))
xmax, ymax = i.pose.position.x + i.length_x / 2, i.pose.position.y + i.length_y / 2
def cell(x, y):
    a, b = int((xmax - x) / res), int((ymax - y) / res)
    return (a, b) if 0 <= a < rows and 0 <= b < cols else None
mask = L['safe_yaw_mask']
tall = [q for q in rocks if q[6] >= 0.10]
print(f'{"rock":13s} {"top":>5s} {"rays":>4s} {"median":>6s} {"min":>5s} {"max":>5s}  (m from outline to first safe cell)')
for n, x, y, yaw, hx, hy, top, box in tall:
    if cell(x - 1.6, y - 1.6) is None or cell(x + 1.6, y + 1.6) is None: continue
    out = []
    for k in range(16):
        th = 2 * math.pi * k / 16; ux, uy = math.cos(th), math.sin(th)
        # outline distance along this ray (rock frame), boxes exact, ellipsoids by radii
        c, s = math.cos(th - yaw), math.sin(th - yaw)
        r0 = min(hx / abs(c) if abs(c) > 1e-6 else 9, hy / abs(s) if abs(s) > 1e-6 else 9) if box else 1 / math.hypot(c / hx, s / hy)
        if any(o[0] != n and math.hypot(o[1] - (x + ux * (r0 + 0.75)), o[2] - (y + uy * (r0 + 0.75))) < 0.9 for o in tall):
            continue  # another tall rock on this ray
        d = 0.0; hit = None
        while d < 1.5:
            q = cell(x + ux * (r0 + d), y + uy * (r0 + d))
            if q is None or not np.isfinite(mask[q]): break
            if mask[q] != 0: hit = d; break
            d += res / 2
        if hit is not None: out.append(hit)
    if out: print(f'{n:13s} {top:5.2f} {len(out):4d} {np.median(out):6.2f} {min(out):5.2f} {max(out):5.2f}')
