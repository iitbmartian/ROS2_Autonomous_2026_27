#!/usr/bin/env python3
"""Last local and global costmap in a rock_field bag, drawn with the world's rock footprints (red,
labelled with height) and the rover track. Args: bag dir, out png."""
import math, re, sys
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
W = '/home/ram/ROS2_Autonomous_2026_27/src/rover_gazebo/worlds/rock_field.sdf'
rocks = []
for m in re.finditer(r'<model name="(rock_\d+)">(.*?)</model>', open(W).read(), re.S):
    x, y, z = [float(v) for v in re.search(r'<pose>([^<]+)</pose>', m.group(2)).group(1).split()[:3]]
    e = re.search(r'<radii>([^<]+)</radii>', m.group(2)); b = re.search(r'<size>([^<]+)</size>', m.group(2))
    if e: a, bb, c = [float(v) for v in e.group(1).split()]; top = z + c
    elif b: a, bb, c = [float(v)/2 for v in b.group(1).split()]; top = z + c
    else: continue
    rocks.append((m.group(1), x, y, max(a, bb), top))
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1]), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
last = {}; track = []
while r.has_next():
    topic, data, t = r.read_next()
    if topic in ('/local_costmap/costmap', '/global_costmap/costmap'):
        last[topic] = deserialize_message(data, get_message(types[topic]))
    elif topic == '/terrain/hazard_points':
        last[topic] = deserialize_message(data, get_message(types[topic]))
    elif topic == '/odom':
        m = deserialize_message(data, get_message(types[topic])); track.append((m.pose.pose.position.x, m.pose.pose.position.y))
track = np.array(track)
from sensor_msgs_py import point_cloud2
hz = np.array([[q[0], q[1]] for q in point_cloud2.read_points(last['/terrain/hazard_points'], ('x', 'y'), skip_nans=True)])
print('hazard frame', last['/terrain/hazard_points'].header.frame_id, len(hz))
fig, axs = plt.subplots(1, 2, figsize=(16, 8))
for ax, k in zip(axs, ('/local_costmap/costmap', '/global_costmap/costmap')):
    m = last[k]; i = m.info
    g = np.array(m.data, float).reshape(i.height, i.width); g[g < 0] = np.nan
    ext = (i.origin.position.x, i.origin.position.x + i.width*i.resolution, i.origin.position.y, i.origin.position.y + i.height*i.resolution)
    im = ax.imshow(g, origin='lower', extent=ext, cmap='viridis', vmin=0, vmax=100)
    for n, x, y, rad, top in rocks:
        if ext[0] < x < ext[1] and ext[2] < y < ext[3]:
            ax.add_patch(plt.Circle((x, y), rad, fill=False, color='r', lw=1))
            if top > 0.05: ax.text(x, y, f'{top*100:.0f}', color='r', fontsize=7, clip_on=True)
    if len(hz): ax.plot(hz[:, 0], hz[:, 1], 'c.', ms=1)
    ax.plot(track[:, 0], track[:, 1], 'w-', lw=0.8); ax.plot(10, 0, 'm*', ms=12)
    ax.set_xlim(max(ext[0], 3), min(ext[1], 12)); ax.set_ylim(max(ext[2], -3), min(ext[3], 4))
    ax.set_title(f'{k} ({m.header.frame_id}), rock top cm in red'); fig.colorbar(im, ax=ax, shrink=0.7)
fig.tight_layout(); fig.savefig(sys.argv[2], dpi=110)
