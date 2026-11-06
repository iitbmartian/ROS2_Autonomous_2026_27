#!/usr/bin/env python3
"""During a stall (rover static for 20 s+): command stats at each stage, and local costmap cost
along the current plan ahead of the rover. Arg: bag dir."""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1]), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ('/odom', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/plan', '/local_costmap/costmap')
d = {k: [] for k in want}
while r.has_next():
    topic, data, t = r.read_next()
    if topic not in want: continue
    m = deserialize_message(data, get_message(types[topic])); t *= 1e-9
    if topic == '/odom':
        p = m.pose.pose; q = p.orientation
        d[topic].append((t, p.position.x, p.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
    elif topic.startswith('/cmd'):
        d[topic].append((t, m.twist.linear.x, m.twist.linear.y, m.twist.angular.z))
    elif topic == '/plan':
        d[topic].append((t, m.header.frame_id, np.array([[q.pose.position.x, q.pose.position.y] for q in m.poses])))
    else:
        d[topic].append((t, m))
od = np.array(d['/odom'])
# stall window: longest run where position stays within 0.15 m
best = (0, 0); i0 = 0
for i in range(len(od)):
    while math.hypot(od[i,1]-od[i0,1], od[i,2]-od[i0,2]) > 0.15: i0 += 1
    if od[i,0]-od[i0,0] > best[1]-best[0]: best = (od[i0,0], od[i,0])
t0, t1 = best
print(f'stall {t1-t0:.0f} s from t={t0-od[0,0]:.0f}s at ({od[od[:,0]>=t0][0,1]:.2f},{od[od[:,0]>=t0][0,2]:.2f})')
for k in ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel'):
    a = np.array([c for c in d[k] if t0 <= c[0] <= t1])
    if len(a) == 0: print(k, 'none'); continue
    print(f'{k:18s} n={len(a)} rate={len(a)/(t1-t0):.1f}Hz vx mean {a[:,1].mean():+.3f} |vx|>0.05 {np.mean(abs(a[:,1])>0.05):.2f} vy {a[:,2].mean():+.3f} wz mean {a[:,3].mean():+.3f} |wz| {abs(a[:,3]).mean():.3f}')
# costmap along plan, mid-stall
tm = (t0+t1)/2
pl = min(d['/plan'], key=lambda p: abs(p[0]-tm)); cm = min(d['/local_costmap/costmap'], key=lambda c: abs(c[0]-tm))[1]
ox, oy, res, w, h = cm.info.origin.position.x, cm.info.origin.position.y, cm.info.resolution, cm.info.width, cm.info.height
g = np.array(cm.data).reshape(h, w)
print('plan frame', pl[1], 'costmap frame', cm.header.frame_id, 'plan len', len(pl[2]))
rx, ry = od[od[:,0]>=tm][0,1:3]
print(f'rover ({rx:.2f},{ry:.2f}) first plan pose {pl[2][0]} last {pl[2][-1]}')
for p in pl[2][::5][:25]:
    i, j = int((p[0]-ox)/res), int((p[1]-oy)/res)
    c = g[j, i] if 0 <= i < w and 0 <= j < h else None
    print(f'  ({p[0]:.2f},{p[1]:.2f}) cost {c}')
# patch around the rover
i, j = int((rx-ox)/res), int((ry-oy)/res)
print('local costmap around rover (rows +y up, 0.1 m step):')
for jj in range(j+20, j-21, -4):
    print(' '.join(f'{g[jj,ii]:3d}' if 0<=ii<w and 0<=jj<h else '  .' for ii in range(i-10, i+31, 2)))
