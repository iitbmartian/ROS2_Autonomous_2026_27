#!/usr/bin/env python3
"""far_south stall from bag_far_south.sh: stall window, commands, speed limit, the plan's end and
cusps, local costmap under the footprint at the rover and at the goal, and the BT events.
Arg: bag dir. Goal (30, -20, yaw -0.6), footprint 1.1 x 0.9 m."""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
GX, GY, GYAW, FL, FW = 30.0, -20.0, -0.6, 1.1, 0.9
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1]), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ('/odom', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/plan', '/local_costmap/costmap',
        '/speed_limit', '/behavior_tree_log')
d = {k: [] for k in want}
while r.has_next():
    topic, data, t = r.read_next()
    if topic not in want: continue
    m = deserialize_message(data, get_message(types[topic])); t *= 1e-9
    if topic == '/odom':
        p = m.pose.pose; q = p.orientation
        d[topic].append((t, p.position.x, p.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
    elif topic.startswith('/cmd'):
        tw = m.twist if hasattr(m, 'twist') else m
        d[topic].append((t, tw.linear.x, tw.linear.y, tw.angular.z))
    elif topic == '/plan':
        d[topic].append((t, np.array([[q.pose.position.x, q.pose.position.y,
                                       2*math.atan2(q.pose.orientation.z, q.pose.orientation.w)] for q in m.poses])))
    elif topic == '/speed_limit':
        d[topic].append((t, m.speed_limit, m.percentage))
    elif topic == '/behavior_tree_log':
        for e in m.event_log:
            if e.current_status in ('FAILURE', 'SUCCESS') or e.node_name.lower().startswith(('backup', 'wait', 'clear', 'compute', 'follow')):
                d[topic].append((t, e.node_name, e.previous_status, e.current_status))
    else:
        d[topic].append((t, m))
od = np.array(d['/odom']); T0 = od[0, 0]
best = (0, 0); i0 = 0
for i in range(len(od)):
    while math.hypot(od[i, 1]-od[i0, 1], od[i, 2]-od[i0, 2]) > 0.15: i0 += 1
    if od[i, 0]-od[i0, 0] > best[1]-best[0]: best = (od[i0, 0], od[i, 0])
t0, t1 = best
k = od[:, 0] >= t0
sx, sy, syaw = od[k][0, 1:4]
print(f'stall {t1-t0:.0f} s from t={t0-T0:.0f} s at ({sx:.2f},{sy:.2f}) yaw {math.degrees(syaw):.0f} deg, '
      f'goal {math.hypot(GX-sx, GY-sy):.2f} m away at bearing {math.degrees(math.atan2(GY-sy, GX-sx)-syaw):.0f} deg from heading')
for key in ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel'):
    a = np.array([c[1:] for c in d[key] if t0 <= c[0] <= t1])
    if len(a) == 0: print(key, 'none'); continue
    print(f'{key:18s} n={len(a)} vx mean {a[:,0].mean():+.3f} min {a[:,0].min():+.3f} max {a[:,0].max():+.3f} '
          f'|vx|>0.03 {np.mean(abs(a[:,0])>0.03):.2f} wz mean {a[:,2].mean():+.3f}')
sl = np.array([c[1:] for c in d['/speed_limit'] if t0 <= c[0] <= t1])
if len(sl): print(f'speed_limit in stall: mean {sl[:,0].mean():.3f} min {sl[:,0].min():.3f} max {sl[:,0].max():.3f} (percentage={bool(sl[0,1])})')
plans = [p for p in d['/plan'] if t0 <= p[0] <= t1]
print(f'plans published in stall: {len(plans)}')
for tp, P in plans[:: max(1, len(plans)//4)][:5]:
    seg = np.diff(P[:, :2], axis=0); hd = np.array([math.cos(a) for a in P[:-1, 2]]), np.array([math.sin(a) for a in P[:-1, 2]])
    fwd = seg[:, 0]*hd[0] + seg[:, 1]*hd[1]
    cusps = int(np.sum(np.diff(np.sign(fwd[abs(fwd) > 1e-4])) != 0))
    L = np.sum(np.hypot(seg[:, 0], seg[:, 1]))
    print(f'  t={tp-T0:.0f} s plan {len(P)} poses {L:.2f} m, cusps {cusps}, reverse share {np.mean(fwd < 0):.2f}, '
          f'start ({P[0,0]:.2f},{P[0,1]:.2f}) end ({P[-1,0]:.2f},{P[-1,1]:.2f},{math.degrees(P[-1,2]):.0f} deg)')
cm = min(d['/local_costmap/costmap'], key=lambda c: abs(c[0]-(t0+t1)/2))[1]
ox, oy, res, w, h = cm.info.origin.position.x, cm.info.origin.position.y, cm.info.resolution, cm.info.width, cm.info.height
g = np.array(cm.data).reshape(h, w)
def foot(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw); vals = []
    for u in np.arange(-FL/2, FL/2+1e-6, res):
        for v in np.arange(-FW/2, FW/2+1e-6, res):
            i, j = int((x+c*u-s*v-ox)/res), int((y+s*u+c*v-oy)/res)
            if 0 <= i < w and 0 <= j < h: vals.append(g[j, i])
    v = np.array(vals); return f'max {v.max()} mean {v.mean():.0f} cells>=253 {np.sum(v>=253)} unknown {np.sum(v==255)} of {len(v)}'
print('costmap frame', cm.header.frame_id)
print('footprint at rover:', foot(sx, sy, syaw))
print('footprint at goal :', foot(GX, GY, GYAW))
i, j = int((GX-ox)/res), int((GY-oy)/res)
print('local costmap around the goal (rows +y up, 0.1 m step, G = goal):')
for jj in range(j+12, j-13, -2):
    print(' '.join(('  G' if (ii, jj) == (i, j) else f'{g[jj,ii]:3d}') if 0<=ii<w and 0<=jj<h else '  .' for ii in range(i-16, i+17, 2)))
ev = [e for e in d['/behavior_tree_log'] if e[0] >= t0 - 30]
print('BT events from 30 s before the stall:')
last = None
for e in ev:
    s = f'{e[1]} {e[2]}->{e[3]}'
    if s != last: print(f'  t={e[0]-T0:6.1f} {s}')
    last = s
