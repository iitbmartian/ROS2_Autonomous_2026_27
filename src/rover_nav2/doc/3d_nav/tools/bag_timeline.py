#!/usr/bin/env python3
"""Timeline of a recorded nav run, one line per interval: odom xy and distance to the goal, the
command at each stage (cmd_vel_nav, cmd_vel_smoothed, cmd_vel), mean |wheel joint velocity| and the
BT nodes that changed state. Shows where a stall sits in the command chain.
Args: bag goal_x goal_y [interval_s]."""
import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, gx, gy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
dt = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/odom', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/joint_states', '/behavior_tree_log']
r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
cls = {t: get_message(types[t]) for t in want if t in types}
t0 = None; last = {}; bt = []; nxt = None
def tw(m): return f'{m.twist.linear.x:+.2f},{m.twist.angular.z:+.2f}' if m else '  --  '
while r.has_next():
    topic, data, t = r.read_next(); t *= 1e-9
    if t0 is None: t0 = t; nxt = t
    m = deserialize_message(data, cls[topic])
    if topic == '/behavior_tree_log':
        for e in m.event_log:
            if e.current_status in ('RUNNING', 'FAILURE') and e.node_name not in bt[-6:]:
                bt.append(f'{e.node_name}:{e.current_status[0]}')
        continue
    last[topic] = m
    if t >= nxt and '/odom' in last:
        p = last['/odom'].pose.pose.position
        js = last.get('/joint_states')
        wv = (sum(abs(v) for n, v in zip(js.name, js.velocity) if 'wheel' in n) /
              max(1, sum(1 for n in js.name if 'wheel' in n))) if js and js.velocity else float('nan')
        print(f'{t - t0:7.1f} xy {p.x:6.2f} {p.y:6.2f} d {math.hypot(gx - p.x, gy - p.y):5.2f} | nav {tw(last.get("/cmd_vel_nav"))}'
              f' sm {tw(last.get("/cmd_vel_smoothed"))} out {tw(last.get("/cmd_vel"))} | wheel {wv:5.2f} | {" ".join(bt[-4:])}')
        bt = []; nxt += dt
