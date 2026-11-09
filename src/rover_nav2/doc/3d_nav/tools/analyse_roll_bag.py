#!/usr/bin/env python3
"""Roll timeline of a recorded run: prints the 8 s before and 1 s after the largest |roll| (IMU,
base frame assumed aligned), with the commanded twist, pose and steer and rocker joints.
Arg: bag dir. Output goes to stdout and <bag>.roll.txt."""
import math, sys, bisect
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1].rstrip('/')
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = {'/imu/data', '/odom', '/joint_states', '/cmd_vel_nav', '/cmd_vel', '/speed_limit'}
S = {k: [] for k in want}
while r.has_next():
    topic, data, t = r.read_next()
    if topic not in want or topic not in types: continue
    m = deserialize_message(data, get_message(types[topic])); t = t * 1e-9
    if topic == '/imu/data':
        q = m.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        S[topic].append((t, math.degrees(roll), math.degrees(pitch)))
    elif topic == '/odom':
        p = m.pose.pose.position; q = m.pose.pose.orientation
        S[topic].append((t, p.x, p.y, math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))),
                         m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z))
    elif topic in ('/cmd_vel_nav', '/cmd_vel'):
        tw = m.twist if hasattr(m, 'twist') else m
        S[topic].append((t, tw.linear.x, tw.linear.y, tw.angular.z))
    elif topic == '/speed_limit':
        S[topic].append((t, m.speed_limit))
    elif topic == '/joint_states':
        S[topic].append((t, dict(zip(m.name, m.position))))

def at(L, t):
    if not L: return None
    i = bisect.bisect_left([x[0] for x in L], t); i = min(max(i, 0), len(L) - 1); return L[i]

imu = S['/imu/data']
out = [f"{bag}: {len(imu)} imu, {len(S['/odom'])} odom, {len(S['/cmd_vel_nav'])} cmd_vel_nav, {len(S['/joint_states'])} joint_states"]
if imu:
    tm, rm, pm = max(imu, key=lambda x: abs(x[1]))
    out.append(f"max |roll| {rm:.1f} deg (pitch {pm:.1f}) at t+{tm - imu[0][0]:.1f} s")
    js0 = at(S['/joint_states'], tm); steer = sorted(k for k in (js0[1] if js0 else {}) if 'steer' in k.lower() or 'str' in k.lower())
    rock = sorted(k for k in (js0[1] if js0 else {}) if k.endswith('LS_joint') or k.endswith('RS_joint'))
    out.append("  dt    roll  pitch |   x      y    yaw  |  vx    vy    wz (odom) | nav vx vy wz | lim | steer deg " + ' '.join(steer) + " | rockers " + ' '.join(rock))
    t = tm - 8.0
    while t <= tm + 1.0:
        i = at(imu, t); o = at(S['/odom'], t); c = at(S['/cmd_vel_nav'], t); l = at(S['/speed_limit'], t); j = at(S['/joint_states'], t)
        line = f"{t - tm:+5.1f} {i[1]:6.1f} {i[2]:6.1f} |"
        line += f" {o[1]:6.2f} {o[2]:6.2f} {o[3]:6.0f} | {o[4]:5.2f} {o[5]:5.2f} {o[6]:5.2f} |" if o else " - |"
        line += f" {c[1]:5.2f} {c[2]:5.2f} {c[3]:5.2f} |" if c else " - |"
        line += f" {l[1]:.2f} |" if l else " - |"
        if j: line += ' ' + ' '.join(f"{math.degrees(j[1][k]):5.0f}" for k in steer) + ' | ' + ' '.join(f"{math.degrees(j[1][k]):5.1f}" for k in rock)
        out.append(line); t += 0.25
txt = '\n'.join(out); print(txt); open(bag + '.roll.txt', 'w').write(txt + '\n')
