#!/usr/bin/env python3
"""Rocker coupling residuals in a bag, relative to their values in the first second (the rest
pose the plugin captured): e1 = BLS + 0.577165 FLS, e2 = BRS + 0.572734 FRS, e3 = BLS + BRS, deg.
Prints the max |e| and the 2 s around it, plus how long FLS sat at its 0.45 rad limit. Arg: bag."""
import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id=''), rosbag2_py.ConverterOptions('', ''))
J = []; R = []
while r.has_next():
    t, d, s = r.read_next()
    if t == '/joint_states':
        m = deserialize_message(d, JointState); p = dict(zip(m.name, m.position))
        if all(k in p for k in ('BLS_joint', 'BRS_joint', 'FLS_joint', 'FRS_joint')):
            J.append((s * 1e-9, p['BLS_joint'], p['BRS_joint'], p['FLS_joint'], p['FRS_joint']))
    elif t == '/imu/data':
        m = deserialize_message(d, Imu); q = m.orientation
        R.append((s * 1e-9, math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))))
def res(j): return (j[1] + 0.577165 * j[3], j[2] + 0.572734 * j[4], j[1] + j[2])
t0 = J[0][0]; base = [sum(res(j)[i] for j in J if j[0] - t0 < 1.0) / max(1, sum(1 for j in J if j[0] - t0 < 1.0)) for i in range(3)]
E = [(j[0], *[math.degrees(res(j)[i] - base[i]) for i in range(3)]) for j in J]
k = max(range(len(E)), key=lambda i: max(abs(E[i][1]), abs(E[i][2]), abs(E[i][3])))
print(f"{sys.argv[1]}: max residual e1 {max(abs(e[1]) for e in E):.1f}, e2 {max(abs(e[2]) for e in E):.1f}, e3 {max(abs(e[3]) for e in E):.1f} deg; worst at t+{E[k][0]-t0:.1f} s")
lim = [j for j in J if abs(j[3]) > 0.445 or abs(j[4]) > 0.445]
print(f"front rocker at its 0.45 rad limit: {len(lim)} samples, about {len(lim)/max(1,len(J))*(J[-1][0]-t0):.1f} s")
import bisect; ts = [x[0] for x in R]
for e in E[::max(1, len(E)//4000)]:
    if abs(e[0] - E[k][0]) <= 2.0 and int((e[0]-E[k][0])*4) != int((e[0]-E[k][0]-0.01)*4):
        i = min(bisect.bisect_left(ts, e[0]), len(R) - 1)
        print(f"  {e[0]-E[k][0]:+5.2f} s  e1 {e[1]:6.1f}  e2 {e[2]:6.1f}  e3 {e[3]:6.1f}  roll {R[i][1]:6.1f}")
