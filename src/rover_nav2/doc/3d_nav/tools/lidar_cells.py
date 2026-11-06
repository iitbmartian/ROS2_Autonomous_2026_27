"""Raw /lidar/points over N scans in odom (gravity aligned), averaged per 5 cm cell, on flat ground behind the rover (x < -0.8). Residual spread by range = the floor any estimator could reach."""
import time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
rclpy.init(); n = Node('lidar_cells', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
tf = Buffer(); TransformListener(tf, n); msgs = []
n.create_subscription(PointCloud2, '/lidar/points', lambda m: msgs.append(m), 40)
t0 = time.time()
while len(msgs) < 40 and time.time() - t0 < 90: rclpy.spin_once(n, timeout_sec=0.2)
t = tf.lookup_transform('odom', msgs[-1].header.frame_id, rclpy.time.Time())
q = t.transform.rotation; tr = t.transform.translation; R = Rotation.from_quat([q.x, q.y, q.z, q.w])
P = np.vstack([R.apply(pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)) + [tr.x, tr.y, tr.z] for m in msgs])
P = P[(P[:, 0] < -0.8) & (np.abs(P[:, 2]) < 0.15)]
ij = np.floor(P[:, :2] / 0.05).astype(int); key = ij[:, 0] * 100000 + ij[:, 1]
u, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
mean = np.bincount(inv, P[:, 2]) / cnt
cx = (np.bincount(inv, P[:, 0]) / cnt); cy = np.bincount(inv, P[:, 1]) / cnt
r = np.hypot(cx, cy)
pr = np.hypot(P[:, 0], P[:, 1])
M = np.column_stack([cx, cy, np.ones_like(cx)]); c, *_ = np.linalg.lstsq(M[cnt >= 3], mean[cnt >= 3], rcond=None)
mean = mean - M @ c
print(f'plane removed: dz/dx {c[0]:.4f}, dz/dy {c[1]:.4f}')
print(f'{len(msgs)} scans, sensor height {tr.z:.3f} m')
for lo in range(1, 6):
    s = (r >= lo) & (r < lo + 1); sp = (pr >= lo) & (pr < lo + 1)
    print(f'range {lo}-{lo+1} m: cells {s.sum()}, median points per cell {np.median(cnt[s]):.0f}, cell-mean std {mean[s].std()*100:.2f} cm, raw point z std {P[sp, 2].std()*100:.2f} cm')
