"""Per-ring height bias of /lidar/points on flat ground: groups returns by elevation angle in the
sensor frame and reports each ring's mean height error and the vertical-angle error it implies.
Run inside simenv.sh with the rover standing on flat ground (z of flat ground = 0 in odom)."""
import math, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
rclpy.init(); n = Node('ring_bias', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
tf = Buffer(); TransformListener(tf, n); msgs = []
n.create_subscription(PointCloud2, '/lidar/points', lambda m: msgs.append(m), 20)
t0 = time.time()
while len(msgs) < 20 and time.time() - t0 < 60: rclpy.spin_once(n, timeout_sec=0.2)
rows = []
for m in msgs[-10:]:
    try: t = tf.lookup_transform('odom', m.header.frame_id, m.header.stamp)
    except Exception: continue
    q = t.transform.rotation; tr = t.transform.translation
    p = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
    el = np.degrees(np.arctan2(p[:, 2], np.hypot(p[:, 0], p[:, 1])))   # sensor frame
    w = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply(p) + [tr.x, tr.y, tr.z]
    h = np.hypot(w[:, 0] - tr.x, w[:, 1] - tr.y)
    ok = (np.abs(w[:, 2]) < 0.15) & (h > 0.8) & (h < 4.5) & (w[:, 0] - tr.x < -0.8)  # flat, behind
    rows.append(np.column_stack([np.round(el[ok], 1), w[ok, 2], h[ok]]))
A = np.vstack(rows); H = tr.z
print(f'sensor height {H:.3f} m, {len(A)} ground returns behind the rover')
print(' ring el(deg)  n    range(m)  mean z err(cm)  implied angle err(deg)')
for e in np.unique(A[:, 0]):
    s = A[:, 0] == e
    if s.sum() < 30: continue
    dz, hh = A[s, 1].mean(), A[s, 2].mean()
    # On flat ground a vertical-angle error d gives dz about -d * r / cos(el) ... use exact geometry.
    true_dep = math.atan2(H, hh); obs_dep = math.atan2(H - dz, hh)
    print(f'{e:9.1f} {s.sum():5d} {hh:8.2f} {dz*100:+10.2f} {math.degrees(obs_dep-true_dep):+14.3f}')
