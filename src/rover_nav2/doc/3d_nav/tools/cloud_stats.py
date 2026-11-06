"""One /lidar/points scan in base_footprint: where the returns land (body, near ground, far)."""
import math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
class S(Node):
    def __init__(s):
        super().__init__('cloud_stats', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        s.tf = Buffer(); s.l = TransformListener(s.tf, s); s.msg = None
        s.create_subscription(PointCloud2, '/lidar/points', lambda m: setattr(s, 'msg', m), 1)
rclpy.init(); n = S(); t0 = time.time()
while time.time() - t0 < 20 and n.msg is None: rclpy.spin_once(n, timeout_sec=0.2)
time.sleep(1); [rclpy.spin_once(n, timeout_sec=0.1) for _ in range(10)]
m = n.msg
tf = n.tf.lookup_transform('base_footprint', m.header.frame_id, rclpy.time.Time())
t = tf.transform.translation; q = tf.transform.rotation
R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
p = np.array([[a, b, c] for a, b, c in pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)])
fin = np.all(np.isfinite(p), axis=1) & (np.linalg.norm(p, axis=1) < 29.9)
P = p[fin] @ R.T + np.array([t.x, t.y, t.z])
r_sensor = np.linalg.norm(p[fin], axis=1)
print('frame', m.header.frame_id, 'sensor at', round(t.x, 2), round(t.y, 2), round(t.z, 2), ' raw points', len(p), ' returns', len(P))
body = (np.abs(P[:, 0]) < 0.65) & (np.abs(P[:, 1]) < 0.55) & (P[:, 2] > -0.05) & (P[:, 2] < 1.0)
ground = np.abs(P[:, 2]) < 0.05
dh = np.hypot(P[:, 0], P[:, 1])
print('inside body box', int(body.sum()), ' ground returns', int(ground.sum()),
      ' ground within 2 m horizontal', int((ground & (dh < 2.0)).sum()),
      ' ground nearest horizontal %.2f m' % (dh[ground].min() if ground.any() else float('nan')))
print('sensor range of returns p5/p50/p95', np.round(np.percentile(r_sensor, [5, 50, 95]), 2),
      ' returns under 1.0 m range (FastDEM range_min)', int((r_sensor < 1.0).sum()))
