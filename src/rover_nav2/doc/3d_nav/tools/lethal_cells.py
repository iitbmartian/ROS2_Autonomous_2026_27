"""Count feature cells over the prototype layer's limits in the last /terrain/local_features of a bag."""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from grid_map_msgs.msg import GridMap
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
last = None
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/terrain/local_features': last = d
m = deserialize_message(last, GridMap)
L = {n: np.array(a.data, dtype=float) for n, a in zip(m.layers, m.data)}
fin = ~np.isnan(L['slope'])
print('finite feature cells', int(fin.sum()), 'of', fin.size)
for name, lim in [('slope', 0.45), ('step', 0.15), ('roughness', 0.05)]:
    v = L[name][fin]
    print(f'{name:9s} > {lim}: {int((v > lim).sum()):5d}   p50 {np.percentile(v,50):.3f}  p99 {np.percentile(v,99):.3f}  max {v.max():.3f}')
print('lethal by any channel:', int(((L['slope'] > 0.45) | (L['step'] > 0.15) | (L['roughness'] > 0.05))[fin].sum()))
