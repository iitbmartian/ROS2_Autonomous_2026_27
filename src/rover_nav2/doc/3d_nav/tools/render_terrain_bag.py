"""Render the last /terrain/local elevation, /terrain/local_features slope and step, and the last
/local_costmap/costmap from a bag to one PNG. Usage: render_terrain_bag.py BAG OUT.png"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid

bag, out = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
last = {}
costmaps = []
while r.has_next():
    topic, data, _ = r.read_next()
    if topic in ('/terrain/local', '/terrain/local_features'):
        last[topic] = data
    elif topic == '/local_costmap/costmap':
        costmaps.append(data)

def layer(msg, name):
    arr = msg.data[msg.layers.index(name)]
    rows = arr.layout.dim[1].size   # grid_map stores column-major: dim[0] cols, dim[1] rows
    cols = arr.layout.dim[0].size
    a = np.array(arr.data, dtype=float).reshape(cols, rows).T
    # Undo the circular buffer, then flip so +x is up and +y is left, as seen from above.
    a = np.roll(a, (-msg.outer_start_index, -msg.inner_start_index), axis=(1, 0))
    return a

elev = deserialize_message(last['/terrain/local'], GridMap)
feat = deserialize_message(last['/terrain/local_features'], GridMap)
# The costmap nearest in stamp to the terrain snapshot, so both show the same moment.
t_terrain = elev.header.stamp.sec + elev.header.stamp.nanosec * 1e-9
cms = [deserialize_message(d, OccupancyGrid) for d in costmaps]
cm = min(cms, key=lambda m: abs(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - t_terrain))
fig, ax = plt.subplots(1, 4, figsize=(20, 5))
for a, (title, img, cmap, lim) in zip(ax, [
        ('elevation [m] /terrain/local', layer(elev, 'elevation'), 'terrain', None),
        ('slope [rad] /terrain/local_features', layer(feat, 'slope'), 'magma', (0, 1.6)),
        ('step [m] /terrain/local_features', layer(feat, 'step'), 'viridis', (0, 0.25))]):
    im = a.imshow(img, cmap=cmap, vmin=None if lim is None else lim[0],
                  vmax=None if lim is None else lim[1])
    a.set_title(title); a.set_xticks([]); a.set_yticks([]); fig.colorbar(im, ax=a, shrink=0.8)
c = np.array(cm.data, dtype=float).reshape(cm.info.height, cm.info.width)
c[c < 0] = np.nan
im = ax[3].imshow(np.flipud(c), cmap='inferno', vmin=0, vmax=100)
ax[3].set_title(f'/local_costmap/costmap [0-100] at {cm.header.stamp.sec}s'); ax[3].set_xticks([]); ax[3].set_yticks([])
fig.colorbar(im, ax=ax[3], shrink=0.8)
fig.suptitle(f'{bag}  terrain stamp {elev.header.stamp.sec}s, 10 x 10 m at '
             f'{elev.info.resolution} m, NaN = white')
fig.tight_layout(); fig.savefig(out, dpi=90)
print('wrote', out)
