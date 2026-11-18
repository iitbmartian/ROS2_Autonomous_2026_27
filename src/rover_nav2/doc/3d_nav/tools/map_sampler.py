#!/usr/bin/env python3
"""Save the local terrain features and the local costmap every N seconds as .npz, a few MB in
total, where a bag of /terrain/local_features would cost about 5 MB/s.

    python3 map_sampler.py OUT_DIR [--every 15] [--features /terrain/local_features]
      [--costmap /local_costmap/costmap]

features_<t>.npz: every layer as a row-major float32 array (rows along -x, like core::TerrainGrid,
the same layout as bag_gridmap.py), plus resolution, centre, frame and stamp.
costmap_<t>.npz: the OccupancyGrid data (rows along +y), resolution, origin, frame, stamp.
<t> is the message stamp in seconds (sim time).
"""
import argparse
import os

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter


def to_matrix(arr, rows, cols, outer_start, inner_start):
    # grid_map stores layers column-major (Eigen default) in a circular buffer.
    m = np.array(arr.data, dtype=np.float32).reshape(cols, rows).T
    return np.roll(np.roll(m, -outer_start, axis=0), -inner_start, axis=1)


class Sampler(Node):
    def __init__(self, a):
        super().__init__('map_sampler', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.a = a
        self.last = {'f': -1e9, 'c': -1e9}
        self.create_subscription(GridMap, a.features, self.on_features, 1)
        self.create_subscription(OccupancyGrid, a.costmap, self.on_costmap, 1)

    def due(self, key, t):
        if t - self.last[key] < self.a.every:
            return False
        self.last[key] = t
        return True

    def on_features(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not self.due('f', t):
            return
        res = m.info.resolution
        rows, cols = int(round(m.info.length_x / res)), int(round(m.info.length_y / res))
        layers = {n: to_matrix(d, rows, cols, m.outer_start_index, m.inner_start_index)
                  for n, d in zip(m.layers, m.data)}
        p = m.info.pose.position
        np.savez_compressed(os.path.join(self.a.out, f'features_{t:.1f}.npz'), **layers,
                            _resolution=res, _center=np.array([p.x, p.y]),
                            _frame=m.header.frame_id, _stamp=t)

    def on_costmap(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not self.due('c', t):
            return
        i = m.info
        np.savez_compressed(
            os.path.join(self.a.out, f'costmap_{t:.1f}.npz'),
            data=np.array(m.data, dtype=np.int8).reshape(i.height, i.width),
            _resolution=i.resolution,
            _origin=np.array([i.origin.position.x, i.origin.position.y]),
            _frame=m.header.frame_id, _stamp=t)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('out')
    p.add_argument('--every', type=float, default=15.0, help='sim s between samples')
    p.add_argument('--features', default='/terrain/local_features')
    p.add_argument('--costmap', default='/local_costmap/costmap')
    a = p.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    n = Sampler(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
