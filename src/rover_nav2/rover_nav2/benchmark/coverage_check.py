"""Near-field map coverage: how close to the rover the elevation map has ground cells.

    ros2 run rover_nav2 coverage_check --duration 30

Phase 1 gate (PHASES.md): with the L2-like lidar, the elevation map has ground cells within
1.0 m of the footprint edge. For each of --bins bearings around the rover, the distance from the
footprint rectangle's edge to the nearest observed cell is measured on every /terrain/local map.
The worst bearing is the number that matters: a single blind direction is enough to drive into.
Also reported: the share of cells in the band from the edge out to --band that are observed.
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from rover_nav2.benchmark.metrics import quat_to_rpy
from rover_nav2.benchmark.terrain_eval import gridmap_cell_xy, gridmap_layer


def edge_distance(lx, ly, half_l, half_w):
    """Distance from points (body frame) to the outside of an axis-aligned rectangle, 0 inside."""
    dx = np.maximum(np.abs(lx) - half_l, 0.0)
    dy = np.maximum(np.abs(ly) - half_w, 0.0)
    return np.hypot(dx, dy)


def nearest_per_bearing(lx, ly, observed, half_l, half_w, bins):
    """Per bearing bin: distance from the footprint edge to the nearest observed cell outside the
    footprint (inf where none)."""
    d = edge_distance(lx, ly, half_l, half_w)
    outside = d > 0
    bearing = np.mod(np.arctan2(ly, lx), 2 * math.pi)
    k = np.minimum((bearing / (2 * math.pi) * bins).astype(int), bins - 1)
    out = np.full(bins, np.inf)
    sel = observed & outside
    np.minimum.at(out, k[sel], d[sel])
    return out


class Coverage(Node):

    def __init__(self, a, half_l, half_w):
        super().__init__('coverage_check', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.a, self.half_l, self.half_w = a, half_l, half_w
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.worst, self.band = [], []
        self.create_subscription(GridMap, a.map_topic, self.on_map, 2)

    def on_map(self, msg):
        z = gridmap_layer(msg, 'elevation')
        if z is None:
            return
        try:
            tf = self.tf.lookup_transform(msg.header.frame_id, self.a.base_frame,
                                          Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        tr, q = tf.transform.translation, tf.transform.rotation
        yaw = quat_to_rpy(q.x, q.y, q.z, q.w)[2]
        X, Y = gridmap_cell_xy(msg, z.shape)
        dx, dy = X - tr.x, Y - tr.y
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        obs = np.isfinite(z)
        per = nearest_per_bearing(lx, ly, obs, self.half_l, self.half_w, self.a.bins)
        self.worst.append(float(np.max(per)))
        d = edge_distance(lx, ly, self.half_l, self.half_w)
        ring = (d > 0) & (d <= self.a.band)
        self.band.append(float(np.count_nonzero(obs & ring)) / max(1, np.count_nonzero(ring)))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--duration', type=float, default=30.0, help='wall s')
    p.add_argument('--bins', type=int, default=36)
    p.add_argument('--band', type=float, default=1.0, help='m beyond the footprint edge')
    p.add_argument('--map-topic', default='/terrain/local')
    p.add_argument('--base-frame', default='base_footprint')
    a = p.parse_args(sys.argv[1:] if argv is None else argv)
    from rover_traversability import physical_params
    phys, _ = physical_params.load()
    rclpy.init()
    n = Coverage(a, phys['footprint_length'] / 2, phys['footprint_width'] / 2)
    t0 = time.time()
    while time.time() - t0 < a.duration:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.worst:
        print('no maps received')
        return 1
    w = np.array(n.worst)
    print(f'{len(w)} maps. Worst-bearing distance from the footprint edge to the nearest '
          f'ground cell: median {np.median(w):.2f} m, worst {w.max():.2f} m '
          f'({a.bins} bearings). Band 0 to {a.band} m observed: median '
          f'{100 * np.median(n.band):.0f} percent.')
    print(f"Phase 1 gate (within 1.0 m everywhere): {'PASS' if np.median(w) <= 1.0 else 'FAIL'}"
          ' on the median map')
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
