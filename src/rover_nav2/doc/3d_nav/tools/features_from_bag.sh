#!/bin/bash
# Replay a terrain bag through terrain_features_node and print native vs bridge feature stats.
# Usage: features_from_bag.sh BAG
source /opt/ros/jazzy/setup.bash; source /home/ram/ROS2_Autonomous_2026_27/install/setup.bash
export ROS_DOMAIN_ID=43 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RCUTILS_LOGGING_BUFFERED_STREAM=0
# The binary itself, not ros2 run: killing the ros2 run wrapper leaves its child running, and a
# stale node publishing on the same topic silently corrupts the comparison.
trap 'kill -INT $NODE $PLAY 2>/dev/null' EXIT
/home/ram/ROS2_Autonomous_2026_27/install/rover_terrain_mapping/lib/rover_terrain_mapping/terrain_features_node --ros-args -p use_sim_time:=true \
  -r /terrain/local_features:=/terrain/local_features_native > /tmp/claude-1000/-home-ram-ROS2-Autonomous-2026-27/a41980dc-3f5f-4288-b3eb-d666f900cf6e/scratchpad/features_node.log 2>&1 &
NODE=$!
sleep 2
ros2 bag play "$1" --clock --topics /tf /tf_static /clock /terrain/local /terrain/local_features > /dev/null 2>&1 &
PLAY=$!
python3 - <<'PY'
import math, time, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from grid_map_msgs.msg import GridMap
got = {}
class S(Node):
    def __init__(s):
        super().__init__('feat_cmp', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        for t in ('/terrain/local_features', '/terrain/local_features_native'):
            s.create_subscription(GridMap, t, lambda m, t=t: got.__setitem__(t, m), 1)
rclpy.init(); n = S(); t0 = time.time()
while time.time() - t0 < 95: rclpy.spin_once(n, timeout_sec=0.2)
def stats(m, name):
    if name not in m.layers: return None
    d = [x for x in m.data[m.layers.index(name)].data if not math.isnan(x)]
    tot = len(m.data[m.layers.index(name)].data)
    if not d: return (0.0, None, None, None)
    d.sort()
    return (round(len(d)/tot, 3), round(d[len(d)//2], 3), round(d[int(0.99*(len(d)-1))], 3), round(d[-1], 3))
import numpy as np, sys
def layer(m, name):
    a = m.data[m.layers.index(name)]
    rows, cols = a.layout.dim[1].size, a.layout.dim[0].size
    g = np.array(a.data, dtype=float).reshape(cols, rows).T
    return np.roll(g, (-m.outer_start_index, -m.inner_start_index), axis=(0, 1))
if '/terrain/local_features_native' in got:
    m = got['/terrain/local_features_native']
    np.savez('/tmp/claude-1000/-home-ram-ROS2-Autonomous-2026-27/a41980dc-3f5f-4288-b3eb-d666f900cf6e/scratchpad/native_features.npz',
             **{l: layer(m, l) for l in m.layers}, cx=m.info.pose.position.x, cy=m.info.pose.position.y,
             length=m.info.length_x, res=m.info.resolution)
for t, m in sorted(got.items()):
    print(t, 'stamp', m.header.stamp.sec)
    for l in ('slope', 'slope_std', 'step', 'roughness', 'negative_obstacle'):
        print(f'   {l:18s} finite/p50/p99/max', stats(m, l))
    if 'slope' in m.layers:
        sl = [x for x in m.data[m.layers.index('slope')].data if not math.isnan(x)]
        st = [x for x in m.data[m.layers.index('step')].data if not math.isnan(x)]
        print(f'   over limits: slope>0.45 {sum(x > 0.45 for x in sl)}, step>0.15 {sum(x > 0.15 for x in st)}')
    if 'negative_obstacle' in m.layers:
        print('   negative_obstacle cells', sum(1 for x in m.data[m.layers.index('negative_obstacle')].data if x == 1.0))
PY
kill -INT $PLAY $NODE 2>/dev/null; sleep 2; kill -9 $PLAY $NODE 2>/dev/null; true
