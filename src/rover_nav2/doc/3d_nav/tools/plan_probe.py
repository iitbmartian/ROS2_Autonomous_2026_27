"""Asks the planner for paths from spawn to points along +x and reports result and time.

    python3 plan_probe.py [DIST ...]

Run inside simenv.sh with a stack up. Prints distance, status, poses in the path, wall time.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

rclpy.init()
n = Node('plan_probe', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
c = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
if not c.wait_for_server(60):
    print('no planner')
    sys.exit(1)
for d in [float(x) for x in sys.argv[1:]] or [5.0, 10.0, 20.0, 30.0, 42.0]:
    g = ComputePathToPose.Goal()
    g.goal = PoseStamped()
    g.goal.header.frame_id = 'map'
    g.goal.pose.position.x = d
    g.goal.pose.orientation.w = 1.0
    g.use_start = False
    t0 = time.time()
    f = c.send_goal_async(g)
    rclpy.spin_until_future_complete(n, f, timeout_sec=30)
    h = f.result()
    r = h.get_result_async()
    rclpy.spin_until_future_complete(n, r, timeout_sec=60)
    res = r.result()
    print(f'{d:5.1f} m  status {res.status}  error {res.result.error_code} '
          f'{res.result.error_msg!r}  poses {len(res.result.path.poses)}  '
          f'planning {res.result.planning_time.sec + res.result.planning_time.nanosec * 1e-9:.2f} s'
          f'  wall {time.time() - t0:.1f} s', flush=True)
n.destroy_node()
rclpy.shutdown()
