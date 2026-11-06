#!/usr/bin/env python3
"""Send one NavigateToPose goal in map and wait for the result. Args: x y yaw timeout_s."""
import math, sys, time
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

x, y, yaw, timeout = (float(a) for a in sys.argv[1:5])
rclpy.init()
n = rclpy.create_node('video_goal')
c = ActionClient(n, NavigateToPose, 'navigate_to_pose')
c.wait_for_server()
g = NavigateToPose.Goal()
g.pose.header.frame_id = 'map'
g.pose.pose.position.x, g.pose.pose.position.y = x, y
g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
f = c.send_goal_async(g)
rclpy.spin_until_future_complete(n, f)
h = f.result()
if not h.accepted:
    print('rejected'); sys.exit(1)
r = h.get_result_async()
# Wall timeout, the sim runs below real time.
rclpy.spin_until_future_complete(n, r, timeout_sec=timeout)
print('status', r.result().status if r.done() else 'timeout')
if not r.done():
    h.cancel_goal_async(); time.sleep(1)
