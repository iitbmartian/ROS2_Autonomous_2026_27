"""Velocity chain while a goal runs: controller output, smoother output, collision monitor output,
collision monitor state and speed limit, sampled for N seconds. Usage: vel_chain.py SECONDS"""
import sys, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav2_msgs.msg import CollisionMonitorState, SpeedLimit
last = {}
rclpy.init(); n = Node('vel_chain')
for t in ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel'):
    n.create_subscription(TwistStamped, t, lambda m, t=t: last.__setitem__(t, (m.twist.linear.x, m.twist.angular.z)), 10)
n.create_subscription(CollisionMonitorState, '/collision_monitor_state', lambda m: last.__setitem__('cm', (m.action_type, m.polygon_name)), 10)
n.create_subscription(SpeedLimit, '/speed_limit', lambda m: last.__setitem__('limit', (m.percentage, round(m.speed_limit, 3))), 10)
end = time.time() + float(sys.argv[1]); nxt = time.time()
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() >= nxt:
        nxt += 2.0
        fmt = lambda k: '(%+.2f, %+.2f)' % last[k] if k in last else '-'
        print(f"nav {fmt('/cmd_vel_nav')}  smooth {fmt('/cmd_vel_smoothed')}  out {fmt('/cmd_vel')}  cm {last.get('cm', '-')}  limit {last.get('limit', '-')}", flush=True)
