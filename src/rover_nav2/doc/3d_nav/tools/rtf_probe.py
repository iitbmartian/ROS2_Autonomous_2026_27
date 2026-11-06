"""Real-time factor from /clock against wall time over --seconds (default 30)."""
import sys, time, rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
s = []
class N(Node):
    def __init__(self):
        super().__init__('rtf_probe')
        self.create_subscription(Clock, '/clock', lambda m: s.append((time.time(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
rclpy.init(); n = N(); t0 = time.time()
while time.time() - t0 < secs: rclpy.spin_once(n, timeout_sec=0.1)
(w0, c0), (w1, c1) = s[0], s[-1]
print(f'{(c1 - c0) / (w1 - w0):.3f}')
