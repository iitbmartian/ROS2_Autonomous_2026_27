"""Print Gazebo's ground-truth x y from /odom once (the sim publishes ground truth there)."""
import rclpy
from nav_msgs.msg import Odometry
rclpy.init()
n = rclpy.create_node('gt_pose_probe')
got = []
n.create_subscription(Odometry, '/odom', lambda m: got.append(m), 10)
while not got:
    rclpy.spin_once(n, timeout_sec=0.5)
p = got[-1].pose.pose.position
print(f'{p.x:.3f} {p.y:.3f}')
