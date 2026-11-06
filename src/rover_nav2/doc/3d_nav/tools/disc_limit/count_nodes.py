import time, rclpy
rclpy.init(); n = rclpy.create_node('zz_probe')
end = time.time() + 15
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.5)
names = sorted(x for x in n.get_node_names() if x.startswith('idle'))
print(len(names), 'of 40 seen;', 'missing:', sorted(set(f'idle{i:02d}' for i in range(40)) - set(names)))
