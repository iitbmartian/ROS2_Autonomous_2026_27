import sys, rclpy
rclpy.init(); n = rclpy.create_node(sys.argv[1]); rclpy.spin(n)
