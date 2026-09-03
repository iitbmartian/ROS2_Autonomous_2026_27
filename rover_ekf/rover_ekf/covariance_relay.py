#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class CovarianceRelay(Node):
    def __init__(self):
        super().__init__('covariance_relay')
        
        # 1. Listen to the raw data coming from the Gazebo bridge
        # self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom/ground_truth', self.odom_callback, 10)
        # self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        # 2. Publish the "fixed" data for the EKF to use
        self.odom_pub = self.create_publisher(Odometry, '/odom_relay', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu_relay', 10)

    def odom_callback(self, msg):
        # The EKF divides by these numbers. If they are 0, it outputs NaN and crashes RViz.
        # We inject a small variance (0.01) into the diagonal elements we care about.
        
        # Pose Covariance (6x6 matrix, flattened to 36 elements)
        msg.pose.covariance[0] = 0.05   # X position
        msg.pose.covariance[7] = 0.05   # Y position
        msg.pose.covariance[35] = 0.01  # Yaw orientation
        
        # Twist Covariance
        msg.twist.covariance[0] = 0.1  # X velocity
        msg.twist.covariance[35] = 0.01 # Yaw velocity
        
        self.odom_pub.publish(msg)

    def imu_callback(self, msg):
        # IMU Covariance (3x3 matrices, flattened to 9 elements)
        variance = 0.01
        
        # Diagonals are at indices 0 (X), 4 (Y), and 8 (Z)
        msg.angular_velocity_covariance = [variance, 0.0, 0.0, 
                                           0.0, variance, 0.0, 
                                           0.0, 0.0, variance]
                                           
        msg.linear_acceleration_covariance = [variance, 0.0, 0.0, 
                                              0.0, variance, 0.0, 
                                              0.0, 0.0, variance]
        msg.orientation_covariance = [
                                    0.1, 0.0, 0.0,
                                    0.0, 0.1, 0.0,
                                    0.0, 0.0, 0.05   # yaw more confident
                                ]
        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CovarianceRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()