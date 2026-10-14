#!/usr/bin/python3
"""Republish /odom as the odom -> base_footprint transform, for watching a bag in RViz.

The sim's TF tree has no odom frame (ground truth reaches ROS only as /odom), so a
replayed bag shows the rover's joints moving but the body standing still. Run this next
to `ros2 bag play ... --clock` and set RViz's fixed frame to "odom":

    ros2 run rover_gazebo odom_tf_replay.py --ros-args -p use_sim_time:=true
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTf(Node):

    def __init__(self):
        super().__init__("odom_tf_replay")
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, "odom", self.on_odom, 50)

    def on_odom(self, m):
        t = TransformStamped()
        t.header.stamp = m.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        p = m.pose.pose.position
        t.transform.translation.x, t.transform.translation.y = p.x, p.y
        t.transform.translation.z = p.z
        t.transform.rotation = m.pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomTf())


if __name__ == "__main__":
    main()
