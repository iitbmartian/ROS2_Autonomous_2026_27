#!/usr/bin/env python3
"""Rebroadcast Gazebo's ground-truth odometry as the odom -> base_footprint TF.

rover_gazebo bridges the simulator's OdometryPublisher onto /odom but publishes
no transform for it, deliberately: rover_gazebo/doc/INTEGRATION.md leaves
odom -> base_footprint to the estimator. RTAB-Map needs that transform to exist.

Normally rgbd_odometry provides it. This node is the alternative, selected with
odom_source:=ground_truth, and it exists for one purpose: when the map comes out
wrong, running with a perfect pose says whether the fault is in the mapping or in
the visual odometry. It is a debugging instrument, not a pose source to build on.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class GroundTruthOdomTf(Node):

    def __init__(self):
        super().__init__("gt_odom_tf")

        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")

        self.odom_frame = self.get_parameter("odom_frame_id").value
        self.base_frame = self.get_parameter("base_frame_id").value

        self._tf = TransformBroadcaster(self)

        # ros_gz_bridge publishes reliable; match it or nothing arrives.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, "odom", self.on_odom, qos)

        self.get_logger().info(
            f"republishing /odom as {self.odom_frame} -> {self.base_frame}")

    def on_odom(self, msg):
        # The frame ids in the bridged message come from the Gazebo plugin and
        # may carry a model prefix, so use the configured names rather than the
        # message's own, which keeps the TF tree matching the URDF.
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(t)


def main():
    rclpy.init()
    node = GroundTruthOdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
