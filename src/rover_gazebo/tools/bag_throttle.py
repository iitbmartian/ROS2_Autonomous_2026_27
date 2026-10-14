#!/usr/bin/python3
"""Republish /joint_states and /tf at a lower rate under /rec/, for small recordings.

Both arrive at the 1 kHz physics rate, which is ~90% of a raw bag and far more than
watching or plotting a run needs. run_stress_suite.sh records /rec/joint_states and
/rec/tf from this instead; play them back with

    ros2 bag play <bag> --clock --remap /rec/joint_states:=/joint_states /rec/tf:=/tf

    ros2 run rover_gazebo bag_throttle.py --ros-args -p rate:=100.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


class Throttle(Node):

    def __init__(self):
        super().__init__("bag_throttle")
        self.declare_parameter("rate", 100.0)
        self.period = 1.0 / self.get_parameter("rate").value
        self.last = {}
        qos = QoSProfile(depth=100)
        for topic, kind in (("/joint_states", JointState), ("/tf", TFMessage)):
            pub = self.create_publisher(kind, "/rec" + topic, qos)
            self.create_subscription(kind, topic,
                                     lambda m, t=topic, p=pub: self.relay(t, p, m), qos)

    def relay(self, topic, pub, msg):
        # Throttle on the message's own stamp (sim time), not the wall clock.
        st = msg.header.stamp if hasattr(msg, "header") else msg.transforms[0].header.stamp \
            if msg.transforms else None
        if st is None:
            return
        t = st.sec + st.nanosec * 1e-9
        if t - self.last.get(topic, -1.0) >= self.period or t < self.last.get(topic, 0.0):
            self.last[topic] = t
            pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(Throttle())


if __name__ == "__main__":
    main()
