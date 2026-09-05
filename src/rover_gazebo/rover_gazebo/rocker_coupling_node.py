#!/usr/bin/env python3
"""Hold the rocker suspension on its kinematic constraints using joint torques.

Gazebo has no solver-level way to enforce these constraints (no <mimic> support), so
this is the only coupling mechanism this package ships. It is the direct descendant of
Unity's RockerDifferentialCoupler.

The real rover's suspension is held together by two connector bars and a differential.
Those are closed loops, so they cannot be written in a URDF. Each one is replaced here
by a stiff virtual constraint

    e  = theta_a  -  ratio * theta_b            the residual, zero when satisfied
    g  = K * e  +  D * edot                     the constraint force
    tau_a -= g                                  equal and opposite, so no net work is
    tau_b += g * ratio                          done when the constraint is satisfied

applied every cycle to both joints. Applying it to both is the whole point: it is what
makes load on a rear wheel show up at the front rocker, the way a steel bar would.

Three constraints, from config/rover_kinematics.yaml:

    left connector    BLS = ratio_L x FLS
    right connector   BRS = ratio_R x FRS
    differential      BLS + BRS = 0

Together they leave the suspension one degree of freedom, which is what the mechanism
actually has.
"""

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Command order must match the joints list of the effort controller.
ORDER = ("FLS_joint", "BLS_joint", "BRS_joint", "FRS_joint")


class RockerCoupling(Node):

    def __init__(self):
        super().__init__("rocker_coupling")

        share = get_package_share_directory("rover_gazebo")
        self.declare_parameter("kinematics_file",
                               os.path.join(share, "config", "rover_kinematics.yaml"))
        # Stiff enough that a rocker carrying a quarter of a 107 kg rover on a 0.22 m
        # arm, about 58 N m, deflects well under a degree.
        self.declare_parameter("stiffness", 8000.0)     # N m per rad of residual
        self.declare_parameter("damping", 1000.0)        # N m per rad/s of residual rate
        self.declare_parameter("max_torque", 400.0)     # N m, per joint
        self.declare_parameter("update_rate", 400.0)    # Hz
        # The uniform-sag mode the constraints forbid is also blocked by the wheels
        # themselves: correcting it needs the rover to roll slightly, and the wheel
        # controller holds them still. So a small residual survives from the moment the
        # rover settles onto the ground, and a pure CAD-pose reference would push
        # against it forever, shoving the rover along. Capturing the rest pose once the
        # rover has settled zeroes that standing push and leaves the coupling to resist
        # changes, which is what it is for. Unity's coupler does the same thing.
        self.declare_parameter("capture_rest", True)
        self.declare_parameter("rest_capture_delay", 2.5)   # s after the first sample

        with open(self.get_parameter("kinematics_file").value) as fh:
            cfg = yaml.safe_load(fh)["rover"]["coupling"]

        fb = cfg["four_bar"]
        # (joint_a, joint_b, ratio) meaning a - ratio * b = constant
        self.constraints = [
            (fb["left"]["rear"], fb["left"]["front"], fb["left"]["ratio"]),
            (fb["right"]["rear"], fb["right"]["front"], fb["right"]["ratio"]),
            (cfg["differential"]["joints"][0], cfg["differential"]["joints"][1], -1.0),
        ]
        for a, b, r in self.constraints:
            self.get_logger().info(f"constraint  {a} = {r:+.6f} x {b}")

        self.rest = None
        self.first_seen = None
        self.pos = {}
        self.vel = {}
        self.residuals = [0.0, 0.0, 0.0]

        self.pub = self.create_publisher(
            Float64MultiArray, "rocker_coupling_controller/commands", 10)
        self.create_subscription(JointState, "joint_states", self.on_state, 10)

        rate = self.get_parameter("update_rate").value
        self.create_timer(1.0 / rate, self.step)
        self.create_timer(2.0, self.report)

    def on_state(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in ORDER:
                self.pos[name] = msg.position[i] if i < len(msg.position) else 0.0
                self.vel[name] = msg.velocity[i] if i < len(msg.velocity) else 0.0

    def step(self):
        if not all(j in self.pos for j in ORDER):
            return

        if self.rest is None:
            if not self.get_parameter("capture_rest").value:
                self.rest = [0.0, 0.0, 0.0]
            else:
                now = self.get_clock().now()
                if self.first_seen is None:
                    self.first_seen = now
                    return
                waited = (now - self.first_seen).nanoseconds * 1e-9
                if waited < self.get_parameter("rest_capture_delay").value:
                    return
                self.rest = [self.pos[a] - r * self.pos[b] for a, b, r in self.constraints]
                self.get_logger().info(
                    "rest pose captured: " + ", ".join(
                        f"{math_degrees(v):+.3f} deg" for v in self.rest))

        k = self.get_parameter("stiffness").value
        d = self.get_parameter("damping").value
        lim = self.get_parameter("max_torque").value

        tau = dict.fromkeys(ORDER, 0.0)
        for i, (a, b, ratio) in enumerate(self.constraints):
            e = (self.pos[a] - ratio * self.pos[b]) - self.rest[i]
            edot = self.vel[a] - ratio * self.vel[b]
            g = k * e + d * edot
            self.residuals[i] = e
            tau[a] -= g
            tau[b] += g * ratio

        data = [max(-lim, min(lim, tau[j])) for j in ORDER]
        self.pub.publish(Float64MultiArray(data=data))

    def report(self):
        if self.rest is None:
            self.get_logger().warn("waiting for rocker joints on /joint_states")
            return
        deg = [math_degrees(e) for e in self.residuals]
        self.get_logger().debug(
            "residuals deg: left %.3f  right %.3f  differential %.3f" % tuple(deg))


def math_degrees(r):
    return r * 57.29577951308232


def main():
    rclpy.init()
    node = RockerCoupling()
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
