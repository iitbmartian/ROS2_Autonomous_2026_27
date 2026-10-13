#!/usr/bin/python3
"""Hold FLS_joint, the suspension's one genuinely free coordinate, near its rest pose.

Used when suspension:=mimic (rover.urdf.xacro), Bullet-Featherstone only. In this mode
BLS/BRS/FRS are no longer driven from here, or from anywhere: they're held by <mimic>
tags on the joints themselves (rover_body.xacro), physics-solver-enforced, to
essentially machine precision -- measured live, matched the CAD-derived ratio
(config/rover_kinematics.yaml) to four decimal places, far tighter than
rocker_coupling_node's four-joint torque approach ever held its own constraints
(settled to roughly 1-2 degrees of residual under DART). DART cannot hold a mimic
constraint at all and silently leaves the follower joint unmoved -- see
doc/VERIFICATION.md -- so suspension:=mimic only works under Bullet-Featherstone;
suspension:=torque (the default, rocker_coupling_node.py) is what every other engine
uses.

That leaves exactly one problem: FLS_joint itself. Mimic only constrains BLS/BRS/FRS
*relative to* FLS -- nothing constrains FLS's own absolute position, and with only
joint damping (no stiffness) resisting it, it is a genuinely free coordinate with
nothing pulling it back toward any particular pose. Measured live: left alone, it
drifts steadily (roughly 1-2 deg/s, not decaying) and reaches its hard stop within
seconds, from a small residual imbalance too small to identify by inspection -- the
same class of problem rocker_coupling_node's own docstring already anticipated for a
naive, uncaptured reference ("a pure CAD-pose reference would push against it forever,
shoving the rover along"), just now showing up on the one coordinate mimic joints
cannot help with instead of on all four.

The fix is the same idea rocker_coupling_node already uses for exactly that reason,
just applied to one joint instead of three constraint pairs: a plain PD spring-damper
holding FLS_joint at wherever it settles after spawning, not at some theoretical
CAD-zero it may never actually rest at, since the small residual imbalance driving the
drift is too small to identify or reliably cancel but small enough to hold against
directly.

    e    = theta_FLS - rest              the residual, zero when at rest
    tau  = -(K * e + D * edot)           restoring torque, negative of the residual

Two extra terms exist, inherited from rocker_coupling_node's own tuning history on
this same suspension before the mimic/Bullet-Featherstone split -- not yet
re-measured in this single-joint form (see doc/VERIFICATION.md). Both default to off:

  max_torque_rate      caps how fast the published torque may change, in N m per
                        second, independent of how large it is allowed to get.
  velocity_filter_tau   a one-pole low-pass, time constant in seconds, on the
                        velocity fed into the D term. 0 disables it.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT = "FLS_joint"


class RockerHold(Node):

    def __init__(self):
        super().__init__("rocker_hold")

        # Carried over from rocker_coupling_node's own four-joint tuning as a
        # starting point only -- live-measured to hold FLS bounded (not runaway) but
        # with real jitter (roughly +/-5 deg around the captured rest pose), so this
        # is not yet a tuned value for this much simpler single-joint case. See
        # doc/VERIFICATION.md.
        self.declare_parameter("stiffness", 8000.0)     # N m per rad of residual
        self.declare_parameter("damping", 1000.0)        # N m per rad/s of residual rate
        self.declare_parameter("max_torque", 400.0)     # N m
        self.declare_parameter("update_rate", 400.0)    # Hz
        # Same rationale as rocker_coupling_node's own capture_rest: whatever small
        # residual imbalance is driving the drift is too small to identify or
        # reliably cancel by fixing the underlying cause, so this holds *against* it
        # at wherever the rover actually settles instead of forever pushing toward a
        # CAD-nominal pose it may never rest at.
        self.declare_parameter("capture_rest", True)
        self.declare_parameter("rest_capture_delay", 2.5)   # s after the first sample
        self.declare_parameter("max_torque_rate", 1.0e6)     # N m/s, effectively unlimited
        self.declare_parameter("velocity_filter_tau", 0.0)   # s, 0 disables the filter

        self.rest = None
        self.first_seen = None
        self.pos = None
        self.vel = None
        self.vel_filt = None
        self.prev_tau = 0.0
        self.residual = 0.0

        self.pub = self.create_publisher(
            Float64MultiArray, "rocker_coupling_controller/commands", 10)
        self.create_subscription(JointState, "joint_states", self.on_state, 10)

        rate = self.get_parameter("update_rate").value
        self.create_timer(1.0 / rate, self.step)
        self.create_timer(2.0, self.report)

    def on_state(self, msg: JointState):
        if JOINT in msg.name:
            i = msg.name.index(JOINT)
            self.pos = msg.position[i]
            self.vel = msg.velocity[i]

    def step(self):
        if self.pos is None:
            return

        if self.rest is None:
            if not self.get_parameter("capture_rest").value:
                self.rest = 0.0
            else:
                now = self.get_clock().now()
                if self.first_seen is None:
                    self.first_seen = now
                    return
                waited = (now - self.first_seen).nanoseconds * 1e-9
                if waited < self.get_parameter("rest_capture_delay").value:
                    return
                self.rest = self.pos
                self.get_logger().info(f"rest pose captured: {math.degrees(self.rest):+.3f} deg")

        k = self.get_parameter("stiffness").value
        d = self.get_parameter("damping").value
        lim = self.get_parameter("max_torque").value
        dt = 1.0 / self.get_parameter("update_rate").value

        vel = self._filtered_velocity(dt)
        e = self.pos - self.rest
        self.residual = e
        tau = -(k * e + d * vel)

        wanted = max(-lim, min(lim, tau))
        max_delta = self.get_parameter("max_torque_rate").value * dt
        delta = max(-max_delta, min(max_delta, wanted - self.prev_tau))
        self.prev_tau = self.prev_tau + delta
        self.pub.publish(Float64MultiArray(data=[self.prev_tau]))

    def _filtered_velocity(self, dt):
        tau_f = self.get_parameter("velocity_filter_tau").value
        if tau_f <= 0.0:
            return self.vel
        alpha = dt / (tau_f + dt)
        prev = self.vel if self.vel_filt is None else self.vel_filt
        self.vel_filt = prev + alpha * (self.vel - prev)
        return self.vel_filt

    def report(self):
        if self.rest is None:
            self.get_logger().warn(f"waiting for {JOINT} on /joint_states")
            return
        self.get_logger().debug(f"residual deg: {math.degrees(self.residual):.3f}")


def main():
    rclpy.init()
    node = RockerHold()
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
