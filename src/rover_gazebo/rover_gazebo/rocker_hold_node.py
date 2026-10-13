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

A second, sharper problem showed up only once this was tested repeatedly rather than
once: which pose gets captured as "rest" is not repeatable. Confirmed live across four
clean, isolated launches, same command every time: FLS settled at -0.007, -0.010,
+20.863 and +21.216 degrees -- a bimodal split, not noise around one value, and the
+21 deg outcomes leave only about 3 degrees of margin to the joint's own +/-25.8 deg
hard limit before any driving load is even applied. That is what turns into the
"random locking into certain positions" symptom: whichever launch happens to swing
hard gets that near-limit pose captured as its permanent hold target, and a joint
parked 3 degrees from a hard stop chatters against it under load.

First hypothesis, ruled out live: an underdamped spawn transient. Unlike
rocker_coupling_node's four *independent* rockers, each damping only its own small
individual inertia while free, the mimic constraint (rover_body.xacro) is enforced by
the physics solver from the very first step, so FLS_joint's damping (5.0,
urdf/rover_body.xacro) has to absorb the *combined* reflected inertia of all four
rocker+wheel assemblies (FRS alone is coupled at very nearly 1:1) the instant the model
spawns. A pure velocity-damping term (no stiffness, so it adds no bias to *where* it
settles) was tried first on exactly that theory -- it narrowed the spread (11 deg,
16 deg on two more launches, instead of 21 deg) but did not fix it, and running the
same pure-damping term for 40 seconds instead of the usual 2.5 (temporarily, for this
measurement only) proved why: FLS never stopped moving. It drifted past 0, oscillated
briefly, then crawled monotonically down to -20 deg and was still moving at 40 s,
matching this docstring's own earlier "drifts steadily (roughly 1-2 deg/s, not
decaying)" description exactly. Pure damping cannot arrest that: a constant bias
torque against constant damping settles at whatever *velocity* balances the two
(tau_bias = D * v_terminal), not at any particular position, so the joint just
crawls forever at a slower, but still nonzero, rate. Whatever rest_capture_delay
happens to be, it grabs an arbitrary point along that ramp -- hence the
non-repeatable capture, not ringing.

The actual fix: hold FLS near zero with real stiffness through the whole settling
window too, not just damping, using the same PD law this node already applies after
capture (see below), just targeting rest=0.0 provisionally instead of returning
early. This is safe specifically *because* rocker_coupling_node's own measured,
real-driving data already answers the "might CAD-zero not be where it actually wants
to rest?" question this node's rest-capture design exists to avoid answering blind:
suspension:=torque settles FLS at -0.0007 deg on this same rover, same world, same
load -- indistinguishable from zero. So anchoring at zero during the settle is not a
guess here, and rest is still re-captured at the end of the delay exactly as before
(now consistently landing close to that anchor, rather than wherever a slow
unbounded crawl happened to be), preserving the "hold wherever it actually settles"
behaviour rocker_coupling_node's own docstring explains the rationale for.

The symptom came back anyway, live-reported after this fix shipped, once driving was
involved rather than only idling at spawn. Re-investigated live rather than guessing
again:

Idle on a fresh launch, `max_torque` at the value the fix above shipped with (2000),
was itself still capable of a real, sustained problem, not just an occasional
close-to-limit capture: one launch settled into FLS oscillating between +13 and
+24 deg indefinitely, `rocker_coupling_controller/commands` pinned at exactly -2000
(the ceiling) over and over rather than converging -- genuine saturating chatter, not
noise. Raising `max_torque` to 8000 fixed *that* specific run cleanly: same launch
conditions, FLS converged to a stable point (no more chatter) inside a few seconds and
held there within a fraction of a degree for the next fifteen. But the point it
converged to was -19.5 deg, not zero, and the steady torque needed to hold it there
turned out to be tiny (about 50 N m) -- meaning -19.5 deg was a genuine, low-energy,
*stable* equilibrium for that particular launch's settling dynamics, not a
torque-starved compromise. Two more identical launches (same command, same world)
landed at +17.916 and +17.905 deg respectively -- close enough to each other that this
is not chaotic run-to-run noise; something about real (non-deterministic) ROS-level
startup/controller-activation timing relative to the physics clock is the more likely
source of the spread than the physics itself.

That reframes the whole problem: raising `max_torque` further changes whether the
settling transient *chatters* against the ceiling, but does not change *where* the
system settles, because the destination is a real mechanical equilibrium, not a
torque-limited stopping point. And live-testing an actual maneuver confirms this
matters well beyond spawn: `measure_rover.py --test spot` (a point turn,
`--yaw-rate 0.7854`), run right after a launch that had already settled near +18 deg,
drove FLS to +25.423 deg and FRS to -25.622 deg -- both within a fraction of a degree
of the +/-25.78 deg hard limit -- with peak chassis roll of 19.7 deg and peak pitch of
12.8 deg. The identical test under suspension:=torque, same machine, same command:
peak roll 1.4 deg, peak pitch 5.4 deg, rockers settling within a degree of zero.
`max_torque` was 8000 for the mimic run; raising it further cannot fix this, because
the mimic joints' residuals during that same test were essentially exact (worst 0.06
deg) -- the constraint is being satisfied precisely, and *that exact satisfaction is
the problem*. suspension:=torque's own history (doc/VERIFICATION.md, "coupling off"
rows) already shows what an *uncoupled* rocker does under exactly this kind of load:
excursions to the same +/-25.8 deg stops this test just reproduced. What keeps
suspension:=torque's rockers near zero under real load is not that it enforces the
suspension's geometric constraint more precisely than mimic does -- it enforces it far
*less* precisely (residuals of 1-2.5 deg while driving, by design, capped at 400 N m
specifically so it yields rather than fights) and that yielding is what absorbs the
load differential between wheels. A rigid <mimic> constraint cannot yield by
definition, so whatever load differential a maneuver creates has nowhere to go except
into FLS_joint's own travel -- no value of stiffness, damping, or max_torque on this
node changes that, because none of them touch the constraint itself, only how FLS is
held once the constraint has already decided where it has to be. This is flagged as a
known, unresolved architectural limitation of the suspension:=mimic path, not
something this node can fix alone -- see doc/updates.md.

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

        # stiffness/damping carried over from rocker_coupling_node's own four-joint
        # tuning as a starting point only, not yet independently tuned for this
        # much simpler single-joint case. max_torque is *not* carried over: see the
        # module docstring for why this joint needs materially more of it than any
        # one of rocker_coupling_node's own four rockers. See doc/VERIFICATION.md.
        self.declare_parameter("stiffness", 8000.0)     # N m per rad of residual
        self.declare_parameter("damping", 1000.0)        # N m per rad/s of residual rate
        self.declare_parameter("max_torque", 2000.0)     # N m
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
        # Full PD (stiffness + damping, same gains as the post-capture hold) applied
        # *during* rest_capture_delay too, targeting 0.0 provisionally, instead of
        # leaving the joint fully free that long. See the module docstring: measured
        # live, a free (or damping-only) FLS_joint never settles under
        # suspension:=mimic -- it crawls steadily toward its hard limit -- because
        # this joint reflects all four rockers' combined inertia through the mimic
        # constraint from the first physics step, unlike rocker_coupling_node's own
        # small, independent rockers, which really do stay put while free. Safe to
        # anchor at zero specifically because suspension:=torque's own real,
        # measured rest position on this rover is -0.0007 deg -- indistinguishable
        # from zero -- so this is not a blind CAD-nominal guess.
        self.declare_parameter("pre_capture_hold", True)

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

        dt = 1.0 / self.get_parameter("update_rate").value

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
                    self._pre_capture_hold(dt)
                    return
                self.rest = self.pos
                self.get_logger().info(f"rest pose captured: {math.degrees(self.rest):+.3f} deg")

        k = self.get_parameter("stiffness").value
        d = self.get_parameter("damping").value
        lim = self.get_parameter("max_torque").value

        vel = self._filtered_velocity(dt)
        e = self.pos - self.rest
        self.residual = e
        tau = -(k * e + d * vel)

        wanted = max(-lim, min(lim, tau))
        max_delta = self.get_parameter("max_torque_rate").value * dt
        delta = max(-max_delta, min(max_delta, wanted - self.prev_tau))
        self.prev_tau = self.prev_tau + delta
        self.pub.publish(Float64MultiArray(data=[self.prev_tau]))

    def _pre_capture_hold(self, dt):
        """Full PD toward 0.0 while waiting out rest_capture_delay, or a no-op.

        See the module docstring: a pure-damping version of this was tried and
        measurably failed to stop FLS_joint crawling toward its limit, since the
        problem is a small constant bias torque, not spawn ringing -- damping alone
        only slows the terminal drift velocity, it cannot zero a constant torque.
        Same gains and rate limiting as the post-capture hold in step(), just
        against a fixed provisional target instead of the not-yet-captured
        self.rest.
        """
        if not self.get_parameter("pre_capture_hold").value:
            return
        k = self.get_parameter("stiffness").value
        d = self.get_parameter("damping").value
        lim = self.get_parameter("max_torque").value
        vel = self._filtered_velocity(dt)
        e = self.pos - 0.0
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
