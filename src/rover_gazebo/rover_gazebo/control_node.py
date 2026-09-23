#!/usr/bin/python3
"""Independently PID-control the 8 explicit-drive joints, one loop per joint.

This is explicit_pid mode's (teleop key 5) counterpart to explicit mode's (key 4)
steer_controller/wheel_controller: same movement -- A/D sweep a shared aim, W/S roll
the wheels along it -- but instead of handing a setpoint to ros2_control's ideal
position/velocity group controllers and trusting whatever Gazebo's hidden internal
gains do with it, each of the 4 steering joints and 4 wheel joints is closed over its
own explicit torque loop here, the same way rocker_coupling_node already closes one
for the suspension rather than relying on a Gazebo-side constraint solver. Four
independent position loops (steering) and four independent velocity loops (wheel),
not one shared setpoint broadcast to all four -- "independent" is the point: nothing
here assumes the four joints of a group move identically, the way steer_controller's
single JointGroupPositionController command implicitly does.

rover_kinematics_node still owns all of explicit_pid mode's actual driving logic --
the aim sweep-and-hold, the W/S speed, the command timeout, C's recentre sweep -- it
is the exact same explicit_step() mode 4 uses. The only difference is where the
result goes: instead of steer_controller/commands and wheel_controller/commands, it
publishes the same per-joint targets on steer_target/wheel_target below, and this
node is the thing that actually turns a target plus the joint's current state into a
torque. rover_kinematics_node also switches which pair of ros2_control controllers is
active (steer_controller/wheel_controller vs steer_pid_controller/wheel_pid_controller)
whenever the mode crosses the 4/5 boundary, so this node's effort output only ever
has actual authority over the joints while explicit_pid mode is selected -- the rest
of the time steer_pid_controller/wheel_pid_controller are loaded but inactive, and
whatever this node publishes to them is simply dropped.

This node also tracks rover/steer_mode itself, purely to stay quiet outside
explicit_pid: step() returns immediately unless the mode is explicit_pid, and the
/joint_states subscription itself only exists while the mode is explicit_pid --
created on entry, destroyed on exit, rather than left running and gated inside the
callback. That distinction matters and was measured, not assumed: joint_state_broadcaster
publishes /joint_states at the controller_manager's own rate (1000 Hz here, matched to
the physics step), and gating *inside* on_state() -- letting the subscription run and
returning early in the callback -- still leaves rclpy/rmw paying to deserialize every
one of those 1000 messages/second into a Python JointState before the callback body
ever gets to check the mode and bail. That cost alone was enough to pin this node at
100% of a core even sitting idle in ackermann, confirmed live by comparing against
rocker_coupling_node -- an unrelated, unmodified node that also subscribes to
/joint_states and was independently found running just as hot, which is what pointed
at the subscription itself rather than anything this node's callbacks do. Tearing the
subscription down when it's not needed avoids paying that cost at all instead of
paying it and discarding the result. The physics step here already runs at
real_time_factor 1.0 with no slack budgeted (worlds/flat.sdf), so this is not a
micro-optimisation: a fourth always-on high-rate Python subscriber, on top of the
three this package already ships, was enough to visibly worsen jitter in ackermann
and explicit too -- modes this node has nothing to do with.

Every gain and effort limit below is a starting point, not a measured value: unlike
the driving geometry (config/rover_kinematics.yaml, derived from the CAD), there is
no source for how much torque a steering knuckle or a wheel actually needs. They are
picked conservatively and sanity-checked in Gazebo for stability -- see
doc/VERIFICATION.md's explicit_pid section -- not tuned against real hardware.

One thing tuning cannot fix: commanding real torque on a steering joint necessarily
applies an equal-and-opposite reaction torque to whatever it's mounted on (Newton's
third law), unlike steer_controller's *position* command interface in mode 4, which
Gazebo realizes kinematically with no such reaction. All four steering joints share
the same axis convention (rover_body.xacro, all four "0 -1 0") and the same
steer_sign (config/rover_kinematics.yaml, all four -1.0) -- there is no left/right
mirroring to make the four reaction torques cancel across the chassis, so they add.
steer_kp/steer_kd and steer_max_effort_rate below are chosen to keep that reaction as
small as this mode can be while still functioning, not to eliminate it -- it cannot
be eliminated without commanding zero torque.

A second, separate problem showed up live and needed its own fix: four *independent*
wheel-speed loops, each tracking the same target correctly on its own, do not
necessarily agree with each other at any given instant -- and a rover whose four
wheels are turning at slightly different speeds is, mechanically, no different from a
skid-steer rig with a wheel-speed mismatch. That is enough to inject real, growing
yaw into the chassis on its own, measured live even driving dead straight with no
steering input at all (see doc/VERIFICATION.md). steer_sync_kp/wheel_sync_kp below
are the fix: each joint's PID output gets a second, explicit term pulling it toward
the other three's *measured* state, not their target -- so all four are nudged to
agree with each other in addition to each independently tracking the commanded
target. This is not a mimic joint wearing a different name: a mimic joint would make
three of the four joints passive followers with no actuator of their own, which is
not how the real rover's steering/drive motors work (each has its own motor, unlike
the suspension's rocker linkage, which really is a passive mechanical coupling --
see rocker_coupling_node) and was confirmed unusable in this simulator regardless
(doc/VERIFICATION.md's mimic-joint section: DART, the physics engine this package
runs on, explicitly rejects mimic constraints outright). All four joints stay
independently actuated and independently PID-controlled; sync is a second, additive
correction layered on top of that, the control-systems equivalent of "gang
synchronization"/"electronic line shafting" between independently-driven motors that
are supposed to move together, not a kinematic constraint standing in for one.

A third problem needed a third, different kind of fix, because it isn't a magnitude
problem at all: holding a steering joint at a fixed angle while the chassis is
rotating is a closed loop that never routes through anything this node measures. The
chassis's rotation physically drags the wheel along with it; that scrubs the joint's
*relative* angle away from its target; the PID (correctly, from its own point of
view) applies torque to correct that; that torque's reaction (same non-cancelling
axis convention as above) pushes the chassis further the same way; repeat. Measured
live, and deliberately ruled out as a magnitude problem before treating it as a
structural one: neither raising kd to 8 (four times the shipped value) nor lowering
steer_max_effort to a quarter of it stopped the chassis from accelerating into a
full, continuing rotation, seconds after the steering target had already gone
perfectly still. A direct measurement (apply a known torque, watch the resulting
chassis angular acceleration) also ruled out an unrealistically low simulated yaw
inertia as the explanation -- the measured effective inertia was large, consistent
with a genuinely heavy rover, not a modelling bug making it easy to spin. The
qualitative signature (unaffected by gain magnitude, unaffected by torque ceiling,
present with a believable inertia) is what a loop closed *outside* the measured
signal looks like: no amount of retuning steer_kp/steer_kd can fix a divergence in a
quantity -- absolute chassis yaw -- that they cannot see, because they only ever see
each joint's angle *relative to the chassis*, which stays deceptively small even
while the chassis itself is spinning out from under it.

chassis_yaw_kd is the fix, and it is a genuinely different mechanism from
steer_kd, not a bigger version of it: it reads real chassis yaw rate from /odom
(ground truth from Gazebo here, not an estimated or noisy sensor) and adds a term
to all four steering joints, proportional to that rate, sized to oppose it -- using
the exact same reaction-torque path that was causing the problem, deliberately this
time, as a brake instead of an accident. It is applied uniformly across all four
joints for the same reason the runaway itself was uniform across all four: the
shared axis convention that makes their individual reactions add on the chassis is
exactly what gives a single shared correction real authority over chassis yaw. This
is layered on top of steer_sync (previous section) and the ordinary target-tracking
PID (JointPID.step()), not a replacement for either.
"""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

# Command order must match steer_pid_controller/wheel_pid_controller's joints list in
# config/controllers.yaml, which in turn mirrors steer_controller/wheel_controller's
# own order -- FL, FR, BL, BR -- so a target array from rover_kinematics_node lines
# up index-for-index with both a joint's state and this node's own command array.
STATIONS = ("FL", "FR", "BL", "BR")

# rover/steer_mode is published only on change, not continuously, so it needs
# TRANSIENT_LOCAL durability (latching) here and on every publisher to it
# (teleop_common.py, measure_rover.py) -- otherwise this node, which connects
# slightly later than most (it loads a YAML file at startup), can lose the race
# and never see the mode it should have started in. Measured live: this is exactly
# what made explicit_pid unresponsive on a fresh launch until switching away and
# back. See teleop_common.py's own copy of this constant for the full explanation.
MODE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
STEER_JOINTS = ("FLE_joint", "FRE_joint", "BLE_joint", "BRE_joint")
WHEEL_JOINTS = ("FLD_joint", "FRD_joint", "BLD_joint", "BRD_joint")


def rate_limit(prev, target, max_delta):
    """Move prev toward target by at most max_delta. Same idea as
    rover_kinematics_node.ramp()/rocker_coupling_node's own torque slew limit: caps
    how fast a published value may change, independent of how large it is allowed to
    get, so a step in the demanded value arrives as a ramp instead of a kick."""
    delta = max(-max_delta, min(max_delta, target - prev))
    return prev + delta


class JointPID:
    """One joint's independent PID loop, position- or velocity-mode.

    Not shared state between joints and not aware of the other three in its group --
    each instance only ever sees its own target, its own measured state and its own
    output, which is what makes the four "independent" rather than four outputs of
    one shared law.

    Anti-windup is conditional integration: the integral only accumulates while doing
    so would not itself push the output past the clamp, so a large, sustained error
    (steering held against a target it physically cannot reach yet) cannot wind the
    integrator up somewhere it then has to unwind before the joint can move back.
    """

    def __init__(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        """Called on a fresh entry into explicit_pid mode so a stale integral or
        derivative from a much earlier session can't leak into a kick on re-entry."""
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error, measured_rate, dt, kp, ki, kd, limit, derivative_on_measurement):
        # Derivative on measurement (steering: rate is the joint's own measured
        # velocity) rather than on the error avoids a kick when the target itself
        # jumps; a velocity loop has no such setpoint jump to protect against here,
        # since rover_kinematics_node ramps the commanded speed before this ever sees
        # it, so it uses a plain finite-difference error derivative instead.
        if derivative_on_measurement:
            d_term = -kd * measured_rate
        else:
            d_term = kd * (error - self.prev_error) / dt if dt > 0.0 else 0.0
        self.prev_error = error

        candidate = kp * error + ki * self.integral + d_term
        if abs(candidate) < limit or candidate * error < 0.0:
            self.integral += error * dt

        output = kp * error + ki * self.integral + d_term
        return max(-limit, min(limit, output))


class ExplicitPidControl(Node):

    def __init__(self):
        super().__init__("explicit_pid_control")

        # For the gang-sync terms in step(): steer_sign/drive_sign per station, same
        # source and same FL/FR/BL/BR index order rover_kinematics_node.py and
        # rocker_coupling_node.py both already use, so a sync correction can be
        # computed in a common sign-normalized frame rather than on raw joint values
        # (BR's drive_sign is -1 while the other three are +1, so its raw velocity
        # has the opposite sign from theirs even when perfectly synchronized -- see
        # the module docstring).
        share = get_package_share_directory("rover_gazebo")
        self.declare_parameter(
            "kinematics_file", os.path.join(share, "config", "rover_kinematics.yaml"))
        with open(self.get_parameter("kinematics_file").value) as fh:
            cfg = yaml.safe_load(fh)["rover"]["wheels"]
        self.steer_signs = tuple(cfg[s]["steer_sign"] for s in STATIONS)
        self.drive_signs = tuple(cfg[s]["drive_sign"] for s in STATIONS)

        # Steering: kept deliberately gentle, well below the xacro's 150 N m
        # interface backstop. Live-measured in Gazebo (see doc/VERIFICATION.md): the
        # torque this loop commands reacts on the chassis (see the module docstring).
        # kp=40/kd=4 (no rate limit) made the whole rover body visibly spin while A/D
        # was held. kp=8/kd=1 (with the rate limiter added) cut that a lot while
        # actively steering, but a much worse problem showed up on release: holding a
        # nonzero angle against a chassis disturbance is a real feedback path (the
        # disturbance twists the joint off target -> the PID corrects -> that
        # correction's own reaction twists the chassis further the same way), and at
        # kp=8/kd=1 it was an outright instability -- the chassis kept accelerating
        # into full, un-damped rotations well after the key was released and the aim
        # had stopped moving. Lowering kp changed that from monotonic runaway into a
        # bounded oscillation; raising kd on top of that damped the oscillation into
        # settling. kp=1/kd=3 is the first combination tried that actually settles
        # rather than spinning or ringing -- slower to reach a commanded angle than
        # kp=8 was, which is the real cost of a stable hold here, not a free lunch.
        self.declare_parameter("steer_kp", 1.0)      # N m / rad
        self.declare_parameter("steer_ki", 0.0)      # N m / (rad s)
        self.declare_parameter("steer_kd", 3.0)      # N m / (rad/s)
        self.declare_parameter("steer_max_effort", 20.0)          # N m
        self.declare_parameter("steer_max_effort_rate", 40.0)     # N m/s, slews a
                                                                   # torque step into
                                                                   # a ramp
        # Gang sync -- see step(). A starting guess, live-tuned like everything else
        # here; 0 would disable it entirely.
        self.declare_parameter("steer_sync_kp", 5.0)  # N m / rad of deviation from
                                                        # the other 3 joints' average
        # Chassis yaw damping -- see the module docstring for why this is a genuinely
        # different mechanism from steer_kd, not a bigger version of it. Sign is not
        # obvious by inspection (depends on the steering joints' axis direction
        # relative to the chassis's own frame after the CAD's Y-up-to-ROS-Z-up
        # rotation) -- verify live that a positive chassis yaw rate produces a
        # torque that measurably slows it, not speeds it up, before trusting this
        # sign; flip it if a live test shows the opposite. 0 disables it entirely.
        self.declare_parameter("chassis_yaw_kd", 15.0)  # N m / (rad/s) of measured
                                                          # chassis yaw rate, applied
                                                          # uniformly to all 4 steer
                                                          # joints
        # kd defaults to 0: differentiating a velocity *error* means differentiating an
        # already-once-differentiated, noisy simulated signal, and at update_rate's
        # 200 Hz that noise is amplified straight into a torque command. kp/ki are
        # both low for the same reason -- a wheel's rotational inertia is small enough
        # that even a modest gain reacting to a noisy velocity reading measurably
        # overshot in Gazebo. Measured directly (see doc/VERIFICATION.md): kp=8/kd=0.5
        # oscillated through zero, sign and all, never settling; kp=2/ki=1/kd=0 was
        # stable but still overshot about 3x; these values track within roughly the
        # commanded speed's own magnitude of ripple, stable and correctly signed, but
        # still not tight -- there is real tuning headroom left here.
        self.declare_parameter("wheel_kp", 0.4)      # N m / (rad/s)
        self.declare_parameter("wheel_ki", 0.4)      # N m / rad
        self.declare_parameter("wheel_kd", 0.0)      # N m / (rad/s^2)
        self.declare_parameter("wheel_max_effort", 60.0)          # N m
        # Generous on purpose -- unlike steering, nothing measured so far points to
        # the wheel loop needing its output slewed; this just gives it the same
        # mechanism, at a ceiling high enough not to change today's behaviour.
        self.declare_parameter("wheel_max_effort_rate", 1000.0)   # N m/s
        # Gang sync -- see step(). This is the one that matters most: four
        # independently-tracked wheel speeds that agree with their own target but
        # not quite with each other behave like a mismatched differential drive,
        # injecting real, growing yaw into the chassis even driving dead straight --
        # measured live, not assumed (see doc/VERIFICATION.md).
        self.declare_parameter("wheel_sync_kp", 2.0)  # N m / (rad/s) of deviation
                                                        # from the other 3 wheels'
                                                        # average speed
        self.declare_parameter("update_rate", 200.0)  # Hz

        self.mode = "ackermann"     # rover_kinematics_node's own default; on_state()
                                     # and step() both no-op until this is
                                     # "explicit_pid" -- see the module docstring
        self.pos = {}
        self.vel = {}
        self.steer_target = None    # rad, FL/FR/BL/BR, from rover_kinematics_node
        self.wheel_target = None    # rad/s, FL/FR/BL/BR
        self.chassis_yaw_rate = 0.0  # rad/s, from /odom -- see on_odom()
        self.steer_pid = {j: JointPID() for j in STEER_JOINTS}
        self.wheel_pid = {j: JointPID() for j in WHEEL_JOINTS}
        self.prev_steer_effort = dict.fromkeys(STEER_JOINTS, 0.0)
        self.prev_wheel_effort = dict.fromkeys(WHEEL_JOINTS, 0.0)

        self.steer_pub = self.create_publisher(
            Float64MultiArray, "steer_pid_controller/commands", 10)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, "wheel_pid_controller/commands", 10)
        # None of these three created here -- see on_mode(). A subscription and a
        # 200 Hz timer are themselves the expense (message deserialization, and the
        # executor waking up 200 times/second to service the timer), not just what
        # their callbacks do -- measured live: gating step()'s *body* on mode while
        # leaving the timer running cut this node from 100% to only ~69% of a core
        # sitting idle, still a real, avoidable cost. All three exist only while
        # explicit_pid actually needs them.
        self.joint_state_sub = None
        self.odom_sub = None
        self.step_timer = None
        self.create_subscription(
            Float64MultiArray, "explicit_pid_control/steer_target", self.on_steer_target, 10)
        self.create_subscription(
            Float64MultiArray, "explicit_pid_control/wheel_target", self.on_wheel_target, 10)
        self.create_subscription(String, "rover/steer_mode", self.on_mode, MODE_QOS)

        rate = self.get_parameter("update_rate").value
        self.dt = 1.0 / rate
        self.create_timer(2.0, self.report)

    def on_mode(self, msg: String):
        name = msg.data.strip().lower()
        entering = name == "explicit_pid" and self.mode != "explicit_pid"
        leaving = name != "explicit_pid" and self.mode == "explicit_pid"
        if entering:
            for pid in list(self.steer_pid.values()) + list(self.wheel_pid.values()):
                pid.reset()
            self.prev_steer_effort = dict.fromkeys(STEER_JOINTS, 0.0)
            self.prev_wheel_effort = dict.fromkeys(WHEEL_JOINTS, 0.0)
            self.pos = {}
            self.vel = {}
            self.steer_target = None
            self.wheel_target = None
            self.chassis_yaw_rate = 0.0
            self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.on_state, 10)
            self.odom_sub = self.create_subscription(
                Odometry, "odom", self.on_odom, 10)
            self.step_timer = self.create_timer(self.dt, self.step)
        elif leaving:
            if self.joint_state_sub is not None:
                self.destroy_subscription(self.joint_state_sub)
                self.joint_state_sub = None
            if self.odom_sub is not None:
                self.destroy_subscription(self.odom_sub)
                self.odom_sub = None
            if self.step_timer is not None:
                self.step_timer.cancel()
                self.destroy_timer(self.step_timer)
                self.step_timer = None
        self.mode = name

    def on_state(self, msg: JointState):
        # No mode check needed: this subscription only exists while explicit_pid is
        # active (created/destroyed in on_mode()).
        for i, name in enumerate(msg.name):
            if name in STEER_JOINTS or name in WHEEL_JOINTS:
                self.pos[name] = msg.position[i] if i < len(msg.position) else 0.0
                self.vel[name] = msg.velocity[i] if i < len(msg.velocity) else 0.0

    def on_odom(self, msg: Odometry):
        # No mode check needed, same as on_state(). Ground truth from Gazebo, not an
        # estimated or noisy sensor -- see the module docstring for why that matters
        # here (the whole point is a clean signal for the one quantity the per-joint
        # PIDs can't see).
        self.chassis_yaw_rate = msg.twist.twist.angular.z

    def on_steer_target(self, msg: Float64MultiArray):
        if len(msg.data) == len(STEER_JOINTS):
            self.steer_target = list(msg.data)
        else:
            self.get_logger().warn(
                f"steer_target has {len(msg.data)} values, want {len(STEER_JOINTS)}; ignored")

    def on_wheel_target(self, msg: Float64MultiArray):
        if len(msg.data) == len(WHEEL_JOINTS):
            self.wheel_target = list(msg.data)
        else:
            self.get_logger().warn(
                f"wheel_target has {len(msg.data)} values, want {len(WHEEL_JOINTS)}; ignored")

    def step(self):
        if self.mode != "explicit_pid":
            return   # nothing has authority over the joints in this mode; do nothing
        if self.steer_target is None or self.wheel_target is None:
            return   # rover_kinematics_node hasn't published a target yet
        if not all(j in self.pos for j in STEER_JOINTS + WHEEL_JOINTS):
            return   # no /joint_states yet

        p = self.get_parameter
        s_kp, s_ki, s_kd = p("steer_kp").value, p("steer_ki").value, p("steer_kd").value
        s_lim = p("steer_max_effort").value
        s_rate = p("steer_max_effort_rate").value * self.dt
        s_sync = p("steer_sync_kp").value
        chassis_kd = p("chassis_yaw_kd").value
        w_kp, w_ki, w_kd = p("wheel_kp").value, p("wheel_ki").value, p("wheel_kd").value
        w_lim = p("wheel_max_effort").value
        w_rate = p("wheel_max_effort_rate").value * self.dt
        w_sync = p("wheel_sync_kp").value

        # Chassis yaw damping -- see the module docstring. Uniform across all four
        # steering joints on purpose: the same shared axis convention that let their
        # individual reactions add up into an unbounded chassis rotation is exactly
        # what gives one shared correction real authority over that rotation. Sign
        # is negative (opposing the measured rate); if a live test shows the chassis
        # spinning *faster* with this enabled, the sign is backwards for this
        # installation's axis convention and chassis_yaw_kd needs to go negative in
        # config/rover_control.yaml, not this line.
        chassis_damp = -chassis_kd * self.chassis_yaw_rate

        # Gang sync: each joint is still driven by its own independent PID against
        # the shared target above, exactly as before -- this is a *second*, explicit
        # correction stacked on top, pulling each joint toward the other three's
        # actual measured state rather than toward the commanded target.
        #
        # It has to be computed in a sign-normalized frame, not on raw joint values:
        # BR's drive_sign is -1 while the other three wheels' is +1 (its joint is
        # mechanically mounted mirrored -- config/rover_kinematics.yaml), so a
        # perfectly synchronized rover driving forward has BR's raw velocity with
        # the *opposite* sign from the other three's, not the same one. Averaging
        # raw values directly (an earlier version of this code did exactly that) is
        # wrong by construction: it reports every wheel as "out of sync" even when
        # all four are exactly on target, worst on BR, injecting a real, continuous,
        # wrong torque on that one corner -- measured live as violent jitter and a
        # chassis that kept rotating after the steering key was released, sometimes
        # a full turn. norm_j = sign_j * measured_j converts each joint into a common
        # "how much does this joint contribute" frame where they should all agree
        # when truly in sync, regardless of which way that joint's own axis points;
        # multiplying the correction by sign_j again converts it back to that
        # joint's own raw effort convention before it's added in. All four steer_sign
        # values happen to be identical today, so this is a no-op for steering, but
        # it is given the same treatment rather than leave two differently-shaped
        # implementations of what is conceptually one mechanism.
        norm_steer = [sign * self.pos[j] for j, sign in zip(STEER_JOINTS, self.steer_signs)]
        avg_norm_steer = sum(norm_steer) / len(norm_steer)
        norm_wheel = [sign * self.vel[j] for j, sign in zip(WHEEL_JOINTS, self.drive_signs)]
        avg_norm_wheel = sum(norm_wheel) / len(norm_wheel)

        steer_effort = []
        for j, target, sign, norm in zip(
                STEER_JOINTS, self.steer_target, self.steer_signs, norm_steer):
            error = target - self.pos[j]
            raw = self.steer_pid[j].step(
                error, self.vel[j], self.dt, s_kp, s_ki, s_kd, s_lim,
                derivative_on_measurement=True)
            sync = s_sync * sign * (avg_norm_steer - norm)
            combined = max(-s_lim, min(s_lim, raw + sync + chassis_damp))
            out = rate_limit(self.prev_steer_effort[j], combined, s_rate)
            self.prev_steer_effort[j] = out
            steer_effort.append(out)

        wheel_effort = []
        for j, target, sign, norm in zip(
                WHEEL_JOINTS, self.wheel_target, self.drive_signs, norm_wheel):
            error = target - self.vel[j]
            raw = self.wheel_pid[j].step(
                error, self.vel[j], self.dt, w_kp, w_ki, w_kd, w_lim,
                derivative_on_measurement=False)
            sync = w_sync * sign * (avg_norm_wheel - norm)
            combined = max(-w_lim, min(w_lim, raw + sync))
            out = rate_limit(self.prev_wheel_effort[j], combined, w_rate)
            self.prev_wheel_effort[j] = out
            wheel_effort.append(out)

        self.steer_pub.publish(Float64MultiArray(data=steer_effort))
        self.wheel_pub.publish(Float64MultiArray(data=wheel_effort))

    def report(self):
        if self.mode != "explicit_pid" or self.steer_target is None or self.wheel_target is None:
            return
        steer_err = [math.degrees(t - self.pos.get(j, t))
                     for j, t in zip(STEER_JOINTS, self.steer_target)]
        wheel_err = [t - self.vel.get(j, t)
                     for j, t in zip(WHEEL_JOINTS, self.wheel_target)]
        self.get_logger().debug(
            "steer error deg [%.2f,%.2f,%.2f,%.2f]  wheel error rad/s [%.2f,%.2f,%.2f,%.2f]"
            % tuple(steer_err + wheel_err))


def main():
    rclpy.init()
    node = ExplicitPidControl()
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
