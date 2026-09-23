#!/usr/bin/python3
"""Turn a Twist into steering angles and wheel speeds for a four-wheel-steer rover.

This is the Gazebo counterpart of Unity's RoverDriveController. Four driving modes.
Three of them ask for a body motion and solve for the wheels, and all three reduce to
the same two lines of algebra: give every wheel the ground velocity its station would
have under the commanded body motion, then point the wheel along it and roll it at that
speed.

    v_wheel = v_body + omega x r

  Ackermann   body moves forward and yaws, so the rover follows an arc. Front and rear
              wheels counter-steer, which roughly halves the turning circle.
  Crab        body translates without yawing, so the rover slides while keeping its
              heading. linear.y is the sideways part.
  Spot        body yaws with no translation, so the rover spins in place.

Writing it as one velocity field rather than three special cases means there is no
singularity at zero speed and spot mode is just the case where linear.x is zero.

Only the speed half of that is ramped, though: v_body changes gradually, but the wheel
angle solved from it is a fresh atan2 every tick, with no such limit. Ramping vx and vy
toward a new direction independently means their ratio, and so the angle, can swing
through 90 degrees in a single tick even while both are still near zero -- pressing a
pure sideways command from a stop asks for a 90 degree wheel snap on the very first
step, well before the body has picked up any real speed. If the four wheels do not
reach that snap in perfect lockstep, which they need not, the mismatch shows up as a
real yaw kick on the chassis. So the solved angle is itself rate-limited per wheel
(max_wheel_turn_rate), and the wheel's driven speed is the desired ground velocity
projected onto wherever the wheel is actually pointed rather than its full magnitude,
so a wheel mid-turn drives less hard rather than scrubbing sideways at full speed. A
wheel is also never asked to swing past a quarter turn from square-on to the body: past
90 degrees the shorter path is to point it the other way and roll it backwards instead,
so the solved angle is folded to whichever of itself or its reverse sits closer to
wherever the wheel already is *before* it gets rate-limited, not after -- folding first
is what keeps that boundary from ever producing a same-tick jump in the published angle.

The fourth mode inverts the relationship. You aim the wheels and the body goes wherever
they take it, instead of asking for a body motion and solving for the wheels.

  Explicit    all four wheels share one steering angle. angular.z is read as a steering
              rate in rad/s, not a body yaw rate, and integrated here into that angle.
              linear.x is then one wheel speed handed to all four. No body kinematics at
              all: nothing here works out where the rover will end up, and nothing stops
              you aiming somewhere that makes the wheels scrub. The current aim goes out
              on rover/steer_aim so a teleop or a test can see where the wheels point.

A fifth mode, explicit_pid (teleop key 5), is the exact same movement as explicit --
it runs through explicit_step() unchanged -- but instead of handing the solved angle
and speed to ros2_control's ideal steer_controller/wheel_controller, it publishes them
as *targets* on explicit_pid_control/steer_target and .../wheel_target for
control_node.py to track with its own independent per-joint PID loops, and this node
switches which pair of ros2_control controllers is active accordingly (see on_mode()).
explicit and explicit_pid never run at once, so there is exactly one control law with
authority over the 8 joints at any time.

Geometry, joint names and joint senses come from config/rover_kinematics.yaml, which is
generated from the CAD by tools/derive_ratios.py. Nothing here is hand-tuned to the
model, so a CAD change needs no edit to this file.
"""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64, Float64MultiArray, String

MODES = ("ackermann", "crab", "spot", "explicit", "explicit_pid")
STATIONS = ("FL", "FR", "BL", "BR")

# rover/steer_mode is published only on change, not continuously, so it needs
# TRANSIENT_LOCAL durability (latching) here and on every publisher to it
# (teleop_common.py, measure_rover.py) and every other subscriber
# (control_node.py) -- otherwise a subscriber that finishes discovery after the
# last mode change simply never receives it and silently stays on its startup
# default. See teleop_common.py's own copy of this constant for the full
# explanation; duplicated rather than imported since this node has no other
# dependency on the teleop module.
MODE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# The two ros2_control controller pairs explicit/explicit_pid hand off between.
# Exactly one pair is ever active on the 8 steer/drive joints at a time -- see
# on_mode() -- so explicit_pid's independent per-joint PID (control_node.py) and
# explicit/ackermann/crab/spot's ideal group controllers never have simultaneous
# authority over a joint.
IDEAL_CONTROLLERS = ("steer_controller", "wheel_controller")
PID_CONTROLLERS = ("steer_pid_controller", "wheel_pid_controller")


def ramp(current, target, max_delta):
    """Move current toward target by at most max_delta."""
    if target > current:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


def angle_ramp(current, target, max_delta):
    """Move an angle toward another by at most max_delta, the short way round.

    Plain ramp() breaks on angles: going from 179 to -179 degrees is a 2 degree turn,
    not a 358 degree one, and it has to be treated that way or a wheel asked to swing
    through the wrap point would refuse to move at all. Returns a value normalised to
    (-pi, pi], so state carried between calls cannot wind up unbounded over time.
    """
    diff = (target - current + math.pi) % (2 * math.pi) - math.pi
    diff = max(-max_delta, min(max_delta, diff))
    return (current + diff + math.pi) % (2 * math.pi) - math.pi


class RoverKinematics(Node):

    def __init__(self):
        super().__init__("rover_kinematics")

        share = get_package_share_directory("rover_gazebo")
        default_cfg = os.path.join(share, "config", "rover_kinematics.yaml")

        self.declare_parameter("kinematics_file", default_cfg)
        self.declare_parameter("max_speed", 1.0)          # m/s
        self.declare_parameter("min_turn_radius", 0.8)    # m, tightest arc
        self.declare_parameter("max_steer_angle", 1.5708) # rad
        self.declare_parameter("max_yaw_rate", 1.0)       # rad/s, ackermann/crab yaw-rate
                                                            # range -- calibrates steer_ramp_time
                                                            # for these modes, mirrors teleop's
                                                            # own max_yaw_rate
        self.declare_parameter("max_spin_rate", 0.7854)   # rad/s, spot mode
        self.declare_parameter("max_wheel_turn_rate", 2.0)  # rad/s, ackermann/crab/spot
                                                             # solved-angle rate limit
        # Explicit mode keeps its own clamp so the aim can be opened up without
        # loosening what the other three modes are allowed to ask for.
        self.declare_parameter("explicit_max_steer_angle", 1.5708)  # rad
        self.declare_parameter("explicit_max_steer_rate", 1.0)      # rad/s, sweep ceiling
        self.declare_parameter("speed_ramp_time", 1.2)    # s, standstill to full speed
        self.declare_parameter("steer_ramp_time", 0.8)    # s, lock to lock
        self.declare_parameter("update_rate", 100.0)      # Hz
        self.declare_parameter("command_timeout", 0.5)    # s before commands lapse
        self.declare_parameter("mode", "ackermann")

        cfg_path = self.get_parameter("kinematics_file").value
        with open(cfg_path) as fh:
            cfg = yaml.safe_load(fh)["rover"]

        self.radius = cfg["geometry"]["wheel_radius"]
        self.stations = [cfg["wheels"][s] for s in STATIONS]
        self.get_logger().info(
            f"loaded {os.path.basename(cfg_path)}: wheelbase "
            f"{cfg['geometry']['wheelbase']:.3f} m, track {cfg['geometry']['track']:.3f} m, "
            f"wheel radius {self.radius:.4f} m")
        self.get_logger().info(
            "steer signs [" + ",".join("+" if s["steer_sign"] > 0 else "-" for s in self.stations)
            + "]  drive signs ["
            + ",".join("+" if s["drive_sign"] > 0 else "-" for s in self.stations) + "]")

        self.mode = self.get_parameter("mode").value
        if self.mode not in MODES:
            self.mode = "ackermann"

        # commanded, then ramped
        self.want = [0.0, 0.0, 0.0]     # vx, vy, omega
        self.have = [0.0, 0.0, 0.0]
        self.last_cmd = self.get_clock().now()

        # Explicit mode state. steer_aim is the one angle all four wheels share;
        # last_fl_angle is whatever the front-left wheel was last told, kept so the
        # mode can take over from the pose the steering is already in.
        self.steer_aim = 0.0
        self.zeroing = False
        self.last_fl_angle = 0.0

        # Continuous per-wheel heading, tracked across ackermann/crab/spot ticks (and
        # kept in sync by explicit mode) so a changing target can be swept toward
        # smoothly instead of solved fresh, and possibly wildly different, every tick.
        self.wheel_angle = [0.0, 0.0, 0.0, 0.0]

        self.steer_pub = self.create_publisher(Float64MultiArray, "steer_controller/commands", 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, "wheel_controller/commands", 10)
        # explicit_pid mode's targets, for control_node.py's independent per-joint PID
        # to track -- same content steer_pub/wheel_pub would carry for explicit mode,
        # just a target for a control loop to close rather than a command ros2_control
        # applies directly. See explicit_step().
        self.steer_target_pub = self.create_publisher(
            Float64MultiArray, "explicit_pid_control/steer_target", 10)
        self.wheel_target_pub = self.create_publisher(
            Float64MultiArray, "explicit_pid_control/wheel_target", 10)
        self.aim_pub = self.create_publisher(Float64, "rover/steer_aim", 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(String, "rover/steer_mode", self.on_mode, MODE_QOS)

        # Hands off between {steer,wheel}_controller (ideal, modes 1-4) and
        # {steer,wheel}_pid_controller (control_node.py, explicit_pid) whenever the
        # mode crosses that boundary -- see on_mode()/_switch_controllers(). Waiting
        # here in __init__, before the node starts spinning, is fine: it is not inside
        # a callback, so there is nothing for a brief block to deadlock against. By the
        # time this node is launched (after joint_state_broadcaster, per
        # launch/rover_sim.launch.py) the controller manager is already up, so this
        # should resolve almost immediately in practice.
        self.switch_cli = self.create_client(
            SwitchController, "/controller_manager/switch_controller")
        if not self.switch_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "controller_manager/switch_controller not available yet; "
                "explicit_pid mode's controller hand-off may fail on first use")

        rate = self.get_parameter("update_rate").value
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.step)

    # -- input ------------------------------------------------------------
    def on_cmd(self, msg: Twist):
        self.want = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.last_cmd = self.get_clock().now()

    def on_mode(self, msg: String):
        name = msg.data.strip().lower()
        if name in MODES:
            if name != self.mode:
                self.get_logger().info(f"steer mode -> {name}")
                entering_explicit = name in ("explicit", "explicit_pid")
                leaving_explicit = self.mode in ("explicit", "explicit_pid")
                if entering_explicit and not leaving_explicit:
                    # Take over from wherever the steering already is, then sweep to
                    # straight ahead, so entering the mode never snaps the wheels.
                    # Coming from spot or Ackermann the four wheels sit at different
                    # angles and one shared aim cannot reproduce four of them, so the
                    # other three match the front-left once before they all sweep
                    # home. Not retriggered swapping explicit <-> explicit_pid
                    # themselves: both already share the same continuous steer_aim,
                    # so there is nothing to take over from.
                    self.steer_aim = self.last_fl_angle
                    self.zeroing = True
                was_pid, now_pid = self.mode == "explicit_pid", name == "explicit_pid"
                if now_pid != was_pid:
                    self._switch_controllers(to_pid=now_pid)
            self.mode = name
        else:
            self.get_logger().warn(f"unknown steer mode '{msg.data}', keeping {self.mode}")

    def _switch_controllers(self, to_pid):
        """Hand the 8 steer/drive joints from the ideal group controllers to
        control_node.py's PID ones, or back. Fire-and-forget on purpose: on_mode() is
        a subscription callback on this node's single-threaded executor, so blocking
        here for the response (e.g. spin_until_future_complete) would deadlock the
        node against itself rather than switch anything."""
        if not self.switch_cli.service_is_ready():
            self.get_logger().warn(
                f"switch_controller service not ready; could not switch controllers "
                f"for explicit_pid={to_pid}")
            return
        req = SwitchController.Request()
        req.activate_controllers = list(PID_CONTROLLERS if to_pid else IDEAL_CONTROLLERS)
        req.deactivate_controllers = list(IDEAL_CONTROLLERS if to_pid else PID_CONTROLLERS)
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout = Duration(sec=2, nanosec=0)

        def done(fut):
            try:
                res = fut.result()
                level = self.get_logger().info if res.ok else self.get_logger().warn
                level(f"controller switch (to_pid={to_pid}): ok={res.ok} {res.message}")
            except Exception as exc:
                self.get_logger().warn(f"controller switch (to_pid={to_pid}) failed: {exc}")

        self.switch_cli.call_async(req).add_done_callback(done)

    # -- main loop --------------------------------------------------------
    def step(self):
        p = self.get_parameter
        top = p("max_speed").value
        timeout = p("command_timeout").value

        want = list(self.want)
        age = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
        if timeout > 0.0 and age > timeout:
            want = [0.0, 0.0, 0.0]

        # Ramp so the rover does not snap between commands, exactly as in Unity. The yaw-rate
        # ramp is calibrated from whichever limit the current mode actually asks for --
        # max_spin_rate in spot, max_yaw_rate everywhere else (crab never sets angular.z, so
        # which constant it uses is a don't-care) -- so steer_ramp_time means lock-to-lock in
        # the mode actually driving, not always spot's own range.
        lin_step = top / max(0.01, p("speed_ramp_time").value) * self.dt
        ang_range = p("max_spin_rate").value if self.mode == "spot" else p("max_yaw_rate").value
        ang_step = 2.0 * ang_range / max(0.01, p("steer_ramp_time").value) * self.dt
        self.have[0] = ramp(self.have[0], max(-top, min(top, want[0])), lin_step)
        self.have[1] = ramp(self.have[1], max(-top, min(top, want[1])), lin_step)

        if self.mode in ("explicit", "explicit_pid"):
            # angular.z is a steering rate here, so it is integrated rather than ramped.
            # have[2] is held at zero so a sweep cannot leak out as a yaw command the
            # moment the mode changes back. Both modes share this exact same step --
            # explicit_step() itself decides, from self.mode, whether the result is an
            # ideal command or a target for control_node.py's PID to track.
            self.have[2] = 0.0
            self.explicit_step(self.have[0], want[2])
            return

        self.have[2] = ramp(self.have[2], want[2], ang_step)

        vx, vy, omega = self.have
        angles, speeds = [], []

        for i, st in enumerate(self.stations):
            x, y = st["x"], st["y"]

            if self.mode == "crab":
                # Every wheel parallel: the body translates and never yaws.
                wx, wy = vx, vy
            elif self.mode == "spot":
                spin = max(-p("max_spin_rate").value, min(p("max_spin_rate").value, omega))
                wx, wy = -spin * y, spin * x
            else:
                # Ackermann. Curvature is capped rather than yaw rate, so the turn
                # stays inside the tightest arc the steering can reach at any speed.
                w = omega
                cap = abs(vx) / max(1e-6, p("min_turn_radius").value)
                w = max(-cap, min(cap, w))
                wx, wy = vx - w * y, w * x

            # Solve the direction this station wants, then fold it to whichever of
            # itself or its reverse (wheel pointed the other way, rolling backwards)
            # sits closer to wherever the wheel already is -- only then let the wheel
            # catch up to that folded target at a bounded rate. Folding relative to the
            # wheel's actual current position, before the ramp, is what keeps the +/-90
            # degree boundary from ever needing a same-tick jump in the published value:
            # crossing it now costs one ordinary max_wheel_turn_rate*dt step like any
            # other tick, because the choice is re-made fresh, from wherever the wheel
            # currently sits, every time -- never carried over stale from a ramp that
            # already overshot past it. (Folding the *ramped* angle after the fact, the
            # way this used to work, let the internal angle overshoot out toward its raw
            # target -- up to 180 degrees away when reversing straight, since atan2(0, vx)
            # is exactly 0 for vx>0 and exactly pi for vx<0 -- and the published angle
            # would jump ~180 degrees in one tick the moment that overshoot crossed 90.)
            raw_target = math.atan2(wy, wx)
            current = self.wheel_angle[i]
            diff = (raw_target - current + math.pi) % (2 * math.pi) - math.pi
            if diff > math.pi / 2:
                target_angle = raw_target - math.pi
            elif diff < -math.pi / 2:
                target_angle = raw_target + math.pi
            else:
                target_angle = raw_target

            max_delta = p("max_wheel_turn_rate").value * self.dt
            self.wheel_angle[i] = angle_ramp(current, target_angle, max_delta)
            angle = self.wheel_angle[i]

            # Drive at however much of the desired ground velocity actually lies along
            # the way the wheel is presently pointed, not its full magnitude, so a wheel
            # still mid-turn eases off instead of scrubbing sideways at full speed. The
            # sign falls out on its own here: if the fold above picked the reversed
            # target, angle points opposite (wx, wy) and this projection comes out
            # negative by itself, so no separate speed flip is needed any more.
            speed = wx * math.cos(angle) + wy * math.sin(angle)

            lim = p("max_steer_angle").value
            angle = max(-lim, min(lim, angle))

            if i == 0:
                self.last_fl_angle = angle

            angles.append(st["steer_sign"] * angle)
            speeds.append(st["drive_sign"] * speed / self.radius)

        self.steer_pub.publish(Float64MultiArray(data=angles))
        self.wheel_pub.publish(Float64MultiArray(data=speeds))

    # -- explicit mode ----------------------------------------------------
    def explicit_step(self, speed, rate):
        """Aim the wheels, then roll them.

        rate is angular.z read as a steering rate rather than a body yaw rate, so the
        aim only moves while a rate is arriving and otherwise stays where it was left.
        That is also what makes the command timeout safe here: it zeroes the rate, so
        the drive coasts down while the wheels stay pointed where you parked them.

        speed is one wheel speed for all four, scaled only by each station's drive_sign.
        There is deliberately no v = v_body + omega x r here; the wheels are simply told
        to spin, and the rover goes wherever they drag it.
        """
        p = self.get_parameter
        lim = p("explicit_max_steer_angle").value
        top_rate = p("explicit_max_steer_rate").value
        rate = max(-top_rate, min(top_rate, rate))

        if rate != 0.0:
            # A or D during the entry sweep takes over rather than being swallowed.
            self.zeroing = False

        if self.zeroing:
            self.steer_aim = ramp(self.steer_aim, 0.0, top_rate * self.dt)
            if abs(self.steer_aim) < 1e-3:
                self.steer_aim = 0.0
                self.zeroing = False
        else:
            self.steer_aim = max(-lim, min(lim, self.steer_aim + rate * self.dt))

        self.last_fl_angle = self.steer_aim
        # Kept in sync so a later switch back to ackermann/crab/spot sweeps from
        # wherever explicit mode actually left the wheels, not a stale value.
        self.wheel_angle = [self.steer_aim] * 4
        self.aim_pub.publish(Float64(data=self.steer_aim))

        angles = [st["steer_sign"] * self.steer_aim for st in self.stations]
        speeds = [st["drive_sign"] * speed / self.radius for st in self.stations]
        if self.mode == "explicit_pid":
            # A target for control_node.py's own PID to track, not a command
            # ros2_control applies directly -- steer_pid_controller/wheel_pid_controller
            # are the ones actually active on the joints in this mode (see on_mode()).
            self.steer_target_pub.publish(Float64MultiArray(data=angles))
            self.wheel_target_pub.publish(Float64MultiArray(data=speeds))
        else:
            self.steer_pub.publish(Float64MultiArray(data=angles))
            self.wheel_pub.publish(Float64MultiArray(data=speeds))


def main():
    rclpy.init()
    node = RoverKinematics()
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
