#!/usr/bin/env python3
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
so a wheel mid-turn drives less hard rather than scrubbing sideways at full speed.

The fourth mode inverts the relationship. You aim the wheels and the body goes wherever
they take it, instead of asking for a body motion and solving for the wheels.

  Explicit    all four wheels share one steering angle. angular.z is read as a steering
              rate in rad/s, not a body yaw rate, and integrated here into that angle.
              linear.x is then one wheel speed handed to all four. No body kinematics at
              all: nothing here works out where the rover will end up, and nothing stops
              you aiming somewhere that makes the wheels scrub. The current aim goes out
              on rover/steer_aim so a teleop or a test can see where the wheels point.

Geometry, joint names and joint senses come from config/rover_kinematics.yaml, which is
generated from the CAD by tools/derive_ratios.py. Nothing here is hand-tuned to the
model, so a CAD change needs no edit to this file.
"""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String

MODES = ("ackermann", "crab", "spot", "explicit")
STATIONS = ("FL", "FR", "BL", "BR")


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
        self.aim_pub = self.create_publisher(Float64, "rover/steer_aim", 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(String, "rover/steer_mode", self.on_mode, 10)

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
                if name == "explicit":
                    # Take over from wherever the steering already is, then sweep to
                    # straight ahead, so entering the mode never snaps the wheels.
                    # Coming from spot or Ackermann the four wheels sit at different
                    # angles and one shared aim cannot reproduce four of them, so the
                    # other three match the front-left once before they all sweep home.
                    self.steer_aim = self.last_fl_angle
                    self.zeroing = True
            self.mode = name
        else:
            self.get_logger().warn(f"unknown steer mode '{msg.data}', keeping {self.mode}")

    # -- main loop --------------------------------------------------------
    def step(self):
        p = self.get_parameter
        top = p("max_speed").value
        timeout = p("command_timeout").value

        want = list(self.want)
        age = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
        if timeout > 0.0 and age > timeout:
            want = [0.0, 0.0, 0.0]

        # Ramp so the rover does not snap between commands, exactly as in Unity.
        lin_step = top / max(0.01, p("speed_ramp_time").value) * self.dt
        ang_step = 2.0 * p("max_spin_rate").value / max(0.01, p("steer_ramp_time").value) * self.dt
        self.have[0] = ramp(self.have[0], max(-top, min(top, want[0])), lin_step)
        self.have[1] = ramp(self.have[1], max(-top, min(top, want[1])), lin_step)

        if self.mode == "explicit":
            # angular.z is a steering rate here, so it is integrated rather than ramped.
            # have[2] is held at zero so a sweep cannot leak out as a yaw command the
            # moment the mode changes back.
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

            # Solve the direction this station wants, then let the wheel catch up to
            # it at a bounded rate rather than snapping there this same tick.
            target_angle = math.atan2(wy, wx)
            max_delta = p("max_wheel_turn_rate").value * self.dt
            self.wheel_angle[i] = angle_ramp(self.wheel_angle[i], target_angle, max_delta)
            angle = self.wheel_angle[i]

            # Drive at however much of the desired ground velocity actually lies along
            # the way the wheel is presently pointed, not its full magnitude, so a wheel
            # still mid-turn eases off instead of scrubbing sideways at full speed.
            speed = wx * math.cos(angle) + wy * math.sin(angle)

            # Past a quarter turn, point the wheel the other way and roll it backwards.
            # Same result, and the steering never has to swing through the far side.
            if angle > math.pi / 2:
                angle -= math.pi
                speed = -speed
            elif angle < -math.pi / 2:
                angle += math.pi
                speed = -speed

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
        self.steer_pub.publish(Float64MultiArray(
            data=[st["steer_sign"] * self.steer_aim for st in self.stations]))
        self.wheel_pub.publish(Float64MultiArray(
            data=[st["drive_sign"] * speed / self.radius for st in self.stations]))


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
