#!/usr/bin/env python3
"""Turn a Twist into steering angles and wheel speeds for a four-wheel-steer rover.

This is the Gazebo counterpart of Unity's RoverDriveController. Three driving modes,
all of which reduce to the same two lines of algebra: give every wheel the ground
velocity its station would have under the commanded body motion, then point the wheel
along it and roll it at that speed.

    v_wheel = v_body + omega x r

  Ackermann   body moves forward and yaws, so the rover follows an arc. Front and rear
              wheels counter-steer, which roughly halves the turning circle.
  Crab        body translates without yawing, so the rover slides while keeping its
              heading. linear.y is the sideways part.
  Spot        body yaws with no translation, so the rover spins in place.

Writing it as one velocity field rather than three special cases means there is no
singularity at zero speed and spot mode is just the case where linear.x is zero.

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
from std_msgs.msg import Float64MultiArray, String

MODES = ("ackermann", "crab", "spot")
STATIONS = ("FL", "FR", "BL", "BR")


def ramp(current, target, max_delta):
    """Move current toward target by at most max_delta."""
    if target > current:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


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

        self.steer_pub = self.create_publisher(Float64MultiArray, "steer_controller/commands", 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, "wheel_controller/commands", 10)
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
        self.have[2] = ramp(self.have[2], want[2], ang_step)

        vx, vy, omega = self.have
        angles, speeds = [], []

        for st in self.stations:
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

            angle = math.atan2(wy, wx)
            speed = math.hypot(wx, wy)

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

            angles.append(st["steer_sign"] * angle)
            speeds.append(st["drive_sign"] * speed / self.radius)

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
