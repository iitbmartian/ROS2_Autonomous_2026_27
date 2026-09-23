#!/usr/bin/env python3
"""Keyboard driving for the rover, with the same keys as the Unity build.

    W / S            forward / reverse
    A / D            left / right, meaning depends on the mode
    C                explicit mode only: sweep the wheels back to straight ahead
    Space            stop driving; the wheels stay pointed where they are
    Shift            boost, hold shift with a letter key
    1 / 2 / 3 / 4    Ackermann / crab / spot / explicit
    Q or Ctrl-C      quit

What A and D do in each mode:

    Ackermann  yaw rate, so the rover follows an arc. Front and rear counter-steer.
    Crab       sideways speed, so the rover slides without changing heading.
    Spot       spin rate. W and S do nothing here.
    Explicit   steering rate: slowly turns all four wheels together, like aiming the
               wheels of a shopping cart. Let go and they hold that angle. W and S then
               drive forward/reverse along wherever the wheels are currently pointed,
               and C sweeps them back to straight. Boost speeds up W/S only, not the
               A/D aiming rate, which is deliberately slow and un-boostable.
               The live angle is shown on screen and read back from rover/steer_aim.
               Expect to overshoot the angle you wanted by roughly hold_time times the
               sweep rate, about six degrees at the defaults, because a terminal key
               lapses rather than releasing. Trim it with a tap of the opposite key.

Publishes geometry_msgs/Twist on cmd_vel and std_msgs/String on rover/steer_mode, and
subscribes to std_msgs/Float64 on rover/steer_aim to show and recentre the explicit
angle. In explicit mode angular.z carries a steering rate rather than a body yaw rate;
rover_kinematics_node integrates it, which is why the angle survives a key release.
Terminal keyboards send no key-release event, so a key is held for hold_time seconds
after the last repeat and then lapses.
"""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, String

MODES = {"1": "ackermann", "2": "crab", "3": "spot", "4": "explicit"}

HELP = __doc__.split("Publishes")[0]


class Teleop(Node):

    def __init__(self):
        super().__init__("teleop_rover")
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("max_yaw_rate", 1.0)      # rad/s, Ackermann
        self.declare_parameter("max_spin_rate", 0.7854)  # rad/s, spot
        self.declare_parameter("max_steer_rate", 0.2618)  # rad/s, ~15 deg/s, explicit mode A/D
        self.declare_parameter("boost", 2.5)
        self.declare_parameter("hold_time", 0.4)         # s a key stays live
        self.declare_parameter("update_rate", 50.0)

        self.cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, "rover/steer_mode", 10)
        self.create_subscription(Float64, "rover/steer_aim", self.on_aim, 10)
        self.mode = "ackermann"
        self.held = {}
        self.aim = 0.0          # rad, echoed back by the kinematics node
        self.recentre = False   # C pressed, sweeping the aim to zero
        self.shown = None       # last aim printed, in whole degrees
        self.dt = 1.0 / self.get_parameter("update_rate").value
        self.create_timer(self.dt, self.step)

    def on_aim(self, msg: Float64):
        self.aim = msg.data

    def press(self, ch):
        now = self.get_clock().now().nanoseconds * 1e-9
        boost = ch.isupper()
        low = ch.lower()

        if low in MODES:
            # Drop any held key rather than let it be reinterpreted under the new
            # mode's semantics for up to hold_time: a body-yaw rate is not a steering
            # rate, and vice versa.
            self.held.clear()
            self.mode = MODES[low]
            self.recentre = False
            self.shown = None       # force the aim readout to redraw
            self.mode_pub.publish(String(data=self.mode))
            print(f"\r  mode: {self.mode}          ")
            return
        if low == " ":
            # Stop means stop driving. It also calls off a sweep in progress, since a
            # sweep is something moving, but it leaves the wheels pointed where they are.
            self.held.clear()
            self.recentre = False
            return
        if low == "c":
            self.recentre = True
            return
        if low in "wasd":
            if low in "ad":
                self.recentre = False   # aiming by hand overrides the sweep home
            self.held[low] = (now, boost)

    def step(self):
        p = self.get_parameter
        now = self.get_clock().now().nanoseconds * 1e-9
        hold = p("hold_time").value
        self.held = {k: v for k, v in self.held.items() if now - v[0] < hold}

        boost = p("boost").value if any(b for _, b in self.held.values()) else 1.0
        speed = p("max_speed").value * boost
        fwd = (1.0 if "w" in self.held else 0.0) - (1.0 if "s" in self.held else 0.0)
        turn = (1.0 if "a" in self.held else 0.0) - (1.0 if "d" in self.held else 0.0)

        t = Twist()
        if self.mode == "crab":
            t.linear.x = fwd * speed
            t.linear.y = turn * speed
        elif self.mode == "spot":
            t.angular.z = turn * p("max_spin_rate").value * boost
        elif self.mode == "explicit":
            # Shift is a drive boost only here, so it is taken from the W/S keys alone
            # rather than from any held key. The sweep rate is deliberately fixed.
            drive_boost = p("boost").value if any(
                b for k, (_, b) in self.held.items() if k in "ws") else 1.0
            rate = p("max_steer_rate").value
            t.linear.x = fwd * p("max_speed").value * drive_boost
            if self.recentre:
                # Saturates at the sweep rate while the aim is large and lands exactly
                # on zero as it closes, so it converges without hunting around straight.
                want = -self.aim / self.dt
                t.angular.z = max(-rate, min(rate, want))
                if abs(self.aim) < 0.005:
                    t.angular.z = 0.0
                    self.recentre = False
            else:
                t.angular.z = turn * rate
            self.report_aim()
        else:
            t.linear.x = fwd * speed
            t.angular.z = turn * p("max_yaw_rate").value
        self.cmd.publish(t)

    def report_aim(self):
        """Show the live steering angle, redrawn only when the whole degree changes.

        sys.stdout.write rather than print: the terminal is in raw mode, where a newline
        moves down without returning to column zero.
        """
        deg = round(math.degrees(self.aim))
        if deg != self.shown:
            self.shown = deg
            sys.stdout.write(f"\r  aim: {deg:+4d} deg   ")
            sys.stdout.flush()


def main():
    rclpy.init()
    node = Teleop()
    print(HELP)
    node.mode_pub.publish(String(data="ackermann"))

    fd = sys.stdin.fileno()
    interactive = os.isatty(fd)
    old = termios.tcgetattr(fd) if interactive else None
    try:
        if interactive:
            tty.setraw(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if interactive and select.select([sys.stdin], [], [], 0.0)[0]:
                ch = sys.stdin.read(1)
                if ch in ("\x03", "q", "Q"):
                    break
                node.press(ch)
    finally:
        if interactive:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
