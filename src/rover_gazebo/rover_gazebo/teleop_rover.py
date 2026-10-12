#!/usr/bin/python3
"""Keyboard driving for the rover, with the same keys as the Unity build.

    W / S               forward / reverse
    A / D               left / right, meaning depends on the mode
    C                   explicit/explicit_pid only: sweep the wheels back to straight
    Space               stop driving; the wheels stay pointed where they are
    Shift               boost, hold shift with a letter key
    1 / 2 / 3 / 4 / 5    Ackermann / crab / spot / explicit / explicit_pid
    Q or Ctrl-C         quit

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
    Explicit_PID  every key does exactly what it does in explicit mode above -- same
               A/D/W/S/C/Space, same aim readout. The only difference is invisible from
               here: rover_kinematics_node hands the solved angle/speed to
               control_node.py's own independent per-joint PID loops (one per steering
               joint, one per wheel joint) instead of straight to the ideal
               steer_controller/wheel_controller, and switches which ros2_control
               controllers are active to match. See rover_kinematics_node.py and
               control_node.py.

Publishes geometry_msgs/Twist on cmd_vel and std_msgs/String on rover/steer_mode, and
subscribes to std_msgs/Float64 on rover/steer_aim to show and recentre the explicit
angle. In explicit/explicit_pid modes angular.z carries a steering rate rather than a
body yaw rate, which rover_kinematics_node integrates, which is why the angle survives
a key release. Terminal keyboards send no key-release event, so a key is held for
hold_time seconds after the last repeat and then lapses -- and, since most terminals
only auto-repeat one held key at a time, holding two keys together (e.g. W+A for an
arc) can drop one of them early. teleop_rover_gui.py reads real per-key press/release
events instead and does not have either limitation; use it if that shows up for you.
Both share the exact same driving logic (teleop_common.TeleopCore), so the two never
disagree about what a key does.
"""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from teleop_common import MODE_KEYS, TeleopCore

HELP = __doc__.split("Publishes")[0]


class Teleop(TeleopCore):

    def __init__(self):
        super().__init__("teleop_rover")
        self.held = {}
        self.shown = None       # last aim printed, in whole degrees
        self.create_timer(self.dt, self.step)

    def press(self, ch):
        now = self.get_clock().now().nanoseconds * 1e-9
        boost = ch.isupper()
        low = ch.lower()

        if low in MODE_KEYS:
            # Drop any held key rather than let it be reinterpreted under the new
            # mode's semantics for up to hold_time: a body-yaw rate is not a steering
            # rate, and vice versa.
            self.held.clear()
            self.set_mode(MODE_KEYS[low])
            self.shown = None       # force the aim readout to redraw
            print(f"\r  mode: {self.mode}          ")
            return
        if low == " ":
            # Stop means stop driving. It also calls off a sweep in progress, since a
            # sweep is something moving, but it leaves the wheels pointed where they are.
            self.held.clear()
            self.stop()
            return
        if low == "c":
            self.recentre_on()
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

        if self.mode in ("explicit", "explicit_pid"):
            # Shift is a drive boost only here, so it is taken from the W/S keys alone
            # rather than from any held key. The sweep rate is deliberately fixed.
            boost_active = any(b for k, (_, b) in self.held.items() if k in "ws")
        else:
            boost_active = any(b for _, b in self.held.values())

        fwd = (1.0 if "w" in self.held else 0.0) - (1.0 if "s" in self.held else 0.0)
        turn = (1.0 if "a" in self.held else 0.0) - (1.0 if "d" in self.held else 0.0)

        self.drive(fwd, turn, boost_active)
        if self.mode in ("explicit", "explicit_pid"):
            self.report_aim()

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
