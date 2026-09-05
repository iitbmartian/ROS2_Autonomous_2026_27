#!/usr/bin/env python3
"""Keyboard driving for the rover, with the same keys as the Unity build.

    W / S        forward / reverse
    A / D        left / right, meaning depends on the mode
    Space        stop
    Shift        boost, hold shift with a letter key
    1 / 2 / 3    Ackermann / crab / spot
    Q or Ctrl-C  quit

What A and D do in each mode:

    Ackermann  yaw rate, so the rover follows an arc. Front and rear counter-steer.
    Crab       sideways speed, so the rover slides without changing heading.
    Spot       spin rate. W and S do nothing here.

Publishes geometry_msgs/Twist on cmd_vel and std_msgs/String on rover/steer_mode.
Terminal keyboards send no key-release event, so a key is held for hold_time seconds
after the last repeat and then lapses.
"""

import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

MODES = {"1": "ackermann", "2": "crab", "3": "spot"}

HELP = __doc__.split("Publishes")[0]


class Teleop(Node):

    def __init__(self):
        super().__init__("teleop_rover")
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("max_yaw_rate", 1.0)      # rad/s, Ackermann
        self.declare_parameter("max_spin_rate", 0.7854)  # rad/s, spot
        self.declare_parameter("boost", 2.5)
        self.declare_parameter("hold_time", 0.4)         # s a key stays live
        self.declare_parameter("update_rate", 50.0)

        self.cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, "rover/steer_mode", 10)
        self.mode = "ackermann"
        self.held = {}
        self.create_timer(1.0 / self.get_parameter("update_rate").value, self.step)

    def press(self, ch):
        now = self.get_clock().now().nanoseconds * 1e-9
        boost = ch.isupper()
        low = ch.lower()

        if low in MODES:
            self.mode = MODES[low]
            self.mode_pub.publish(String(data=self.mode))
            print(f"\r  mode: {self.mode}          ")
            return
        if low == " ":
            self.held.clear()
            return
        if low in "wasd":
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
        else:
            t.linear.x = fwd * speed
            t.angular.z = turn * p("max_yaw_rate").value
        self.cmd.publish(t)


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
