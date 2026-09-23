#!/usr/bin/env python3
"""Drive the rover through one scripted manoeuvre and report what it did.

This is the harness behind doc/VERIFICATION.md. Each run launches nothing itself: point
it at an already-running simulation, name a test, and it commands the rover, records
odometry and joint states, and prints a one-line JSON summary.

    ros2 run rover_gazebo measure_rover.py --test forward --duration 5

Tests

    settle      no command; how far does it wander while it beds in
    forward     straight ahead; distance, lateral drift, yaw drift
    arc_left    arc at the tightest radius; degrees of turn
    arc_right   the mirror of it
    reverse_arc wheels turned, driving backwards; a car turns the other way
    crab        sideways without changing heading
    spot        spin in place; degrees turned and how far the body slid
    obstacle    drive straight over whatever the world puts in the way

Every test also reports chassis roll and pitch, the free rocker's travel, and the
residual of each suspension constraint, so a run says not only where the rover went
but whether the suspension stayed together on the way.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ROCKERS = ("FLS_joint", "BLS_joint", "BRS_joint", "FRS_joint")

# The constraints the suspension is meant to satisfy, as (a, b, ratio) meaning
# a - ratio * b should stay at zero. Mirrors config/rover_kinematics.yaml.
CONSTRAINTS = (("BLS_joint", "FLS_joint", -0.577165),
               ("BRS_joint", "FRS_joint", -0.572734),
               ("BLS_joint", "BRS_joint", -1.0))


def rpy(q):
    """Roll, pitch, yaw from a quaternion."""
    sr = 2.0 * (q.w * q.x + q.y * q.z)
    cr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    sp = 2.0 * (q.w * q.y - q.z * q.x)
    sp = max(-1.0, min(1.0, sp))
    sy = 2.0 * (q.w * q.z + q.x * q.y)
    cy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sr, cr), math.asin(sp), math.atan2(sy, cy)


class Measure(Node):

    def __init__(self, args):
        super().__init__("measure_rover")
        self.args = args
        self.odom = None
        self.joints = {}
        self.samples = []
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.create_subscription(JointState, "joint_states", self.on_joints, 20)
        self.cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.mode = self.create_publisher(String, "rover/steer_mode", 10)

    def on_odom(self, msg):
        self.odom = msg

    def on_joints(self, msg):
        for i, n in enumerate(msg.name):
            if i < len(msg.position):
                self.joints[n] = msg.position[i]

    # -- helpers ----------------------------------------------------------
    def spin(self, seconds, twist=None):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            if twist is not None:
                self.cmd.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            self.record()

    def record(self):
        if self.odom is None:
            return
        p = self.odom.pose.pose.position
        r, pi, y = rpy(self.odom.pose.pose.orientation)
        v = self.odom.twist.twist.linear
        self.samples.append({
            "x": p.x, "y": p.y, "z": p.z,
            "roll": r, "pitch": pi, "yaw": y,
            "speed": math.hypot(v.x, v.y),
            "rockers": {j: self.joints.get(j, float("nan")) for j in ROCKERS},
        })

    def wait_ready(self, timeout=30.0):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is not None and self.joints:
                return True
        return False

    def set_coupling_gains(self, stiffness, damping):
        cli = self.create_client(SetParameters, "/rocker_coupling/set_parameters")
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("no rocker_coupling node; stiffness left alone")
            return False
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name=n, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))
            for n, v in (("stiffness", stiffness), ("damping", damping)) if v is not None]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.done()

    # -- the manoeuvres ---------------------------------------------------
    def run(self):
        a = self.args
        if not self.wait_ready():
            print(json.dumps({"error": "no odometry or joint states"}))
            return 1

        if a.coupling_stiffness is not None or a.coupling_damping is not None:
            self.set_coupling_gains(a.coupling_stiffness, a.coupling_damping)

        # Let it bed in before anything is commanded, so settling is not counted
        # against the manoeuvre.
        self.spin(a.settle)
        self.samples.clear()
        self.record()
        start = dict(self.samples[0])

        t = Twist()
        mode = "ackermann"
        if a.test == "settle":
            pass
        elif a.test == "forward":
            t.linear.x = a.speed
        elif a.test == "arc_left":
            t.linear.x, t.angular.z = a.speed, a.yaw_rate
        elif a.test == "arc_right":
            t.linear.x, t.angular.z = a.speed, -a.yaw_rate
        elif a.test == "reverse_arc":
            t.linear.x, t.angular.z = -a.speed, -a.yaw_rate
        elif a.test == "crab":
            mode, t.linear.y = "crab", -a.speed        # slide right
        elif a.test == "spot":
            mode, t.angular.z = "spot", -a.yaw_rate    # spin clockwise
        elif a.test == "obstacle":
            t.linear.x = a.speed
        else:
            print(json.dumps({"error": f"unknown test {a.test}"}))
            return 1

        self.mode.publish(String(data=mode))
        self.spin(0.5)
        self.spin(a.duration, t)
        self.cmd.publish(Twist())
        self.spin(0.5)

        return self.summarise(start, mode)

    def summarise(self, start, mode):
        s = self.samples
        end = s[-1]
        dx, dy = end["x"] - start["x"], end["y"] - start["y"]

        # Rotate the displacement into the heading the rover started with, so
        # "forward" and "sideways" mean what they say.
        c, sn = math.cos(start["yaw"]), math.sin(start["yaw"])
        fwd = c * dx + sn * dy
        lat = -sn * dx + c * dy

        yaw = 0.0
        prev = start["yaw"]
        for smp in s:                       # accumulate, so turns past 180 deg count
            d = smp["yaw"] - prev
            yaw += math.atan2(math.sin(d), math.cos(d))
            prev = smp["yaw"]

        # Peak residual is dominated by the drop onto the ground at spawn, so report
        # the settled value alongside it. The settled value is what says whether the
        # suspension is actually held together while driving.
        tail = s[-max(1, len(s) // 10):]
        residuals, settled = {}, {}
        for a_, b_, ratio in CONSTRAINTS:
            key = f"{a_}-{ratio:+.4f}x{b_}"

            def resid(smp, a_=a_, b_=b_, ratio=ratio):
                ra, rb = smp["rockers"][a_], smp["rockers"][b_]
                return abs(ra - ratio * rb) if ra == ra and rb == rb else 0.0

            residuals[key] = round(math.degrees(max(resid(x) for x in s)), 3)
            settled[key] = round(math.degrees(
                sum(resid(x) for x in tail) / len(tail)), 3)

        out = {
            "test": self.args.test,
            "mode": mode,
            "duration_s": self.args.duration,
            "forward_m": round(fwd, 4),
            "lateral_m": round(lat, 4),
            "distance_m": round(math.hypot(dx, dy), 4),
            "yaw_deg": round(math.degrees(yaw), 2),
            "peak_speed_mps": round(max(x["speed"] for x in s), 4),
            "peak_roll_deg": round(max(abs(math.degrees(x["roll"])) for x in s), 3),
            "peak_pitch_deg": round(max(abs(math.degrees(x["pitch"])) for x in s), 3),
            "peak_tilt_deg": round(max(
                math.degrees(math.hypot(x["roll"], x["pitch"])) for x in s), 3),
            "rocker_travel_deg": {
                j: round(math.degrees(max(x["rockers"][j] for x in s)
                                      - min(x["rockers"][j] for x in s)), 3)
                for j in ROCKERS if s[0]["rockers"][j] == s[0]["rockers"][j]
            },
            "worst_constraint_residual_deg": residuals,
            "settled_constraint_residual_deg": settled,
            "settled_rocker_deg": {
                j: round(math.degrees(sum(x["rockers"][j] for x in tail) / len(tail)), 3)
                for j in ROCKERS if s[0]["rockers"][j] == s[0]["rockers"][j]
            },
            "peak_z_m": round(max(x["z"] for x in s), 4),
            "min_z_m": round(min(x["z"] for x in s), 4),
            "settled_z_m": round(sum(x["z"] for x in tail) / len(tail), 4),
            "settled_roll_deg": round(math.degrees(
                sum(x["roll"] for x in tail) / len(tail)), 3),
            "settled_pitch_deg": round(math.degrees(
                sum(x["pitch"] for x in tail) / len(tail)), 3),
            "samples": len(s),
        }
        print(json.dumps(out))
        return 0


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--test", required=True)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--settle", type=float, default=3.0)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--yaw-rate", type=float, default=1.0)
    ap.add_argument("--coupling-damping", type=float, default=None,
                    help="set with --coupling-stiffness; zero both to switch the "
                         "suspension coupling off entirely")
    ap.add_argument("--coupling-stiffness", type=float, default=None,
                    help="override the coupling node's stiffness first; 0 turns the "
                         "suspension coupling off, for a before-and-after comparison")
    a = ap.parse_args(argv[1:])

    rclpy.init()
    node = Measure(a)
    try:
        code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
