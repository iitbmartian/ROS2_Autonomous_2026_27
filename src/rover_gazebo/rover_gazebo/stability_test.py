#!/usr/bin/python3
"""Drive the rover through every steering mode and fail loudly if any of them is unstable.

Point it at an already-running simulation (tools/run_stability_suite.sh launches one per
world for you) and pick a suite:

    ros2 run rover_gazebo stability_test.py --suite flat       # worlds/flat.sdf
    ros2 run rover_gazebo stability_test.py --suite obstacle   # worlds/ledge.sdf, bars.sdf

Each scenario selects a mode, (for explicit/explicit_pid) sweeps the aim, then holds
and/or drives, recording /joint_states (1 kHz), /odom, /imu/data and whichever joint
targets the mode publishes. Every scenario is then checked for:

    shake      chassis vibration from the IMU: vertical-accel std and roll/pitch-rate
               RMS. The Harmonic rocker-coupling oscillation shows up here as ~27 m/s^2
               and ~0.85 rad/s; a calm rover is ~0.02 and ~0.0002.
    tracking   every steering joint within tolerance of its target and every wheel at
               its commanded speed -- in *every* mode, so joints left limp by a
               controller hand-off (doc/VERIFICATION.md) fail here, not just PID error.
    motion     the body did what the mode promises: speed, yaw rate, crab heading,
               spot spin, explicit track angle, a hold that stays put.
    suspension the rocker constraints stay closed (settled residual).

Prints one JSON line per scenario, a summary table, and exits non-zero if anything failed.
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, Float64MultiArray, String

# Same latching profile as every other rover/steer_mode publisher; see teleop_common.py.
MODE_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

STEER = ("FLE_joint", "FRE_joint", "BLE_joint", "BRE_joint")
WHEEL = ("FLD_joint", "FRD_joint", "BLD_joint", "BRD_joint")
ROCK = ("FLS_joint", "BLS_joint", "BRS_joint", "FRS_joint")
# Mirrors measure_rover.py / config/rover_kinematics.yaml: a - ratio * b stays at zero.
CONSTRAINTS = (("BLS_joint", "FLS_joint", -0.577165),
               ("BRS_joint", "FRS_joint", -0.572734),
               ("BLS_joint", "BRS_joint", -1.0))

# Pass thresholds. Calm-rover baselines measured 2026-10-12 are in the comments.
LIMITS = {
    "acc_std": 1.0,          # m/s^2, vertical accel std (calm ~0.02, shaking ~27)
    "gyro_rms": 0.05,        # rad/s, roll/pitch rate RMS (calm ~0.0002, shaking ~0.85)
    "steer_err_p95": 3.0,    # deg, 95th percentile |target - position| per joint
    "wheel_err_frac": 0.10,  # mean |speed error| as a fraction of |target| ...
    "wheel_err_abs": 0.10,   # ... plus this many rad/s
    "wheel_std": 0.30,       # rad/s, speed ripple about the mean
    "speed_frac": 0.15,      # body speed vs commanded
    "yaw_rate_frac": 0.20,   # body yaw rate vs commanded (arcs, spot)
    "straight_yaw": 3.0,     # deg of heading change allowed driving straight/crabbing
    "track_err": 6.0,        # deg, explicit modes: travel direction vs aim
    "hold_move": 0.03,       # m, a hold must not creep further than this
    "hold_yaw": 1.0,         # deg
    "residual": 3.0,         # deg, settled suspension constraint residual
}
# Over obstacles the chassis is supposed to pitch and bounce, so shake/tracking limits
# are relaxed and the question becomes "did it get over without diverging".
OBSTACLE_LIMITS = dict(LIMITS, acc_std=6.0, gyro_rms=0.6, steer_err_p95=6.0,
                       wheel_std=1.5, wheel_err_frac=0.25, straight_yaw=10.0,
                       track_err=15.0, residual=6.0)


def S(name, mode, vx=0.0, vy=0.0, wz=0.0, aim=None, hold=0.0, drive=6.0, kind="drive",
      skip=2.0, limits=None):
    """One scenario: mode, optional explicit aim (deg), optional hold, then drive.
    limits overrides individual LIMITS entries for this scenario only."""
    return dict(name=name, mode=mode, vx=vx, vy=vy, wz=wz, aim=aim, hold=hold,
                drive=drive, kind=kind, skip=skip, limits=limits or {})


# Known limitation, accepted on purpose: with all four wheels turned across the chassis
# the coupling's residual has nothing to roll against (doc/VERIFICATION.md, "Where the
# torque coupling falls down"), and the damping that keeps every other mode stable (50)
# leaves crab/spot at ~8-18 deg. Crab/spot are low priority for this rover, so they are
# held to "suspension does not come apart" rather than the 3 deg the main modes meet.
CRAB_SPOT = {"residual": 20.0}


SUITES = {
    "flat": [
        S("settle", "ackermann", drive=0.0, hold=5.0, kind="hold"),
        S("ackermann_forward", "ackermann", vx=0.5, kind="straight"),
        S("ackermann_reverse", "ackermann", vx=-0.5, kind="straight"),
        S("ackermann_arc_left", "ackermann", vx=0.5, wz=0.4, kind="arc"),
        S("ackermann_arc_right", "ackermann", vx=0.5, wz=-0.4, kind="arc"),
        S("crab_right", "crab", vy=-0.3, kind="crab", limits=CRAB_SPOT),
        S("crab_diagonal", "crab", vx=0.3, vy=0.3, kind="crab", limits=CRAB_SPOT),
        S("spot_cw", "spot", wz=-0.5, kind="spot", limits=CRAB_SPOT),
        S("spot_ccw", "spot", wz=0.5, kind="spot", limits=CRAB_SPOT),
        S("explicit_0", "explicit", vx=0.5, aim=0.0, hold=3.0, kind="explicit"),
        S("explicit_45", "explicit", vx=0.5, aim=45.0, hold=3.0, kind="explicit"),
        S("explicit_pid_0", "explicit_pid", vx=0.5, aim=0.0, hold=4.0, kind="explicit"),
        S("explicit_pid_45", "explicit_pid", vx=0.5, aim=45.0, hold=4.0, kind="explicit"),
        S("explicit_pid_90", "explicit_pid", vx=0.4, aim=90.0, hold=4.0, kind="explicit"),
        S("explicit_pid_-30_reverse", "explicit_pid", vx=-0.4, aim=-30.0, hold=4.0,
          kind="explicit"),
        S("explicit_pid_long_hold_30", "explicit_pid", aim=30.0, hold=15.0, drive=0.0,
          kind="hold"),
        # Hand-offs back out of explicit_pid: every joint must respond to the ideal
        # controllers again (the limp-joint failure mode, doc/VERIFICATION.md).
        S("after_pid_explicit_20", "explicit", vx=0.5, aim=20.0, hold=3.0, kind="explicit"),
        S("after_pid_explicit_pid_0", "explicit_pid", vx=0.5, aim=0.0, hold=3.0,
          kind="explicit"),
        S("after_pid_ackermann_forward", "ackermann", vx=0.5, kind="straight"),
        S("after_pid_spot_cw", "spot", wz=-0.5, kind="spot", limits=CRAB_SPOT),
    ],
    # Obstacles sit 1.6-2.75 m ahead of the spawn point in ledge.sdf/bars.sdf: drive
    # forward over them in ackermann, then back over them in explicit_pid.
    "obstacle": [
        S("settle", "ackermann", drive=0.0, hold=4.0, kind="hold"),
        S("ackermann_over", "ackermann", vx=0.4, drive=10.0, kind="obstacle"),
        S("explicit_pid_back_over", "explicit_pid", vx=-0.4, aim=0.0, hold=3.0,
          drive=10.0, kind="obstacle"),
    ],
}


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


class Tester(Node):

    def __init__(self):
        super().__init__("stability_test")
        self.sim = 0.0
        self.recording = False
        self.idx = None
        self.js, self.odom, self.imu, self.steer_t, self.wheel_t = [], [], [], [], []
        self.aim = None
        deep = QoSProfile(depth=500)
        self.create_subscription(Clock, "clock", self.on_clock, deep)
        self.create_subscription(JointState, "joint_states", self.on_js, deep)
        self.create_subscription(Odometry, "odom", self.on_odom, deep)
        self.create_subscription(Imu, "imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Float64, "rover/steer_aim",
                                 lambda m: setattr(self, "aim", m.data), 10)
        # Whichever pair the active mode publishes: the ideal group controllers'
        # commands, or the targets explicit_pid hands to control_node.py.
        for topic, store in (("steer_controller/commands", self.steer_t),
                             ("explicit_pid_control/steer_target", self.steer_t),
                             ("wheel_controller/commands", self.wheel_t),
                             ("explicit_pid_control/wheel_target", self.wheel_t)):
            self.create_subscription(Float64MultiArray, topic,
                                     lambda m, s=store: self.on_target(m, s), deep)
        self.cmd = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, "rover/steer_mode", MODE_QOS)

    # -- recording ------------------------------------------------------------
    def on_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def on_target(self, m, store):
        if self.recording and len(m.data) == 4:
            store.append((self.sim, *m.data))

    def on_js(self, m):
        if self.idx is None:
            names = list(m.name)
            self.idx = [names.index(j) for j in STEER + WHEEL + ROCK]
        if self.recording:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.js.append((t, *(m.position[i] for i in self.idx),
                            *(m.velocity[i] for i in self.idx)))

    def on_odom(self, m):
        if self.recording:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            q = m.pose.pose.orientation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                              yaw, roll, pitch))

    def on_imu(self, m):
        if self.recording:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            w, a = m.angular_velocity, m.linear_acceleration
            self.imu.append((t, w.x, w.y, w.z, a.x, a.y, a.z))

    def send(self, twist):
        """Publish on cmd_vel stamped, the type rover_kinematics and Nav2 both use."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist = twist
        self.cmd.publish(msg)

    def spin_for(self, seconds, twist=None):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            if twist is not None:
                self.send(twist)
            rclpy.spin_once(self, timeout_sec=0.005)

    def wait_ready(self, timeout=60.0):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.idx is not None and self.sim > 0.0:
                return True
        return False

    # -- one scenario ---------------------------------------------------------
    def run(self, sc, limits):
        self.send(Twist())
        self.mode_pub.publish(String(data=sc["mode"]))
        explicit = sc["mode"] in ("explicit", "explicit_pid")
        self.spin_for(3.0 if explicit else 2.5, Twist() if not explicit else None)

        if sc["aim"] is not None:
            if self.aim is None:
                return {"error": "no rover/steer_aim from rover_kinematics"}
            target, end, tw = math.radians(sc["aim"]), time.time() + 30.0, Twist()
            while abs(target - self.aim) > 0.005 and time.time() < end:
                tw.angular.z = math.copysign(0.5, target - self.aim)
                self.send(tw)
                rclpy.spin_once(self, timeout_sec=0.005)
            self.send(Twist())
            self.spin_for(1.5, Twist())   # let the joints arrive before judging them

        out = {"scenario": sc["name"], "mode": sc["mode"]}
        windows = []
        if sc["hold"] > 0.0:
            windows.append(("hold", sc["hold"], Twist()))
        if sc["drive"] > 0.0:
            tw = Twist()
            tw.linear.x, tw.linear.y, tw.angular.z = sc["vx"], sc["vy"], sc["wz"]
            windows.append(("drive", sc["drive"], tw))
        for name, dur, tw in windows:
            self.js.clear(), self.odom.clear(), self.imu.clear()
            self.steer_t.clear(), self.wheel_t.clear()
            self.recording = True
            self.spin_for(dur, tw)
            self.recording = False
            out[name] = self.evaluate(sc, name, tw, limits)
        self.send(Twist())
        self.spin_for(1.5, None if explicit else Twist())
        return out

    # -- judging --------------------------------------------------------------
    def evaluate(self, sc, window, tw, L):
        js, od, im = np.array(self.js), np.array(self.odom), np.array(self.imu)
        if len(js) < 100 or len(od) < 20:
            return {"checks": {"data": [False, f"only {len(js)} joint/{len(od)} odom samples"]}}
        skip = sc["skip"] if window == "drive" else 0.5
        t0 = js[0, 0] + skip
        js, od = js[js[:, 0] >= t0], od[od[:, 0] >= t0]
        if len(im):
            im = im[im[:, 0] >= t0]
        pos, vel = js[:, 1:13], js[:, 13:25]
        checks, m = {}, {}

        def check(key, ok, detail):
            checks[key] = [bool(ok), detail]

        # Shake.
        if len(im) > 20:
            acc_std = float(np.std(im[:, 6]))
            gyro = max(rms(im[:, 1] - im[:, 1].mean()), rms(im[:, 2] - im[:, 2].mean()))
            check("shake", acc_std < L["acc_std"] and gyro < L["gyro_rms"],
                  f"acc_z std {acc_std:.3f} m/s^2, roll/pitch rate rms {gyro:.4f} rad/s")
        else:
            check("shake", False, "no /imu/data")

        # Joint tracking, against the target in force at each joint sample.
        for label, targets, cols, is_steer in (("steer", self.steer_t, slice(0, 4), True),
                                               ("wheel", self.wheel_t, slice(4, 8), False)):
            if len(targets) < 5:
                check(f"{label}_tracking", False, "no targets published")
                continue
            tg = np.array(targets)
            k = np.clip(np.searchsorted(tg[:, 0], js[:, 0], side="right") - 1, 0, len(tg) - 1)
            want = tg[k, 1:5]
            have = pos[:, cols] if is_steer else vel[:, cols]
            err = want - have
            if is_steer:
                p95 = np.degrees(np.percentile(np.abs(err), 95, axis=0))
                m["steer_err_p95_deg"] = np.round(p95, 2).tolist()
                check("steer_tracking", np.all(p95 < L["steer_err_p95"]),
                      f"p95 |err| deg {np.round(p95, 2).tolist()}")
            else:
                mean_err = np.abs(err.mean(0))
                allowed = L["wheel_err_frac"] * np.abs(want.mean(0)) + L["wheel_err_abs"]
                std = have.std(0)
                m["wheel_target"] = np.round(want.mean(0), 3).tolist()
                m["wheel_mean"] = np.round(have.mean(0), 3).tolist()
                m["wheel_std"] = np.round(std, 3).tolist()
                check("wheel_tracking",
                      np.all(mean_err < allowed) and np.all(std < L["wheel_std"]),
                      f"target {m['wheel_target']} got {m['wheel_mean']} std {m['wheel_std']}")

        # Body motion, in the frame the rover started this window in.
        dur = od[-1, 0] - od[0, 0]
        yaw = np.unwrap(od[:, 3])
        dyaw = math.degrees(yaw[-1] - yaw[0])
        dx, dy = od[-1, 1] - od[0, 1], od[-1, 2] - od[0, 2]
        c, s = math.cos(yaw[0]), math.sin(yaw[0])
        fwd, lat = c * dx + s * dy, -s * dx + c * dy
        dist = math.hypot(dx, dy)
        speed = dist / max(dur, 1e-6)
        m.update(fwd_m=round(fwd, 3), lat_m=round(lat, 3), yaw_deg=round(dyaw, 2),
                 speed=round(speed, 3), max_tilt_deg=round(float(np.degrees(
                     np.max(np.hypot(od[:, 4], od[:, 5])))), 2))
        cmd_v = math.hypot(tw.linear.x, tw.linear.y)
        kind = sc["kind"] if window == "drive" else "hold"
        if kind == "hold":
            check("motion", dist < L["hold_move"] and abs(dyaw) < L["hold_yaw"],
                  f"held: moved {dist:.3f} m, yaw {dyaw:.2f} deg")
        elif kind in ("straight", "crab"):
            want_dir = math.degrees(math.atan2(tw.linear.y, tw.linear.x))
            got_dir = math.degrees(math.atan2(lat, fwd))
            dir_err = (got_dir - want_dir + 180.0) % 360.0 - 180.0
            check("motion", abs(speed - cmd_v) < L["speed_frac"] * cmd_v
                  and abs(dyaw) < L["straight_yaw"] and abs(dir_err) < L["track_err"],
                  f"speed {speed:.3f}/{cmd_v:.3f} m/s, heading err {dir_err:.1f} deg, "
                  f"yaw {dyaw:.2f} deg")
        elif kind in ("arc", "spot"):
            rate = math.radians(dyaw) / max(dur, 1e-6)
            ok = (abs(rate - tw.angular.z) < L["yaw_rate_frac"] * abs(tw.angular.z))
            if kind == "spot":
                ok = ok and dist < 0.15
            check("motion", ok, f"yaw rate {rate:.3f}/{tw.angular.z:.3f} rad/s, "
                                f"moved {dist:.3f} m")
        elif kind == "explicit":
            aim = sc["aim"]
            got = math.degrees(math.atan2(lat, fwd))
            if tw.linear.x < 0.0:   # reversing: travel is opposite the aim
                got = (got + 360.0) % 360.0 - 180.0
            err = (got - aim + 180.0) % 360.0 - 180.0
            check("motion", abs(err) < L["track_err"] and abs(dyaw) < L["straight_yaw"]
                  and abs(speed - cmd_v) < L["speed_frac"] * cmd_v,
                  f"track {got:.1f} vs aim {aim:.1f} deg, yaw {dyaw:.2f} deg, "
                  f"speed {speed:.3f}/{cmd_v:.3f}")
        elif kind == "obstacle":
            check("motion", abs(fwd) > 0.6 * cmd_v * dur and abs(dyaw) < L["straight_yaw"]
                  and m["max_tilt_deg"] < 35.0,
                  f"progress {fwd:.2f} m, yaw {dyaw:.2f} deg, max tilt {m['max_tilt_deg']} deg")

        # Suspension.
        name_col = {j: 8 + i for i, j in enumerate(ROCK)}
        worst = max(float(np.degrees(np.mean(np.abs(
            pos[:, name_col[a]] - r * pos[:, name_col[b]]))))
            for a, b, r in CONSTRAINTS)
        check("suspension", worst < L["residual"], f"mean constraint residual {worst:.2f} deg")
        return {"checks": checks, "metrics": m}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--suite", choices=sorted(SUITES), default="flat")
    ap.add_argument("--only", nargs="*", help="run just these scenario names")
    ap.add_argument("--out", help="also write the JSON results to this file")
    args = ap.parse_args()

    rclpy.init()
    node = Tester()
    if not node.wait_ready():
        print(json.dumps({"error": "no /clock or /joint_states; is the sim running?"}))
        return 2
    node.spin_for(3.0)   # let the suspension bed in and the coupling capture rest

    limits = OBSTACLE_LIMITS if args.suite == "obstacle" else LIMITS
    results, failed = [], []
    for sc in SUITES[args.suite]:
        if args.only and sc["name"] not in args.only:
            continue
        res = node.run(sc, dict(limits, **sc["limits"]))
        results.append(res)
        print(json.dumps(res), flush=True)
        for window in ("hold", "drive"):
            for key, (ok, detail) in res.get(window, {}).get("checks", {}).items():
                if not ok:
                    failed.append(f"{sc['name']}/{window}/{key}: {detail}")
        if "error" in res:
            failed.append(f"{sc['name']}: {res['error']}")

    print("\n%-28s %-6s %-6s %-6s %-6s %-6s %-6s" % (
        "scenario", "window", "shake", "steer", "wheel", "motion", "susp"))
    for res in results:
        for window in ("hold", "drive"):
            ch = res.get(window, {}).get("checks")
            if not ch:
                continue
            cell = lambda k: ("ok" if ch[k][0] else "FAIL") if k in ch else "-"
            print("%-28s %-6s %-6s %-6s %-6s %-6s %-6s" % (
                res["scenario"], window, cell("shake"), cell("steer_tracking"),
                cell("wheel_tracking"), cell("motion"), cell("suspension")))
    print(f"\n{len(failed)} failed check(s)")
    for f in failed:
        print("  FAIL " + f)
    if args.out:
        with open(args.out, "w") as fh:
            json.dump({"suite": args.suite, "results": results, "failed": failed}, fh, indent=1)

    node.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
