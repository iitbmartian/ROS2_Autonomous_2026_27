#!/usr/bin/python3
"""Plot a recorded stress run (an mcap bag from run_stress_suite.sh) to one PNG.

    ros2 run rover_gazebo plot_run.py <bag_dir> <out.png> [--title "slopes / climb"]

Six panels, one quantity each, all on sim time: top-down path, chassis roll/pitch,
steering error per joint, wheel-speed error per joint, IMU vertical acceleration, and
the three suspension constraint residuals. Grey verticals mark mode changes.
"""

import argparse
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

STEER = ("FLE_joint", "FRE_joint", "BLE_joint", "BRE_joint")
WHEEL = ("FLD_joint", "FRD_joint", "BLD_joint", "BRD_joint")
CONSTRAINTS = (("BLS_joint", "FLS_joint", -0.577165, "left four-bar"),
               ("BRS_joint", "FRS_joint", -0.572734, "right four-bar"),
               ("BLS_joint", "BRS_joint", -1.0, "differential"))
# Validated default categorical slots 1-4 (dataviz palette), fixed order FL FR BL BR.
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
STATIONS = ("FL", "FR", "BL", "BR")


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    d = {k: [] for k in ("js", "odom", "imu", "steer_t", "wheel_t", "mode", "clock")}
    names, sim = None, 0.0
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in types:
            continue
        m = deserialize_message(raw, types[topic])
        if topic == "/clock":
            sim = m.clock.sec + m.clock.nanosec * 1e-9
        elif topic in ("/joint_states", "/rec/joint_states"):   # /rec/: bag_throttle.py
            if names is None:
                names = list(m.name)
            d["js"].append((stamp(m), *m.position, *m.velocity))
        elif topic == "/odom":
            q = m.pose.pose.orientation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            d["odom"].append((stamp(m), m.pose.pose.position.x, m.pose.pose.position.y,
                              roll, pitch))
        elif topic == "/imu/data":
            d["imu"].append((stamp(m), m.linear_acceleration.z))
        elif topic in ("/steer_controller/commands", "/explicit_pid_control/steer_target"):
            if len(m.data) == 4:
                d["steer_t"].append((sim, *m.data))
        elif topic in ("/wheel_controller/commands", "/explicit_pid_control/wheel_target"):
            if len(m.data) == 4:
                d["wheel_t"].append((sim, *m.data))
        elif topic == "/rover/steer_mode":
            d["mode"].append((sim, m.data))
    return names, {k: (v if k == "mode" else np.array(v, dtype=float)) for k, v in d.items()}


def track_error(js, cols, targets, use_vel, n):
    """target - measured, the target in force at each joint sample."""
    if len(targets) < 2 or len(js) < 2:
        return None
    k = np.clip(np.searchsorted(targets[:, 0], js[:, 0], side="right") - 1, 0,
                len(targets) - 1)
    have = js[:, 1 + n + np.array(cols)] if use_vel else js[:, 1 + np.array(cols)]
    return targets[k, 1:5] - have


def style(ax, title, ylabel):
    ax.set_title(title, loc="left", fontsize=10, color=INK)
    ax.set_ylabel(ylabel, fontsize=8, color=MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.grid(True, color=GRID, linewidth=0.6)
    for s in ax.spines.values():
        s.set_visible(False)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--title", default="")
    a = ap.parse_args()

    names, d = read(a.bag)
    if names is None or len(d["odom"]) < 2:
        print("bag has no /joint_states or /odom")
        return 1
    js, od = d["js"], d["odom"]
    n = len(names)
    t0 = js[0, 0]
    col = {j: names.index(j) for j in names}

    fig, axes = plt.subplots(3, 2, figsize=(13, 11), constrained_layout=True)
    fig.patch.set_facecolor("#fcfcfb")
    fig.suptitle(a.title or a.bag, x=0.01, ha="left", fontsize=12, color=INK)

    ax = axes[0, 0]
    ax.plot(od[:, 1], od[:, 2], color=SERIES[0], linewidth=2)
    ax.plot(od[0, 1], od[0, 2], "o", color=SERIES[0], markersize=8)
    ax.annotate("start", (od[0, 1], od[0, 2]), xytext=(6, 6), textcoords="offset points",
                fontsize=8, color=MUTED)
    ax.annotate("end", (od[-1, 1], od[-1, 2]), xytext=(6, -10), textcoords="offset points",
                fontsize=8, color=MUTED)
    ax.set_aspect("equal", adjustable="datalim")
    style(ax, "Path, top down (odom)", "y (m)")
    ax.set_xlabel("x (m)", fontsize=8, color=MUTED)

    panels = []
    ax = axes[0, 1]
    for i, (lbl, c) in enumerate((("roll", 3), ("pitch", 4))):
        ax.plot(od[:, 0] - t0, np.degrees(od[:, c]), color=SERIES[i], linewidth=1.5, label=lbl)
    style(ax, "Chassis roll and pitch", "deg")
    ax.legend(fontsize=8, frameon=False)
    panels.append(ax)

    for ax, label, cols, tg, use_vel, unit, conv in (
            (axes[1, 0], "Steering error per joint (target - actual)",
             [col[j] for j in STEER], d["steer_t"], False, "deg", np.degrees),
            (axes[1, 1], "Wheel speed error per joint (target - actual)",
             [col[j] for j in WHEEL], d["wheel_t"], True, "rad/s", lambda x: x)):
        err = track_error(js, cols, tg, use_vel, n)
        if err is not None:
            for i in range(4):
                ax.plot(js[:, 0] - t0, conv(err[:, i]), color=SERIES[i], linewidth=1.0,
                        label=STATIONS[i])
            ax.legend(fontsize=8, frameon=False, ncol=4)
        style(ax, label, unit)
        panels.append(ax)

    ax = axes[2, 0]
    if len(d["imu"]):
        ax.plot(d["imu"][:, 0] - t0, d["imu"][:, 1], color=SERIES[0], linewidth=0.8)
    style(ax, "IMU vertical acceleration (shake shows here)", "m/s²")
    panels.append(ax)

    ax = axes[2, 1]
    for i, (a_, b_, r, lbl) in enumerate(CONSTRAINTS):
        res = np.degrees(js[:, 1 + col[a_]] - r * js[:, 1 + col[b_]])
        ax.plot(js[:, 0] - t0, res - res[0], color=SERIES[i], linewidth=1.2, label=lbl)
    style(ax, "Suspension constraint residual (change from start)", "deg")
    ax.legend(fontsize=8, frameon=False)
    panels.append(ax)

    for ax in panels:
        ax.set_xlabel("sim time (s)", fontsize=8, color=MUTED)
        top = ax.get_ylim()[1]
        for t, mode in d["mode"]:
            ax.axvline(t - t0, color=MUTED, linewidth=0.6, alpha=0.5)
            ax.text(t - t0, top, f" {mode}", fontsize=6, color=MUTED, va="top", rotation=90)

    fig.savefig(a.out, dpi=110, facecolor=fig.get_facecolor())
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
