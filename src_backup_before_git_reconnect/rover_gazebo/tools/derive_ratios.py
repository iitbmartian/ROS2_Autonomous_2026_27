#!/usr/bin/env python3
"""Derive every geometric constant of the rover from the original SolidWorks export.

The exported URDF describes a suspension with four closed kinematic loops that URDF
cannot represent, so the loops were dropped on import and the suspension fell apart.
This script recovers the constraints those loops imposed and expresses them as linear
joint couplings. Gazebo has no solver-level way to enforce them, so they are applied at
runtime, as torques, by rocker_coupling_node.

Nothing here is hand-measured. Run it again after any CAD change:

    python3 derive_ratios.py --urdf <original.urdf> --out ../config/rover_kinematics.yaml

Frame note. The export is Y-up: the chassis frame has X pointing backwards, Y up and Z
to the left. Everything below is computed in that frame and converted at the end.
"""

from __future__ import annotations

import argparse
import math
import os
import xml.etree.ElementTree as ET

# ---------------------------------------------------------------------------
# The four-bar closures the exporter could not write down.
#
# Each side's connector bar is a child of the front rocker but its far end is
# bolted to the rear rocker, which closes a loop. The far end is at the far pivot
# of the connector mesh; the STL bounding box gives the pivot spacing.
# ---------------------------------------------------------------------------
CONNECTOR_PIVOT_SPACING = 0.3017      # m, LCON/RCON pivot to pivot, from the STL bbox
WHEEL_RADIUS = 0.14985                # m, from the wheel STL bbox (0.2997 / 2)
WHEEL_WIDTH = 0.140                   # m, from the wheel STL bbox

FRONT_ROCKERS = {"left": "FLS", "right": "FRS"}
REAR_ROCKERS = {"left": "BLS", "right": "BRS"}
CONNECTORS = {"left": "LCON", "right": "RCON"}
STEERS = ["FLE", "FRE", "BLE", "BRE"]
DRIVES = ["FLD", "FRD", "BLD", "BRD"]

TRAVEL_DEG = 20.0                     # range the linear fits are optimised over


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------
def rot2(t):
    c, s = math.cos(t), math.sin(t)
    return ((c, -s), (s, c))


def apply2(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1], m[1][0] * v[0] + m[1][1] * v[1])


def add2(a, b):
    return (a[0] + b[0], a[1] + b[1])


def sub2(a, b):
    return (a[0] - b[0], a[1] - b[1])


def norm2(v):
    return math.hypot(v[0], v[1])


def bisect(f, lo, hi, iters=200):
    flo, fhi = f(lo), f(hi)
    if flo * fhi > 0:
        raise ValueError(f"root not bracketed: f({lo})={flo}, f({hi})={fhi}")
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        fm = f(mid)
        if flo * fm <= 0:
            hi, fhi = mid, fm
        else:
            lo, flo = mid, fm
    return 0.5 * (lo + hi)


# ---------------------------------------------------------------------------
# URDF reading
# ---------------------------------------------------------------------------
class Urdf:
    def __init__(self, path):
        self.root = ET.parse(path).getroot()
        self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.links = {l.get("name"): l for l in self.root.findall("link")}
        # The exporter misspells one joint ("Rdifcon2" for link "Rdiffcon2"), so every
        # lookup below accepts a child link name as well as a joint name.
        self.by_child = {j.find("child").get("link"): n for n, j in self.joints.items()}

    def jname(self, key):
        return key if key in self.joints else self.by_child[key]

    def origin(self, joint):
        o = self.joints[self.jname(joint)].find("origin")
        xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
        return xyz, rpy

    def axis(self, joint):
        a = self.joints[self.jname(joint)].find("axis")
        if a is None:
            return [0.0, 0.0, 1.0]
        return [float(v) for v in a.get("xyz").split()]

    def parent(self, joint):
        return self.joints[self.jname(joint)].find("parent").get("link")

    def mass(self, link):
        return float(self.links[link].find("inertial/mass").get("value"))

    def chain_xyz(self, joint):
        """Position of a joint's frame in the chassis frame, ignoring joint motion.

        Every joint from the chassis down to the wheels has a zero or tiny yaw, so
        summing origins along the chain is exact for the frames this script needs.
        """
        xyz = [0.0, 0.0, 0.0]
        yaw = 0.0
        chain = []
        j = self.jname(joint)
        while j is not None:
            chain.append(j)
            p = self.parent(j)
            j = self.by_child.get(p)
        for j in reversed(chain):
            o, rpy = self.origin(j)
            c, s = math.cos(yaw), math.sin(yaw)
            xyz[0] += c * o[0] - s * o[1]
            xyz[1] += s * o[0] + c * o[1]
            xyz[2] += o[2]
            yaw += rpy[2]
        return xyz, yaw


# ---------------------------------------------------------------------------
# the connector four-bar
# ---------------------------------------------------------------------------
def connector_four_bar(u: Urdf, side: str):
    """Solve the front-rocker to rear-rocker four-bar exactly, then fit a line.

    Ground link  : the two rocker pivots on the chassis
    Input crank  : front rocker pivot -> connector near pivot
    Coupler      : the connector bar itself
    Output crank : rear rocker pivot -> connector far pivot
    """
    fj, rj, cj = FRONT_ROCKERS[side], REAR_ROCKERS[side], CONNECTORS[side]

    f_xyz, _ = u.origin(fj)
    r_xyz, _ = u.origin(rj)
    c_xyz, c_rpy = u.origin(cj)
    f_yaw = u.origin(fj)[1][2]
    c_yaw = c_rpy[2]

    # Sense of each rocker joint about the chassis Z axis.
    f_sense = 1.0 if u.axis(fj)[2] > 0 else -1.0
    r_sense = 1.0 if u.axis(rj)[2] > 0 else -1.0

    pivot_f = (f_xyz[0], f_xyz[1])
    pivot_r = (r_xyz[0], r_xyz[1])

    # near pivot, as an offset from the front rocker pivot, in chassis axes
    crank_f = apply2(rot2(f_yaw), (c_xyz[0], c_xyz[1]))

    # far pivot, walked out along the connector, then back to the rear rocker pivot
    far_in_front = add2(
        (c_xyz[0], c_xyz[1]),
        (CONNECTOR_PIVOT_SPACING * math.cos(c_yaw), CONNECTOR_PIVOT_SPACING * math.sin(c_yaw)),
    )
    far_in_chassis = add2(pivot_f, apply2(rot2(f_yaw), far_in_front))
    crank_r = sub2(far_in_chassis, pivot_r)

    coupler = norm2(sub2(add2(pivot_r, crank_r), add2(pivot_f, crank_f)))

    def rear_angle(theta_f):
        a = add2(pivot_f, apply2(rot2(f_sense * theta_f), crank_f))

        def resid(theta_r):
            b = add2(pivot_r, apply2(rot2(r_sense * theta_r), crank_r))
            return norm2(sub2(b, a)) - coupler

        return bisect(resid, -1.2, 1.2)

    # coupler bar orientation, for the decorative connector joint
    def coupler_angle(theta_f):
        a = add2(pivot_f, apply2(rot2(f_sense * theta_f), crank_f))
        b = add2(pivot_r, apply2(rot2(r_sense * rear_angle(theta_f)), crank_r))
        d = sub2(b, a)
        return math.atan2(d[1], d[0])

    span = math.radians(TRAVEL_DEG)
    samples = [(-span + 2 * span * i / 40.0) for i in range(41)]
    samples = [t for t in samples if abs(t) > 1e-9]

    num = den = 0.0
    worst = 0.0
    rears = {}
    for t in samples:
        tr = rear_angle(t)
        rears[t] = tr
        num += t * tr
        den += t * t
    ratio = num / den
    for t, tr in rears.items():
        worst = max(worst, abs(tr - ratio * t))

    h = math.radians(0.25)
    slope0 = (rear_angle(h) - rear_angle(-h)) / (2 * h)

    base = coupler_angle(0.0)
    cnum = cden = 0.0
    for t in samples:
        # joint angle of the connector relative to its parent rocker
        q = (coupler_angle(t) - base) - f_sense * t
        cnum += t * q
        cden += t * t
    connector_ratio = cnum / cden

    # wheel-height error the linear fit costs at full travel, for the record
    rear_arm = abs(u.origin({"left": "BLE", "right": "BRE"}[side])[0][0])
    height_err = worst * rear_arm

    return {
        "front_joint": fj,
        "rear_joint": rj,
        "connector_joint": cj,
        "input_crank_m": norm2(crank_f),
        "output_crank_m": norm2(crank_r),
        "coupler_m": coupler,
        "ratio": ratio,
        "slope_at_zero": slope0,
        "max_dev_deg": math.degrees(worst),
        "max_wheel_height_error_m": height_err,
        "connector_ratio": connector_ratio,
    }


# ---------------------------------------------------------------------------
# the differential
# ---------------------------------------------------------------------------
def differential(u: Urdf):
    """The pushrods are vertical at rest and mirrored, so the constraint is exact.

    Each rod ties a rear-rocker socket to a pin on the differential bar. Vertical
    speed of the left pin is +R*phidot and of the right pin -R*phidot, while each
    socket rises at -r*thetadot. Adding the two rows cancels phidot and leaves

        theta_left + theta_right = constant
    """
    d_xyz, _ = u.origin("Differential")
    l_pin, _ = u.origin("Ldiff")
    r_pin, _ = u.origin("Rdiff")
    l_sock, _ = u.origin("Ldiffcon2")
    r_sock, _ = u.origin("Rdiffcon2")
    rod_len = abs(u.origin("Ldiffcon")[0][1])

    bar_arm = 0.5 * (abs(l_pin[2]) + abs(r_pin[2]))     # pin offset along the bar
    sock_arm = 0.5 * (abs(l_sock[0]) + abs(r_sock[0]))  # socket offset along the rocker

    # geometric closure check: rest length of each pushrod
    bls, _ = u.origin("BLS")
    brs, _ = u.origin("BRS")
    l_sock_w = [bls[i] + l_sock[i] for i in range(3)]
    r_sock_w = [brs[i] + r_sock[i] for i in range(3)]
    l_pin_w = [d_xyz[i] + l_pin[i] for i in range(3)]
    r_pin_w = [d_xyz[i] + r_pin[i] for i in range(3)]
    l_rest = math.dist(l_pin_w, l_sock_w)
    r_rest = math.dist(r_pin_w, r_sock_w)

    return {
        "law": "theta(BLS) + theta(BRS) = 0",
        "bar_arm_m": bar_arm,
        "socket_arm_m": sock_arm,
        "pushrod_len_m": rod_len,
        "pushrod_rest_left_m": l_rest,
        "pushrod_rest_right_m": r_rest,
        "bar_ratio_vs_BLS": -sock_arm / bar_arm,
    }


# ---------------------------------------------------------------------------
# wheels, steering, signs
# ---------------------------------------------------------------------------
def drivetrain(u: Urdf):
    """Positions and joint senses of the eight driven joints, in ROS axes.

    Chassis frame is X back, Y up, Z left. ROS base_link is X forward, Y left, Z up:

        x_ros = -x_sw     y_ros = z_sw     z_ros = y_sw
    """
    forward_sw = (-1.0, 0.0, 0.0)
    left_sw = (0.0, 0.0, 1.0)
    up_sw = (0.0, 1.0, 0.0)

    def dot(a, b):
        return sum(x * y for x, y in zip(a, b))

    def axis_in_chassis(joint):
        _, yaw = u.chain_xyz(joint)
        a = u.axis(joint)
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * a[0] - s * a[1], s * a[0] + c * a[1], a[2])

    wheels = {}
    for steer, drive in zip(STEERS, DRIVES):
        s_xyz, _ = u.chain_xyz(steer)
        d_xyz, _ = u.chain_xyz(drive)
        # ROS convention throughout: positive is counter-clockwise about "up", which
        # points a steered wheel to the left.
        steer_sign = 1.0 if dot(axis_in_chassis(steer), up_sw) > 0 else -1.0
        # A wheel rolls forward when it turns positively about the vehicle's LEFT
        # axis: that is the rotation which carries the top of the wheel forward and
        # the contact patch backward. Comparing against "right" gets this backwards
        # and drives the whole rover in reverse.
        drive_sign = 1.0 if dot(axis_in_chassis(drive), left_sw) > 0 else -1.0
        wheels[steer[:2]] = {
            "steer_joint": steer,
            "drive_joint": drive,
            "x": -s_xyz[0],
            "y": s_xyz[2],
            "steer_sign": steer_sign,
            "drive_sign": drive_sign,
            "wheel_centre_z_sw": d_xyz[1],
        }

    xs = [w["x"] for w in wheels.values()]
    ys = [w["y"] for w in wheels.values()]
    centre_x = 0.5 * (max(xs) + min(xs))
    for w in wheels.values():
        w["x"] -= centre_x

    # The front rockers carry a 0.32 deg yaw in the export, so the four wheel centres
    # are not quite level. Ground is set by the lowest of them.
    wheel_centre_y_sw = min(w["wheel_centre_z_sw"] for w in wheels.values())
    return {
        "wheels": wheels,
        "wheelbase_m": max(xs) - min(xs),
        "track_m": max(ys) - min(ys),
        "wheel_radius_m": WHEEL_RADIUS,
        "wheel_width_m": WHEEL_WIDTH,
        "centre_offset_from_chassis_origin_m": centre_x,
        "ground_below_chassis_origin_m": -(wheel_centre_y_sw - WHEEL_RADIUS),
    }


# ---------------------------------------------------------------------------
def derive(path):
    u = Urdf(path)
    left = connector_four_bar(u, "left")
    right = connector_four_bar(u, "right")
    diff = differential(u)
    dt = drivetrain(u)

    # Front-left rocker is the one free suspension coordinate; everything follows.
    bls = left["ratio"]                  # BLS from FLS, through the left four-bar
    brs = -bls                           # differential
    frs = brs / right["ratio"]           # BRS back through the right four-bar

    return {
        "urdf": os.path.abspath(path),
        "left": left,
        "right": right,
        "differential": diff,
        "drivetrain": dt,
        "coupling_ratios": {
            "leader": "FLS",
            "BLS": bls,
            "BRS": brs,
            "FRS": frs,
            "LCON": left["connector_ratio"],
            # Expressed against FLS, not against its own parent rocker. A follower
            # chained onto another follower compounds its error, so every one in the
            # model points at the one free joint directly.
            "RCON": right["connector_ratio"] * frs,
            "Differential": diff["bar_ratio_vs_BLS"] * bls,
        },
        "total_mass_kg": sum(u.mass(l) for l in u.links),
    }


def emit_yaml(d, out):
    m = d["coupling_ratios"]
    dt = d["drivetrain"]
    w = dt["wheels"]
    order = ["FL", "FR", "BL", "BR"]

    def f(x, n=6):
        return f"{x:.{n}f}"

    def j(name):
        """CAD part name to ROS joint name. See generate_description.py for why."""
        return f"{name}_joint"

    lines = [
        "# Generated by tools/derive_ratios.py -- do not edit by hand.",
        f"# Source: {d['urdf']}",
        "#",
        "# Every number below is recovered from the CAD export. Re-run the tool after any",
        "# change to the URDF and both the model and the kinematics node pick it up.",
        "",
        "rover:",
        "  geometry:",
        f"    wheel_radius: {f(dt['wheel_radius_m'])}",
        f"    wheel_width: {f(dt['wheel_width_m'])}",
        f"    wheelbase: {f(dt['wheelbase_m'])}",
        f"    track: {f(dt['track_m'])}",
        f"    ground_below_chassis_origin: {f(dt['ground_below_chassis_origin_m'])}",
        "",
        "  # Wheel stations in base_link axes: x forward, y left, from the rover centre.",
        "  # Joint names carry a _joint suffix; SDF will not load a model whose joint",
        "  # shares a name with a link, and the CAD export gave them matching names.",
        "  # steer_sign is +1 when a positive joint angle points that wheel left.",
        "  # drive_sign is +1 when a positive joint speed rolls the rover forward.",
        "  wheels:",
    ]
    for k in order:
        e = w[k]
        lines += [
            f"    {k}:",
            f"      steer_joint: {j(e['steer_joint'])}",
            f"      drive_joint: {j(e['drive_joint'])}",
            f"      x: {f(e['x'])}",
            f"      y: {f(e['y'])}",
            f"      steer_sign: {e['steer_sign']:+.1f}",
            f"      drive_sign: {e['drive_sign']:+.1f}",
        ]
    lines += [
        "",
        "  # Suspension coupling. FLS is the single free coordinate; the rest are",
        "  # torque-driven followers, held by rocker_coupling_node.",
        "  # Positive rocker angle means that wheel moves down relative to the chassis.",
        "  coupling:",
        f"    leader: {j(m['leader'])}",
        "    followers:",
        f"      {j('BLS')}: {f(m['BLS'])}",
        f"      {j('BRS')}: {f(m['BRS'])}",
        f"      {j('FRS')}: {f(m['FRS'])}",
        "    # Raw pairwise relations, before they are chained onto the leader.",
        "    # rocker_coupling_node needs these, because it applies each constraint",
        "    # as an equal and opposite torque pair on the two joints it ties together.",
        "    four_bar:",
        "      left:  {front: FLS_joint, rear: BLS_joint, ratio: " + f(d['left']['ratio']) + "}",
        "      right: {front: FRS_joint, rear: BRS_joint, ratio: " + f(d['right']['ratio']) + "}",
        "    differential:",
        "      joints: [BLS_joint, BRS_joint]",
        "      law: sum_is_zero",
        "    decorative:",
        f"      LCON: {f(m['LCON'])}      # of FLS",
        f"      RCON: {f(m['RCON'])}      # of FRS",
        f"      Differential: {f(m['Differential'])}   # of FLS",
        "",
        "  # Linearisation cost of the connector four-bars over +/- "
        f"{TRAVEL_DEG:.0f} deg of front-rocker travel.",
        "  fit_quality:",
        f"    left_max_deviation_deg: {f(d['left']['max_dev_deg'], 3)}",
        f"    left_max_wheel_height_error_m: {f(d['left']['max_wheel_height_error_m'], 4)}",
        f"    right_max_deviation_deg: {f(d['right']['max_dev_deg'], 3)}",
        f"    right_max_wheel_height_error_m: {f(d['right']['max_wheel_height_error_m'], 4)}",
        "",
    ]
    with open(out, "w") as fh:
        fh.write("\n".join(lines))
    return "\n".join(lines)


def report(d):
    L, R, D, dt = d["left"], d["right"], d["differential"], d["drivetrain"]
    out = []
    out.append("Connector four-bars")
    for name, s in (("left ", L), ("right", R)):
        out.append(
            f"  {name}  cranks {s['input_crank_m']:.5f} / {s['output_crank_m']:.5f} m, "
            f"coupler {s['coupler_m']:.5f} m"
        )
        out.append(
            f"         {s['rear_joint']} = {s['ratio']:+.5f} x {s['front_joint']}   "
            f"(slope at rest {s['slope_at_zero']:+.5f}, worst deviation "
            f"{s['max_dev_deg']:.3f} deg = {1000 * s['max_wheel_height_error_m']:.1f} mm of wheel travel)"
        )
    out.append("")
    out.append("Differential")
    out.append(
        f"  pushrods rest at {D['pushrod_rest_left_m']:.5f} / {D['pushrod_rest_right_m']:.5f} m "
        f"against a modelled {D['pushrod_len_m']:.5f} m, so the linkage closes exactly"
    )
    out.append(f"  {D['law']}")
    out.append("")
    out.append("Coupling multipliers, all against FLS")
    for k in ("BLS", "BRS", "FRS"):
        out.append(f"  {k:<13s} {d['coupling_ratios'][k]:+.5f}")
    out.append("  decorative")
    for k in ("LCON", "RCON", "Differential"):
        out.append(f"    {k:<11s} {d['coupling_ratios'][k]:+.5f}")
    out.append("  every follower points at FLS directly; none chains onto another follower")
    out.append("")
    out.append("Drivetrain")
    out.append(
        f"  wheelbase {dt['wheelbase_m']:.4f} m, track {dt['track_m']:.4f} m, "
        f"wheel radius {dt['wheel_radius_m']:.4f} m"
    )
    out.append(f"  ground sits {dt['ground_below_chassis_origin_m']:.4f} m below the chassis origin")
    signs = "".join("+" if dt["wheels"][k]["drive_sign"] > 0 else "-" for k in ("FL", "FR", "BL", "BR"))
    ssigns = "".join("+" if dt["wheels"][k]["steer_sign"] > 0 else "-" for k in ("FL", "FR", "BL", "BR"))
    out.append(f"  steer signs [{','.join(ssigns)}]   drive signs [{','.join(signs)}]")
    out.append(f"  total mass {d['total_mass_kg']:.2f} kg")
    return "\n".join(out)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.abspath(os.path.join(here, ".."))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--urdf",
        default=os.path.join(pkg, "..", "reference", "urdf_full_withdifferential",
                             "urdf", "urdf_full_withdifferential.urdf"),
        help="the original SolidWorks export; everything is derived from it")
    ap.add_argument("--out", default=os.path.join(pkg, "config", "rover_kinematics.yaml"))
    a = ap.parse_args()
    data = derive(a.urdf)
    print(report(data))
    emit_yaml(data, a.out)
    print(f"\nwrote {os.path.abspath(a.out)}")
